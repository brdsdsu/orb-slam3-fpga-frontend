library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.pyramid_test_params_pkg.all;

-- Production-parameter pyramid testbench for extractor_top's strict-cell gate.
--
-- Reproduces the real hardware scenario the single-image extractor_top_tb never
-- covered: the 8 EuRoC-mono pyramid levels streamed back-to-back through ONE
-- extractor_top at MAX_WIDTH=752 (=> MAX_CELL_COLS=20, CCW=5 -- the wide-grid
-- path), each level with its own cfg geometry and a reset between (mimicking the
-- FSM's per-level core_rst). Vectors from pyramid_tb_generator.py:
--   testbenches/pyramid_pixels.hex     all levels' pixels, raster, level order
--   testbenches/pyramid_expected.txt   all levels' GATED keypoints, level order
--   pyramid_test_params_pkg.vhd        per-level dims / grid / counts
--
-- Per level: reset, program cfg, stream W*H pixels, check the emitted gated
-- keypoint stream against the golden (coords/score/flags/strict exact, angle
-- within tolerance), and check o_supp_count == PYR_NSUPP(level). Each level's
-- golden is computed INDEPENDENTLY, so any gate-state leak across the inter-level
-- reset makes the next level mismatch. Fails (severity failure) on any mismatch.
--
-- Score configs: the FAST default checks pyramid_expected.txt (response column
-- = FAST score); the Harris run overrides both generics:
--   -gG_USE_HARRIS=true -gG_EXPECTED_FILE=testbenches/pyramid_expected_harris.txt
entity extractor_pyramid_tb is
    generic (
        G_USE_HARRIS    : boolean := false;
        G_EXPECTED_FILE : string  := "testbenches/pyramid_expected.txt"
    );
end entity;

architecture sim of extractor_pyramid_tb is
    constant CLK_PERIOD   : time     := 4 ns;
    constant FIFO_DEPTH   : positive := 1024;   -- holds a level's transient occupancy
    constant CORDIC_WIDTH : natural  := 24;
    constant MAXW         : positive := PYR_MAX_WIDTH;   -- 752 -> CCW=5
    constant FIFO_CNT_W   : positive := ceil_log2(FIFO_DEPTH + 1);

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal i_valid : std_logic := '0';
    signal i_pixel : pixel_t   := (others => '0');
    signal i_sof   : std_logic := '0';

    signal cfg_wcell : coord_t := (others => '0');
    signal cfg_hcell : coord_t := (others => '0');
    signal cfg_ncols : coord_t := (others => '0');
    signal cfg_nrows : coord_t := (others => '0');
    signal img_w     : coord_t := (others => '0');
    signal img_h     : coord_t := (others => '0');

    signal o_valid         : std_logic;
    signal o_x             : coord_t;
    signal o_y             : coord_t;
    signal o_response      : signed(31 downto 0);
    signal o_is_brighter   : std_logic;
    signal o_passed_strict : std_logic;
    signal o_angle         : signed(CORDIC_WIDTH-1 downto 0);
    signal o_drop_count    : unsigned(31 downto 0);
    signal o_supp_count    : unsigned(31 downto 0);
    signal o_fifo_count    : unsigned(FIFO_CNT_W-1 downto 0);

    signal errors_seq : natural := 0;   -- sequencer-side (supp/drop) errors
    signal errors_chk : natural := 0;   -- checker-side (keypoint) errors
    signal done       : boolean := false;
    signal kp_seen      : natural := 0;   -- free-running count of o_valid pulses (diagnostic)
    signal chk_consumed : natural := 0;   -- keypoints the checker actually caught (diagnostic)

    -- Sequencer <-> checker handshake (arm a level, wait for its keypoints)
    signal lvl_go   : natural := 0;   -- sequencer increments to arm level cur_lvl
    signal lvl_done : natural := 0;   -- checker increments after collecting the level
    signal cur_lvl  : natural := 0;

    -- Shortest-path angular distance (handles +-pi wrap); same as extractor_top_tb.
    function angle_diff(a, b : signed) return natural is
        constant FULL_CIRCLE : integer := 2**(CORDIC_WIDTH-1);
        variable d : integer;
    begin
        d := abs(to_integer(a) - to_integer(b));
        if d > FULL_CIRCLE / 2 then
            d := FULL_CIRCLE - d;
        end if;
        return d;
    end function;

begin

    dut : entity work.extractor_top
        generic map (
            MAX_WIDTH            => MAXW,
            THRESHOLD_PERMISSIVE => PYR_TH_PERM,
            THRESHOLD_STRICT     => PYR_TH_STRICT,
            FIFO_DEPTH           => FIFO_DEPTH,
            USE_HARRIS           => G_USE_HARRIS
        )
        port map (
            clk             => clk,
            rst             => rst,
            img_width       => img_w,
            img_height      => img_h,
            cfg_wcell       => cfg_wcell,
            cfg_hcell       => cfg_hcell,
            cfg_ncols       => cfg_ncols,
            cfg_nrows       => cfg_nrows,
            i_valid         => i_valid,
            i_pixel         => i_pixel,
            i_sof           => i_sof,
            o_valid         => o_valid,
            o_x             => o_x,
            o_y             => o_y,
            o_response      => o_response,
            o_is_brighter   => o_is_brighter,
            o_passed_strict => o_passed_strict,
            o_angle         => o_angle,
            o_drop_count    => o_drop_count,
            o_supp_count    => o_supp_count,
            o_fifo_count    => o_fifo_count
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Free-running diagnostic: count every emitted keypoint (all levels).
    monitor : process(clk)
    begin
        if rising_edge(clk) then
            if o_valid = '1' then
                kp_seen <= kp_seen + 1;
            end if;
        end if;
    end process;

    -- ============================================================
    -- Sequencer: for each level -> reset, program cfg, stream pixels,
    -- wait for the checker to collect the level, verify o_supp_count.
    -- ============================================================
    sequencer : process
        file     pix_file : text open read_mode is "testbenches/pyramid_pixels.hex";
        variable line_in  : line;
        variable pix_v    : std_logic_vector(7 downto 0);
    begin
        for L in 0 to PYR_NLEVELS-1 loop
            -- Inter-level reset (mimics the FSM's per-level core_rst pulse).
            rst     <= '1';
            i_valid <= '0';
            i_sof   <= '0';
            wait for 5 * CLK_PERIOD;

            -- Program this level's dimensions + selection-cell geometry.
            img_w     <= to_unsigned(PYR_W(L),     COORD_WIDTH);
            img_h     <= to_unsigned(PYR_H(L),     COORD_WIDTH);
            cfg_wcell <= to_unsigned(PYR_WCELL(L), COORD_WIDTH);
            cfg_hcell <= to_unsigned(PYR_HCELL(L), COORD_WIDTH);
            cfg_ncols <= to_unsigned(PYR_NCOLS(L), COORD_WIDTH);
            cfg_nrows <= to_unsigned(PYR_NROWS(L), COORD_WIDTH);
            wait for 2 * CLK_PERIOD;
            rst <= '0';
            wait until rising_edge(clk);

            -- Arm the checker for level L, then stream the level's pixels.
            cur_lvl <= L;
            lvl_go  <= lvl_go + 1;
            wait until rising_edge(clk);

            for p in 0 to PYR_W(L) * PYR_H(L) - 1 loop
                readline(pix_file, line_in);
                hread(line_in, pix_v);
                i_pixel <= unsigned(pix_v);
                i_valid <= '1';
                if p = 0 then i_sof <= '1'; else i_sof <= '0'; end if;
                wait until rising_edge(clk);
            end loop;
            i_valid <= '0';
            i_sof   <= '0';

            -- Wait for the checker to collect all PYR_NKP(L) keypoints of this level.
            -- Guarded wait: the checker may ALREADY have finished (the last image rows
            -- can be corner-free, so the DUT drains before streaming ends) -- a bare
            -- 'wait until lvl_done = lvl_go' would then hang waiting for an event that
            -- never comes.
            while lvl_done /= lvl_go loop
                wait on lvl_done, lvl_go;
            end loop;

            -- Gate telemetry: exact suppressed-corner count for this level.
            if to_integer(o_supp_count) /= PYR_NSUPP(L) then
                report "L" & to_string(L) & " SUPPCNT = " &
                       to_string(to_integer(o_supp_count)) & ", expected " &
                       to_string(PYR_NSUPP(L)) severity error;
                errors_seq <= errors_seq + 1;
            end if;
            -- No FIFO drops expected (density tuned under drain capacity).
            if to_integer(o_drop_count) /= 0 then
                report "L" & to_string(L) & " DROPCNT = " &
                       to_string(to_integer(o_drop_count)) &
                       " (image too dense -- regenerate vectors)" severity error;
                errors_seq <= errors_seq + 1;
            end if;
            report "L" & to_string(L) & " (" & to_string(PYR_W(L)) & "x" &
                   to_string(PYR_H(L)) & "): " & to_string(PYR_NKP(L)) &
                   " keypoints OK, suppressed " & to_string(to_integer(o_supp_count));
        end loop;

        wait for CLK_PERIOD;   -- let the checker's last increments settle
        assert errors_seq = 0 and errors_chk = 0
            report "PYRAMID TEST FAILED (" & to_string(errors_seq) & " seq + " &
                   to_string(errors_chk) & " checker errors)" severity failure;
        report "PYRAMID TEST PASSED (all " & to_string(PYR_NLEVELS) &
               " levels, production params MAX_WIDTH=" & to_string(MAXW) & ")"
            severity note;
        done <= true;
        wait;
    end process;

    -- ============================================================
    -- Checker: for each armed level, read PYR_NKP(L) golden keypoints and
    -- check the emitted stream in raster order.
    -- ============================================================
    checker : process
        file     exp_file : text open read_mode is G_EXPECTED_FILE;
        variable line_in  : line;
        variable ex, ey, es, eb, estrict, etol : integer;
        variable eangle_v, em10_v, em01_v : std_logic_vector(CORDIC_WIDTH-1 downto 0);
        variable eangle   : signed(CORDIC_WIDTH-1 downto 0);
        variable eresp_v  : std_logic_vector(31 downto 0);
        variable eresp    : signed(31 downto 0);
        variable diff     : natural;
        variable max_diff : natural := 0;
        variable L        : natural := 0;   -- checker's OWN level counter (no cur_lvl race)
        variable stall    : natural;
    begin
        loop
            -- Wait until level L is armed. lvl_go counts armings, so level L is armed
            -- when lvl_go reaches L+1. Using the checker's own L (not cur_lvl) avoids
            -- any cross-delta sampling of the sequencer's level index.
            while (lvl_go < L + 1) and not done loop
                wait on lvl_go, done;
            end loop;
            exit when done;

            for k in 0 to PYR_NKP(L)-1 loop
                readline(exp_file, line_in);
                read(line_in, ex);
                read(line_in, ey);
                read(line_in, es);
                read(line_in, eb);
                read(line_in, estrict);
                hread(line_in, eangle_v); eangle := signed(eangle_v);
                read(line_in, etol);
                hread(line_in, em10_v);
                hread(line_in, em01_v);
                hread(line_in, eresp_v); eresp := signed(eresp_v);

                stall := 0;
                loop
                    wait until rising_edge(clk);
                    wait for 1 ps;
                    exit when o_valid = '1';
                    stall := stall + 1;
                    if stall > 100000 then
                        report "L" & to_string(L) & ": STALLED after " & to_string(k) &
                               " of " & to_string(PYR_NKP(L)) & " keypoints -- DUT emitted " &
                               "fewer than expected. DROPCNT=" &
                               to_string(to_integer(o_drop_count)) & ", SUPPCNT=" &
                               to_string(to_integer(o_supp_count)) & " (model expects SUPP=" &
                               to_string(PYR_NSUPP(L)) & "). FIFO drops => density too high; " &
                               "SUPPCNT>model => gate over-suppresses (the bug)." severity failure;
                    end if;
                end loop;

                if to_integer(unsigned(o_x)) /= ex then
                    report "L" & to_string(L) & " KP " & to_string(k) & ": o_x = " &
                           to_string(to_integer(unsigned(o_x))) & ", expected " &
                           to_string(ex) & " (o_y=" & to_string(to_integer(unsigned(o_y))) &
                           ", exp_y=" & to_string(ey) & ")" severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                if to_integer(unsigned(o_y)) /= ey then
                    report "L" & to_string(L) & " KP " & to_string(k) & ": o_y = " &
                           to_string(to_integer(unsigned(o_y))) & ", expected " &
                           to_string(ey) severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                if o_response /= eresp then
                    report "L" & to_string(L) & " KP " & to_string(k) & " (" &
                           to_string(ex) & "," & to_string(ey) & "): o_response = " &
                           to_string(to_integer(o_response)) & ", expected " &
                           to_string(to_integer(eresp)) severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                if (o_is_brighter = '1' and eb /= 1) or
                   (o_is_brighter = '0' and eb /= 0) then
                    report "L" & to_string(L) & " KP " & to_string(k) &
                           ": o_is_brighter mismatch" severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                if (o_passed_strict = '1' and estrict /= 1) or
                   (o_passed_strict = '0' and estrict /= 0) then
                    report "L" & to_string(L) & " KP " & to_string(k) & " (" &
                           to_string(ex) & "," & to_string(ey) & ", score=" &
                           to_string(es) & "): o_passed_strict = " &
                           to_string(o_passed_strict) & ", expected " &
                           to_string(estrict) severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                diff := angle_diff(o_angle, eangle);
                if diff > max_diff then max_diff := diff; end if;
                if diff > etol then
                    report "L" & to_string(L) & " KP " & to_string(k) & " (" &
                           to_string(ex) & "," & to_string(ey) & "): angle diff " &
                           to_string(diff) & " exceeds tol " & to_string(etol)
                        severity error;
                    errors_chk <= errors_chk + 1;
                end if;
                chk_consumed <= chk_consumed + 1;   -- diagnostic: keypoints caught so far
            end loop;

            lvl_done <= lvl_go;   -- signal the sequencer this level is complete
            L := L + 1;
            exit when L = PYR_NLEVELS;
        end loop;
        report "Checker done. Max angle diff over all levels: " & to_string(max_diff);
        wait;
    end process;

    -- ============================================================
    -- Watchdog: generous bound over the whole pyramid.
    -- ============================================================
    watchdog : process
    begin
        -- Full pyramid: ~13.3k keypoints at ~140 cycles each + streaming ~= 7-10 ms.
        -- 15 ms is a generous upper bound; a healthy run sets 'done' well before this.
        wait for 15 ms;
        if not done then
            report "WATCHDOG at level " & to_string(cur_lvl) &
                   ": DUT emitted kp_seen=" & to_string(kp_seen) &
                   ", checker consumed chk_consumed=" & to_string(chk_consumed) &
                   " (level expects " & to_string(PYR_NKP(cur_lvl)) & "), DROPCNT=" &
                   to_string(to_integer(o_drop_count)) & ", SUPPCNT=" &
                   to_string(to_integer(o_supp_count)) & "." severity failure;
        end if;
        wait;
    end process;

end architecture;
