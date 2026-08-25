library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.extractor_test_params_pkg.all;

-- Testbench for extractor_top (window + FAST-9 detect/NMS + orientation).
--
-- Config from extractor_test_params_pkg; data from extractor_top_generator.py
-- (reference model = the full software ORB front-end):
--   testbenches/extractor_pixels.hex     raster pixel stream (W*H bytes)
--   testbenches/extractor_expected.txt   per kp: x y score brighter strict angle tol m10 m01
--
-- Producer streams the image one pixel/cycle (i_sof on the first pixel);
-- consumer checks the N keypoints in raster order -- coords/response/flags/
-- strict exact, angle within the per-vector tolerance. A watchdog fails the
-- run on a dropped corner or deadlock. Fails (severity failure) on any mismatch.
--
-- Score configs (mirrors TOP's G_SCORE_TYPE): the default run checks the FAST
-- build against extractor_expected.txt (response column = FAST score). The
-- Harris run overrides both generics:
--   -gG_USE_HARRIS=true -gG_EXPECTED_FILE=testbenches/extractor_expected_harris.txt
-- (same corner set -- detection/NMS/gate are FAST-based in both modes -- only
-- the response column differs).
entity extractor_top_tb is
	generic (
		G_USE_HARRIS    : boolean := false;
		G_EXPECTED_FILE : string  := "testbenches/extractor_expected.txt"
	);
end entity;

architecture sim of extractor_top_tb is
    constant CLK_PERIOD   : time     := 4 ns;
    constant FIFO_DEPTH   : positive := 256;
    constant CORDIC_WIDTH : natural  := 24;

    constant W : positive := TEST_IMG_WIDTH;
    constant H : positive := TEST_IMG_HEIGHT;
    constant N : natural  := TEST_N_EXPECTED;

    constant FIFO_CNT_W : positive := ceil_log2(FIFO_DEPTH + 1);

    -- DUT ports
    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal i_valid : std_logic := '0';
    signal i_pixel : pixel_t   := (others => '0');
    signal i_sof   : std_logic := '0';

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

    signal errors : natural := 0;
    signal done   : boolean := false;

    -- Shortest-path angular distance (handles +-pi wrap)
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
			MAX_WIDTH			 => W,
            THRESHOLD_PERMISSIVE => TEST_TH_PERM,
            THRESHOLD_STRICT     => TEST_TH_STRICT,
            FIFO_DEPTH           => FIFO_DEPTH,
            USE_HARRIS           => G_USE_HARRIS
        )
        port map (
            clk             => clk,
            rst             => rst,
			img_width		=> to_unsigned(W, COORD_WIDTH),
			img_height		=> to_unsigned(H, COORD_WIDTH),
			-- Gate ON: per-level grid geometry (== ComputeCellGrid, from the generator).
			cfg_wcell		=> to_unsigned(TEST_WCELL, COORD_WIDTH),
			cfg_hcell		=> to_unsigned(TEST_HCELL, COORD_WIDTH),
			cfg_ncols		=> to_unsigned(TEST_NCOLS, COORD_WIDTH),
			cfg_nrows		=> to_unsigned(TEST_NROWS, COORD_WIDTH),
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

    rst_proc : process
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait;
    end process;

    -- ============================================================
    -- Producer: stream the whole image in raster order, one pixel/cycle.
    -- i_sof is asserted coincident with the first valid pixel (assumed
    -- window_NxN convention: sof marks the first pixel of the frame).
    -- ============================================================
    producer : process
        file     pix_file : text open read_mode is "testbenches/extractor_pixels.hex";
        variable line_in  : line;
        variable pix_v    : std_logic_vector(7 downto 0);
    begin
        wait until rst = '0';
        wait until rising_edge(clk);

        for p in 0 to W*H-1 loop
            readline(pix_file, line_in);
            hread(line_in, pix_v);
            i_pixel <= unsigned(pix_v);
            i_valid <= '1';
            if p = 0 then
                i_sof <= '1';
            else
                i_sof <= '0';
            end if;
            wait until rising_edge(clk);
        end loop;

        i_valid <= '0';
        i_sof   <= '0';
        wait;
    end process;

    -- ============================================================
    -- Consumer: collect N keypoints in raster order, check each.
    -- ============================================================
    consumer : process
        file     exp_file : text open read_mode is G_EXPECTED_FILE;
        variable line_in  : line;
        variable ex, ey, es, eb, estrict, etol : integer;
        variable eangle_v, em10_v, em01_v : std_logic_vector(CORDIC_WIDTH-1 downto 0);
        variable eangle   : signed(CORDIC_WIDTH-1 downto 0);
        variable eresp_v  : std_logic_vector(31 downto 0);
        variable eresp    : signed(31 downto 0);
        variable diff     : natural;
        variable max_diff : natural := 0;
    begin
        wait until rst = '0';

        for k in 0 to N-1 loop
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

            -- Wait for the next emitted keypoint
            loop
                wait until rising_edge(clk);
                wait for 1 ps;
                exit when o_valid = '1';
            end loop;

            -- Coordinates: exact (catches window coord-convention / alignment errors)
            if to_integer(unsigned(o_x)) /= ex then
                report "KP " & to_string(k) & ": o_x = " &
                       to_string(to_integer(unsigned(o_x))) &
                       ", expected " & to_string(ex) &
                       " (o_y=" & to_string(to_integer(unsigned(o_y))) &
                       ", exp_y=" & to_string(ey) & ")" severity error;
                errors <= errors + 1;
            end if;
            if to_integer(unsigned(o_y)) /= ey then
                report "KP " & to_string(k) & ": o_y = " &
                       to_string(to_integer(unsigned(o_y))) &
                       ", expected " & to_string(ey) severity error;
                errors <= errors + 1;
            end if;

            -- Response / flags: exact. In the FAST config the expected response
            -- equals the FAST score column; in the Harris config it is the
            -- integer Harris response (same corner set either way).
            if o_response /= eresp then
                report "KP " & to_string(k) & " (" & to_string(ex) & "," &
                       to_string(ey) & "): o_response = " &
                       to_string(to_integer(o_response)) &
                       ", expected " & to_string(to_integer(eresp)) severity error;
                errors <= errors + 1;
            end if;
            if (o_is_brighter = '1' and eb /= 1) or
               (o_is_brighter = '0' and eb /= 0) then
                report "KP " & to_string(k) & ": o_is_brighter = " &
                       to_string(o_is_brighter) & ", expected " &
                       to_string(eb) severity error;
                errors <= errors + 1;
            end if;
            if (o_passed_strict = '1' and estrict /= 1) or
               (o_passed_strict = '0' and estrict /= 0) then
                report "KP " & to_string(k) & " (" & to_string(ex) & "," &
                       to_string(ey) & ", score=" & to_string(es) &
                       "): o_passed_strict = " & to_string(o_passed_strict) &
                       ", expected " & to_string(estrict) severity error;
                errors <= errors + 1;
            end if;

            -- Angle: within per-vector CORDIC tolerance
            diff := angle_diff(o_angle, eangle);
            if diff > max_diff then
                max_diff := diff;
            end if;
            if diff > etol then
                report "KP " & to_string(k) & " (" & to_string(ex) & "," &
                       to_string(ey) & "): angle diff " & to_string(diff) &
                       " exceeds tol " & to_string(etol) &
                       " (got " & to_string(to_integer(o_angle)) &
                       ", exp " & to_string(to_integer(eangle)) &
                       ", m10=" & to_string(to_integer(signed(em10_v))) &
                       ", m01=" & to_string(to_integer(signed(em01_v))) & ")"
                    severity error;
                errors <= errors + 1;
            end if;
        end loop;

        report "Collected " & to_string(N) & " keypoints. Max angle diff: " &
               to_string(max_diff) & ". Drops: " &
               to_string(to_integer(o_drop_count));
        report "Gate-suppressed corners: " & to_string(to_integer(o_supp_count)) &
               " (expected " & to_string(TEST_N_SUPPRESSED) & ")";
        assert to_integer(o_supp_count) = TEST_N_SUPPRESSED
            report "GATE SUPPRESS COUNT MISMATCH: got " &
                   to_string(to_integer(o_supp_count)) & ", expected " &
                   to_string(TEST_N_SUPPRESSED) severity failure;
        assert errors = 0 report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;

    -- ============================================================
    -- Watchdog: if the consumer hasn't collected all N keypoints within a
    -- generous bound, the DUT dropped a corner or deadlocked.
    -- Bound: stream time (W*H) + drain (N * ~60 cyc/corner) + margin.
    -- ============================================================
    watchdog : process
        constant TIMEOUT_CYCLES : natural := W*H + N*80 + 20000;
    begin
        wait for TIMEOUT_CYCLES * CLK_PERIOD;
        if not done then
            report "WATCHDOG TIMEOUT: only saw part of the keypoint stream " &
                   "(drop or deadlock). Drops reported = " &
                   to_string(to_integer(o_drop_count)) severity failure;
        end if;
        wait;
    end process;

end architecture;
