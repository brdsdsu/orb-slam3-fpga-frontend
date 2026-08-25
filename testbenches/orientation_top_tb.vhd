library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.umax_pkg.all;
use work.orientation_test_params_pkg.all;

-- Testbench for orientation_top (corner_fifo -> moment_processor -> CORDIC -> keypoint).
--
-- Vectors from orientation_top_generator.py (reference model = ORB-SLAM3 angle):
--   testbenches/orientation_push.hex   producer input: x y score brighter + 31x31 patch
--   testbenches/orientation_check.hex  expected: x y score brighter angle tol m10 m01
-- N = TEST_N_VECTORS < FIFO_DEPTH, so the back-to-back push cannot drop corners.
--
-- Producer pushes all N corners; consumer collects N outputs in FIFO order and
-- checks metadata exactly plus angle within the per-vector CORDIC tolerance
-- (wrap-aware angle_diff). Fails (severity failure) on any mismatch.
--
-- Score configs: one check file serves both. o_hresponse is checked against the
-- file's Harris column when G_USE_HARRIS (run with -gG_USE_HARRIS=true), and
-- against constant 0 in the FAST default (the unit is not generated there).
entity orientation_top_tb is
    generic (
        G_USE_HARRIS : boolean := false
    );
end entity;

architecture sim of orientation_top_tb is
    constant CLK_PERIOD   : time     := 4 ns;
    constant FIFO_DEPTH   : positive := 256;
    constant CORDIC_WIDTH : natural  := 24;
    constant N            : natural  := TEST_N_VECTORS;

    constant FIFO_CNT_W : positive := ceil_log2(FIFO_DEPTH + 1);

    -- DUT signals
    signal clk           : std_logic := '0';
    signal rst           : std_logic := '1';
    signal i_valid       : std_logic := '0';
    signal i_x           : coord_t   := (others => '0');
    signal i_y           : coord_t   := (others => '0');
    signal i_score       : pixel_t   := (others => '0');
    signal i_is_brighter : std_logic := '0';
    signal i_patch       : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    signal o_valid       : std_logic;
    signal o_x           : coord_t;
    signal o_y           : coord_t;
    signal o_score       : pixel_t;
    signal o_is_brighter : std_logic;
    signal o_angle       : signed(CORDIC_WIDTH-1 downto 0);
    signal o_hresponse   : signed(31 downto 0);
    signal o_drop_count  : unsigned(31 downto 0);
    signal o_fifo_count  : unsigned(FIFO_CNT_W-1 downto 0);

    signal errors  : natural := 0;
    signal done    : boolean := false;

    -- Shortest-path angular distance, accounting for +-pi wrap
    function angle_diff(a, b : signed) return natural is
        constant FULL_CIRCLE : integer := 2**(CORDIC_WIDTH-1);  -- 2pi span
        variable d : integer;
    begin
        d := abs(to_integer(a) - to_integer(b));
        if d > FULL_CIRCLE / 2 then            -- > pi: take the short way round
            d := FULL_CIRCLE - d;
        end if;
        return d;
    end function;

begin

    dut : entity work.orientation_top
        generic map (
            FIFO_DEPTH => FIFO_DEPTH,
            USE_HARRIS => G_USE_HARRIS
        )
        port map (
            clk           => clk,
            rst           => rst,
            i_valid       => i_valid,
            i_x           => i_x,
            i_y           => i_y,
            i_score       => i_score,
            i_is_brighter => i_is_brighter,
            i_patch       => i_patch,
            o_valid       => o_valid,
            o_x           => o_x,
            o_y           => o_y,
            o_score       => o_score,
            o_is_brighter => o_is_brighter,
            o_angle       => o_angle,
            o_hresponse   => o_hresponse,
            o_drop_count  => o_drop_count,
            o_fifo_count  => o_fifo_count
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Reset driver
    rst_proc : process
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait;
    end process;

    -- ============================================================
    -- Producer: push all N corners back-to-back (tests FIFO buffering).
    -- N < FIFO_DEPTH guarantees no drops.
    -- ============================================================
    producer : process
        file     push_file : text open read_mode is "testbenches/orientation_push.hex";
        variable line_in   : line;
        variable x_int, y_int, sc_int, br_int : integer;
        variable pix_v     : std_logic_vector(7 downto 0);
        variable patch_v   : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    begin
        wait until rst = '0';
        wait until rising_edge(clk);

        for k in 0 to N-1 loop
            readline(push_file, line_in);
            read(line_in, x_int);
            read(line_in, y_int);
            read(line_in, sc_int);
            read(line_in, br_int);
            for row in 0 to PATCH_SIZE-1 loop
                for col in 0 to PATCH_SIZE-1 loop
                    hread(line_in, pix_v);
                    patch_v(row, col) := unsigned(pix_v);
                end loop;
            end loop;

            i_x           <= to_unsigned(x_int, COORD_WIDTH);
            i_y           <= to_unsigned(y_int, COORD_WIDTH);
            i_score       <= to_unsigned(sc_int, PIXEL_WIDTH);
            i_is_brighter <= '1' when br_int = 1 else '0';
            i_patch       <= patch_v;
            i_valid       <= '1';
            wait until rising_edge(clk);
        end loop;

        i_valid <= '0';
        wait;
    end process;

    -- ============================================================
    -- Consumer: collect N outputs in order, check each.
    -- Order is preserved by the FIFO, so the k-th output corresponds
    -- to the k-th pushed corner.
    -- ============================================================
    consumer : process
        file     check_file : text open read_mode is "testbenches/orientation_check.hex";
        variable line_in    : line;
        variable ex, ey, es, eb, etol : integer;
        variable eangle_v   : std_logic_vector(CORDIC_WIDTH-1 downto 0);
        variable em10_v     : std_logic_vector(CORDIC_WIDTH-1 downto 0);
        variable em01_v     : std_logic_vector(CORDIC_WIDTH-1 downto 0);
        variable eangle     : signed(CORDIC_WIDTH-1 downto 0);
        variable ehresp_v   : std_logic_vector(31 downto 0);
        variable ehresp     : signed(31 downto 0);
        variable diff       : natural;
        variable max_diff   : natural := 0;
    begin
        wait until rst = '0';

        for k in 0 to N-1 loop
            readline(check_file, line_in);
            read(line_in, ex);
            read(line_in, ey);
            read(line_in, es);
            read(line_in, eb);
            hread(line_in, eangle_v); eangle := signed(eangle_v);
            read(line_in, etol);
            hread(line_in, em10_v);
            hread(line_in, em01_v);
            hread(line_in, ehresp_v);
            if G_USE_HARRIS then
                ehresp := signed(ehresp_v);
            else
                ehresp := (others => '0');   -- unit not generated: constant 0
            end if;

            -- Wait for the next output pulse
            loop
                wait until rising_edge(clk);
                wait for 1 ps;             -- let DUT outputs settle past delta cycles
                exit when o_valid = '1';
            end loop;

            -- Metadata: must match exactly (catches any swap/misrouting)
            if to_integer(unsigned(o_x)) /= ex then
                report "Corner " & to_string(k) & ": o_x = " &
                       to_string(to_integer(unsigned(o_x))) &
                       ", expected " & to_string(ex) severity error;
                errors <= errors + 1;
            end if;
            if to_integer(unsigned(o_y)) /= ey then
                report "Corner " & to_string(k) & ": o_y = " &
                       to_string(to_integer(unsigned(o_y))) &
                       ", expected " & to_string(ey) severity error;
                errors <= errors + 1;
            end if;
            if to_integer(unsigned(o_score)) /= es then
                report "Corner " & to_string(k) & ": o_score = " &
                       to_string(to_integer(unsigned(o_score))) &
                       ", expected " & to_string(es) severity error;
                errors <= errors + 1;
            end if;
            if (o_is_brighter = '1' and eb /= 1) or
               (o_is_brighter = '0' and eb /= 0) then
                report "Corner " & to_string(k) & ": o_is_brighter = " &
                       to_string(o_is_brighter) &
                       ", expected " & to_string(eb) severity error;
                errors <= errors + 1;
            end if;
            if o_hresponse /= ehresp then
                report "Corner " & to_string(k) & ": o_hresponse = " &
                       to_string(to_integer(o_hresponse)) &
                       ", expected " & to_string(to_integer(ehresp)) severity error;
                errors <= errors + 1;
            end if;

            -- Angle: within per-vector CORDIC tolerance
            diff := angle_diff(o_angle, eangle);
            if diff > max_diff then
                max_diff := diff;
            end if;
            if diff > etol then
                report "Corner " & to_string(k) & ": angle diff " &
                       to_string(diff) & " exceeds tol " & to_string(etol) &
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
        assert errors = 0 report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;
	

end architecture;