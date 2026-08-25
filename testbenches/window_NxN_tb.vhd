library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.window_test_params_pkg.all;

-- Testbench for window_NxN (sliding NxN window generator).
--
-- Config from window_test_params_pkg; data from window_NxN_generator.py:
--   testbenches/window_stream.hex     stimulus: one (pixel, sof) per line
--   testbenches/window_expected.hex   golden: x  y  then N*N patch bytes
--
-- Two checker variants selected by image size (USE_PRELOAD): small images load
-- all expected windows into memory; large ones stream them from file to bound
-- ModelSim memory. Both compare coordinates and every patch pixel on each
-- o_valid and require the window COUNT to match exactly. Fails on any mismatch.
entity window_NxN_tb is
end entity;

architecture sim of window_NxN_tb is
    -----------------------------------------------------------------
    -- Test configuration
    -----------------------------------------------------------------
    constant CLK_PERIOD     : time    := 4 ns;
    constant N              : natural := TEST_N;
    constant IMG_WIDTH      : natural := TEST_IMG_WIDTH;
    constant IMG_HEIGHT     : natural := TEST_IMG_HEIGHT;

    constant DRAIN_CYCLES   : natural := 8;
    constant STREAM_FILE    : string  := "testbenches/window_stream.hex";
    constant EXPECTED_FILE  : string  := "testbenches/window_expected.hex";

    -- Switch between preload and streaming strategies based on image size.
    -- 128x128 with N=7 = 14884 windows ≈ ~45 MB in ModelSim's representation,
    -- which is comfortable. Above that, switch to streaming.
    constant PRELOAD_THRESHOLD : natural := 128 * 128;
    constant USE_PRELOAD       : boolean :=
        (IMG_WIDTH * IMG_HEIGHT) <= PRELOAD_THRESHOLD;

    -----------------------------------------------------------------
    -- DUT signals
    -----------------------------------------------------------------
    signal clk     : std_logic := '0';
    signal rst     : std_logic := '1';
    signal i_valid : std_logic := '0';
    signal i_pixel : pixel_t   := (others => '0');
    signal i_sof   : std_logic := '0';
    signal o_valid : std_logic;
    signal o_x     : coord_t;
    signal o_y     : coord_t;
    signal o_patch : patch_arr_t(0 to N-1, 0 to N-1);

    -----------------------------------------------------------------
    -- Shared bookkeeping (used by both checker variants)
    -----------------------------------------------------------------
    signal errors        : natural := 0;
    signal checks        : natural := 0;
    signal stimulus_done : boolean := false;
    signal done          : boolean := false;
begin

    dut : entity work.window_nxn
        generic map (
            N          => N,
			MAX_WIDTH  => IMG_WIDTH
        )
        port map (
            clk			=> clk,
            rst			=> rst,
			img_width	=> to_unsigned(IMG_WIDTH, COORD_WIDTH),
			img_height	=> to_unsigned(IMG_HEIGHT, COORD_WIDTH),
            i_valid		=> i_valid,
            i_pixel		=> i_pixel,
            i_sof		=> i_sof,
            o_valid		=> o_valid,
            o_x			=> o_x,
            o_y			=> o_y,
            o_patch		=> o_patch
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -----------------------------------------------------------------
    -- Stimulus (identical for both modes)
    -----------------------------------------------------------------
    stim : process
        file     stream_file : text open read_mode is STREAM_FILE;
        variable line_in     : line;
        variable pix_v       : std_logic_vector(7 downto 0);
        variable sof_int     : integer;
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait until rising_edge(clk);

        while not endfile(stream_file) loop
            readline(stream_file, line_in);
            hread(line_in, pix_v);
            read(line_in, sof_int);

            i_pixel <= unsigned(pix_v);
            i_sof   <= '1' when sof_int = 1 else '0';
            i_valid <= '1';

            wait until rising_edge(clk);
        end loop;

        i_valid <= '0';
        i_sof   <= '0';

        wait for DRAIN_CYCLES * CLK_PERIOD;

        stimulus_done <= true;
        wait;
    end process;

    -----------------------------------------------------------------
    -- Mode A: Preload all expected windows into memory at startup.
    -- Fast for small images; memory-heavy for large ones.
    -----------------------------------------------------------------
    gen_preload : if USE_PRELOAD generate
        constant MAX_EXPECTED : natural :=
            (IMG_HEIGHT - N + 1) * (IMG_WIDTH - N + 1);

        type exp_x_arr_t is array (0 to MAX_EXPECTED-1) of coord_t;
        type exp_y_arr_t is array (0 to MAX_EXPECTED-1) of coord_t;
        type exp_patch_arr_t is array (0 to MAX_EXPECTED-1)
                                  of patch_arr_t(0 to N-1, 0 to N-1);

        signal exp_x     : exp_x_arr_t;
        signal exp_y     : exp_y_arr_t;
        signal exp_patch : exp_patch_arr_t;
        signal exp_count : natural := 0;
        signal exp_rd    : natural := 0;
        signal loaded    : boolean := false;
    begin

        ---------------------------------------------------------
        -- Load expected outputs at startup
        ---------------------------------------------------------
        loader : process
            file     exp_file : text open read_mode is EXPECTED_FILE;
            variable line_in  : line;
            variable x_v      : std_logic_vector(15 downto 0);
            variable y_v      : std_logic_vector(15 downto 0);
            variable pix_v    : std_logic_vector(7 downto 0);
            variable idx      : natural := 0;
            variable tmp_patch : patch_arr_t(0 to N-1, 0 to N-1);
        begin
            while not endfile(exp_file) loop
                readline(exp_file, line_in);
                hread(line_in, x_v);
                hread(line_in, y_v);
                exp_x(idx) <= resize(unsigned(x_v), COORD_WIDTH);
                exp_y(idx) <= resize(unsigned(y_v), COORD_WIDTH);

                for row in 0 to N-1 loop
                    for col in 0 to N-1 loop
                        hread(line_in, pix_v);
                        tmp_patch(row, col) := unsigned(pix_v);
                    end loop;
                end loop;
                exp_patch(idx) <= tmp_patch;

                idx := idx + 1;
            end loop;

            exp_count <= idx;
            loaded    <= true;
            report "[PRELOAD mode] Loaded " & to_string(idx) & " expected windows.";
            wait;
        end process;

        ---------------------------------------------------------
        -- Checker: clocked, compares against in-memory arrays
        ---------------------------------------------------------
        check_preload : process(clk)
            variable mismatch : boolean;
        begin
            if rising_edge(clk) then
                if o_valid = '1' then
                    mismatch := false;

                    if o_x /= exp_x(exp_rd) then
                        report "X MISMATCH at check " & to_string(checks) &
                               ": got " & to_string(to_integer(o_x)) &
                               ", expected " & to_string(to_integer(exp_x(exp_rd)))
                            severity error;
                        mismatch := true;
                    end if;
                    if o_y /= exp_y(exp_rd) then
                        report "Y MISMATCH at check " & to_string(checks) &
                               ": got " & to_string(to_integer(o_y)) &
                               ", expected " & to_string(to_integer(exp_y(exp_rd)))
                            severity error;
                        mismatch := true;
                    end if;
                    for row in 0 to N-1 loop
                        for col in 0 to N-1 loop
                            if o_patch(row, col) /= exp_patch(exp_rd)(row, col) then
                                report "PATCH MISMATCH at check " & to_string(checks) &
                                       " (x=" & to_string(to_integer(o_x)) &
                                       ", y=" & to_string(to_integer(o_y)) &
                                       ") at [" & to_string(row) & "," & to_string(col) &
                                       "]: got " & to_string(to_integer(o_patch(row, col))) &
                                       ", expected " & to_string(to_integer(exp_patch(exp_rd)(row, col)))
                                    severity error;
                                mismatch := true;
                            end if;
                        end loop;
                    end loop;

                    if mismatch then
                        errors <= errors + 1;
                    end if;
                    checks <= checks + 1;
                    exp_rd <= exp_rd + 1;
                end if;
            end if;
        end process;

        ---------------------------------------------------------
        -- End-of-test reporter (preload mode)
        ---------------------------------------------------------
        reporter_preload : process
        begin
            wait until stimulus_done;
            wait for CLK_PERIOD;

            report "Checked " & to_string(checks) &
                   " of " & to_string(exp_count) &
                   " expected windows, " & to_string(errors) & " errors.";
            if checks /= exp_count then
                report "Mismatch in window count: expected " &
                       to_string(exp_count) & ", got " & to_string(checks)
                    severity error;
            end if;
            assert errors = 0 and checks = exp_count
                report "TEST FAILED" severity failure;
            report "TEST PASSED" severity note;
            done <= true;
            wait;
        end process;

    end generate gen_preload;

    -----------------------------------------------------------------
    -- Mode B: Stream expected outputs from file as DUT produces them.
    -- Bounded memory usage; works for any image size.
    -----------------------------------------------------------------
    gen_streaming : if not USE_PRELOAD generate
    begin

        check_streaming : process
            file     exp_file : text open read_mode is EXPECTED_FILE;
            variable line_in  : line;
            variable x_v      : std_logic_vector(15 downto 0);
            variable y_v      : std_logic_vector(15 downto 0);
            variable pix_v    : std_logic_vector(7 downto 0);
            variable exp_x_v  : coord_t;
            variable exp_y_v  : coord_t;
            variable exp_patch_v : patch_arr_t(0 to N-1, 0 to N-1);
            variable mismatch : boolean;
        begin
            report "[STREAMING mode] Reading expected windows from file as they're produced.";

            wait until rst = '0';
            wait until rising_edge(clk);

            loop
                -- Wait for next valid DUT output or end of stimulus
                loop
                    wait until rising_edge(clk);
                    exit when o_valid = '1' or stimulus_done;
                end loop;

                exit when stimulus_done and o_valid /= '1';

                if endfile(exp_file) then
                    report "DUT produced more outputs than expected (extra at check "
                           & to_string(checks) & ")"
                        severity error;
                    errors <= errors + 1;
                    exit;
                end if;

                readline(exp_file, line_in);
                hread(line_in, x_v);
                hread(line_in, y_v);
                exp_x_v := resize(unsigned(x_v), COORD_WIDTH);
                exp_y_v := resize(unsigned(y_v), COORD_WIDTH);

                for row in 0 to N-1 loop
                    for col in 0 to N-1 loop
                        hread(line_in, pix_v);
                        exp_patch_v(row, col) := unsigned(pix_v);
                    end loop;
                end loop;

                mismatch := false;
                if o_x /= exp_x_v then
                    report "X MISMATCH at check " & to_string(checks) &
                           ": got " & to_string(to_integer(o_x)) &
                           ", expected " & to_string(to_integer(exp_x_v))
                        severity error;
                    mismatch := true;
                end if;
                if o_y /= exp_y_v then
                    report "Y MISMATCH at check " & to_string(checks) &
                           ": got " & to_string(to_integer(o_y)) &
                           ", expected " & to_string(to_integer(exp_y_v))
                        severity error;
                    mismatch := true;
                end if;
                for row in 0 to N-1 loop
                    for col in 0 to N-1 loop
                        if o_patch(row, col) /= exp_patch_v(row, col) then
                            report "PATCH MISMATCH at check " & to_string(checks) &
                                   " (x=" & to_string(to_integer(o_x)) &
                                   ", y=" & to_string(to_integer(o_y)) &
                                   ") at [" & to_string(row) & "," & to_string(col) &
                                   "]: got " & to_string(to_integer(o_patch(row, col))) &
                                   ", expected " & to_string(to_integer(exp_patch_v(row, col)))
                                severity error;
                            mismatch := true;
                        end if;
                    end loop;
                end loop;

                if mismatch then
                    errors <= errors + 1;
                end if;
                checks <= checks + 1;
            end loop;

            if not endfile(exp_file) then
                report "DUT produced fewer outputs than expected"
                    severity error;
            end if;

            report "Checked " & to_string(checks) &
                   " windows, " & to_string(errors) & " errors.";
            assert errors = 0 report "TEST FAILED" severity failure;
            report "TEST PASSED" severity note;
            done <= true;
            wait;
        end process;

    end generate gen_streaming;

end architecture;