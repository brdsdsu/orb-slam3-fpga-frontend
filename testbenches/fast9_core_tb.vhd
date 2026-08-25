library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.fast_pkg.all;

-- Testbench for fast9_core (FAST-9 corner detector).
--
-- Vectors: testbenches/vectors_fast9.hex, from fast9_core_generator.py
-- (reference model = OpenCV FAST). One vector per line, hex bytes:
--   center  c0..c15  threshold  is_corner  is_brighter.
--
-- Self-checking with an expected-value queue (DUT latency 3). is_brighter is
-- only checked when a corner is expected -- the DUT leaves it undefined when no
-- corner is detected. Fails on any mismatch.
entity fast9_core_tb is
end entity;

architecture sim of fast9_core_tb is
    -----------------------------------------------------------------
    -- Test configuration
    -----------------------------------------------------------------
    constant CLK_PERIOD     : time    := 4 ns;   -- 250 MHz
    constant PIPELINE_DEPTH : natural := 3;      -- fast9_core latency
    constant QUEUE_MARGIN   : natural := 4;
    constant QUEUE_DEPTH    : natural := PIPELINE_DEPTH * QUEUE_MARGIN;
    constant DRAIN_CYCLES   : natural := PIPELINE_DEPTH + 4;
    constant VECTOR_FILE    : string  := "testbenches/vectors_fast9.hex";

    -----------------------------------------------------------------
    -- DUT signals
    -----------------------------------------------------------------
    signal clk         : std_logic := '0';
    signal rst         : std_logic := '1';
    signal i_valid     : std_logic := '0';
    signal i_center    : pixel_t     := (others => '0');
    signal i_circle    : circle_t    := (others => (others => '0'));
    signal i_threshold : threshold_t := (others => '0');
    signal o_valid       : std_logic;
    signal o_is_corner   : std_logic;
    signal o_is_brighter : std_logic;

    -----------------------------------------------------------------
    -- Expected-value circular buffer
    -- Each slot holds (is_corner, is_brighter) as a 2-bit value
    -----------------------------------------------------------------
    type exp_queue_t is array (natural range <>) of std_logic_vector(1 downto 0);
    signal exp_queue : exp_queue_t(0 to QUEUE_DEPTH-1) := (others => "00");
    signal exp_wr    : natural := 0;
    signal exp_rd    : natural := 0;

    signal errors    : natural := 0;
    signal checks    : natural := 0;
    signal done      : boolean := false;
begin

    dut : entity work.fast9_core
        port map (
            clk           => clk,
            rst           => rst,
            i_valid       => i_valid,
            i_center      => i_center,
            i_circle      => i_circle,
            i_threshold   => i_threshold,
            o_valid       => o_valid,
            o_is_corner   => o_is_corner,
            o_is_brighter => o_is_brighter
        );

    -- Clock
    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Stimulus + expected-value tracking
    stim : process
        file     vec_file : text open read_mode is VECTOR_FILE;
        variable line_in  : line;
        variable hex_v    : std_logic_vector(7 downto 0);
        variable c_var    : pixel_t;
        variable circ_var : circle_t;
        variable t_var    : threshold_t;
        variable corner_var   : std_logic;
        variable brighter_var : std_logic;
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait until rising_edge(clk);

        while not endfile(vec_file) loop
            readline(vec_file, line_in);

            -- Parse: center c0..c15 threshold is_corner is_brighter
            hread(line_in, hex_v); c_var := unsigned(hex_v);
            for k in 0 to 15 loop
                hread(line_in, hex_v);
                circ_var(k) := unsigned(hex_v);
            end loop;
            hread(line_in, hex_v); t_var := unsigned(hex_v);
            hread(line_in, hex_v); corner_var   := hex_v(0);
            hread(line_in, hex_v); brighter_var := hex_v(0);

            -- Drive one cycle
            i_center    <= c_var;
            i_circle    <= circ_var;
            i_threshold <= t_var;
            i_valid     <= '1';

            -- Push expected (is_corner in bit 1, is_brighter in bit 0)
            exp_queue(exp_wr mod QUEUE_DEPTH) <= corner_var & brighter_var;
            exp_wr <= exp_wr + 1;

            wait until rising_edge(clk);
        end loop;

        i_valid <= '0';

        -- Drain pipeline
        wait for DRAIN_CYCLES * CLK_PERIOD;

        report "Checked " & to_string(checks) &
               " vectors, " & to_string(errors) & " errors.";
        assert errors = 0 report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;

    -- Output checker
    check : process(clk)
        variable expected : std_logic_vector(1 downto 0);
        variable actual   : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if o_valid = '1' then
                expected := exp_queue(exp_rd mod QUEUE_DEPTH);
                actual   := o_is_corner & o_is_brighter;

                -- Only check is_brighter when is_corner is set:
                -- the DUT's is_brighter output is undefined when no corner.
                if expected(1) = '1' then
                    if actual /= expected then
                        report "MISMATCH at check " & to_string(checks) &
                               ": got (corner=" & to_string(o_is_corner) &
                               ", brighter=" & to_string(o_is_brighter) &
                               "), expected (corner=" & to_string(expected(1)) &
                               ", brighter=" & to_string(expected(0)) & ")"
                            severity error;
                        errors <= errors + 1;
                    end if;
                else
                    -- No corner expected: only check is_corner bit
                    if o_is_corner /= expected(1) then
                        report "MISMATCH at check " & to_string(checks) &
                               ": got corner=" & to_string(o_is_corner) &
                               ", expected corner=" & to_string(expected(1))
                            severity error;
                        errors <= errors + 1;
                    end if;
                end if;
                checks <= checks + 1;
                exp_rd <= exp_rd + 1;
            end if;
        end if;
    end process;

end architecture;