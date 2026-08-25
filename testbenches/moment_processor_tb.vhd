library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.umax_pkg.all;
use work.moment_processor_test_params_pkg.all;

-- Testbench for moment_processor (intensity-centroid moments m_10 / m_01).
--
-- Vectors: testbenches/moment_vectors.hex (TEST_N_VECTORS lines from
-- moment_processor_generator.py, reference model = ORB-SLAM3 IC_Angle moments).
-- Per line: 961 patch bytes (31x31, row-major) then expected m_10, m_01 as
-- 24-bit two's-complement hex.
--
-- One sequential process (the DUT handles one corner at a time): wait o_ready,
-- pulse i_start, wait o_done, compare both moments exactly. Fails on any
-- mismatch or if fewer than TEST_N_VECTORS vectors were checked.
entity moment_processor_tb is
end entity;

architecture sim of moment_processor_tb is
    constant CLK_PERIOD  : time    := 4 ns;
    constant DRAIN_CYCLES : natural := 32;
    constant VECTOR_FILE : string  := "testbenches/moment_vectors.hex";

    -- DUT signals
    signal clk     : std_logic := '0';
    signal rst     : std_logic := '1';
    signal i_start : std_logic := '0';
    signal i_patch : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    signal o_done  : std_logic;
    signal o_m_10  : signed(23 downto 0);
    signal o_m_01  : signed(23 downto 0);
    signal o_ready : std_logic;

    -- Tracking
    signal errors  : natural := 0;
    signal checks  : natural := 0;
    signal done    : boolean := false;
begin

    dut : entity work.moment_processor
        port map (
            clk     => clk,
            rst     => rst,
            i_start => i_start,
            i_patch => i_patch,
            o_done  => o_done,
            o_m_10  => o_m_10,
            o_m_01  => o_m_01,
            o_ready => o_ready
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Single process drives both stimulus and checking, since the
    -- one-corner-at-a-time pattern is sequential: drive patch, wait for
    -- done, compare. No need for parallel checker process.
    test : process
        file     vec_file  : text open read_mode is VECTOR_FILE;
        variable line_in   : line;
        variable pix_v     : std_logic_vector(7 downto 0);
        variable m10_v     : std_logic_vector(23 downto 0);
        variable m01_v     : std_logic_vector(23 downto 0);
        variable exp_m10   : signed(23 downto 0);
        variable exp_m01   : signed(23 downto 0);
        variable patch_v   : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
        variable mismatch  : boolean;
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait until rising_edge(clk);

        for vec_idx in 0 to TEST_N_VECTORS-1 loop
            readline(vec_file, line_in);

            -- Parse 961 patch pixels
            for row in 0 to PATCH_SIZE-1 loop
                for col in 0 to PATCH_SIZE-1 loop
                    hread(line_in, pix_v);
                    patch_v(row, col) := unsigned(pix_v);
                end loop;
            end loop;

            -- Parse expected moments (24-bit two's complement)
            hread(line_in, m10_v);
            hread(line_in, m01_v);
            exp_m10 := signed(m10_v);
            exp_m01 := signed(m01_v);

            -- Wait for DUT ready, then drive start pulse
            wait until o_ready = '1' and rising_edge(clk);
            i_patch <= patch_v;
            i_start <= '1';
            wait until rising_edge(clk);
            i_start <= '0';

            -- Wait for done
            wait until o_done = '1';

            -- Compare (sampling values on the o_done cycle)
            mismatch := false;
            if o_m_10 /= exp_m10 then
                report "M_10 MISMATCH at check " & to_string(checks) &
                       ": got " & to_string(to_integer(o_m_10)) &
                       ", expected " & to_string(to_integer(exp_m10))
                    severity error;
                mismatch := true;
            end if;
            if o_m_01 /= exp_m01 then
                report "M_01 MISMATCH at check " & to_string(checks) &
                       ": got " & to_string(to_integer(o_m_01)) &
                       ", expected " & to_string(to_integer(exp_m01))
                    severity error;
                mismatch := true;
            end if;

            if mismatch then
                errors <= errors + 1;
            end if;
            checks <= checks + 1;

            wait until rising_edge(clk);
        end loop;

        -- Drain
        wait for DRAIN_CYCLES * CLK_PERIOD;

        report "Checked " & to_string(checks) &
               " of " & to_string(TEST_N_VECTORS) &
               " vectors, " & to_string(errors) & " errors.";
        assert errors = 0 and checks = TEST_N_VECTORS
            report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;

end architecture;