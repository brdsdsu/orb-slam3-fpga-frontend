library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;
use work.umax_pkg.all;
use work.harris_test_params_pkg.all;

-- Testbench for harris_response (integer Harris corner response).
--
-- Vectors: testbenches/harris_vectors.hex (TEST_N_VECTORS lines from
-- harris_response_generator.py, reference model = the integer transform of
-- OpenCV's HarrisResponses, bit-exact spec shared with the RTL).
-- Per line: 961 patch bytes (31x31, row-major) then the expected response as
-- 32-bit two's-complement hex.
--
-- One sequential process (the DUT handles one corner at a time): wait o_ready,
-- pulse i_start, wait o_done, compare the response exactly. Fails on any
-- mismatch or if fewer than TEST_N_VECTORS vectors were checked.
entity harris_response_tb is
end entity;

architecture sim of harris_response_tb is
    constant CLK_PERIOD   : time    := 4 ns;
    constant DRAIN_CYCLES : natural := 32;
    constant VECTOR_FILE  : string  := "testbenches/harris_vectors.hex";

    -- DUT signals
    signal clk        : std_logic := '0';
    signal rst        : std_logic := '1';
    signal i_start    : std_logic := '0';
    signal i_patch    : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    signal o_done     : std_logic;
    signal o_response : signed(31 downto 0);
    signal o_ready    : std_logic;

    -- Tracking
    signal errors : natural := 0;
    signal checks : natural := 0;
    signal done   : boolean := false;
begin

    dut : entity work.harris_response
        port map (
            clk        => clk,
            rst        => rst,
            i_start    => i_start,
            i_patch    => i_patch,
            o_done     => o_done,
            o_response => o_response,
            o_ready    => o_ready
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Single process drives both stimulus and checking (one corner at a time,
    -- same pattern as moment_processor_tb).
    test : process
        file     vec_file : text open read_mode is VECTOR_FILE;
        variable line_in  : line;
        variable pix_v    : std_logic_vector(7 downto 0);
        variable resp_v   : std_logic_vector(31 downto 0);
        variable exp_resp : signed(31 downto 0);
        variable patch_v  : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
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

            -- Parse expected response (32-bit two's complement)
            hread(line_in, resp_v);
            exp_resp := signed(resp_v);

            -- Wait for DUT ready, then drive start pulse
            wait until o_ready = '1' and rising_edge(clk);
            i_patch <= patch_v;
            i_start <= '1';
            wait until rising_edge(clk);
            i_start <= '0';

            -- Wait for done
            wait until o_done = '1';

            -- Compare (sampling the value on the o_done cycle)
            if o_response /= exp_resp then
                report "RESPONSE MISMATCH at check " & to_string(checks) &
                       ": got " & to_string(to_integer(o_response)) &
                       ", expected " & to_string(to_integer(exp_resp))
                    severity error;
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
