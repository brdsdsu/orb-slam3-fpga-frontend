library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

use work.feature_pkg.all;   -- pixel_t, patch_arr_t
use work.fast_pkg.all;
use work.umax_pkg.all;      -- PATCH_SIZE

-- Testbench for fast_nms (FAST score + 3x3 non-maximum suppression).
--
-- Vectors: testbenches/fast_nms_vectors.hex, from fast_nms_generator.py. Per
-- line: 961 patch bytes (31x31, row-major) then expected centre score and
-- local-max flag.
--
-- SCOPE OF THIS TESTBENCH (2026-08-11). The generator is an INDEPENDENT
-- reimplementation of OpenCV's cornerScore<16> and 3x3 suppression, not a call
-- into cv2. The score half is now confirmed bit-exact against cv2 (9857/9857 on
-- a real MH01 frame). The TIE RULE half is NOT: generator and RTL both use '>='
-- (keep plateaus) while OpenCV uses '>' (drop ties), so they agree with each
-- other and this testbench passes on a rule that differs from OpenCV. A shared
-- oracle cannot detect an error it shares. See fast_nms.vhd and progress.md
-- [F.24]. The deviation is documented, not fixed.
--
-- Self-checking with expected-value queues (DUT latency 5). Compares both the
-- centre score and the local-max verdict on every o_valid. Fails on any
-- mismatch.
entity fast_nms_tb is
end entity;

architecture sim of fast_nms_tb is
    -----------------------------------------------------------------
    -- Test configuration
    -----------------------------------------------------------------
    constant CLK_PERIOD     : time     := 4 ns;   -- 250 MHz
    constant N              : positive := PATCH_SIZE;        -- 31
    constant PIPELINE_DEPTH : natural  := 5;                 -- fast_nms latency
    constant QUEUE_DEPTH    : natural  := PIPELINE_DEPTH * 4;
    constant DRAIN_CYCLES   : natural  := PIPELINE_DEPTH + 4;
    constant VECTOR_FILE    : string   := "testbenches/fast_nms_vectors.hex";

    -----------------------------------------------------------------
    -- DUT signals
    -----------------------------------------------------------------
    signal clk            : std_logic := '0';
    signal rst            : std_logic := '1';
    signal i_valid        : std_logic := '0';
    signal i_patch        : patch_arr_t(0 to N-1, 0 to N-1) :=
                                (others => (others => (others => '0')));
    signal o_valid        : std_logic;
    signal o_center_score : pixel_t;
    signal o_is_local_max : std_logic;

    -----------------------------------------------------------------
    -- Expected-value circular buffers
    -----------------------------------------------------------------
    type score_q_t is array (natural range <>) of pixel_t;
    type bit_q_t   is array (natural range <>) of std_logic;
    signal exp_score : score_q_t(0 to QUEUE_DEPTH-1) := (others => (others => '0'));
    signal exp_lmax  : bit_q_t  (0 to QUEUE_DEPTH-1) := (others => '0');
    signal exp_wr    : natural := 0;
    signal exp_rd    : natural := 0;

    signal errors    : natural := 0;
    signal checks    : natural := 0;
    signal done      : boolean := false;
begin

    dut : entity work.fast_nms
        generic map ( N => N )
        port map (
            clk => clk, rst => rst,
            i_valid => i_valid, i_patch => i_patch,
            o_valid => o_valid,
            o_center_score => o_center_score,
            o_is_local_max => o_is_local_max
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    -- Stimulus + expected-value tracking
    stim : process
        file     vec_file : text open read_mode is VECTOR_FILE;
        variable line_in  : line;
        variable hex_v    : std_logic_vector(7 downto 0);
        variable patch_v  : patch_arr_t(0 to N-1, 0 to N-1);
        variable score_v  : pixel_t;
        variable lmax_v   : std_logic;
    begin
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        wait until rising_edge(clk);

        while not endfile(vec_file) loop
            readline(vec_file, line_in);

            -- 961 patch bytes, row-major (r outer, c inner)
            for r in 0 to N-1 loop
                for c in 0 to N-1 loop
                    hread(line_in, hex_v);
                    patch_v(r, c) := unsigned(hex_v);
                end loop;
            end loop;
            -- expected centre score, then local-max flag (00 / 01)
            hread(line_in, hex_v); score_v := unsigned(hex_v);
            hread(line_in, hex_v); lmax_v  := hex_v(0);

            -- Drive one cycle
            i_patch <= patch_v;
            i_valid <= '1';

            exp_score(exp_wr mod QUEUE_DEPTH) <= score_v;
            exp_lmax (exp_wr mod QUEUE_DEPTH) <= lmax_v;
            exp_wr <= exp_wr + 1;

            wait until rising_edge(clk);
        end loop;

        i_valid <= '0';
        wait for DRAIN_CYCLES * CLK_PERIOD;

        report "Checked " & to_string(checks) & " vectors, " &
               to_string(errors) & " errors.";
        assert errors = 0 report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;

    -- Output checker
    check : process(clk)
        variable e_score : pixel_t;
        variable e_lmax  : std_logic;
    begin
        if rising_edge(clk) then
            if o_valid = '1' then
                e_score := exp_score(exp_rd mod QUEUE_DEPTH);
                e_lmax  := exp_lmax (exp_rd mod QUEUE_DEPTH);
                if o_center_score /= e_score then
                    report "SCORE MISMATCH at check " & to_string(checks) &
                           ": got " & to_string(o_center_score) &
                           ", expected " & to_string(e_score) severity error;
                    errors <= errors + 1;
                elsif o_is_local_max /= e_lmax then
                    report "LMAX MISMATCH at check " & to_string(checks) &
                           ": got " & to_string(o_is_local_max) &
                           ", expected " & to_string(e_lmax) severity error;
                    errors <= errors + 1;
                end if;
                checks <= checks + 1;
                exp_rd <= exp_rd + 1;
            end if;
        end if;
    end process;

end architecture;