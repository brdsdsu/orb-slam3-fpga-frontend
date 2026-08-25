library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.umax_pkg.all;

-- Pipelined moment computation for ORB-SLAM3's intensity centroid orientation.
--
-- Bit-exact m_10/m_01 vs the sequential version (same products, same resize
-- points, ACC_WIDTH=24; summation reordered into balanced adder trees, which is
-- exact modulo 2**ACC_WIDTH in two's complement). The i_start/o_done/o_ready
-- handshake is preserved, so orientation_top and the module TB are unaffected.
--
-- Algorithm (matches ORB-SLAM3's IC_Angle()):
--   m_10 = sum over center row of u*pixel[0,u]
--        + sum_{v=1..15} sum_{|u|<=u_max[v]} u*(pixel[+v,u]+pixel[-v,u])
--   m_01 = sum_{v=1..15} v * sum_{|u|<=u_max[v]} (pixel[+v,u]-pixel[-v,u])
--
-- Microarchitecture: one row (pair) fed per cycle (v = 0..15); the per-row work
-- is a 6-stage pipeline so no single cycle holds the row-mux -> reduce ->
-- multiply -> accumulate chain that limited fmax:
--   S_SETUP    latch i_patch, clear accumulators (pipeline already drained)
--   stage 1    form: select row pair by v, mask by u_max[v], build per-lane m_10
--              product u*(pix_plus[+pix_minus]) and per-lane difference
--   stage 2a   reduce part 1: eight balanced 4-input partial sums
--   stage 2b   reduce part 2: combine partials -> row m_10 contribution, v_sum
--   stage 3a   multiply: full-width product v*v_sum, NOT truncated
--   stage 3b   register the product (pure copy)
--   stage 4    accumulate: m_10_acc += row_m10 ; m_01_acc += trunc(product)
-- Registering the native product across stages 3a and 3b (operands already come
-- from stage-2 registers) lets the synthesiser pack the DSP48E2 input, MREG and
-- PREG registers, so the multiply is fully pipelined inside the DSP. The
-- truncation to ACC_WIDTH is deferred to stage 4 and is loss-free (|v*v_sum| <
-- 2**23), so it remains bit-identical to the reference. o_done pulses for one
-- cycle when the v=15 contribution is accumulated, with final m_10/m_01 valid on
-- that cycle, and o_ready re-asserts the same cycle. Latency i_start -> o_done is
-- 23 cycles.

entity moment_processor is
	port (
		clk     : in  std_logic;
		rst     : in  std_logic;

		i_start : in  std_logic;
		i_patch : in  patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

		o_done  : out std_logic;
		o_m_10  : out signed(MOMENT_WIDTH-1 downto 0);
		o_m_01  : out signed(MOMENT_WIDTH-1 downto 0);

		o_ready : out std_logic
	);
end entity;

architecture rtl of moment_processor is

	constant CR         : natural := HALF_PATCH_SIZE;   -- center row/col index
	constant ACC_WIDTH  : natural := MOMENT_WIDTH;   -- 24, shared via umax_pkg
	constant V_WIDTH    : natural := 5;                  -- v_count as signed
	constant PROD_WIDTH : natural := V_WIDTH + ACC_WIDTH; -- v * v_sum, 29 bits

	-- One lane per column offset u; lane index l maps to u = l - CR (-15..+15).
	type prod_arr_t is array (0 to PATCH_SIZE-1) of signed(ACC_WIDTH-1 downto 0);
	-- Named row_diff_arr_t (not diff_arr_t) to avoid colliding with
	-- fast_pkg.diff_arr_t should fast_pkg ever be used here.
	type row_diff_arr_t is array (0 to PATCH_SIZE-1) of signed(9 downto 0);
	type part_arr_t is array (0 to 7) of signed(ACC_WIDTH-1 downto 0);

	-- Control FSM
	type state_t is (S_IDLE, S_SETUP, S_COMPUTE, S_DRAIN);
	signal state   : state_t := S_IDLE;
	signal v_count : natural range 0 to HALF_PATCH_SIZE := 0;

	signal patch_r : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

	-- Stage 1: per-lane m_10 product and difference (masked by u_max[v])
	signal s1_valid : std_logic := '0';
	signal s1_last  : std_logic := '0';
	signal s1_v     : natural range 0 to HALF_PATCH_SIZE := 0;
	signal s1_prod  : prod_arr_t;
	signal s1_diff  : row_diff_arr_t;

	-- Stage 2a: eight balanced 4-input partial sums
	signal s2a_valid : std_logic := '0';
	signal s2a_last  : std_logic := '0';
	signal s2a_v     : natural range 0 to HALF_PATCH_SIZE := 0;
	signal s2a_m10   : part_arr_t;
	signal s2a_vs    : part_arr_t;

	-- Stage 2b: combined row contribution and v_sum
	signal s2_valid   : std_logic := '0';
	signal s2_last    : std_logic := '0';
	signal s2_v       : natural range 0 to HALF_PATCH_SIZE := 0;
	signal s2_row_m10 : signed(ACC_WIDTH-1 downto 0) := (others => '0');
	signal s2_v_sum   : signed(ACC_WIDTH-1 downto 0) := (others => '0');

	-- Stage 3a: full-width product (DSP MREG), m_10 contribution passed through
	signal s3a_valid   : std_logic := '0';
	signal s3a_last    : std_logic := '0';
	signal s3a_row_m10 : signed(ACC_WIDTH-1 downto 0) := (others => '0');
	signal s3a_prod    : signed(PROD_WIDTH-1 downto 0) := (others => '0');

	-- Stage 3b: registered product (DSP PREG), m_10 contribution passed through
	signal s3b_valid   : std_logic := '0';
	signal s3b_last    : std_logic := '0';
	signal s3b_row_m10 : signed(ACC_WIDTH-1 downto 0) := (others => '0');
	signal s3b_prod    : signed(PROD_WIDTH-1 downto 0) := (others => '0');

	-- Stage 4: accumulators + done
	signal m_10_acc : signed(ACC_WIDTH-1 downto 0) := (others => '0');
	signal m_01_acc : signed(ACC_WIDTH-1 downto 0) := (others => '0');
	signal done_r   : std_logic := '0';

	-- Helper: unsigned pixel to signed (sign-extended), 9-bit nonnegative
	function pix_s(p : pixel_t) return signed is
	begin
		return signed('0' & p);
	end function;

begin

	o_done  <= done_r;
	o_m_10  <= m_10_acc;
	o_m_01  <= m_01_acc;
	o_ready <= '1' when state = S_IDLE else '0';

	-- ================================================================
	-- Control FSM: feed v = 0..15 into the pipeline, one per cycle, then
	-- drain. S_DRAIN ends when the last row reaches the accumulators.
	-- ================================================================
	p_fsm : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				state   <= S_IDLE;
				v_count <= 0;
			else
				case state is
					when S_IDLE =>
						v_count <= 0;
						if i_start = '1' then
							state <= S_SETUP;
						end if;

					when S_SETUP =>
						v_count <= 0;
						state   <= S_COMPUTE;

					when S_COMPUTE =>
						if v_count = HALF_PATCH_SIZE then
							state <= S_DRAIN;
						else
							v_count <= v_count + 1;
						end if;

					when S_DRAIN =>
						if s3b_valid = '1' and s3b_last = '1' then
							state <= S_IDLE;
						end if;
				end case;
			end if;
		end if;
	end process;

	-- Latch the patch during S_SETUP; stable for the whole corner (the pipeline
	-- drains before the next S_SETUP, so no read is mid-flight).
	p_patch : process(clk)
	begin
		if rising_edge(clk) then
			if state = S_SETUP then
				patch_r <= i_patch;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 1: select the row pair for v_count, mask columns by u_max[v],
	-- and build the per-lane m_10 product (u*sum) and difference.
	-- Center row (v=0): sum = pix_plus only, diff = 0 (m_01 unaffected).
	-- ================================================================
	p_stage1 : process(clk)
		variable u_max_v : natural range 0 to HALF_PATCH_SIZE;
		variable pp, pm  : signed(8 downto 0);
		variable sum10   : signed(9 downto 0);
		variable dif10   : signed(9 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s1_valid <= '0';
				s1_last  <= '0';
			else
				s1_valid <= '0';
				s1_last  <= '0';

				if state = S_COMPUTE then
					s1_valid <= '1';
					s1_v     <= v_count;
					if v_count = HALF_PATCH_SIZE then
						s1_last <= '1';
					end if;

					u_max_v := U_MAX(v_count);
					for l in 0 to PATCH_SIZE-1 loop
						if abs(l - CR) <= u_max_v then
							pp := pix_s(patch_r(CR + v_count, l));
							if v_count = 0 then
								sum10 := resize(pp, 10);
								dif10 := (others => '0');
							else
								pm    := pix_s(patch_r(CR - v_count, l));
								sum10 := resize(pp, 10) + resize(pm, 10);
								dif10 := resize(pp, 10) - resize(pm, 10);
							end if;
						else
							sum10 := (others => '0');
							dif10 := (others => '0');
						end if;

						-- u = l - CR is a per-lane compile-time constant, so this
						-- is a constant-coefficient multiply (LUT shift-add).
						s1_prod(l) <= resize(to_signed(l - CR, 6) * sum10, ACC_WIDTH);
						s1_diff(l) <= dif10;
					end loop;
				end if;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 2a: eight balanced 4-input partial sums (last group has 3).
	-- Explicit parentheses force a balanced tree instead of a ripple.
	-- ================================================================
	p_stage2a : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s2a_valid <= '0';
				s2a_last  <= '0';
			else
				s2a_valid <= s1_valid;
				s2a_last  <= s1_last;
				s2a_v     <= s1_v;

				s2a_m10(0) <= (s1_prod(0)  + s1_prod(1))  + (s1_prod(2)  + s1_prod(3));
				s2a_m10(1) <= (s1_prod(4)  + s1_prod(5))  + (s1_prod(6)  + s1_prod(7));
				s2a_m10(2) <= (s1_prod(8)  + s1_prod(9))  + (s1_prod(10) + s1_prod(11));
				s2a_m10(3) <= (s1_prod(12) + s1_prod(13)) + (s1_prod(14) + s1_prod(15));
				s2a_m10(4) <= (s1_prod(16) + s1_prod(17)) + (s1_prod(18) + s1_prod(19));
				s2a_m10(5) <= (s1_prod(20) + s1_prod(21)) + (s1_prod(22) + s1_prod(23));
				s2a_m10(6) <= (s1_prod(24) + s1_prod(25)) + (s1_prod(26) + s1_prod(27));
				s2a_m10(7) <= (s1_prod(28) + s1_prod(29)) +  s1_prod(30);

				s2a_vs(0) <= (resize(s1_diff(0),  ACC_WIDTH) + resize(s1_diff(1),  ACC_WIDTH))
						   + (resize(s1_diff(2),  ACC_WIDTH) + resize(s1_diff(3),  ACC_WIDTH));
				s2a_vs(1) <= (resize(s1_diff(4),  ACC_WIDTH) + resize(s1_diff(5),  ACC_WIDTH))
						   + (resize(s1_diff(6),  ACC_WIDTH) + resize(s1_diff(7),  ACC_WIDTH));
				s2a_vs(2) <= (resize(s1_diff(8),  ACC_WIDTH) + resize(s1_diff(9),  ACC_WIDTH))
						   + (resize(s1_diff(10), ACC_WIDTH) + resize(s1_diff(11), ACC_WIDTH));
				s2a_vs(3) <= (resize(s1_diff(12), ACC_WIDTH) + resize(s1_diff(13), ACC_WIDTH))
						   + (resize(s1_diff(14), ACC_WIDTH) + resize(s1_diff(15), ACC_WIDTH));
				s2a_vs(4) <= (resize(s1_diff(16), ACC_WIDTH) + resize(s1_diff(17), ACC_WIDTH))
						   + (resize(s1_diff(18), ACC_WIDTH) + resize(s1_diff(19), ACC_WIDTH));
				s2a_vs(5) <= (resize(s1_diff(20), ACC_WIDTH) + resize(s1_diff(21), ACC_WIDTH))
						   + (resize(s1_diff(22), ACC_WIDTH) + resize(s1_diff(23), ACC_WIDTH));
				s2a_vs(6) <= (resize(s1_diff(24), ACC_WIDTH) + resize(s1_diff(25), ACC_WIDTH))
						   + (resize(s1_diff(26), ACC_WIDTH) + resize(s1_diff(27), ACC_WIDTH));
				s2a_vs(7) <= (resize(s1_diff(28), ACC_WIDTH) + resize(s1_diff(29), ACC_WIDTH))
						   +  resize(s1_diff(30), ACC_WIDTH);
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 2b: balanced combine of the eight partials -> row m_10, v_sum.
	-- ================================================================
	p_stage2b : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s2_valid <= '0';
				s2_last  <= '0';
			else
				s2_valid <= s2a_valid;
				s2_last  <= s2a_last;
				s2_v     <= s2a_v;

				s2_row_m10 <= ((s2a_m10(0) + s2a_m10(1)) + (s2a_m10(2) + s2a_m10(3)))
							+ ((s2a_m10(4) + s2a_m10(5)) + (s2a_m10(6) + s2a_m10(7)));
				s2_v_sum   <= ((s2a_vs(0)  + s2a_vs(1))  + (s2a_vs(2)  + s2a_vs(3)))
							+ ((s2a_vs(4)  + s2a_vs(5))  + (s2a_vs(6)  + s2a_vs(7)));
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 3a: full-width product v * v_sum, registered. Operands come from
	-- stage-2 registers and the product is NOT truncated here, so the DSP48E2
	-- input register and MREG can be used. m_10 contribution passes through.
	-- ================================================================
	p_stage3a : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s3a_valid <= '0';
				s3a_last  <= '0';
			else
				s3a_valid   <= s2_valid;
				s3a_last    <= s2_last;
				s3a_row_m10 <= s2_row_m10;
				s3a_prod    <= to_signed(s2_v, V_WIDTH) * s2_v_sum;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 3b: pure register of the product -> DSP48E2 PREG. Together with
	-- stage 3a this fully pipelines the multiply inside the DSP.
	-- ================================================================
	p_stage3b : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s3b_valid <= '0';
				s3b_last  <= '0';
			else
				s3b_valid   <= s3a_valid;
				s3b_last    <= s3a_last;
				s3b_row_m10 <= s3a_row_m10;
				s3b_prod    <= s3a_prod;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 4: accumulate. One row's contribution lands per cycle. The
	-- accumulators clear in S_SETUP (pipeline empty), and done_r pulses the
	-- cycle after the v=15 contribution is accumulated. The resize of the
	-- 29-bit product to 24 bits is loss-free (|v*v_sum| < 2**23).
	-- ================================================================
	p_accumulate : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				m_10_acc <= (others => '0');
				m_01_acc <= (others => '0');
				done_r   <= '0';
			else
				done_r <= '0';

				if state = S_SETUP then
					m_10_acc <= (others => '0');
					m_01_acc <= (others => '0');
				elsif s3b_valid = '1' then
					m_10_acc <= m_10_acc + s3b_row_m10;
					m_01_acc <= m_01_acc + resize(s3b_prod, ACC_WIDTH);
					if s3b_last = '1' then
						done_r <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;

end architecture;