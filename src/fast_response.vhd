----------------------------------------------------------------------------------
-- FAST corner SCORE (equivalent to OpenCV's cornerScore).
--
-- Produces the FAST response at the patch centre: a strength value used by the
-- non-maximum suppression in fast_nms. fast9_core decides IF the centre is a
-- corner at a fixed threshold; this module measures HOW strong that corner is.
--
-- Datapath: d[k] = centre - circle[k] over the 16 circle pixels; reduce over
-- every contiguous 9-pixel arc to the best brighter-arc and darker-arc margins,
-- take the larger of the two, subtract 1 and saturate to [0,255]. 4-stage
-- pipeline (latency 4, one result per clock); per-stage logic/bit budget below.
--
-- Stage | Operation                   | Logic levels             | Bits
-- S1    | 16x signed subtract         | 1 (carry-prop on 9 bits) | 9
-- S2    | 16x 9-input min/max trees   | 4 each                   | 9
-- S3    | 16-input max + 16-input min | 4 each                   | 9
-- S4    | Compare, subtract, saturate | ~3                       | 10
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.fast_pkg.all;

entity fast_response is
	port (
		clk	  : in  std_logic;
		rst	  : in  std_logic;

		i_valid  : in  std_logic;
		i_center : in  pixel_t;
		i_circle : in  circle_t;

		o_valid  : out std_logic;
		o_score  : out pixel_t
	);
end entity;

architecture rtl of fast_response is
	-- Pipeline registers
	signal d_s1			: diff_arr_t := (others => (others => '0'));
	signal v_s1			: std_logic := '0';

	signal arc_min_s2	: diff_arr_t := (others => (others => '0'));
	signal arc_max_s2	: diff_arr_t := (others => (others => '0'));
	signal v_s2			: std_logic := '0';

	signal brighter_s3	: diff_t := (others => '0');
	signal darker_s3	: diff_t := (others => '0');
	signal v_s3			: std_logic := '0';

	signal score_s4		: pixel_t := (others => '0');
	signal v_s4			: std_logic := '0';
begin

	-- S1: 16 parallel signed subtractions, d[k] = center - circle[k]
	-- d_s1 datapath free-runs (no reset); only v_s1 carries reset. Stale d_s1 is
	-- never consumed because v_s1 (and the rest of the valid chain) gates output.
	p_s1 : process(clk)
	begin
		if rising_edge(clk) then
			for k in 0 to CIRCLE_SIZE-1 loop
				d_s1(k) <= signed(resize(i_center, DIFF_WIDTH))
						 - signed(resize(i_circle(k), DIFF_WIDTH));
			end loop;

			if rst = '1' then
				v_s1 <= '0';
			else
				v_s1 <= i_valid;
			end if;
		end if;
	end process;

	-- S2: 16 parallel 9-input min/max trees (4 logic levels each, all combinational)
	-- arc_min_s2/arc_max_s2 free-run (no reset); only v_s2 carries reset.
	p_s2 : process(clk)
	begin
		if rising_edge(clk) then
			for s in 0 to CIRCLE_SIZE-1 loop
				arc_min_s2(s) <= arc_min_9(d_s1, s);
				arc_max_s2(s) <= arc_max_9(d_s1, s);
			end loop;

			if rst = '1' then
				v_s2 <= '0';
			else
				v_s2 <= v_s1;
			end if;
		end if;
	end process;

	-- S3: 16-input reductions (4 logic levels)
	--   brighter = max over s of arc_min[s]   -- strongest "brighter" arc
	--   darker   = min over s of arc_max[s]   -- strongest "darker"  arc (negative)
	p_s3 : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				v_s3 <= '0';
				brighter_s3 <= (others => '0');
				darker_s3   <= (others => '0');
			else
				v_s3		<= v_s2;
				brighter_s3 <= reduce_max(arc_min_s2);
				darker_s3   <= reduce_min(arc_max_s2);
			end if;
		end if;
	end process;

	-- S4: final score = max(brighter, -darker) - 1, saturated to [0, 255]
	p_s4 : process(clk)
		variable neg_darker : signed(DIFF_WIDTH downto 0);   -- one extra bit
		variable best	   : signed(DIFF_WIDTH downto 0);
		variable scored	 : signed(DIFF_WIDTH downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				v_s4	 <= '0';
				score_s4 <= (others => '0');
			else
				v_s4 <= v_s3;

				neg_darker := -resize(darker_s3, DIFF_WIDTH+1);
				if resize(brighter_s3, DIFF_WIDTH+1) >= neg_darker then
					best := resize(brighter_s3, DIFF_WIDTH+1);
				else
					best := neg_darker;
				end if;

				scored := best - 1;

				-- Saturate to unsigned 8-bit
				if scored < 0 then
					score_s4 <= (others => '0');
				elsif scored > 255 then
					score_s4 <= (others => '1');
				else
					score_s4 <= unsigned(scored(PIXEL_WIDTH-1 downto 0));
				end if;
			end if;
		end if;
	end process;

	o_valid <= v_s4;
	o_score <= score_s4;

end architecture;