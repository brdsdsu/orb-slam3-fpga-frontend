library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.fast_pkg.all;

-- FAST-9 corner detector (decision only, no score).
--
-- Reports whether the patch centre is a FAST-9 corner at i_threshold: a corner
-- exists when some contiguous arc of ARC_LENGTH (9) pixels on the 16-pixel
-- Bresenham circle is either ALL brighter than centre+threshold or ALL darker
-- than centre-threshold. o_is_brighter distinguishes the two polarities. The
-- corner STRENGTH is produced separately by fast_response / fast_nms.
--
-- 3-stage pipeline (latency 3, one result per clock):
--   S1  16 parallel threshold compares -> brighter[k], darker[k]
--   S2  16 parallel 9-input arc ANDs    -> "arc starting at k is all-bright/dark"
--   S3  OR-reduce the 16 arcs per polarity -> is_corner, is_brighter
--
-- The S1 compares use 9-/10-bit intermediates so centre+threshold (>255) and
-- centre-threshold (<0) cannot wrap. i_circle index order follows OpenCV's
-- offsets16[] (see extract_fast_circle in fast_pkg).
entity fast9_core is
	port (
		clk         : in  std_logic;
		rst         : in  std_logic;

		i_valid     : in  std_logic;
		i_center    : in  pixel_t;
		i_circle    : in  circle_t;
		i_threshold : in  threshold_t;

		o_valid     : out std_logic;
		o_is_corner : out std_logic;
		o_is_brighter : out std_logic
	);
end entity;

architecture rtl of fast9_core is
	-- Stage 1: per-pixel threshold compares
	--   brighter[k] = '1' iff circle[k] > center + threshold
	--   darker[k]   = '1' iff circle[k] < center - threshold
	signal brighter_s1 : bool16_t := (others => '0');
	signal darker_s1   : bool16_t := (others => '0');
	signal v_s1        : std_logic := '0';

	-- Stage 2: per-arc 9-input AND for each polarity
	signal br_arc_s2   : bool16_t := (others => '0');
	signal dk_arc_s2   : bool16_t := (others => '0');
	signal v_s2        : std_logic := '0';

	-- Stage 3: 16-input OR per polarity, combine
	signal is_brighter_s3 : std_logic := '0';
	signal is_darker_s3   : std_logic := '0';
	signal v_s3           : std_logic := '0';
begin

	-- S1: 16 parallel threshold comparisons
	--   Use 9-bit unsigned math to avoid wraparound on (center + threshold) > 255
	--   and (center - threshold) < 0. Saturation-on-overflow gives the correct
	--   boolean result naturally because circle[k] is in [0, 255].
	-- Datapath free-runs (no reset); only the valid chain carries reset, same
	-- policy as fast_response. Stale compare results are never consumed because
	-- v_s1..v_s3 gate the output.
	p_s1 : process(clk)
		variable c_ext : unsigned(PIXEL_WIDTH downto 0);   -- 9-bit
		variable t_ext : unsigned(PIXEL_WIDTH downto 0);
		variable upper : unsigned(PIXEL_WIDTH downto 0);   -- center + thr
		variable lower : signed(PIXEL_WIDTH+1 downto 0);   -- center - thr, may be negative
		variable px_ext : unsigned(PIXEL_WIDTH downto 0);
	begin
		if rising_edge(clk) then
			c_ext := '0' & i_center;
			t_ext := '0' & i_threshold;
			upper := c_ext + t_ext;                         -- 9-bit, range [0..510]
			lower := signed('0' & c_ext) - signed('0' & t_ext);  -- 10-bit signed

			for k in 0 to CIRCLE_SIZE-1 loop
				px_ext := '0' & i_circle(k);

				-- brighter: circle[k] > center + threshold
				if px_ext > upper then
					brighter_s1(k) <= '1';
				else
					brighter_s1(k) <= '0';
				end if;

				-- darker: circle[k] < center - threshold
				if signed('0' & px_ext) < lower then
					darker_s1(k) <= '1';
				else
					darker_s1(k) <= '0';
				end if;
			end loop;

			if rst = '1' then
				v_s1 <= '0';
			else
				v_s1 <= i_valid;
			end if;
		end if;
	end process;

	-- S2: 16 parallel 9-input AND trees, for each polarity
	--   br_arc_s2(s) = '1' iff all 9 pixels in the arc starting at s are brighter
	p_s2 : process(clk)
	begin
		if rising_edge(clk) then
			for s in 0 to CIRCLE_SIZE-1 loop
				br_arc_s2(s) <= arc_and_9(brighter_s1, s);
				dk_arc_s2(s) <= arc_and_9(darker_s1, s);
			end loop;

			if rst = '1' then
				v_s2 <= '0';
			else
				v_s2 <= v_s1;
			end if;
		end if;
	end process;

	-- S3: any arc passes the test → corner detected
	p_s3 : process(clk)
	begin
		if rising_edge(clk) then
			is_brighter_s3 <= reduce_or(br_arc_s2);
			is_darker_s3   <= reduce_or(dk_arc_s2);

			if rst = '1' then
				v_s3 <= '0';
			else
				v_s3 <= v_s2;
			end if;
		end if;
	end process;

	o_valid       <= v_s3;
	o_is_corner   <= is_brighter_s3 or is_darker_s3;
	o_is_brighter <= is_brighter_s3;

end architecture;