library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;   -- pixel_t, patch_arr_t
use work.fast_pkg.all;      -- fast_circle_t, extract_fast_circle_at
use work.umax_pkg.all;      -- PATCH_SIZE

-- 3x3 non-maximum suppression on the FAST score field.
--
-- Stock ORB-SLAM3 detects with OpenCV FAST(..., nonmaxSuppression=true): it keeps
-- only corners that are a local maximum of the FAST score in their 3x3 neighbourhood.
-- Without this suppression the PL emits every permissive corner, flooding the corner
-- FIFO (measured DROPCNT ~87% before the zero-drop backpressure existed; today the
-- flood would cost STALLCNT stalls instead) and diverging from the SW corner set.
--
-- The 31x31 orientation patch already holds every pixel needed to score the 8
-- neighbours (FAST needs only a radius-3 circle; centre +/-1 reaches patch index
-- [11..19] inside [0..30]). So all 9 scores are computed from the SAME patch, in
-- parallel -- no line buffers, no row latency. A neighbour's score here is bit-
-- identical to the score it gets as its own window centre (same radius-3 circle of
-- physical pixels), so this is exactly the 3x3 score-field local max.
--
-- Latency: 4 (fast_response pipeline) + 1 (registered compare) = 5; extractor_top
-- tracks this as NMS_LAT. o_center_score and o_is_local_max emerge together so the
-- consumer aligns to one number.
entity fast_nms is
	generic (
		N : positive := PATCH_SIZE   -- patch size (31)
	);
	port (
		clk : in  std_logic;
		rst : in  std_logic;

		i_valid : in  std_logic;
		i_patch : in  patch_arr_t(0 to N-1, 0 to N-1);

		o_valid        : out std_logic;
		o_center_score : out pixel_t := (others => '0');   -- FAST score at the patch centre (== fast_response).
		                                                   -- Default init: kills the t=0 iteration-0 metavalue
		                                                   -- warning on extractor_top's is_strict '>=' compare.
		o_is_local_max : out std_logic  -- centre is a 3x3 local maximum of the score field
	);
end entity;

architecture rtl of fast_nms is

	constant NUM_POS : natural := 9;   -- centre + 8 neighbours

	type offset_t     is record dr : integer; dc : integer; end record;
	type offset_arr_t is array (0 to NUM_POS-1) of offset_t;
	-- index 0 = centre; 1..8 = the 8-neighbourhood
	constant OFFS : offset_arr_t := (
		0 => (dr =>  0, dc =>  0),
		1 => (dr => -1, dc => -1), 2 => (dr => -1, dc => 0), 3 => (dr => -1, dc => 1),
		4 => (dr =>  0, dc => -1),                            5 => (dr =>  0, dc => 1),
		6 => (dr =>  1, dc => -1), 7 => (dr =>  1, dc => 0), 8 => (dr =>  1, dc => 1)
	);

	type score_arr_t is array (0 to NUM_POS-1) of pixel_t;
	signal scores  : score_arr_t;   -- the 9 FAST scores @ T+RESP_LAT
	signal v_score : std_logic;     -- valid @ T+RESP_LAT (from the centre scorer)

	signal v_r      : std_logic := '0';                 -- registered outputs @ T+RESP_LAT+1
	signal cscore_r : pixel_t   := (others => '0');
	signal lmax_r   : std_logic := '0';

begin

	-- The +/-1 neighbour circles must stay inside the patch: centre index N/2,
	-- neighbour offset 1, circle radius 3 -> N/2 + 4 <= N-1, i.e. N >= 9.
	-- extract_fast_circle_at checks this only at simulation time; fail
	-- elaboration statically here (static condition, checked by synthesis too).
	assert N >= 9
		report "fast_nms: N must be >= 9 (3x3 NMS on a radius-3 circle), got " & to_string(N)
		severity failure;

	-- Nine FAST scorers, one per position, all fed from the SAME patch. fast_response
	-- is combinational-in / 4-stage-pipelined, so all nine scores land together.
	gen_score : for k in 0 to NUM_POS-1 generate
		signal circ : fast_circle_t;
		signal vk   : std_logic;
	begin
		circ <= extract_fast_circle_at(i_patch, OFFS(k).dr, OFFS(k).dc);

		u_resp : entity work.fast_response
			port map (
				clk      => clk,
				rst      => rst,
				i_valid  => i_valid,
				i_center => circ.center,
				i_circle => circ.circle,
				o_valid  => vk,
				o_score  => scores(k)
			);

		gen_v : if k = 0 generate
			v_score <= vk;   -- all nine share timing; use the centre's valid
		end generate;
	end generate;

	-- Register the centre score, validity and the NMS verdict together so the verdict
	-- never sits in the consumer's FIFO-write-enable path.
	p_nms : process(clk)
		variable is_max : std_logic;
	begin
		if rising_edge(clk) then
			if rst = '1' then
				v_r      <= '0';
				cscore_r <= (others => '0');
				lmax_r   <= '0';
			else
				v_r      <= v_score;
				cscore_r <= scores(0);

				-- Centre is a local maximum iff no neighbour STRICTLY exceeds it.
				-- This is the '>=' (plateau-keeping) rule.
				--
				-- KNOWN DEVIATION (2026-08-11): this does NOT match OpenCV. FAST_t
				-- keeps a corner only when "score > n" holds for all 8 neighbours
				-- (opencv 4.5.4 modules/features2d/src/fast.cpp), i.e. it DROPS
				-- ties, while this keeps both members of a tied pair. Pinned on a
				-- real MH01 frame: cv2 9857 corners == the '>' rule exactly (sym
				-- diff 0), while this rule keeps 11926 (+2069, superset). After the
				-- per-cell selection the residual is 518 candidates of 4563 (11.4%),
				-- about twice the deliberate seam residual (274, 6.0%).
				--
				-- NOT FIXED ON PURPOSE: the one-character fix below changes the
				-- synthesized design and would invalidate the whole on-board
				-- measurement campaign. Documented in the thesis instead.
				-- To drop-on-tie (== OpenCV), change "scores(0) < scores(k)" to "<=".
				is_max := '1';
				for k in 1 to NUM_POS-1 loop
					if scores(0) < scores(k) then
						is_max := '0';
					end if;
				end loop;
				lmax_r <= is_max;
			end if;
		end if;
	end process;

	o_valid        <= v_r;
	o_center_score <= cscore_r;
	o_is_local_max <= lmax_r;

end architecture;