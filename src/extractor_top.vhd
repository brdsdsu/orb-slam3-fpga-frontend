library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;   -- pixel_t, coord_t, patch_arr_t, ceil_log2, COORD_WIDTH, PIXEL_WIDTH
use work.fast_pkg.all;      -- circle_t, threshold_t, fast_circle_t, extract_fast_circle
use work.umax_pkg.all;      -- PATCH_SIZE, MOMENT_WIDTH

-- Top-level Level-3 feature extractor.
--
-- Pixel stream -> window_NxN (N=31) -> { fast9_core (detect), fast_nms (score +
-- 3x3 non-max suppression) } in parallel, plus orientation_top consuming the full
-- 31x31 patch. A corner is pushed (with its patch) into orientation_top only when
-- fast9_core flags it AND fast_nms says it is a 3x3 local maximum of the FAST score
-- field -- matching OpenCV FAST(..., nonmaxSuppression=true). orientation_top emits
-- (x, y, score, is_brighter, angle); the strict-threshold annotation is recomputed
-- at the output from the pass-through score.
--
-- NMS rationale: without suppression every permissive corner is pushed, which
-- floods the corner FIFO (drops before the zero-drop backpressure existed,
-- massive input stalls today) and diverges from the SW corner set.
-- fast_nms scores the centre and its 8 neighbours from the SAME patch (FAST needs
-- only a radius-3 circle, which is inside the 31x31 patch for centre +/-1), so NMS
-- costs no line buffers and no row latency.
--
-- Runtime sizing: img_width / img_height arrive on ports and are forwarded to
-- window_NxN so a single instance serves every pyramid level. The window line-buffer
-- DEPTH is fixed at elaboration via MAX_WIDTH (size for level 0).
--
-- Dual-threshold scheme (matches ORB-SLAM3's iniThFAST/minThFAST adaptation):
--   Detection runs at THRESHOLD_PERMISSIVE (minThFAST, e.g. 7) -- the floor.
--   Each emitted keypoint also carries passed_strict = (score >= THRESHOLD_STRICT).
--   Since the score is passed through orientation_top unchanged, the annotation is
--   recomputed at the OUTPUT -- orientation_top stays untouched.
--
-- Scoring selection (USE_HARRIS, elaboration-time): the keypoint's o_response is
-- either the FAST score (zero-extended, USE_HARRIS = false) or the Harris response
-- computed per retained corner in orientation_top's drain path (USE_HARRIS = true,
-- == OpenCV ORB's HARRIS_SCORE mode: FAST detection and FAST-score NMS are
-- IDENTICAL in both modes, Harris only re-scores the survivors). The dual-threshold
-- annotation and the strict-cell gate always work on the FAST score, so the corner
-- SET never depends on USE_HARRIS -- only the ranking value does.
--
-- Pipeline alignment (into orientation_top):
--   window output (patch, x, y, valid) ............ cycle T
--   fast9_core   o_is_corner/o_is_brighter ........ cycle T + FAST9_LAT (3)
--   fast_nms     o_center_score/o_is_local_max .... cycle T + NMS_LAT  (5)
-- The score (via fast_nms) is the latest signal, so everything aligns to
-- T + ALIGN_LAT (= NMS_LAT):
--   - patch / coords / valid delayed by ALIGN_LAT
--   - fast9 outputs delayed by a further FAST9_DLY (= ALIGN_LAT - FAST9_LAT)
--   - fast_nms score/local-max used directly (already at T + NMS_LAT)
-- The latency constants are coupled: static asserts check their mutual consistency,
-- and a simulation-time self-check (p_lat_check) compares them against the ACTUAL
-- sub-module valid chains, so a sub-module latency change fails the first testbench
-- run instead of silently misaligning the pipeline.

entity extractor_top is
	generic (
		MAX_WIDTH            : positive := 752;  -- window line-buffer depth (level 0 width)
		THRESHOLD_PERMISSIVE : natural  := 7;    -- ORB-SLAM3 minThFAST: detection floor
		THRESHOLD_STRICT     : natural  := 20;   -- ORB-SLAM3 iniThFAST: strict annotation
		FIFO_DEPTH           : positive := 256;
		URAM_COLS            : positive := 32;   -- corner_fifo UltraRAM width split (pass-through)
		USE_HARRIS           : boolean  := false -- o_response = Harris response instead of FAST score
	);
	port (
		clk : in std_logic;
		rst : in std_logic;

		-- Active image dimensions for the current frame/level.
		img_width  : in coord_t;
		img_height : in coord_t;

		-- Per-level selection-cell geometry (== ORBextractor.cc ComputeCellGrid /
		-- SelectPerCellHW). Written per level by the driver; drives the strict-cell
		-- FIFO gate so the PL and SW tile the image identically. Left at 0 (reset
		-- default) the gate is inert -- safe checks below can never assert.
		cfg_wcell  : in coord_t;
		cfg_hcell  : in coord_t;
		cfg_ncols  : in coord_t;
		cfg_nrows  : in coord_t;

		-- Streaming pixel input
		i_valid : in std_logic;
		i_pixel : in pixel_t;
		i_sof   : in std_logic;

		-- Keypoint output stream
		o_valid         : out std_logic;
		o_x             : out coord_t;
		o_y             : out coord_t;
		-- Ranking response: FAST score zero-extended (USE_HARRIS = false) or the
		-- signed Harris response (USE_HARRIS = true). passed_strict is FAST-based
		-- in BOTH modes (recomputed from the pass-through FAST score).
		o_response      : out signed(31 downto 0);
		o_is_brighter   : out std_logic;
		o_passed_strict : out std_logic;                 -- '1' if FAST score >= THRESHOLD_STRICT
		o_angle         : out signed(MOMENT_WIDTH-1 downto 0);

		-- Status / monitoring
		o_drop_count    : out unsigned(31 downto 0);
		o_supp_count    : out unsigned(31 downto 0);   -- corners suppressed by the strict-cell gate
		o_fifo_count    : out unsigned(ceil_log2(FIFO_DEPTH+1)-1 downto 0);
		-- Corner-FIFO backpressure: wire into the pixel-stream tready gate so
		-- the input stalls before the FIFO can fill (makes drops impossible).
		o_prog_full     : out std_logic
	);
end entity;

architecture rtl of extractor_top is

	constant N         : positive := PATCH_SIZE;   -- 31 (window size)

	-- Sub-module latencies (input valid -> output). Guarded by asserts below.
	constant FAST9_LAT : natural  := 3;            -- fast9_core latency
	constant NMS_LAT   : natural  := 5;            -- fast_nms latency (fast_response 4 + 1 reg NMS compare)

	-- Align everything to the slowest path (the score, now via fast_nms). The fast9
	-- outputs, which arrive FAST9_DLY cycles earlier, are delayed by FAST9_DLY.
	constant ALIGN_LAT : natural  := NMS_LAT;
	constant FAST9_DLY : natural  := ALIGN_LAT - FAST9_LAT;   -- currently 2

	constant THRESH_PERM_C   : threshold_t := to_unsigned(THRESHOLD_PERMISSIVE, PIXEL_WIDTH);
	constant THRESH_STRICT_C : pixel_t     := to_unsigned(THRESHOLD_STRICT, PIXEL_WIDTH);

	-- ================================================================
	-- Strict-cell FIFO gate: suppress permissive corners that land in a
	-- selection cell already holding a strict corner (== they would be
	-- dropped by SelectPerCellHW anyway). All grid math == ORBextractor.cc.
	-- ================================================================
	constant EDGE_THRESHOLD : natural := 19;                        -- == ORB-SLAM3 EDGE_THRESHOLD
	constant MIN_BORDER     : natural := EDGE_THRESHOLD - 3;        -- minBorderX = 16
	constant FAST_BORDER    : natural := 3;
	constant W_CELL_NOM     : natural := 35;                        -- == ORB-SLAM3 grid cell size W
	constant GRID_ORIGIN    : natural := MIN_BORDER + FAST_BORDER;  -- first cell coord = 19
	constant GUARD          : natural := FAST_BORDER;               -- conservative seam margin

	-- Derived at elaboration: widest possible cell-column count over all levels
	-- (cols <= MAX_WIDTH). Sizes the per-cell-column strict-seen storage.
	constant MAX_CELL_COLS  : natural := (MAX_WIDTH - 2*MIN_BORDER) / W_CELL_NOM;  -- = 20 @752
	constant CCW            : natural := ceil_log2(MAX_CELL_COLS);                 -- = 5
	constant GRID_X0_C      : coord_t := to_unsigned(GRID_ORIGIN, COORD_WIDTH);
	constant GRID_Y0_C      : coord_t := to_unsigned(GRID_ORIGIN, COORD_WIDTH);
	-- cellcol_live is already 1-registered (aligns with x_dly(0)); add ALIGN_LAT-1
	-- more stages to line up with x_dly(ALIGN_LAT-1) at the push.
	constant CELL_ALIGN     : natural := ALIGN_LAT - 1;

	-- live grid tracking (off win_x / win_y, gated by win_valid)
	signal cellcol_live : unsigned(CCW-1 downto 0) := (others => '0');
	signal phase_x      : coord_t := (others => '0');
	signal band_live    : coord_t := (others => '0');
	signal phase_y      : coord_t := (others => '0');
	signal safe_x_live  : std_logic := '0';
	signal safe_y_live  : std_logic := '0';
	signal prev_y       : coord_t := (others => '0');

	-- aligned-to-push copies (delay index 0..CELL_ALIGN-1). Local array types so
	-- these decls do not forward-reference coord_dly_t (declared further below).
	type   ccol_dly_t is array (natural range <>) of unsigned(CCW-1 downto 0);
	type   cbnd_dly_t is array (natural range <>) of coord_t;
	signal cellcol_q : ccol_dly_t(0 to CELL_ALIGN-1) := (others => (others => '0'));
	signal safe_q    : std_logic_vector(0 to CELL_ALIGN-1) := (others => '0');
	signal band_q    : cbnd_dly_t(0 to CELL_ALIGN-1) := (others => (others => '0'));

	-- gate state. strict_seen sized to 2**CCW so any CCW-bit index is in range.
	signal strict_seen : std_logic_vector(0 to 2**CCW - 1) := (others => '0');
	signal band_prev   : coord_t := (others => '0');
	signal is_strict   : std_logic;
	signal base_push   : std_logic;
	signal suppress    : std_logic;
	signal supp_cnt    : unsigned(31 downto 0) := (others => '0');

	-- Window outputs
	signal win_valid : std_logic;
	signal win_x     : coord_t;
	signal win_y     : coord_t;
	signal win_patch : patch_arr_t(0 to N-1, 0 to N-1);

	-- Combinational FAST circle extracted from the window patch (for detection)
	signal fc : fast_circle_t;

	-- fast9_core outputs (detection at the permissive threshold). f9_valid is
	-- consumed by the p_lat_check alignment assertion below.
	signal f9_valid       : std_logic;
	signal f9_is_corner   : std_logic;
	signal f9_is_brighter : std_logic;

	-- fast_nms outputs (centre score + 3x3 local-max verdict). nms_valid is
	-- consumed by the p_lat_check alignment assertion below.
	signal nms_valid : std_logic;
	signal nms_score : pixel_t := (others => '0');  -- init: avoid t=0 metavalue on is_strict '>='
	signal nms_lmax  : std_logic;

	-- Alignment delay lines (depth ALIGN_LAT) for patch/coords/valid
	type coord_dly_t is array (natural range <>) of coord_t;
	type patch_dly_t is array (natural range <>) of patch_arr_t(0 to N-1, 0 to N-1);

	signal x_dly     : coord_dly_t(0 to ALIGN_LAT-1);
	signal y_dly     : coord_dly_t(0 to ALIGN_LAT-1);
	signal patch_dly : patch_dly_t(0 to ALIGN_LAT-1);
	signal valid_dly : std_logic_vector(0 to ALIGN_LAT-1) := (others => '0');

	-- fast9 outputs delayed by FAST9_DLY to align with the score at T+ALIGN_LAT.
	-- Generalised to a FAST9_DLY-deep shift register (was a single stage when
	-- FAST9_DLY=1; the NMS latency makes it 2).
	signal corner_dly   : std_logic_vector(0 to FAST9_DLY-1) := (others => '0');
	signal brighter_dly : std_logic_vector(0 to FAST9_DLY-1) := (others => '0');

	signal push_valid : std_logic;

	-- orientation_top outputs (wired to internal signals so the score is visible
	-- for the strict-threshold recompute)
	signal orient_valid       : std_logic;
	signal orient_x           : coord_t;
	signal orient_y           : coord_t;
	signal orient_score       : pixel_t := (others => '0');  -- init: avoid t=0 metavalue on o_passed_strict '>='
	signal orient_is_brighter : std_logic;
	signal orient_angle       : signed(MOMENT_WIDTH-1 downto 0);
	signal orient_hresp       : signed(31 downto 0);

begin

	-----------------------------------------------------------------
	-- Compile-time check: FIFO_DEPTH must be a power of 2 (equal ceil_log2
	-- for N and N+1 means non-power-of-2; static, checked at elaboration).
	-----------------------------------------------------------------
	assert ceil_log2(FIFO_DEPTH) /= ceil_log2(FIFO_DEPTH + 1)
		report "FIFO_DEPTH must be a positive power of 2, got " & to_string(FIFO_DEPTH)
		severity failure;

	-- ================================================================
	-- Latency-assumption guards (static, elaboration-time).
	-- ================================================================
	assert NMS_LAT >= FAST9_LAT
		report "Alignment assumes fast_nms is the slowest path; "
			 & "FAST9_LAT exceeds NMS_LAT -- delay-line scheme needs rework."
		severity failure;

	assert FAST9_DLY >= 1
		report "FAST9_DLY must be >= 1 (fast9 outputs lead the score)."
		severity failure;

	-- Simulation-time self-check: the window valid delayed by FAST9_LAT /
	-- ALIGN_LAT must land exactly on the sub-modules' actual valid outputs
	-- (both are pure reset-gated shift chains of win_valid, so they must
	-- match every cycle). Catches any fast9_core / fast_nms latency change
	-- in the first TB run. Dynamic condition: ignored by synthesis.
	p_lat_check : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '0' then
				assert f9_valid = valid_dly(FAST9_LAT-1)
					report "extractor_top: FAST9_LAT no longer matches fast9_core's actual latency"
					severity failure;
				assert nms_valid = valid_dly(ALIGN_LAT-1)
					report "extractor_top: NMS_LAT no longer matches fast_nms's actual latency"
					severity failure;
			end if;
		end if;
	end process;

	-----------------------------------------------------------------
	-- THRESHOLD_STRICT should be at least THRESHOLD_PERMISSIVE
	-----------------------------------------------------------------
	assert THRESHOLD_STRICT >= THRESHOLD_PERMISSIVE
		report "THRESHOLD_STRICT " & to_string(THRESHOLD_STRICT) & " should be at least "
			 & "THRESHOLD_PERMISSIVE " & to_string(THRESHOLD_PERMISSIVE)
		severity warning;

	-- ================================================================
	-- Sliding window (single shared 31x31 source for FAST + orientation)
	-- ================================================================
	win : entity work.window_NxN
		generic map (
			N         => N,
			MAX_WIDTH => MAX_WIDTH
		)
		port map (
			clk        => clk,
			rst        => rst,
			img_width  => img_width,
			img_height => img_height,
			i_valid    => i_valid,
			i_pixel    => i_pixel,
			i_sof      => i_sof,
			o_valid    => win_valid,
			o_x        => win_x,
			o_y        => win_y,
			o_patch    => win_patch
		);

	-- Extract FAST center + Bresenham circle from the patch (combinational, for detect).
	fc <= extract_fast_circle(win_patch);

	-- ================================================================
	-- FAST detection (permissive threshold) and NMS scoring (parallel)
	-- ================================================================
	detect : entity work.fast9_core
		port map (
			clk           => clk,
			rst           => rst,
			i_valid       => win_valid,
			i_center      => fc.center,
			i_circle      => fc.circle,
			i_threshold   => THRESH_PERM_C,
			o_valid       => f9_valid,
			o_is_corner   => f9_is_corner,
			o_is_brighter => f9_is_brighter
		);

	-- fast_nms computes the centre FAST score AND whether the centre is a 3x3 local
	-- maximum of the score field, both from win_patch (no line buffers).
	nmsu : entity work.fast_nms
		generic map (
			N => N
		)
		port map (
			clk            => clk,
			rst            => rst,
			i_valid        => win_valid,
			i_patch        => win_patch,
			o_valid        => nms_valid,
			o_center_score => nms_score,
			o_is_local_max => nms_lmax
		);

	-- ================================================================
	-- Alignment: delay patch/coords/valid by ALIGN_LAT, fast9 outputs by FAST9_DLY
	-- so all meet the fast_nms score/verdict at T+ALIGN_LAT.
	-- ================================================================
	p_align : process(clk)
	begin
		if rising_edge(clk) then
			-- Datapath alignment: free-running (no reset), gated by valid_dly below.
			x_dly(0)     <= win_x;
			y_dly(0)     <= win_y;
			patch_dly(0) <= win_patch;
			for s in 1 to ALIGN_LAT-1 loop
				x_dly(s)     <= x_dly(s-1);
				y_dly(s)     <= y_dly(s-1);
				patch_dly(s) <= patch_dly(s-1);
			end loop;

			-- Control/valid path: reset so stale datapath contents are never used.
			if rst = '1' then
				valid_dly    <= (others => '0');
				corner_dly   <= (others => '0');
				brighter_dly <= (others => '0');
				safe_q       <= (others => '0');
			else
				valid_dly(0) <= win_valid;
				for s in 1 to ALIGN_LAT-1 loop
					valid_dly(s) <= valid_dly(s-1);
				end loop;

				corner_dly(0)   <= f9_is_corner;
				brighter_dly(0) <= f9_is_brighter;
				for s in 1 to FAST9_DLY-1 loop
					corner_dly(s)   <= corner_dly(s-1);
					brighter_dly(s) <= brighter_dly(s-1);
				end loop;

				-- Gate metadata aligned to the push: cellcol_live/safe/band are
				-- already 1-registered, so CELL_ALIGN(=ALIGN_LAT-1) more stages line
				-- them up with x_dly(ALIGN_LAT-1).
				cellcol_q(0) <= cellcol_live;
				safe_q(0)    <= safe_x_live and safe_y_live;
				band_q(0)    <= band_live;
				for s in 1 to CELL_ALIGN-1 loop
					cellcol_q(s) <= cellcol_q(s-1);
					safe_q(s)    <= safe_q(s-1);
					band_q(s)    <= band_q(s-1);
				end loop;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Live selection-cell tracking off the window raster (win_x/win_y).
	--   cellcol = floor((x-GRID_ORIGIN)/wCell), band = floor((y-GRID_ORIGIN)/hCell)
	--   phase_*  = position within the cell (0 at GRID_ORIGIN)
	--   safe_*   = conservative interior (inside the outer ring, GUARD px off the
	--              cell seams) where this mapping provably equals cell_of.
	-- Integer-domain bound checks so cfg_ncols<2 / cfg_wcell edges can't underflow.
	-- ================================================================
	p_cellgrid : process(clk)
		variable new_row   : boolean;
		variable v_cellcol : unsigned(CCW-1 downto 0);
		variable v_phase   : coord_t;
		variable v_band    : coord_t;
		variable v_phasey  : coord_t;
	begin
		if rising_edge(clk) then
			if rst = '1' then
				cellcol_live <= (others => '0'); phase_x <= (others => '0'); safe_x_live <= '0';
				band_live    <= (others => '0'); phase_y <= (others => '0'); safe_y_live <= '0';
				prev_y       <= (others => '0');
			elsif win_valid = '1' then
				new_row := (win_y /= prev_y);
				prev_y  <= win_y;

				-- X / column: cell + phase of the CURRENT win_x computed into variables,
				-- so the safe check and the registered outputs describe the same pixel.
				if win_x <= GRID_X0_C then
					v_cellcol := (others => '0');
					v_phase   := (others => '0');
				elsif phase_x = cfg_wcell - to_unsigned(1, COORD_WIDTH) then
					v_phase := (others => '0');
					if cellcol_live /= to_unsigned(2**CCW - 1, CCW) then
						v_cellcol := cellcol_live + 1;   -- saturate, never wrap
					else
						v_cellcol := cellcol_live;
					end if;
				else
					v_phase   := phase_x + 1;
					v_cellcol := cellcol_live;
				end if;
				cellcol_live <= v_cellcol;
				phase_x      <= v_phase;
				if  win_x > GRID_X0_C
				and to_integer(v_cellcol) >= 1
				and to_integer(v_cellcol) <= to_integer(cfg_ncols) - 2
				and to_integer(v_phase)   >= GUARD
				and to_integer(v_phase)   <= to_integer(cfg_wcell) - 1 - GUARD then
					safe_x_live <= '1';
				else
					safe_x_live <= '0';
				end if;

				-- Y / band: advance once per new row; the value persists across the row.
				if new_row then
					if win_y <= GRID_Y0_C then
						v_band   := (others => '0');
						v_phasey := (others => '0');
					elsif phase_y = cfg_hcell - to_unsigned(1, COORD_WIDTH) then
						v_phasey := (others => '0');
						v_band   := band_live + 1;
					else
						v_phasey := phase_y + 1;
						v_band   := band_live;
					end if;
					band_live <= v_band;
					phase_y   <= v_phasey;
					if  win_y > GRID_Y0_C
					and to_integer(v_band)   >= 1
					and to_integer(v_band)   <= to_integer(cfg_nrows) - 2
					and to_integer(v_phasey) >= GUARD
					and to_integer(v_phasey) <= to_integer(cfg_hcell) - 1 - GUARD then
						safe_y_live <= '1';
					else
						safe_y_live <= '0';
					end if;
				end if;
			end if;
		end if;
	end process;

	-- At T+ALIGN_LAT: aligned patch/coords, aligned corner flag, the score, and the
	-- NMS verdict all correspond to window position T. base_push = local-maximum
	-- corner; the gate then drops a permissive corner whose safe cell already holds
	-- a strict one (== SelectPerCellHW would discard it), shrinking the FIFO flood.
	base_push <= valid_dly(ALIGN_LAT-1) and corner_dly(FAST9_DLY-1) and nms_lmax;
	is_strict <= '1' when nms_score >= THRESH_STRICT_C else '0';

	suppress   <= base_push and (not is_strict) and safe_q(CELL_ALIGN-1)
	                         and strict_seen(to_integer(cellcol_q(CELL_ALIGN-1)));
	push_valid <= base_push and (not suppress);

	-- Strict-seen memory: one bit per cell-column, cleared at each cell-row band
	-- boundary. Set (and read) in the push time frame via the aligned copies.
	p_seen : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				strict_seen <= (others => '0');
				band_prev   <= (others => '0');
				supp_cnt    <= (others => '0');
			else
				if band_q(CELL_ALIGN-1) /= band_prev then   -- new band: clear history
					strict_seen <= (others => '0');
					band_prev   <= band_q(CELL_ALIGN-1);
				end if;
				if base_push = '1' and is_strict = '1' and safe_q(CELL_ALIGN-1) = '1' then
					strict_seen(to_integer(cellcol_q(CELL_ALIGN-1))) <= '1';
				end if;
				if suppress = '1' then
					supp_cnt <= supp_cnt + 1;
				end if;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Orientation (FIFO -> moment_processor -> CORDIC), emits keypoints.
	-- ================================================================
	orient : entity work.orientation_top
		generic map (
			FIFO_DEPTH => FIFO_DEPTH,
			URAM_COLS  => URAM_COLS,
			USE_HARRIS => USE_HARRIS
		)
		port map (
			clk           => clk,
			rst           => rst,
			i_valid       => push_valid,
			i_x           => x_dly(ALIGN_LAT-1),
			i_y           => y_dly(ALIGN_LAT-1),
			i_score       => nms_score,
			i_is_brighter => brighter_dly(FAST9_DLY-1),
			i_patch       => patch_dly(ALIGN_LAT-1),
			o_valid       => orient_valid,
			o_x           => orient_x,
			o_y           => orient_y,
			o_score       => orient_score,
			o_is_brighter => orient_is_brighter,
			o_angle       => orient_angle,
			o_hresponse   => orient_hresp,
			o_drop_count  => o_drop_count,
			o_fifo_count  => o_fifo_count,
			o_prog_full   => o_prog_full
		);

	-- ================================================================
	-- Output stage: pass orientation results through, recompute strict annotation.
	-- The strict annotation ALWAYS uses the pass-through FAST score; only the
	-- ranking response is selected by USE_HARRIS (at elaboration).
	-- ================================================================
	o_valid         <= orient_valid;
	o_x             <= orient_x;
	o_y             <= orient_y;
	o_is_brighter   <= orient_is_brighter;
	o_angle         <= orient_angle;
	o_passed_strict <= '1' when orient_score >= THRESH_STRICT_C else '0';
	o_supp_count    <= supp_cnt;

	gen_resp : if USE_HARRIS generate
		o_response <= orient_hresp;
	else generate
		o_response <= signed(resize(orient_score, 32));
	end generate;

end architecture;