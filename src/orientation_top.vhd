library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.umax_pkg.all;   -- PATCH_SIZE, MOMENT_WIDTH

-- Orientation pipeline for ORB-SLAM3 keypoints.
--
-- Consumes corners (with their 31x31 patch) from an internal corner_fifo,
-- computes the intensity-centroid moments (moment_processor), derives the
-- orientation angle (cordic_atan2), and emits one enriched keypoint per
-- corner: (x, y, score, is_brighter, angle).
--
-- USE_HARRIS additionally instantiates harris_response in stage M, running in
-- parallel with the moment processor on the SAME latched patch (the circle-
-- masked FIFO patch covers the 7x7 Harris block + gradient halo, see
-- harris_response). Its 19-cycle latency is always shorter than the moment
-- processor's 23, so stage M's initiation interval is unchanged; the result is
-- carried through the handoff slot and emitted on o_hresponse alongside the
-- keypoint. With USE_HARRIS false the unit is not synthesized and o_hresponse
-- is constant zero. The FAST score (o_score) is carried in BOTH modes -- the
-- strict-threshold annotation upstream is always FAST-based.
--
-- Architecture (two-stage macro-pipeline):
--   - corner_fifo buffers bursts of corners. o_prog_full is exported for the
--     upstream pixel-stream tready gate (TOP), which stalls the input before
--     the FIFO can fill -- tail-drop (drop counted) remains only as a canary.
--   - Stage M owns the FIFO pop + moment_processor: it pops one corner, walks
--     it through the moment processor, and parks the result (m_10/m_01 plus the
--     x/y/score/is_brighter metadata) in a one-entry HANDOFF slot.
--   - Stage C owns the CORDIC + emit: it takes a parked moment result, runs
--     the angle through CORDIC, and emits the enriched keypoint.
--   The two stages run concurrently: while stage C runs CORDIC on corner N,
--   stage M is already popping and computing moments for corner N+1. Stage M
--   takes 28 cycles per corner (capture + pop + 23-cycle moment latency +
--   handoff + the return through M_IDLE) and stage C 26 (consume + 24-cycle
--   CORDIC), so the steady-state initiation interval is max(28,26) = 28 cycles
--   per corner instead of the ~54 of a fully-sequential machine -- ~2x the
--   drain rate, which bounds how long the corner-FIFO backpressure stalls the
--   pixel input under a corner flood (STALLCNT; tail-drop itself is gated off
--   upstream).
--
--   Output ordering is preserved: there is a single corner path (FIFO order ->
--   stage M -> handoff -> stage C -> output), so the k-th emitted keypoint
--   corresponds to the k-th popped corner. Bit-exact with the sequential
--   version: identical datapaths (same moment_processor, same CORDIC), only the
--   control flow is overlapped.
--
-- CORDIC is instantiated at DATA_WIDTH=24, ITERATIONS=22 so the 24-bit
-- moments feed directly with no scaling (bit-exact with software fastAtan2
-- up to CORDIC's ~0.0012 degree approximation error).

entity orientation_top is
	generic (
		FIFO_DEPTH : positive := 256;
		URAM_COLS  : positive := 32;   -- corner_fifo UltraRAM width split (pass-through)
		USE_HARRIS : boolean  := false -- synthesize harris_response alongside the moments
	);
	port (
		clk : in std_logic;
		rst : in std_logic;

		-- Input: corners from FAST detection (push interface to internal FIFO)
		i_valid       : in  std_logic;   -- '1' when a corner is presented
		i_x           : in  coord_t;
		i_y           : in  coord_t;
		i_score       : in  pixel_t;
		i_is_brighter : in  std_logic;
		i_patch       : in  patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

		-- Output: enriched keypoints (single-cycle valid pulse)
		o_valid       : out std_logic;
		o_x           : out coord_t;
		o_y           : out coord_t;
		-- Default init on o_score: kills the t=0 iteration-0 metavalue warning
		-- on extractor_top's o_passed_strict '>=' compare.
		o_score       : out pixel_t := (others => '0');
		o_is_brighter : out std_logic;
		o_angle       : out signed(MOMENT_WIDTH-1 downto 0);   -- CORDIC fixed-point angle
		-- Harris response of the corner (USE_HARRIS builds; constant 0 otherwise)
		o_hresponse   : out signed(31 downto 0);

		-- Status / monitoring
		o_drop_count  : out unsigned(31 downto 0);
		o_fifo_count  : out unsigned(ceil_log2(FIFO_DEPTH+1)-1 downto 0);
		-- Corner-FIFO backpressure, for the upstream pixel-stream tready gate
		o_prog_full   : out std_logic

	);
end entity;

architecture rtl of orientation_top is

	-- CORDIC configuration. DATA_WIDTH = MOMENT_WIDTH so the moments feed the
	-- CORDIC directly with no scaling.
	constant CORDIC_WIDTH : natural := MOMENT_WIDTH;
	constant CORDIC_ITERS : natural := 22;

	-- empty pixel constant
	constant ZERO_PIXEL : pixel_t := (others => '0');

	-- ----------------------------------------------------------------
	-- FIFO interface signals
	-- ----------------------------------------------------------------
	signal fifo_rd_en          : std_logic := '0';
	signal fifo_rd_x           : coord_t;
	signal fifo_rd_y           : coord_t;
	signal fifo_rd_score       : pixel_t;
	signal fifo_rd_is_brighter : std_logic;
	signal fifo_rd_patch       : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
	signal fifo_empty          : std_logic;
	signal fifo_full           : std_logic;
	signal fifo_rd_valid       : std_logic;

	-- o_empty/o_full are observed via fifo_rd_valid instead (show-ahead FIFO:
	-- rd_valid is the correct head-available signal). Kept wired for waveform
	-- visibility during bring-up.
	attribute unused : boolean;
	attribute unused of fifo_empty : signal is true;
	attribute unused of fifo_full  : signal is true;

	-- ----------------------------------------------------------------
	-- moment_processor interface
	-- ----------------------------------------------------------------
	signal mp_start : std_logic := '0';
	signal mp_patch : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
	signal mp_done  : std_logic;
	signal mp_m_10  : signed(MOMENT_WIDTH-1 downto 0);
	signal mp_m_01  : signed(MOMENT_WIDTH-1 downto 0);
	signal mp_ready : std_logic;

	-- ----------------------------------------------------------------
	-- harris_response interface (instantiated only when USE_HARRIS).
	-- Started together with the moment processor on the same mp_patch.
	-- ----------------------------------------------------------------
	signal hr_done : std_logic := '0';
	signal hr_resp : signed(31 downto 0) := (others => '0');

	-- ----------------------------------------------------------------
	-- cordic interface
	-- ----------------------------------------------------------------
	signal cordic_valid_i : std_logic := '0';
	signal cordic_x       : signed(CORDIC_WIDTH-1 downto 0);
	signal cordic_y       : signed(CORDIC_WIDTH-1 downto 0);
	signal cordic_angle   : signed(CORDIC_WIDTH-1 downto 0);
	signal cordic_valid_o : std_logic;

	-- ================================================================
	-- Two-stage control: stage M (FIFO pop + moment) and stage C
	-- (CORDIC + emit), coupled by a one-entry handoff slot.
	-- ================================================================
	-- Stage M FSM
	type mstate_t is (
		M_IDLE,      -- wait for FIFO non-empty and moment processor ready
		M_CAPTURE,   -- latch the now-stable fifo_rd_* head (no rd_en yet)
		M_POP,       -- assert rd_en + mp_start; mp_patch already stable
		M_WAIT_MP,   -- wait for moment_processor o_done, latch moments
		M_HANDOFF    -- write the handoff slot once it is free
	);
	signal mstate : mstate_t := M_IDLE;

	-- Stage C FSM
	type cstate_t is (
		C_IDLE,         -- wait for a parked moment result in the handoff slot
		C_WAIT_CORDIC   -- wait for cordic valid_o, then emit
	);
	signal cstate : cstate_t := C_IDLE;

	-- Stage M parked metadata (held across the moment computation)
	signal m_x           : coord_t := (others => '0');
	signal m_y           : coord_t := (others => '0');
	signal m_score       : pixel_t := (others => '0');
	signal m_is_brighter : std_logic := '0';

	-- Stage M latched moments (captured on mp_done)
	signal mm_10 : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal mm_01 : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');

	-- Stage M latched Harris result (captured on hr_done, which always leads
	-- mp_done). h_seen records that the capture happened for this corner.
	signal hh_resp : signed(31 downto 0) := (others => '0');
	signal h_seen  : std_logic := '0';

	-- Handoff slot (stage M -> stage C). Single entry with a valid flag.
	signal ho_valid       : std_logic := '0';
	signal ho_m10         : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal ho_m01         : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal ho_x           : coord_t := (others => '0');
	signal ho_y           : coord_t := (others => '0');
	signal ho_score       : pixel_t := (others => '0');
	signal ho_is_brighter : std_logic := '0';
	signal ho_hresp       : signed(31 downto 0) := (others => '0');

	-- Stage C registered CORDIC operands + metadata (latched at consume so a
	-- concurrent stage-M handoff write cannot disturb the in-flight CORDIC).
	signal c_m10         : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal c_m01         : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal c_x           : coord_t := (others => '0');
	signal c_y           : coord_t := (others => '0');
	signal c_score       : pixel_t := (others => '0');
	signal c_is_brighter : std_logic := '0';
	signal c_hresp       : signed(31 downto 0) := (others => '0');

	-- Output registers
	signal out_valid       : std_logic := '0';
	signal out_x           : coord_t := (others => '0');
	signal out_y           : coord_t := (others => '0');
	signal out_score       : pixel_t := (others => '0');
	signal out_is_brighter : std_logic := '0';
	signal out_angle       : signed(MOMENT_WIDTH-1 downto 0) := (others => '0');
	signal out_hresp       : signed(31 downto 0) := (others => '0');

begin

	-- ================================================================
	-- Submodule instantiations
	-- ================================================================
	fifo : entity work.corner_fifo
		generic map (
			FIFO_DEPTH => FIFO_DEPTH,
			URAM_COLS  => URAM_COLS
		)
		port map (
			clk            => clk,
			rst            => rst,
			wr_en          => i_valid,
			wr_x           => i_x,
			wr_y           => i_y,
			wr_score       => i_score,
			wr_is_brighter => i_is_brighter,
			wr_patch       => i_patch,
			rd_en          => fifo_rd_en,
			rd_x           => fifo_rd_x,
			rd_y           => fifo_rd_y,
			rd_score       => fifo_rd_score,
			rd_is_brighter => fifo_rd_is_brighter,
			rd_patch       => fifo_rd_patch,
			rd_valid       => fifo_rd_valid,
			o_empty        => fifo_empty,
			o_full         => fifo_full,
			o_prog_full    => o_prog_full,
			o_count        => o_fifo_count,
			o_drop_count   => o_drop_count
		);

	mp : entity work.moment_processor
		port map (
			clk     => clk,
			rst     => rst,
			i_start => mp_start,
			i_patch => mp_patch,
			o_done  => mp_done,
			o_m_10  => mp_m_10,
			o_m_01  => mp_m_01,
			o_ready => mp_ready
		);

	-- Harris scorer: same handshake and patch as the moment processor, running
	-- concurrently in stage M. Not generated in FAST-score builds (hr_done /
	-- hr_resp keep their '0' defaults, and the M_WAIT_MP guard short-circuits
	-- on the generic, so the h_seen machinery is optimized away).
	gen_harris : if USE_HARRIS generate
		hr : entity work.harris_response
			port map (
				clk        => clk,
				rst        => rst,
				i_start    => mp_start,
				i_patch    => mp_patch,
				o_done     => hr_done,
				o_response => hr_resp,
				o_ready    => open   -- always back in idle before mp_ready re-asserts
			);
	end generate;

	cordic : entity work.cordic_atan2
		generic map (
			DATA_WIDTH => CORDIC_WIDTH,
			ITERATIONS => CORDIC_ITERS
		)
		port map (
			clk     => clk,
			rst     => rst,
			valid_i => cordic_valid_i,
			x_i     => cordic_x,
			y_i     => cordic_y,
			angle_o => cordic_angle,
			valid_o => cordic_valid_o
		);

	-- CORDIC inputs: m_10 is the x-axis moment, m_01 the y-axis moment.
	-- ORB-SLAM3 computes fastAtan2(m_01, m_10), i.e. angle = atan2(y=m_01, x=m_10).
	-- Driven from the stage-C registered operands, captured the cycle cordic_valid_i
	-- is asserted, so they are stable for the whole CORDIC run.
	cordic_x <= c_m10;
	cordic_y <= c_m01;

	-- Output port wiring
	o_valid       <= out_valid;
	o_x           <= out_x;
	o_y           <= out_y;
	o_score       <= out_score;
	o_is_brighter <= out_is_brighter;
	o_angle       <= out_angle;
	o_hresponse   <= out_hresp;

	-- ================================================================
	-- Main control: stage M and stage C in one clocked process (single
	-- driver for the shared handoff slot). Stage C is evaluated before
	-- stage M so that when C consumes the slot in the same cycle M wants
	-- to refill it, M's write (textually later) wins -- a clean back-to-
	-- back handoff with no bubble.
	-- ================================================================
	p_pipe : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				-- Synchronous reset: both FSMs, control pulses, slot, outputs
				mstate         <= M_IDLE;
				cstate         <= C_IDLE;
				fifo_rd_en     <= '0';
				mp_start       <= '0';
				cordic_valid_i <= '0';
				out_valid      <= '0';
				ho_valid       <= '0';
				-- Stage M metadata + moments
				m_x           <= (others => '0');
				m_y           <= (others => '0');
				m_score       <= (others => '0');
				m_is_brighter <= '0';
				mm_10         <= (others => '0');
				mm_01         <= (others => '0');
				hh_resp       <= (others => '0');
				h_seen        <= '0';
				-- Handoff payload
				ho_m10        <= (others => '0');
				ho_m01        <= (others => '0');
				ho_x          <= (others => '0');
				ho_y          <= (others => '0');
				ho_score      <= (others => '0');
				ho_is_brighter <= '0';
				ho_hresp      <= (others => '0');
				-- Stage C operands + metadata
				c_m10         <= (others => '0');
				c_m01         <= (others => '0');
				c_x           <= (others => '0');
				c_y           <= (others => '0');
				c_score       <= (others => '0');
				c_is_brighter <= '0';
				c_hresp       <= (others => '0');
				-- Output registers
				out_x           <= (others => '0');
				out_y           <= (others => '0');
				out_score       <= (others => '0');
				out_is_brighter <= '0';
				out_angle       <= (others => '0');
				out_hresp       <= (others => '0');
				-- Clear mp_patch
				mp_patch <= (others => (others => ZERO_PIXEL));
			else
				-- Default: deassert single-cycle pulses
				fifo_rd_en     <= '0';
				mp_start       <= '0';
				cordic_valid_i <= '0';
				out_valid      <= '0';

				-- ----------------------------------------------------
				-- Stage C: handoff slot -> CORDIC -> emit
				-- ----------------------------------------------------
				case cstate is

					when C_IDLE =>
						if ho_valid = '1' then
							-- Consume the parked result. Register the operands and
							-- metadata so a same-cycle stage-M refill of the slot
							-- cannot disturb this CORDIC run.
							c_m10         <= ho_m10;
							c_m01         <= ho_m01;
							c_x           <= ho_x;
							c_y           <= ho_y;
							c_score       <= ho_score;
							c_is_brighter <= ho_is_brighter;
							c_hresp       <= ho_hresp;
							cordic_valid_i <= '1';
							ho_valid       <= '0';   -- slot freed (M may overwrite below)
							cstate         <= C_WAIT_CORDIC;
						end if;

					when C_WAIT_CORDIC =>
						if cordic_valid_o = '1' then
							-- Emit the enriched keypoint as a single-cycle pulse.
							out_angle       <= cordic_angle;
							out_x           <= c_x;
							out_y           <= c_y;
							out_score       <= c_score;
							out_is_brighter <= c_is_brighter;
							out_hresp       <= c_hresp;
							out_valid       <= '1';
							cstate          <= C_IDLE;
						end if;

				end case;

				-- ----------------------------------------------------
				-- Stage M: FIFO pop -> moment_processor -> handoff slot
				-- ----------------------------------------------------
				case mstate is

					when M_IDLE =>
						-- FIFO reports a valid head and the moment processor is free.
						-- Wait one settle cycle (M_CAPTURE) before trusting rd_* data,
						-- because rd_valid leads rd_data_r by a cycle in this FIFO.
						if fifo_rd_valid = '1' and mp_ready = '1' then
							mstate <= M_CAPTURE;
						end if;

					when M_CAPTURE =>
						-- rd_* data is now stable. Latch the head entry. Do NOT assert
						-- rd_en yet -- sample with the FIFO not mid-advance.
						m_x           <= fifo_rd_x;
						m_y           <= fifo_rd_y;
						m_score       <= fifo_rd_score;
						m_is_brighter <= fifo_rd_is_brighter;
						mp_patch      <= fifo_rd_patch;
						mstate        <= M_POP;

					when M_POP =>
						-- Metadata + patch are latched. Pop the FIFO (advance past this
						-- entry) and start the moment processor (and, in USE_HARRIS
						-- builds, the Harris scorer -- same start, same patch).
						fifo_rd_en <= '1';
						mp_start   <= '1';
						h_seen     <= '0';
						mstate     <= M_WAIT_MP;

					when M_WAIT_MP =>
						-- Wait for BOTH units. hr_done always leads mp_done (19 vs 23
						-- cycles), so the Harris term never adds a cycle; it is kept as
						-- a guard (with the same-cycle hr_done escape) so a latency
						-- change in either unit cannot silently truncate the response.
						-- In FAST builds the generic short-circuits the whole term.
						if mp_done = '1' and (not USE_HARRIS or h_seen = '1' or hr_done = '1') then
							mm_10  <= mp_m_10;
							mm_01  <= mp_m_01;
							mstate <= M_HANDOFF;
						end if;

					when M_HANDOFF =>
						-- Write the parked moments into the handoff slot once it is
						-- free. The slot is free if it was already empty, or if stage C
						-- is consuming it this very cycle (cstate = C_IDLE with a valid
						-- entry) -- in which case this textually-later write wins and the
						-- slot is refilled back-to-back with no bubble.
						if ho_valid = '0' or cstate = C_IDLE then
							ho_m10         <= mm_10;
							ho_m01         <= mm_01;
							ho_x           <= m_x;
							ho_y           <= m_y;
							ho_score       <= m_score;
							ho_is_brighter <= m_is_brighter;
							ho_hresp       <= hh_resp;
							ho_valid       <= '1';
							mstate         <= M_IDLE;
						end if;

				end case;

				-- Harris capture, textually AFTER the FSM case: if hr_done ever
				-- coincided with M_POP's h_seen clear (it cannot -- the previous
				-- corner's done pulses before its handoff), the capture wins.
				if USE_HARRIS and hr_done = '1' then
					hh_resp <= hr_resp;
					h_seen  <= '1';
				end if;
			end if;
		end if;
	end process;

end architecture;
