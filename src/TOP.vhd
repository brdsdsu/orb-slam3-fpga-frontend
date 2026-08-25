--------------------------------------------------------------------------------
-- TOP.vhd
-- AXI wrapper around extractor_top (Level-3: window + FAST-9 + orientation).
--
-- Interfaces:
--   * AXI4-Stream slave  (pixels in, raster order, from AXI DMA MM2S)
--   * AXI4-Stream master (keypoints out, 128-bit packed, to AXI DMA S2MM)
--   * AXI4-Lite slave    (config / status)
--
-- Single clock domain (aclk). DMA stream clock == PL fabric clock == AXI-Lite
-- clock by design (no CDC for first bring-up).
--
-- Per-frame operation: the control FSM holds the core in reset while idle, so
-- every frame (every pyramid level) starts from clean counters. The PS sets
-- image dims + enable, waits for STATUS.done, clears enable, repeats per level.
--
-- Keypoint packing v3 (128-bit, LSB-first field layout for easy C-struct parse;
-- layout version is coupled to the BUILD ID -- v3 since 0xC0DE0003):
--   word0 [ 15:  0] = x (11b used)     word2 [ 87: 64] = angle (24b two's compl.)
--   word0 [ 31: 16] = y (11b used)     word2 [    88 ] = is_brighter
--   word1 [ 63: 32] = response, int32  word2 [    89 ] = passed_strict
--     (G_SCORE_TYPE=1: FAST score      word2 [ 95: 90] = reserved (0)
--      0..255 zero-extended;           word3 [127: 96] = 0x00000000 (real kp)
--      G_SCORE_TYPE=0: signed Harris                    0xFFFFFFFF (EOF sentinel)
--      response, see harris_response)
--
-- NOTE: G_KP_FIFO_DEPTH must be a power of two (pointer wrap relies on it).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;   -- ceil_log2, coord_t, pixel_t, COORD_WIDTH, PIXEL_WIDTH
use work.umax_pkg.all;      -- PATCH_SIZE (minimum legal image dimension), MOMENT_WIDTH (angle width)

entity TOP is
	generic (
		-- Packed keypoint word (layout above assumes 128)
		G_KP_WIDTH        : positive := 128;

		-- Output keypoint FIFO (MUST be a power of two, and must exceed
		-- G_PROG_FULL_GAP -- the gap is RESERVED space; see asserts below)
		G_KP_FIFO_DEPTH   : positive := 1024;



		-- Drain detection: cycles with no core keypoint after the last input pixel.
		-- Must exceed (tail compute latency) + (max gap between drained keypoints).
		G_IDLE_GAP        : positive := 512;

		-- Forwarded to extractor_top
		G_MAX_WIDTH       : positive := 752;    -- window line-buffer depth (level 0)
		G_THRESH_PERM     : natural  := 7;      -- ORB-SLAM3 minThFAST (baked in)
		G_THRESH_STRICT   : natural  := 20;     -- ORB-SLAM3 iniThFAST (baked in)

		-- Keypoint ranking response, mirroring the OpenCV ORB scoreType enum
		-- (ORBextractor.h: HARRIS_SCORE = 0, FAST_SCORE = 1). Elaboration-time:
		-- only the selected scoring datapath is synthesized. Detection, NMS and
		-- passed_strict are FAST-based in BOTH modes; readback: THRESH[16].
		G_SCORE_TYPE      : natural  := 1;      -- 0 = HARRIS_SCORE, 1 = FAST_SCORE
		G_CORE_FIFO_DEPTH : positive := 512;    -- extractor_top's internal corner FIFO
		G_CORE_URAM_COLS  : positive := 32;     -- corner-FIFO UltraRAM columns (32 -> 32 URAM + 52 BRAM36
		                                        -- per instance; sized so TWO instances fit the K26)
		G_PROG_FULL_GAP   : positive := 530;	-- must be >= G_CORE_FIFO_DEPTH + pipeline tail (~16)

		-- AXI4-Lite
		C_S_AXI_DATA_WIDTH : integer := 32;
		C_S_AXI_ADDR_WIDTH : integer := 6
	);
	port (
		aclk        : in  std_logic;
		aresetn     : in  std_logic;

		------------------------------------------------------------------
		-- AXI4-Stream slave : pixel input (from DMA MM2S)
		------------------------------------------------------------------
		s_axis_tdata  : in  std_logic_vector(PIXEL_WIDTH-1 downto 0);
		s_axis_tkeep  : in  std_logic_vector((PIXEL_WIDTH/8)-1 downto 0);
		s_axis_tvalid : in  std_logic;
		s_axis_tready : out std_logic;
		s_axis_tlast  : in  std_logic;

		------------------------------------------------------------------
		-- AXI4-Stream master : keypoint output (to DMA S2MM)
		------------------------------------------------------------------
		m_axis_tdata  : out std_logic_vector(G_KP_WIDTH-1 downto 0);
		m_axis_tkeep  : out std_logic_vector((G_KP_WIDTH/8)-1 downto 0);
		m_axis_tvalid : out std_logic;
		m_axis_tready : in  std_logic;
		m_axis_tlast  : out std_logic;

		------------------------------------------------------------------
		-- AXI4-Lite slave : control / status
		------------------------------------------------------------------
		s_axi_awaddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		s_axi_awprot  : in  std_logic_vector(2 downto 0);
		s_axi_awvalid : in  std_logic;
		s_axi_awready : out std_logic;
		s_axi_wdata   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		s_axi_wstrb   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		s_axi_wvalid  : in  std_logic;
		s_axi_wready  : out std_logic;
		s_axi_bresp   : out std_logic_vector(1 downto 0);
		s_axi_bvalid  : out std_logic;
		s_axi_bready  : in  std_logic;
		s_axi_araddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		s_axi_arprot  : in  std_logic_vector(2 downto 0);
		s_axi_arvalid : in  std_logic;
		s_axi_arready : out std_logic;
		s_axi_rdata   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		s_axi_rresp   : out std_logic_vector(1 downto 0);
		s_axi_rvalid  : out std_logic;
		s_axi_rready  : in  std_logic
	);
end entity TOP;

architecture rtl of TOP is

	-- AXI sideband signals intentionally unused by this slave.
	attribute unused : boolean;
	attribute unused of s_axis_tkeep : signal is true;
	attribute unused of s_axi_awprot : signal is true;
	attribute unused of s_axi_arprot : signal is true;

	----------------------------------------------------------------------------
	-- Register map (byte-offset INDICES, 32-bit registers).
	-- Named OFF_* to avoid colliding with the reg_* signals (VHDL folds case).
	----------------------------------------------------------------------------
	constant OFF_CTRL    : integer := 0;  -- 0x00 RW [0]=enable [1]=soft_reset
	constant OFF_STATUS  : integer := 1;  -- 0x04 RO [0]=busy [1]=done [2]=overflow [3]=cfg_error
	constant OFF_WIDTH   : integer := 2;  -- 0x08 RW image width  (lower COORD_WIDTH b)
	constant OFF_HEIGHT  : integer := 3;  -- 0x0C RW image height (lower COORD_WIDTH b)
	constant OFF_THRESH  : integer := 4;  -- 0x10 RO [7:0]=permissive [15:8]=strict (synth)
	constant OFF_KPCOUNT : integer := 5;  -- 0x14 RO real keypoints emitted this frame
	constant OFF_DROPCNT : integer := 6;  -- 0x18 RO core corner-FIFO drop count
	constant OFF_ID      : integer := 7;  -- 0x1C RO build ID
	constant OFF_CELLDIM : integer := 8;  -- 0x20 RW [15:0]=wCell  [31:16]=hCell (strict-cell gate)
	constant OFF_CELLNUM : integer := 9;  -- 0x24 RW [15:0]=nCols  [31:16]=nRows (strict-cell gate)
	constant OFF_SUPPCNT : integer := 10; -- 0x28 RO gate-suppressed permissive-corner count
	constant OFF_STALLCNT : integer := 11; -- 0x2C RO cycles the pixel input stalled on corner-FIFO
	                                       --      backpressure this frame (throughput cost of the
	                                       --      zero-drop guarantee; DROPCNT should now stay 0)

	-- 0003: keypoint packing v3 (32-bit response in word1, angle24 + flags in
	--       word2), G_SCORE_TYPE generic (FAST/Harris), THRESH[16] readback
	-- 0002: +STALLCNT register (0x2C), corner-FIFO backpressure + URAM split
	constant BUILD_ID_HEX : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) :=
		std_logic_vector(resize(unsigned'(x"C0DE0003"), C_S_AXI_DATA_WIDTH));
	constant ADDR_LSB     : integer := 2;

	-- EOF sentinel: top 32 bits (word3) = all ones, rest zero. Built by a
	-- function: an aggregate with a generic-dependent (non-locally-static)
	-- choice plus "others" is rejected by LRM-strict tools (incl. Vivado).
	function eof_word_init return std_logic_vector is
		variable v : std_logic_vector(G_KP_WIDTH-1 downto 0) := (others => '0');
	begin
		v(G_KP_WIDTH-1 downto G_KP_WIDTH-32) := (others => '1');
		return v;
	end function;
	constant EOF_WORD : std_logic_vector(G_KP_WIDTH-1 downto 0) := eof_word_init;

	----------------------------------------------------------------------------
	-- AXI4-Lite internal
	----------------------------------------------------------------------------
	signal axi_awready : std_logic := '0';
	signal axi_wready  : std_logic := '0';
	signal axi_bvalid  : std_logic := '0';
	signal axi_bresp   : std_logic_vector(1 downto 0) := "00";
	signal axi_arready : std_logic := '0';
	signal axi_rvalid  : std_logic := '0';
	signal axi_rresp   : std_logic_vector(1 downto 0) := "00";
	signal axi_rdata   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
	signal axi_awaddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
	signal axi_araddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
	signal aw_en       : std_logic := '1';

	-- Software-visible read/write registers
	signal reg_ctrl    : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
	signal reg_width   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
	signal reg_height  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
	-- Strict-cell gate geometry (wCell|hCell, nCols|nRows). Reset default 0 keeps
	-- the gate inert until the driver programs it per level.
	signal reg_celldim : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
	signal reg_cellnum : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');

	signal sw_enable     : std_logic;
	signal sw_soft_reset : std_logic;

	----------------------------------------------------------------------------
	-- Control FSM
	----------------------------------------------------------------------------
	type ctrl_state_t is (S_IDLE, S_RECV, S_DRAIN, S_FLUSH, S_DONE);
	signal cstate : ctrl_state_t := S_IDLE;

	signal idle_reset : std_logic;
	signal core_rst   : std_logic;

	signal busy       : std_logic := '0';
	signal done       : std_logic := '0';
	signal overflow   : std_logic := '0';
	signal cfg_valid : std_logic;
	signal cfg_error : std_logic := '0';
	signal kp_count   : unsigned(31 downto 0) := (others => '0');
	signal gap_cnt    : unsigned(15 downto 0) := (others => '0');
	signal sof_pulse  : std_logic := '0';
	signal first_beat : std_logic := '0';

	----------------------------------------------------------------------------
	-- Core <-> wrapper
	----------------------------------------------------------------------------
	signal core_pixel_valid : std_logic;
	signal core_kp_valid    : std_logic;
	signal core_kp_x        : coord_t;
	signal core_kp_y        : coord_t;
	signal core_kp_resp     : signed(31 downto 0);
	signal core_kp_brighter : std_logic;
	signal core_kp_strict   : std_logic;
	signal core_kp_angle    : signed(MOMENT_WIDTH-1 downto 0);
	signal core_drop_count  : unsigned(31 downto 0);
	signal core_supp_count  : unsigned(31 downto 0);
	-- Corner-FIFO backpressure from the core, plus a counter of the pixel
	-- beats it actually delayed (STALLCNT: the measured cost of zero drops).
	signal core_prog_full   : std_logic;
	signal stall_cnt        : unsigned(31 downto 0) := (others => '0');

	----------------------------------------------------------------------------
	-- Output keypoint FIFO (FWFT, count-based)
	----------------------------------------------------------------------------
	type fifo_mem_t is array (0 to G_KP_FIFO_DEPTH-1) of std_logic_vector(G_KP_WIDTH-1 downto 0);
	signal fifo_mem  : fifo_mem_t;
	signal wr_ptr    : unsigned(ceil_log2(G_KP_FIFO_DEPTH)-1 downto 0) := (others => '0');
	signal rd_ptr    : unsigned(ceil_log2(G_KP_FIFO_DEPTH)-1 downto 0) := (others => '0');
	signal fifo_cnt  : unsigned(ceil_log2(G_KP_FIFO_DEPTH)   downto 0) := (others => '0');
	signal fifo_full : std_logic;
	signal fifo_empty: std_logic;
	signal prog_full : std_logic;
	signal fifo_din  : std_logic_vector(G_KP_WIDTH-1 downto 0);
	signal fifo_dout : std_logic_vector(G_KP_WIDTH-1 downto 0);
	signal fifo_wr   : std_logic;
	signal fifo_rd   : std_logic;
	-- Asynchronous read (line: fifo_dout <= fifo_mem(...)) forces DISTRIBUTED
	-- RAM -- BRAM has synchronous read ports only. Deliberate: LUTs are the
	-- abundant resource here, and the freed BRAM (corner_fifo now takes ~36%
	-- of BRAM36 + 32 URAM) is reserved for a second extractor instance.
	attribute ram_style : string;
	attribute ram_style of fifo_mem : signal is "distributed";

	signal send_sentinel : std_logic := '0';
	signal kp_word       : std_logic_vector(G_KP_WIDTH-1 downto 0);

begin

	-- G_KP_FIFO_DEPTH must be a power of two (pointer wrap relies on it).
	-- Same ceil_log2 trick as corner_fifo: equal ceilings mean non-power-of-2.
	assert ceil_log2(G_KP_FIFO_DEPTH) /= ceil_log2(G_KP_FIFO_DEPTH + 1)
		report "G_KP_FIFO_DEPTH must be a power of two, got " & to_string(G_KP_FIFO_DEPTH)
		severity failure;

	-- The prog-full gap is reserved headroom INSIDE the FIFO; if it meets or
	-- exceeds the depth, prog_full is permanently asserted (the comparison
	-- threshold goes non-positive) and tready never rises: instant wedge.
	assert G_KP_FIFO_DEPTH > G_PROG_FULL_GAP
		report "G_KP_FIFO_DEPTH (" & to_string(G_KP_FIFO_DEPTH) & ") must exceed "
			 & "G_PROG_FULL_GAP (" & to_string(G_PROG_FULL_GAP) & ")."
		severity failure;

	assert G_PROG_FULL_GAP >= G_CORE_FIFO_DEPTH + 16
		report "G_PROG_FULL_GAP (" & to_string(G_PROG_FULL_GAP) & ") is too small."
		severity failure;

	assert C_S_AXI_DATA_WIDTH = 32
		report "TOP assumes a 32-bit AXI4-Lite data bus (register packing is hardcoded)."
		severity failure;

	-- G_SCORE_TYPE mirrors the two-valued OpenCV enum; anything else is a typo.
	assert G_SCORE_TYPE <= 1
		report "G_SCORE_TYPE must be 0 (HARRIS_SCORE) or 1 (FAST_SCORE), got "
			 & to_string(G_SCORE_TYPE)
		severity failure;

	-- gap_cnt is 16 bits; a larger G_IDLE_GAP would silently truncate in the
	-- to_unsigned compare and the drain heuristic would misfire.
	assert G_IDLE_GAP <= 2**16 - 1
		report "G_IDLE_GAP (" & to_string(G_IDLE_GAP) & ") must fit the 16-bit gap counter."
		severity failure;

	----------------------------------------------------------------------------
	-- Control decode / reset
	----------------------------------------------------------------------------
	sw_enable     <= reg_ctrl(0);
	sw_soft_reset <= reg_ctrl(1);
	idle_reset    <= '1' when cstate = S_IDLE else '0';
	-- Active-HIGH synchronous reset to the core (matches extractor_top.rst).
	-- Held during S_IDLE so each frame/level starts with clean counters.
	core_rst      <= (not aresetn) or sw_soft_reset or idle_reset;

	----------------------------------------------------------------------------
	-- Configuration sanity gate: refuse to start on dimensions the window core
	-- cannot address. Checks the FULL 32-bit registers so values that would
	-- alias after truncation to COORD_WIDTH bits are also rejected.
	--   width  in [PATCH_SIZE .. G_MAX_WIDTH]      (line-buffer depth bound)
	--   height in [PATCH_SIZE .. 2^COORD_WIDTH-1]  (coord_t representable)
	-- Catches the reset default (WIDTH=0, i.e. the PS forgot to program dims),
	-- which would otherwise underflow the line-wrap compare and index the
	-- line buffers out of bounds.
	----------------------------------------------------------------------------
	cfg_valid <= '1' when (unsigned(reg_width)  >= PATCH_SIZE)
					  and (unsigned(reg_width)  <= G_MAX_WIDTH)
					  and (unsigned(reg_height) >= PATCH_SIZE)
					  and (unsigned(reg_height) <= 2**COORD_WIDTH - 1)
			else '0';

	----------------------------------------------------------------------------
	-- AXI4-Stream slave : pixel input + backpressure. Two stall sources:
	--   prog_full      -- output KP FIFO within G_PROG_FULL_GAP of full
	--   core_prog_full -- corner FIFO nearly full (drain-rate bound); stalling
	--                     here makes corner tail-drop structurally impossible
	-- Neither drain path depends on the pixel input, so no deadlock cycle.
	----------------------------------------------------------------------------
	s_axis_tready    <= '1' when (cstate = S_RECV and prog_full = '0'
	                                             and core_prog_full = '0') else '0';
	core_pixel_valid <= s_axis_tvalid and (s_axis_tready);

	----------------------------------------------------------------------------
	-- Keypoint packing
	----------------------------------------------------------------------------
	process(all)
		variable w : std_logic_vector(G_KP_WIDTH-1 downto 0);
	begin
		w := (others => '0');
		w(15 downto 0)   := std_logic_vector(resize(core_kp_x, 16));
		w(31 downto 16)  := std_logic_vector(resize(core_kp_y, 16));
		w(63 downto 32)  := std_logic_vector(core_kp_resp);
		w(87 downto 64)  := std_logic_vector(core_kp_angle);   -- 24b, PS sign-extends
		w(88)            := core_kp_brighter;
		w(89)            := core_kp_strict;
		-- w(95 downto 90) stay 0 (reserved)
		-- w(127 downto 96) stays 0 -> real keypoint marker
		fifo_din <= w;
	end process;

	----------------------------------------------------------------------------
	-- Output keypoint FIFO
	----------------------------------------------------------------------------
	fifo_empty <= '1' when fifo_cnt = 0 else '0';
	fifo_full  <= '1' when fifo_cnt = G_KP_FIFO_DEPTH else '0';
	prog_full  <= '1' when fifo_cnt >= (G_KP_FIFO_DEPTH - G_PROG_FULL_GAP) else '0';
	fifo_dout  <= fifo_mem(to_integer(rd_ptr));

	fifo_wr <= core_kp_valid and (not fifo_full);
	fifo_rd <= m_axis_tvalid and m_axis_tready and (not send_sentinel);

	process(aclk)
	begin
		if rising_edge(aclk) then
			-- Pointers also clear throughout S_IDLE (per-frame clean): defensive
			-- against any straggler keypoint surviving the drain heuristic and
			-- leaking into the next frame's output.
			if aresetn = '0' or sw_soft_reset = '1' or idle_reset = '1' then
				wr_ptr   <= (others => '0');
				rd_ptr   <= (others => '0');
				fifo_cnt <= (others => '0');
			else
				if fifo_wr = '1' and fifo_rd = '1' then
					fifo_mem(to_integer(wr_ptr)) <= fifo_din;
					wr_ptr <= wr_ptr + 1;
					rd_ptr <= rd_ptr + 1;
				elsif fifo_wr = '1' then
					fifo_mem(to_integer(wr_ptr)) <= fifo_din;
					wr_ptr   <= wr_ptr + 1;
					fifo_cnt <= fifo_cnt + 1;
				elsif fifo_rd = '1' then
					rd_ptr   <= rd_ptr + 1;
					fifo_cnt <= fifo_cnt - 1;
				end if;
			end if;
		end if;
	end process;

	----------------------------------------------------------------------------
	-- AXI4-Stream master : keypoints out (FIFO drain + sentinel EOF)
	----------------------------------------------------------------------------
	kp_word       <= fifo_dout when send_sentinel = '0' else EOF_WORD;
	m_axis_tdata  <= kp_word;
	m_axis_tkeep  <= (others => '1');
	m_axis_tvalid <= (not fifo_empty) or send_sentinel;
	m_axis_tlast  <= send_sentinel;

----------------------------------------------------------------------------
	-- Control FSM + status counters
	-- Reset: aresetn (system) or sw_soft_reset (CTRL[1]). The soft reset
	-- recovers a wedged control FSM as well as the core. soft_reset is NOT
	-- self-clearing: the PS writes 1, then writes 0.
	----------------------------------------------------------------------------
	process(aclk)
	begin
		if rising_edge(aclk) then
			if aresetn = '0' or sw_soft_reset = '1' then
				cstate        <= S_IDLE;
				busy          <= '0';
				done          <= '0';
				overflow      <= '0';
				cfg_error     <= '0';
				kp_count      <= (others => '0');
				stall_cnt     <= (others => '0');
				gap_cnt       <= (others => '0');
				sof_pulse     <= '0';
				first_beat    <= '0';
				send_sentinel <= '0';
			else
				sof_pulse <= '0';  -- default: single-cycle pulse

				-- overflow canary: core produced a keypoint but the output FIFO was full
				if core_kp_valid = '1' and fifo_full = '1' then
					overflow <= '1';
				end if;

				-- STALLCNT: pixel beats delayed by corner-FIFO backpressure (the
				-- DMA had data, we said not-ready because of core_prog_full)
				if cstate = S_RECV and core_prog_full = '1' and s_axis_tvalid = '1' then
					stall_cnt <= stall_cnt + 1;
				end if;

				-- count real keypoints accepted by S2MM
				if m_axis_tvalid = '1' and m_axis_tready = '1' and send_sentinel = '0' then
					kp_count <= kp_count + 1;
				end if;

				case cstate is
					----------------------------------------------------------
					when S_IDLE =>
						-- core held in reset here (idle_reset)
						busy          <= '0';
						send_sentinel <= '0';
						if sw_enable = '1' then
							if cfg_valid = '1' then
								cfg_error  <= '0';
								busy       <= '1';
								done       <= '0';
								overflow   <= '0';
								kp_count   <= (others => '0');
								stall_cnt  <= (others => '0');
								gap_cnt    <= (others => '0');
								first_beat <= '1';   -- next accepted pixel is start-of-frame
								cstate     <= S_RECV;
							else
								-- Invalid dimensions: refuse to start. busy stays
								-- 0, the PS sees STATUS.cfg_error. Cleared on the
								-- next successful start (or soft reset).
								cfg_error <= '1';
							end if;
						end if;

					----------------------------------------------------------
					when S_RECV =>
						if core_pixel_valid = '1' then
							if first_beat = '1' then
								sof_pulse  <= '1';
								first_beat <= '0';
							end if;
							if s_axis_tlast = '1' then
								gap_cnt <= (others => '0');
								cstate  <= S_DRAIN;
							end if;
						end if;

					----------------------------------------------------------
					when S_DRAIN =>
						-- wait for the pipeline tail to stop producing keypoints
						if core_kp_valid = '1' then
							gap_cnt <= (others => '0');
						elsif gap_cnt = to_unsigned(G_IDLE_GAP, gap_cnt'length) then
							cstate <= S_FLUSH;
						else
							gap_cnt <= gap_cnt + 1;
						end if;

					----------------------------------------------------------
					when S_FLUSH =>
						-- all real keypoints drained, then emit one EOF beat.
						-- Assert first, handshake against the CURRENT value: the
						-- beat only completes on a cycle where tvalid and tready
						-- are genuinely both high.
						if fifo_empty = '1' then
							send_sentinel <= '1';
						end if;
						if send_sentinel = '1' and m_axis_tready = '1' then
							send_sentinel <= '0';
							cstate        <= S_DONE;
						end if;

					----------------------------------------------------------
					when S_DONE =>
						busy <= '0';
						done <= '1';
						if sw_enable = '0' then   -- PS acks by clearing enable
							cstate <= S_IDLE;
						end if;
				end case;
			end if;
		end if;
	end process;


	----------------------------------------------------------------------------
	-- Core: extractor_top (direct entity instantiation -- no component decl,
	-- so the port list can never drift out of sync with the entity).
	----------------------------------------------------------------------------
	u_core : entity work.extractor_top
		generic map (
			MAX_WIDTH            => G_MAX_WIDTH,
			THRESHOLD_PERMISSIVE => G_THRESH_PERM,
			THRESHOLD_STRICT     => G_THRESH_STRICT,
			FIFO_DEPTH           => G_CORE_FIFO_DEPTH,
			URAM_COLS            => G_CORE_URAM_COLS,
			USE_HARRIS           => G_SCORE_TYPE = 0
		)
		port map (
			clk             => aclk,
			rst             => core_rst,
			img_width       => unsigned(reg_width (COORD_WIDTH-1 downto 0)),
			img_height      => unsigned(reg_height(COORD_WIDTH-1 downto 0)),
			cfg_wcell       => unsigned(reg_celldim(COORD_WIDTH-1      downto 0)),
			cfg_hcell       => unsigned(reg_celldim(16+COORD_WIDTH-1   downto 16)),
			cfg_ncols       => unsigned(reg_cellnum(COORD_WIDTH-1      downto 0)),
			cfg_nrows       => unsigned(reg_cellnum(16+COORD_WIDTH-1   downto 16)),
			i_valid         => core_pixel_valid,
			i_pixel         => unsigned(s_axis_tdata),
			i_sof           => sof_pulse,
			o_valid         => core_kp_valid,
			o_x             => core_kp_x,
			o_y             => core_kp_y,
			o_response      => core_kp_resp,
			o_is_brighter   => core_kp_brighter,
			o_passed_strict => core_kp_strict,
			o_angle         => core_kp_angle,
			o_drop_count    => core_drop_count,
			o_supp_count    => core_supp_count,
			o_fifo_count    => open,
			o_prog_full     => core_prog_full
		);

	----------------------------------------------------------------------------
	-- AXI4-Lite : write channel
	-- Note: aresetn is treated as a SYNCHRONOUS active-low reset here, for
	-- consistency with the synchronous-reset core. Honors wstrb byte lanes.
	----------------------------------------------------------------------------
	s_axi_awready <= axi_awready;
	s_axi_wready  <= axi_wready;
	s_axi_bresp   <= axi_bresp;
	s_axi_bvalid  <= axi_bvalid;

	process(aclk)
		variable waddr : integer range 0 to 2**(C_S_AXI_ADDR_WIDTH-ADDR_LSB)-1;
	begin
		if rising_edge(aclk) then
			if aresetn = '0' then
				axi_awready <= '0';
				axi_wready  <= '0';
				axi_bvalid  <= '0';
				axi_bresp   <= "00";
				aw_en       <= '1';
				axi_awaddr  <= (others => '0');
				reg_ctrl    <= (others => '0');
				reg_width   <= (others => '0');
				reg_height  <= (others => '0');
				reg_celldim <= (others => '0');
				reg_cellnum <= (others => '0');
			else
				if axi_awready = '0' and s_axi_awvalid = '1' and s_axi_wvalid = '1' and aw_en = '1' then
					axi_awready <= '1';
					axi_awaddr  <= s_axi_awaddr;
					aw_en       <= '0';
				elsif s_axi_bready = '1' and axi_bvalid = '1' then
					aw_en       <= '1';
					axi_awready <= '0';
				else
					axi_awready <= '0';
				end if;

				if axi_wready = '0' and s_axi_wvalid = '1' and s_axi_awvalid = '1' and aw_en = '1' then
					axi_wready <= '1';
				else
					axi_wready <= '0';
				end if;

				if axi_awready = '1' and s_axi_awvalid = '1' and axi_wready = '1' and s_axi_wvalid = '1' then
					waddr := to_integer(unsigned(axi_awaddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB)));
					case waddr is
						when OFF_CTRL =>
							for b in 0 to (C_S_AXI_DATA_WIDTH/8)-1 loop
								if s_axi_wstrb(b) = '1' then
									reg_ctrl(b*8+7 downto b*8) <= s_axi_wdata(b*8+7 downto b*8);
								end if;
							end loop;
						when OFF_WIDTH =>
							for b in 0 to (C_S_AXI_DATA_WIDTH/8)-1 loop
								if s_axi_wstrb(b) = '1' then
									reg_width(b*8+7 downto b*8) <= s_axi_wdata(b*8+7 downto b*8);
								end if;
							end loop;
						when OFF_HEIGHT =>
							for b in 0 to (C_S_AXI_DATA_WIDTH/8)-1 loop
								if s_axi_wstrb(b) = '1' then
									reg_height(b*8+7 downto b*8) <= s_axi_wdata(b*8+7 downto b*8);
								end if;
							end loop;
						when OFF_CELLDIM =>
							for b in 0 to (C_S_AXI_DATA_WIDTH/8)-1 loop
								if s_axi_wstrb(b) = '1' then
									reg_celldim(b*8+7 downto b*8) <= s_axi_wdata(b*8+7 downto b*8);
								end if;
							end loop;
						when OFF_CELLNUM =>
							for b in 0 to (C_S_AXI_DATA_WIDTH/8)-1 loop
								if s_axi_wstrb(b) = '1' then
									reg_cellnum(b*8+7 downto b*8) <= s_axi_wdata(b*8+7 downto b*8);
								end if;
							end loop;
						when others => null;  -- RO / unmapped (incl. OFF_THRESH)
					end case;
					axi_bvalid <= '1';
					axi_bresp  <= "00";
				elsif s_axi_bready = '1' and axi_bvalid = '1' then
					axi_bvalid <= '0';
				end if;
			end if;
		end if;
	end process;

	----------------------------------------------------------------------------
	-- AXI4-Lite : read channel  (synchronous active-low reset, as above)
	----------------------------------------------------------------------------
	s_axi_arready <= axi_arready;
	s_axi_rvalid  <= axi_rvalid;
	s_axi_rresp   <= axi_rresp;
	s_axi_rdata   <= axi_rdata;

	process(aclk)
		variable raddr : integer range 0 to 2**(C_S_AXI_ADDR_WIDTH-ADDR_LSB)-1;
	begin
		if rising_edge(aclk) then
			if aresetn = '0' then
				axi_arready <= '0';
				axi_rvalid  <= '0';
				axi_rresp   <= "00";
				axi_araddr  <= (others => '0');
				axi_rdata   <= (others => '0');
			else
				if axi_arready = '0' and s_axi_arvalid = '1' then
					axi_arready <= '1';
					axi_araddr  <= s_axi_araddr;
				else
					axi_arready <= '0';
				end if;

				if axi_arready = '1' and s_axi_arvalid = '1' and axi_rvalid = '0' then
					axi_rvalid <= '1';
					axi_rresp  <= "00";
					raddr := to_integer(unsigned(axi_araddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB)));
					case raddr is
						when OFF_CTRL    => axi_rdata <= reg_ctrl;
						when OFF_STATUS  => axi_rdata <= (others => '0');
											axi_rdata(0) <= busy;
											axi_rdata(1) <= done;
											axi_rdata(2) <= overflow;
											axi_rdata(3) <= cfg_error;
						when OFF_WIDTH   => axi_rdata <= reg_width;
						when OFF_HEIGHT  => axi_rdata <= reg_height;
						when OFF_THRESH  => axi_rdata <= (others => '0');
											axi_rdata(7 downto 0)  <= std_logic_vector(to_unsigned(G_THRESH_PERM,   8));
											axi_rdata(15 downto 8) <= std_logic_vector(to_unsigned(G_THRESH_STRICT, 8));
											-- [16] = score type (OpenCV enum: 0=HARRIS, 1=FAST)
											axi_rdata(16)          <= to_unsigned(G_SCORE_TYPE, 1)(0);
						when OFF_KPCOUNT => axi_rdata <= std_logic_vector(resize(kp_count,        C_S_AXI_DATA_WIDTH));
						when OFF_DROPCNT => axi_rdata <= std_logic_vector(resize(core_drop_count, C_S_AXI_DATA_WIDTH));
						when OFF_ID      => axi_rdata <= BUILD_ID_HEX;
						when OFF_CELLDIM => axi_rdata <= reg_celldim;
						when OFF_CELLNUM => axi_rdata <= reg_cellnum;
						when OFF_SUPPCNT => axi_rdata <= std_logic_vector(resize(core_supp_count, C_S_AXI_DATA_WIDTH));
						when OFF_STALLCNT => axi_rdata <= std_logic_vector(resize(stall_cnt,     C_S_AXI_DATA_WIDTH));
						when others      => axi_rdata <= (others => '0');
					end case;
				elsif axi_rvalid = '1' and s_axi_rready = '1' then
					axi_rvalid <= '0';
				end if;
			end if;
		end if;
	end process;

end architecture rtl;