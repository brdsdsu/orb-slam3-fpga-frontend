library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.umax_pkg.all;   -- PATCH_SIZE

-- Synchronous FIFO for ORB-SLAM3 corner data, including the orientation
-- patch. Pushes from FAST detection (when is_corner='1'), pops from the
-- moment processor.
--
-- Behavior on full FIFO write:
--   The corner is silently dropped (tail-drop), and o_drop_count
--   increments. Status-only — no data corruption, no deadlock.
--   NOTE: with o_prog_full wired into the upstream pixel-stream tready
--   gate (see TOP), the FIFO can no longer fill and tail-drop is
--   structurally unreachable; o_drop_count remains as a canary.
--
-- Show-ahead semantics:
--   When rd_valid='1', rd_* outputs show the next-to-be-read entry. Assert
--   rd_en for one cycle to advance to the next entry.
--   CAVEAT: on the empty -> non-empty transition rd_valid rises one cycle
--   BEFORE rd_data_r holds the new entry (the synchronous RAM read issued
--   at the write edge still returns the old word). Consumers must wait one
--   cycle after rd_valid rises before sampling rd_* -- orientation_top's
--   M_CAPTURE state exists exactly for this.
--   When rd_valid='0' the rd_* outputs HOLD their last value (don't-care);
--   they are qualified by rd_valid only.
--
-- Storage:
--   Each entry packs (x, y, score, is_brighter, patch) into one wide word.
--   Only the CIRCLE_PIXELS (749) pixels inside the u_max orientation circle
--   are stored -- moment_processor provably never reads the square's corner
--   pixels (its stage 1 masks by U_MAX), and unpack returns them as zeros.
--   Entry width: 31 header + 749*8 patch = 6023 bits (was 7719 full-square).
--
--   The entry is split width-wise across two memories: the top URAM_COLS*72
--   bits in UltraRAM (fixed 4096x72 primitives -- abundant and otherwise
--   idle on the K26), the remaining bits in block RAM. At the depth 512 TOP
--   instantiates with URAM_COLS=32: 32 URAM (50% of 64) + 52 BRAM36 (36% of
--   144), vs. 108 BRAM36 (75%) for the old single-array full-square version
--   -- freeing BRAM headroom for a second extractor instance (stereo).

entity corner_fifo is
	generic (
		FIFO_DEPTH    : positive := 256;  -- MUST BE POWER OF 2
		URAM_COLS     : positive := 32;   -- 72-bit UltraRAM columns holding the entry's top bits
		PROG_FULL_GAP : positive := 32    -- o_prog_full asserts within this many entries of full;
		                                  -- must exceed the upstream pipeline tail (~8 pushes max)
	);
	port (
		clk : in std_logic;
		rst : in std_logic;

		-- Write side
		wr_en          : in  std_logic;
		wr_x           : in  coord_t;
		wr_y           : in  coord_t;
		wr_score       : in  pixel_t;
		wr_is_brighter : in  std_logic;
		wr_patch       : in  patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

		-- Read side (show-ahead)
		rd_en          : in  std_logic;
		rd_x           : out coord_t;
		rd_y           : out coord_t;
		rd_score       : out pixel_t;
		rd_is_brighter : out std_logic;
		rd_patch       : out patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
		rd_valid       : out std_logic;   -- '1' when rd_* outputs hold a valid head entry

		-- Status
		o_empty        : out std_logic;
		o_full         : out std_logic;
		-- Backpressure: asserted within PROG_FULL_GAP entries of full. Wire
		-- into the upstream pixel-stream tready gate to make drops impossible.
		o_prog_full    : out std_logic;
		o_count        : out unsigned(ceil_log2(FIFO_DEPTH+1)-1 downto 0);
		o_drop_count   : out unsigned(31 downto 0)

	);
end entity;

architecture rtl of corner_fifo is

	-----------------------------------------------------------------
	-- Width arithmetic
	-----------------------------------------------------------------
	-- Each entry: x(11) + y(11) + score(8) + is_brighter(1) + patch(749*8),
	-- circle pixels only (see umax_pkg.in_circle / header comment).
	constant ENTRY_HEADER_WIDTH : natural := 2 * COORD_WIDTH + PIXEL_WIDTH + 1;
	constant PATCH_WIDTH        : natural := CIRCLE_PIXELS * PIXEL_WIDTH;
	constant ENTRY_WIDTH        : natural := ENTRY_HEADER_WIDTH + PATCH_WIDTH;

	-- URAM/BRAM width split. A URAM primitive is a fixed 4096x72; each 72-bit
	-- slice of entry width costs one URAM regardless of FIFO_DEPTH (<= 4096).
	constant URAM_WIDTH : natural := URAM_COLS * 72;
	constant BRAM_WIDTH : natural := ENTRY_WIDTH - URAM_WIDTH;

	constant ADDR_WIDTH         : natural := ceil_log2(FIFO_DEPTH);
	constant COUNT_WIDTH        : natural := ceil_log2(FIFO_DEPTH+1);

	constant DROP_MAX : unsigned(31 downto 0) := (others => '1');


	-----------------------------------------------------------------
	-- Storage: one logical entry array, physically split width-wise into an
	-- UltraRAM part (top URAM_WIDTH bits) and a block-RAM part (rest). The
	-- explicit ram_style attributes pin the mapping; without them Vivado may
	-- map either memory to distributed RAM (LUTRAM), which would consume the
	-- entire chip's LUT budget at these widths.
	-----------------------------------------------------------------
	type mem_u_t is array (0 to FIFO_DEPTH-1) of std_logic_vector(URAM_WIDTH-1 downto 0);
	type mem_b_t is array (0 to FIFO_DEPTH-1) of std_logic_vector(BRAM_WIDTH-1 downto 0);
	signal mem_u : mem_u_t;
	signal mem_b : mem_b_t;

	attribute ram_style : string;
	attribute ram_style of mem_u : signal is "ultra";
	attribute ram_style of mem_b : signal is "block";

	-----------------------------------------------------------------
	-- Pointers and counters
	-----------------------------------------------------------------
	signal wr_ptr      : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
	signal rd_ptr      : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
	signal count_r     : unsigned(COUNT_WIDTH-1 downto 0) := (others => '0');
	signal drop_count_r : unsigned(31 downto 0) := (others => '0');

	-- Shared push/pop decisions and next-cycle count, computed ONCE so the
	-- write, read and count processes can never disagree.
	signal do_push    : boolean;
	signal do_pop     : boolean;
	signal next_count : unsigned(COUNT_WIDTH-1 downto 0);

	-- Read address/enable for the memories, shared by both parts so they can
	-- never disagree. rd_ptr_nxt is the post-edge head (pre-fetch address).
	signal rd_ptr_nxt  : unsigned(ADDR_WIDTH-1 downto 0);
	signal rd_ce       : std_logic;

	-- Read data: registered outputs of the two memory parts (kept as separate
	-- signals so each maps onto its primitive's output register; no reset and
	-- no constant loads on these -- the enable-only template both block RAM
	-- and UltraRAM inference support). rd_data_r is their concatenation.
	signal rd_data_u_r : std_logic_vector(URAM_WIDTH-1 downto 0);
	signal rd_data_b_r : std_logic_vector(BRAM_WIDTH-1 downto 0);
	signal rd_data_r   : std_logic_vector(ENTRY_WIDTH-1 downto 0);
	signal rd_valid_r  : std_logic := '0';

	-----------------------------------------------------------------
	-- Packing / unpacking helpers
	-----------------------------------------------------------------
	function pack_entry(
		px         : coord_t;
		py         : coord_t;
		ps         : pixel_t;
		pb         : std_logic;
		pat        : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1)
	) return std_logic_vector is
		variable v : std_logic_vector(ENTRY_WIDTH-1 downto 0);
		variable idx : natural range 0 to PATCH_WIDTH;
	begin
		-- Header layout: [x | y | score | is_brighter] in MSB-to-LSB order
		v(ENTRY_WIDTH-1 downto ENTRY_WIDTH-COORD_WIDTH) := std_logic_vector(px);
		v(ENTRY_WIDTH-COORD_WIDTH-1 downto ENTRY_WIDTH-2*COORD_WIDTH) := std_logic_vector(py);
		v(ENTRY_WIDTH-2*COORD_WIDTH-1 downto ENTRY_WIDTH-2*COORD_WIDTH-PIXEL_WIDTH) := std_logic_vector(ps);
		v(ENTRY_WIDTH-2*COORD_WIDTH-PIXEL_WIDTH-1) := pb;

		-- Patch: row-major, starting just below the header. Only in-circle
		-- pixels are stored; the loop bounds are static, so the skip costs
		-- nothing in hardware (pure wiring).
		idx := PATCH_WIDTH;
		for row in 0 to PATCH_SIZE-1 loop
			for col in 0 to PATCH_SIZE-1 loop
				if in_circle(row, col) then
					v(idx-1 downto idx-PIXEL_WIDTH) := std_logic_vector(pat(row, col));
					idx := idx - PIXEL_WIDTH;
				end if;
			end loop;
		end loop;

		return v;
	end function;

	procedure unpack_entry(
		v   : in  std_logic_vector;
		px  : out coord_t;
		py  : out coord_t;
		ps  : out pixel_t;
		pb  : out std_logic;
		pat : out patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1)
	) is
		variable idx : natural range 0 to PATCH_WIDTH;
	begin
		px := unsigned(v(ENTRY_WIDTH-1 downto ENTRY_WIDTH-COORD_WIDTH));
		py := unsigned(v(ENTRY_WIDTH-COORD_WIDTH-1 downto ENTRY_WIDTH-2*COORD_WIDTH));
		ps := unsigned(v(ENTRY_WIDTH-2*COORD_WIDTH-1 downto ENTRY_WIDTH-2*COORD_WIDTH-PIXEL_WIDTH));
		pb := v(ENTRY_WIDTH-2*COORD_WIDTH-PIXEL_WIDTH-1);

		-- Out-of-circle pixels are not stored: returned as zeros. Safe --
		-- moment_processor's stage-1 U_MAX mask never reads them anyway.
		idx := PATCH_WIDTH;
		for row in 0 to PATCH_SIZE-1 loop
			for col in 0 to PATCH_SIZE-1 loop
				if in_circle(row, col) then
					pat(row, col) := unsigned(v(idx-1 downto idx-PIXEL_WIDTH));
					idx := idx - PIXEL_WIDTH;
				else
					pat(row, col) := (others => '0');
				end if;
			end loop;
		end loop;
	end procedure;

	-- Combinational decoding of read data (for show-ahead output)
	signal rd_x_c          : coord_t;
	signal rd_y_c          : coord_t;
	signal rd_score_c      : pixel_t;
	signal rd_is_brighter_c : std_logic;
	signal rd_patch_c      : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

begin

	-- wire valid signal to output
	rd_valid <= rd_valid_r;

	-----------------------------------------------------------------
	-- Compile-time check: FIFO_DEPTH must be a power of 2 (makes pointer
	-- wrap free). Equal ceilings mean non-power-of-2, e.g.
	-- ceil(log2(7)) = 3 = ceil(log2(8)) -> assert fires, as wanted.
	-- Static condition: evaluated at elaboration by simulators and Vivado.
	-----------------------------------------------------------------
	assert ceil_log2(FIFO_DEPTH) /= ceil_log2(FIFO_DEPTH + 1)
		report "FIFO_DEPTH must be a positive power of 2, got " & to_string(FIFO_DEPTH)
		severity failure;

	-- The URAM part must leave at least one bit for the BRAM part (BRAM_WIDTH
	-- is a natural: a too-large URAM_COLS would already fail elaboration, but
	-- with a far less readable message).
	assert URAM_WIDTH < ENTRY_WIDTH
		report "URAM_COLS (" & to_string(URAM_COLS) & ") covers the whole entry width ("
			 & to_string(ENTRY_WIDTH) & " bits); reduce it so a BRAM part remains."
		severity failure;

	-- A gap >= FIFO_DEPTH would assert o_prog_full permanently: with the
	-- upstream tready gate wired, the pixel stream would wedge instantly.
	assert PROG_FULL_GAP < FIFO_DEPTH
		report "PROG_FULL_GAP (" & to_string(PROG_FULL_GAP) & ") must be smaller than "
			 & "FIFO_DEPTH (" & to_string(FIFO_DEPTH) & ")."
		severity failure;

	-----------------------------------------------------------------
	-- Status outputs
	-----------------------------------------------------------------
	o_empty      <= '1' when count_r = 0 else '0';
	o_full       <= '1' when count_r = FIFO_DEPTH else '0';
	o_prog_full  <= '1' when count_r >= FIFO_DEPTH - PROG_FULL_GAP else '0';
	o_count      <= count_r;
	o_drop_count <= drop_count_r;

	-----------------------------------------------------------------
	-- Push/pop arbitration and next-cycle count (single source of truth
	-- for the write, read and count processes)
	-----------------------------------------------------------------
	do_push <= (wr_en = '1') and (count_r /= FIFO_DEPTH);
	do_pop  <= (rd_en = '1') and (count_r /= 0);

	next_count <= count_r + 1 when do_push and not do_pop else
	              count_r - 1 when do_pop and not do_push else
	              count_r;

	-----------------------------------------------------------------
	-- Write side: push if wr_en and not full; otherwise drop-and-count
	-----------------------------------------------------------------
	p_write : process(clk)
		variable entry_v : std_logic_vector(ENTRY_WIDTH-1 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				wr_ptr       <= (others => '0');
				drop_count_r <= (others => '0');
			else
				if do_push then
					-- Push: write entry (both width parts, same address) and
					-- advance pointer
					entry_v := pack_entry(wr_x, wr_y, wr_score,
										  wr_is_brighter, wr_patch);
					mem_u(to_integer(wr_ptr)) <= entry_v(ENTRY_WIDTH-1 downto BRAM_WIDTH);
					mem_b(to_integer(wr_ptr)) <= entry_v(BRAM_WIDTH-1 downto 0);
					wr_ptr <= wr_ptr + 1;   -- power-of-2 wrap is automatic
				elsif wr_en = '1' then
					-- Full: tail-drop. Don't write, increment drop count
					-- with saturation at 2^32-1.
					if drop_count_r /= DROP_MAX then
						drop_count_r <= drop_count_r + 1;
					end if;
				end if;
			end if;
		end if;
	end process;

	-- Read side (show-ahead with 1-cycle RAM read latency):
	--   When rd_valid='1', rd_* outputs show the head entry.
	--   After asserting rd_en, the next head appears at rd_* one cycle later.
	--   Consumers should treat reads as: observe data, assert rd_en, wait one
	--   cycle, observe next data.
	--   Note the empty -> non-empty caveat in the file header: rd_valid rises
	--   one cycle before rd_data_r holds the freshly written entry (the RAM
	--   read at the write edge returns the old word; the next read fixes it).
	--
	--   rd_ptr_nxt is what rd_ptr will be AFTER this cycle's edge; we
	--   pre-fetch from this future address so the new head is visible at
	--   rd_* immediately after the rd_en pulse. rd_ce (read enable) uses
	--   next_count, the shared next-cycle count, so it can never disagree
	--   with p_count. When rd_ce='0' the data registers hold their last
	--   value; rd_valid_r ('0' then) is the only qualifier.
	-----------------------------------------------------------------
	rd_ptr_nxt <= rd_ptr + 1 when do_pop else rd_ptr;
	rd_ce      <= '1' when next_count /= 0 else '0';

	p_read : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				rd_ptr     <= (others => '0');
				rd_valid_r <= '0';
			else
				rd_ptr     <= rd_ptr_nxt;
				rd_valid_r <= rd_ce;
			end if;
		end if;
	end process;

	-- Memory read: reset-free, enable-only -- the template whose output
	-- registers map directly onto the BRAM/URAM primitives (a reset or a
	-- constant-load branch here can defeat UltraRAM inference and pull
	-- ENTRY_WIDTH fabric registers instead).
	p_mem_read : process(clk)
	begin
		if rising_edge(clk) then
			if rd_ce = '1' then
				rd_data_u_r <= mem_u(to_integer(rd_ptr_nxt));
				rd_data_b_r <= mem_b(to_integer(rd_ptr_nxt));
			end if;
		end if;
	end process;

	rd_data_r <= rd_data_u_r & rd_data_b_r;


	-----------------------------------------------------------------
	-- Count maintenance: registers the shared next_count
	-----------------------------------------------------------------
	p_count : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				count_r <= (others => '0');
			else
				count_r <= next_count;
			end if;
		end if;
	end process;

	-----------------------------------------------------------------
	-- Unpack registered read data for the output ports
	-----------------------------------------------------------------
	process(all)
		variable px : coord_t;
		variable py : coord_t;
		variable ps : pixel_t;
		variable pb : std_logic;
		variable pat : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
	begin
		unpack_entry(rd_data_r, px, py, ps, pb, pat);
		rd_x_c          <= px;
		rd_y_c          <= py;
		rd_score_c      <= ps;
		rd_is_brighter_c <= pb;
		rd_patch_c      <= pat;
	end process;

	rd_x           <= rd_x_c;
	rd_y           <= rd_y_c;
	rd_score       <= rd_score_c;
	rd_is_brighter <= rd_is_brighter_c;
	rd_patch       <= rd_patch_c;

end architecture;