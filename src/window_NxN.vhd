library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;

-- Sliding NxN window generator for a raster-order pixel stream.
--
-- Architecture: (N-1) line buffers + N×N register array.
--
-- Latency: one cycle from i_pixel being clocked in to that pixel
--          appearing in patch(N-1, N-1). Once enough pixels have streamed
--          to fill the line buffers, every cycle yields a new window
--          shifted right by one pixel; line wraps yield a window shifted
--          down by one line.
--
-- Border policy: o_valid = '0' for any center position where the full
-- N×N window would extend beyond the image boundary, i.e. an N/2-pixel
-- border is skipped. (For N=7 that matches cv::FAST's 3-pixel border; the
-- N=31 instance in extractor_top skips the 15-pixel border the full
-- orientation patch needs.)
--
-- Runtime sizing: the active image width/height arrive on ports
-- (img_width, img_height) so one instance can be reused for every pyramid
-- level. Line-buffer DEPTH is fixed at elaboration via MAX_WIDTH (size for
-- the largest level, i.e. level 0); smaller levels use a prefix of each buffer.

entity window_NxN is
	generic (
		N         : natural := 7;     -- window size (MUST BE ODD)
		MAX_WIDTH : natural := 752    -- line-buffer depth: size for the LARGEST
									  -- pyramid level (level 0). EuRoC = 752.
	);
	port (
		clk        : in  std_logic;
		rst        : in  std_logic;

		-- Active image dimensions for the CURRENT frame/level.
		-- Must be stable from before rst deasserts through the whole frame.
		-- Both MUST be > 0 whenever i_valid is high: img_width = 0 underflows the
		-- line-wrap compare (img_width - 1) and indexes the line buffers out of
		-- range. A full window additionally needs width/height >= N.
		img_width  : in  coord_t;   -- active image width  (MUST be > 0)
		img_height : in  coord_t;   -- active image height (MUST be > 0)

		i_valid    : in  std_logic;
		i_pixel    : in  pixel_t;
		i_sof      : in  std_logic;

		o_valid    : out std_logic;
		o_x        : out coord_t;
		o_y        : out coord_t;
		o_patch    : out patch_arr_t(0 to N-1, 0 to N-1)
	);
end entity;

architecture rtl of window_NxN is

	-- i_sof is informational only; per-frame reset (rst) handles frame
	-- boundaries. Wired through the hierarchy for future per-level counter
	-- reset without a full rst, but not consumed by this module today.
	attribute unused : boolean;
	attribute unused of i_sof : signal is true;

	constant HALF : natural := N / 2;

	-----------------------------------------------------------------
	-- Input pixel position tracking
	-- (in_x, in_y) is the position of the pixel being clocked in
	-- THIS cycle, when i_valid='1'. The counters advance after the edge.
	-----------------------------------------------------------------
	signal in_x : coord_t := (others => '0');
	signal in_y : coord_t := (others => '0');

	-----------------------------------------------------------------
	-- N-1 line buffers, packed by image column. line_mem(x) stores the full
	-- vertical history for column x: lane 0 is one row above the current input
	-- pixel, lane 1 is two rows above, etc. This flat word memory avoids the
	-- 3D-array RAM inference failure that turns the line buffer into FFs.
	-----------------------------------------------------------------
	constant LINE_WORD_WIDTH : natural := (N-1) * PIXEL_WIDTH;
	subtype line_word_t is std_logic_vector(LINE_WORD_WIDTH-1 downto 0);
	type line_mem_t is array (0 to MAX_WIDTH-1) of line_word_t;
	signal line_mem : line_mem_t := (others => (others => '0'));
	signal line_word_rd : line_word_t;

	attribute ram_style : string;
	attribute ram_style of line_mem : signal is "distributed";

	function get_line_pixel(w : line_word_t; idx : natural) return pixel_t is
	begin
		return unsigned(w((idx+1)*PIXEL_WIDTH-1 downto idx*PIXEL_WIDTH));
	end function;

	procedure set_line_pixel(
		variable w : inout line_word_t;
		idx        : in natural;
		pix        : in pixel_t
	) is
	begin
		w((idx+1)*PIXEL_WIDTH-1 downto idx*PIXEL_WIDTH) := std_logic_vector(pix);
	end procedure;

	-----------------------------------------------------------------
	-- N×N shift-register window. After each clock, the new pixel and
	-- the (N-1) line-buffer values populate the rightmost column.
	-- Convention: patch(row, col), row 0 = top (oldest line), col 0 = left.
	-----------------------------------------------------------------
	signal patch : patch_arr_t(0 to N-1, 0 to N-1) := (others => (others => (others => '0')));

	-----------------------------------------------------------------
	-- Window center coordinates and validity.
	-- The window in `patch` is centered on (in_x - HALF, in_y - HALF)
	-- once line buffers are filled. Validity requires that center
	-- to be at least HALF away from every image border.
	-----------------------------------------------------------------
	signal center_valid : std_logic := '0';
	signal center_x     : coord_t := (others => '0');
	signal center_y     : coord_t := (others => '0');

begin

	-- Asynchronous read + synchronous write infers distributed RAM and preserves
	-- the original same-cycle "read old column, then update it" behavior.
	line_word_rd <= line_mem(to_integer(in_x));

	-----------------------------------------------------------------
	-- Compile/elaboration-time parameter check (static condition, so both
	-- simulators and Vivado synthesis evaluate it at elaboration).
	-----------------------------------------------------------------
	assert (N mod 2) = 1
		report "window_NxN: N must be odd, got " & to_string(N)
		severity failure;

	-----------------------------------------------------------------
	-- Main pipeline: counters, line buffers, patch register
	-- All in ONE clocked process to guarantee single-driver semantics
	-- on every signal.
	-----------------------------------------------------------------
	p_main : process(clk)
		variable col_in : patch_arr_t(0 to N-1, 0 to 0);
		variable old_word : line_word_t;
		variable new_word : line_word_t;
	begin
		if rising_edge(clk) then
			if rst = '1' then
				in_x         <= (others => '0');
				in_y         <= (others => '0');
				center_valid <= '0';
				center_x     <= (others => '0');
				center_y     <= (others => '0');

			elsif i_valid = '1' then

				-- Runtime contract check (simulation-only; synthesis ignores
				-- runtime assertions, so this costs no hardware). img_width = 0 would make
				-- (img_width - 1) wrap to all-ones and index the line buffers out of
				-- bounds. TOP's cfg_valid gate already enforces the stronger
				-- width/height >= PATCH_SIZE before streaming; this guards standalone
				-- instantiations of window_NxN.
				assert img_width > 0 and img_height > 0
					report "window_NxN: img_width and img_height must be > 0 while i_valid is high"
					severity failure;

				---------------------------------------------------------
				-- Step 1: Read column from line buffers (pre-update)
				-- col_in(0)   = new input pixel (newest)
				-- col_in(k+1) = pixel from line_mem lane k at column in_x
				--               (which is k+1 rows above current row)
				---------------------------------------------------------
				old_word := line_word_rd;
				col_in(0, 0) := i_pixel;
				for k in 0 to N-2 loop
					col_in(k+1, 0) := get_line_pixel(old_word, k);
				end loop;

				---------------------------------------------------------
				-- Step 2: Update line buffers (cascade)
				-- lane 0 receives the new pixel.
				-- lane k for k>=1 receives what lane k-1 held.
				---------------------------------------------------------
				new_word := (others => '0');
				set_line_pixel(new_word, 0, i_pixel);
				for k in 1 to N-2 loop
					set_line_pixel(new_word, k, get_line_pixel(old_word, k-1));
				end loop;
				line_mem(to_integer(in_x)) <= new_word;

				---------------------------------------------------------
				-- Step 3: Update patch register
				-- Shift everything left by one column, load rightmost
				-- column from col_in. col_in(0) is newest (bottom row of
				-- patch), col_in(N-1) is oldest (top row of patch).
				---------------------------------------------------------
				for row in 0 to N-1 loop
					for col in 0 to N-2 loop
						patch(row, col) <= patch(row, col+1);
					end loop;
				end loop;
				for row in 0 to N-1 loop
					patch(N-1-row, N-1) <= col_in(row, 0);
				end loop;

				---------------------------------------------------------
				-- Step 4: Update window-center coordinates and validity.
				-- After this edge, patch is centered on (in_x-HALF, in_y-HALF)
				-- and spans x in [in_x-2*HALF, in_x], y in [in_y-2*HALF, in_y].
				-- Full-window-inside-image requires:
				--   left   : in_x - 2*HALF >= 0      -> in_x >= 2*HALF
				--   top    : in_y - 2*HALF >= 0      -> in_y >= 2*HALF
				--   right  : in_x <= img_width  - 1  (always true while streaming)
				--   bottom : in_y <= img_height - 1  (always true for an exact frame)
				-- The 2*HALF lower bounds and the (-1) upper bounds are asymmetric
				-- on purpose: the center lags the input pixel by HALF, so the last
				-- HALF rows/cols are never centers. The right/bottom terms are
				-- DEFENSIVE -- they keep o_valid low if the stream over-runs the
				-- declared dimensions (e.g. a miscounted DMA descriptor on hardware).
				-- For an exact frame they are tautological, so behavior is unchanged.
				---------------------------------------------------------
				if  in_x >= 2*HALF and in_x <= img_width  - 1
				and in_y >= 2*HALF and in_y <= img_height - 1 then
					center_valid <= '1';
					center_x     <= in_x - HALF;
					center_y     <= in_y - HALF;
				else
					center_valid <= '0';
				end if;

				---------------------------------------------------------
				-- Step 5: Advance input pixel position counters
				-- i_sof is informational only; rst handles initialization.
				---------------------------------------------------------
				if in_x = img_width - 1 then
					in_x <= (others => '0');
					in_y <= in_y + 1;
				else
					in_x <= in_x + 1;
				end if;

			else
				-- i_valid = '0': hold all state, suppress output
				center_valid <= '0';
			end if;
		end if;
	end process;

	-----------------------------------------------------------------
	-- Outputs
	-----------------------------------------------------------------
	o_valid <= center_valid;
	o_x     <= center_x;
	o_y     <= center_y;
	o_patch <= patch;

end architecture;
