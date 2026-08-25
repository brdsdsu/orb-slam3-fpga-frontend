library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;

-- FAST-specific types and helpers.
-- Depends on feature_pkg for universal types (pixel_t, patch_arr_t, min_s, max_s).
package fast_pkg is
	constant DIFF_WIDTH  : natural := PIXEL_WIDTH + 1;   -- signed: [-255, +255]
	constant CIRCLE_SIZE : natural := 16;                -- Bresenham circle
	constant ARC_LENGTH  : natural := 9;                 -- FAST-9 contiguous arc

	subtype diff_t      is signed(DIFF_WIDTH-1 downto 0);
	subtype threshold_t is unsigned(PIXEL_WIDTH-1 downto 0);

	type circle_t   is array (0 to CIRCLE_SIZE-1) of pixel_t;
	type diff_arr_t is array (0 to CIRCLE_SIZE-1) of diff_t;
	type bool16_t   is array (0 to CIRCLE_SIZE-1) of std_logic;

	-- Bundle of pixels consumed by FAST scoring and detection.
	-- Both fast9_core and fast_response take (center, circle) inputs;
	-- this record lets extractor_top pass them together.
	type fast_circle_t is record
		center : pixel_t;
		circle : circle_t;
	end record;

	-- 9-input min/max over a circular arc starting at index `start`
	function arc_min_9(d : diff_arr_t; start : natural) return diff_t;
	function arc_max_9(d : diff_arr_t; start : natural) return diff_t;

	-- 16-input reductions
	function reduce_max(a : diff_arr_t) return diff_t;
	function reduce_min(a : diff_arr_t) return diff_t;

	-- Boolean arc helpers (used by fast9_core)
	function arc_and_9(b : bool16_t; start : natural) return std_logic;
	function reduce_or(b : bool16_t) return std_logic;

	-- Extract FAST center + Bresenham-16 circle from a 2D pixel patch.
	-- Accepts any patch size >= 7x7 (asserts at simulation time if smaller).
	-- The center is taken to be at the patch midpoint (patch'length(1)/2,
	-- patch'length(2)/2), so the patch dimensions should be odd. Indices
	-- match OpenCV's offsets16[] in fast_score.cpp.
	function extract_fast_circle(patch : patch_arr_t) return fast_circle_t;

	-- Extract the FAST center + Bresenham-16 circle around a point offset (dr, dc)
	-- from the patch midpoint. (0,0) == the original extract_fast_circle.
	function extract_fast_circle_at(patch : patch_arr_t; dr : integer; dc : integer)
		return fast_circle_t;


end package;


package body fast_pkg is

	-- Internal scratch types for the balanced reduction trees. Sizing the
	-- levels exactly keeps the trees lint-clean (no partially-used arrays).
	type d9_t is array (0 to 8) of diff_t;
	type d8_t is array (0 to 7) of diff_t;
	type d5_t is array (0 to 4) of diff_t;
	type d4_t is array (0 to 3) of diff_t;
	type d3_t is array (0 to 2) of diff_t;
	type d2_t is array (0 to 1) of diff_t;

	-- 9-input arc min as a balanced tree (depth 4) instead of a linear fold
	-- (depth 8). min_s is associative, commutative and idempotent, so the
	-- result is the exact minimum of the same nine elements. Specialized for
	-- ARC_LENGTH = 9 on a CIRCLE_SIZE = 16 circle (FAST-9).
	function arc_min_9(d : diff_arr_t; start : natural) return diff_t is
		variable w  : d9_t;
		variable r5 : d5_t;
		variable r3 : d3_t;
		variable r2 : d2_t;
	begin
		for i in 0 to ARC_LENGTH-1 loop
			w(i) := d((start + i) mod CIRCLE_SIZE);
		end loop;
		-- 9 -> 5
		r5(0) := min_s(w(0), w(1));  r5(1) := min_s(w(2), w(3));
		r5(2) := min_s(w(4), w(5));  r5(3) := min_s(w(6), w(7));  r5(4) := w(8);
		-- 5 -> 3
		r3(0) := min_s(r5(0), r5(1));  r3(1) := min_s(r5(2), r5(3));  r3(2) := r5(4);
		-- 3 -> 2
		r2(0) := min_s(r3(0), r3(1));  r2(1) := r3(2);
		-- 2 -> 1
		return min_s(r2(0), r2(1));
	end function;

	function arc_max_9(d : diff_arr_t; start : natural) return diff_t is
		variable w  : d9_t;
		variable r5 : d5_t;
		variable r3 : d3_t;
		variable r2 : d2_t;
	begin
		for i in 0 to ARC_LENGTH-1 loop
			w(i) := d((start + i) mod CIRCLE_SIZE);
		end loop;
		-- 9 -> 5
		r5(0) := max_s(w(0), w(1));  r5(1) := max_s(w(2), w(3));
		r5(2) := max_s(w(4), w(5));  r5(3) := max_s(w(6), w(7));  r5(4) := w(8);
		-- 5 -> 3
		r3(0) := max_s(r5(0), r5(1));  r3(1) := max_s(r5(2), r5(3));  r3(2) := r5(4);
		-- 3 -> 2
		r2(0) := max_s(r3(0), r3(1));  r2(1) := r3(2);
		-- 2 -> 1
		return max_s(r2(0), r2(1));
	end function;

	-- 16-input reductions as balanced trees (depth 4) instead of linear folds
	-- (depth 15). Same associativity/commutativity/idempotence argument, so
	-- the result is bit-identical to the previous implementation.
	function reduce_max(a : diff_arr_t) return diff_t is
		variable l1 : d8_t;
		variable l2 : d4_t;
		variable l3 : d2_t;
	begin
		for i in 0 to 7 loop  l1(i) := max_s(a(2*i),  a(2*i+1));  end loop;  -- 16 -> 8
		for i in 0 to 3 loop  l2(i) := max_s(l1(2*i), l1(2*i+1)); end loop;  --  8 -> 4
		for i in 0 to 1 loop  l3(i) := max_s(l2(2*i), l2(2*i+1)); end loop;  --  4 -> 2
		return max_s(l3(0), l3(1));                                          --  2 -> 1
	end function;

	function reduce_min(a : diff_arr_t) return diff_t is
		variable l1 : d8_t;
		variable l2 : d4_t;
		variable l3 : d2_t;
	begin
		for i in 0 to 7 loop  l1(i) := min_s(a(2*i),  a(2*i+1));  end loop;  -- 16 -> 8
		for i in 0 to 3 loop  l2(i) := min_s(l1(2*i), l1(2*i+1)); end loop;  --  8 -> 4
		for i in 0 to 1 loop  l3(i) := min_s(l2(2*i), l2(2*i+1)); end loop;  --  4 -> 2
		return min_s(l3(0), l3(1));                                          --  2 -> 1
	end function;

	function arc_and_9(b : bool16_t; start : natural) return std_logic is
		variable r : std_logic;
	begin
		r := b(start mod CIRCLE_SIZE);
		for i in 1 to ARC_LENGTH-1 loop
			r := r and b((start + i) mod CIRCLE_SIZE);
		end loop;
		return r;
	end function;

	function reduce_or(b : bool16_t) return std_logic is
		variable r : std_logic := '0';
	begin
		for i in 0 to b'length-1 loop
			r := r or b(i);
		end loop;
		return r;
	end function;

	-- extract_fast_circle is the (dr,dc) = (0,0) special case of
	-- extract_fast_circle_at, so it just forwards.
	function extract_fast_circle(patch : patch_arr_t) return fast_circle_t is
	begin
		return extract_fast_circle_at(patch, 0, 0);
	end function;

	function extract_fast_circle_at(patch : patch_arr_t; dr : integer; dc : integer)
		return fast_circle_t is
		constant ROWS : natural := patch'length(1);
		constant COLS : natural := patch'length(2);
		constant CR   : integer := ROWS / 2 + dr;   -- offset center row
		constant CC   : integer := COLS / 2 + dc;   -- offset center col
		variable r    : fast_circle_t;
	begin
		assert ROWS >= 7 and COLS >= 7
			report "extract_fast_circle_at requires patch >= 7x7, got " &
				   to_string(ROWS) & "x" & to_string(COLS)
			severity failure;
		-- The radius-3 circle around the offset center must stay inside the patch.
		assert CR-3 >= 0 and CR+3 <= ROWS-1 and CC-3 >= 0 and CC+3 <= COLS-1
			report "extract_fast_circle_at: offset center (" & to_string(CR) & "," &
				   to_string(CC) & ") circle leaves the patch"
			severity failure;

		r.center := patch(CR, CC);
		-- OpenCV offsets16[] (row, col), clockwise from 3 o'clock.
		r.circle(0)  := patch(CR + 0, CC + 3);
		r.circle(1)  := patch(CR + 1, CC + 3);
		r.circle(2)  := patch(CR + 2, CC + 2);
		r.circle(3)  := patch(CR + 3, CC + 1);
		r.circle(4)  := patch(CR + 3, CC + 0);
		r.circle(5)  := patch(CR + 3, CC - 1);
		r.circle(6)  := patch(CR + 2, CC - 2);
		r.circle(7)  := patch(CR + 1, CC - 3);
		r.circle(8)  := patch(CR + 0, CC - 3);
		r.circle(9)  := patch(CR - 1, CC - 3);
		r.circle(10) := patch(CR - 2, CC - 2);
		r.circle(11) := patch(CR - 3, CC - 1);
		r.circle(12) := patch(CR - 3, CC + 0);
		r.circle(13) := patch(CR - 3, CC + 1);
		r.circle(14) := patch(CR - 2, CC + 2);
		r.circle(15) := patch(CR - 1, CC + 3);
		return r;
	end function;

end package body;