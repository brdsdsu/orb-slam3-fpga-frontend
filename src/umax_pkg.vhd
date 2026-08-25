library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use work.feature_pkg.all;

-- u_max[v] lookup table for ORB-SLAM3's intensity centroid orientation.
--
-- Defines the half-width of each row in the circular orientation patch.
-- For row offset v (1..HALF_PATCH_SIZE), pixels at column offsets
-- u in [-u_max[v], +u_max[v]] are included in the moment computation.
--
-- The table is built at elaboration time using the same construction
-- OpenCV/ORB-SLAM3 uses (cvRound(sqrt(r^2 - v^2)) followed by symmetry
-- enforcement), so any change to HALF_PATCH_SIZE produces a correctly
-- updated table with zero synthesized hardware overhead.

package umax_pkg is

	constant HALF_PATCH_SIZE : natural := 15;
	constant PATCH_SIZE      : natural := 2 * HALF_PATCH_SIZE + 1;   -- 31

	-- Type: u_max(0..HALF_PATCH_SIZE), each value in [0, HALF_PATCH_SIZE]
	type umax_arr_t is array (0 to HALF_PATCH_SIZE) of natural;

	-- Built at elaboration time, matches OpenCV construction exactly.
	function build_umax return umax_arr_t;
	-- Deferred constant: declared here, given its value in the package body
	-- (= build_umax). Deferring keeps the function definition out of the header.
	constant U_MAX : umax_arr_t;

	-- Width of the intensity-centroid moments (m_10/m_01) and of the CORDIC
	-- angle derived from them. Shared by moment_processor's accumulators and
	-- ports, orientation_top's CORDIC instance, and every o_angle port up the
	-- hierarchy, so the widths cannot drift apart.
	constant MOMENT_WIDTH : natural := 24;

	-- Circle membership for 0-based patch indices: pixel (row, col) lies inside
	-- the orientation circle iff |col-center| <= u_max(|row-center|). This is
	-- exactly the set of pixels moment_processor reads (its stage 1 masks the
	-- rest to zero), so corner_fifo stores ONLY these pixels per entry.
	function in_circle(row : natural; col : natural) return boolean;

	-- Number of pixels inside the circle (749 for HALF_PATCH_SIZE = 15, vs.
	-- 961 for the full square). Sizes corner_fifo's per-entry patch storage.
	constant CIRCLE_PIXELS : natural;

end package;


package body umax_pkg is

	function build_umax return umax_arr_t is
		variable result : umax_arr_t := (others => 0);
		variable vmax   : natural;
		variable vmin   : natural;
		variable hp2    : real;
		variable v0     : natural;
	begin

		-- Match OpenCV's construction in ORBextractor constructor
		hp2  := real(HALF_PATCH_SIZE * HALF_PATCH_SIZE);
		vmax := natural(floor(real(HALF_PATCH_SIZE) * sqrt(2.0) / 2.0 + 1.0));
		vmin := natural(ceil (real(HALF_PATCH_SIZE) * sqrt(2.0) / 2.0));

		-- First pass: circle equation rounded
		for v in 0 to vmax loop
			result(v) := natural(round(sqrt(hp2 - real(v * v))));
		end loop;

		-- Second pass: enforce symmetry (matches OpenCV's loop exactly)
		v0 := 0;
		for v in HALF_PATCH_SIZE downto vmin loop
			while v0 < HALF_PATCH_SIZE and result(v0) = result(v0 + 1) loop
				v0 := v0 + 1;
			end loop;
			result(v) := v0;
			v0 := v0 + 1;
		end loop;

		return result;
	end function;

	constant U_MAX : umax_arr_t := build_umax;

	function in_circle(row : natural; col : natural) return boolean is
	begin
		-- natural - natural is evaluated as integer, so the abs() is safe
		return abs(col - HALF_PATCH_SIZE) <= U_MAX(abs(row - HALF_PATCH_SIZE));
	end function;

	-- Body-local helper (not visible outside): counts the circle pixels once
	-- at elaboration to give CIRCLE_PIXELS its deferred value.
	function count_circle_pixels return natural is
		variable n : natural := 0;
	begin
		for r in 0 to PATCH_SIZE-1 loop
			for c in 0 to PATCH_SIZE-1 loop
				if in_circle(r, c) then
					n := n + 1;
				end if;
			end loop;
		end loop;
		return n;
	end function;

	constant CIRCLE_PIXELS : natural := count_circle_pixels;

end package body;