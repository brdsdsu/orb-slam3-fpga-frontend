library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Universal types shared across all feature extraction modules
-- (FAST, orientation, line buffers, etc.).
-- Algorithm-specific types live in their own packages (fast_pkg, umax_pkg).
--
-- NOTE: this package is compiled as VHDL-93/2002 when packaging the Vivado IP
-- (it is referenced by the plain-VHDL wrapper orb_feature_top). Do not add
-- VHDL-2008 constructs here (to_string, process(all), ...).
package feature_pkg is
	constant PIXEL_WIDTH : natural := 8;
	subtype pixel_t is unsigned(PIXEL_WIDTH-1 downto 0);

	-- Image coordinate width. 11 bits addresses up to 2048x2048 images,
	-- which covers every pyramid level of the target sensors (EuRoC 752x480
	-- and smaller). Widen COORD_WIDTH here if a larger sensor is ever used.
	constant COORD_WIDTH : natural := 11;
	subtype coord_t is unsigned(COORD_WIDTH-1 downto 0);

	-- Generic 2D pixel patch, declared unconstrained so the same type
	-- definition is reused for any N (each port/signal fixes its own bounds,
	-- e.g. patch_arr_t(0 to N-1, 0 to N-1)).
	type patch_arr_t is array (natural range <>, natural range <>) of pixel_t;

	-- Generic signed min/max helpers, useful for any reduction tree
	function min_s(a, b : signed) return signed;
	function max_s(a, b : signed) return signed;

	-- take the ceiling logarithm (base 2)
	function ceil_log2(n : positive) return natural;


end package;

package body feature_pkg is
	function min_s(a, b : signed) return signed is
	begin
		if a < b then return a; else return b; end if;
	end function;

	function max_s(a, b : signed) return signed is
	begin
		if a > b then return a; else return b; end if;
	end function;

	function ceil_log2(n : positive) return natural is
		variable v      : natural := n - 1;
		variable result : natural := 0;
	begin
		while v > 0 loop
			v := v / 2;
			result := result + 1;
		end loop;
		return result;
	end function;

end package body;