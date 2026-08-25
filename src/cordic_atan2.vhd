-- =============================================================================
-- File	: cordic_atan2.vhd
-- Purpose : Pipelined CORDIC atan2 for FPGA
--		   Equivalent to OpenCV cv::fastAtan2() – full four-quadrant coverage
--
-- Algorithm : CORDIC Vectoring Mode
--   Each stage i rotates the (x,y) vector toward the +x axis.
--   The accumulated rotation angle z converges to atan2(y,x).
--
-- Fixed-point output format
--   angle_o = round( atan2(y,x) * 2^(DATA_WIDTH-2) / PI )
--   i.e. +2^(DATA_WIDTH-2) ≡ +PI,  -2^(DATA_WIDTH-2) ≡ -PI.
--   DATA_WIDTH=16: +16384 ≡ +PI.
--   DATA_WIDTH=24 (as instantiated by orientation_top): +4194304 (2^22) ≡ +PI.
--   Full range: -PI .. +PI  (same sign convention as OpenCV, result in radians)
--
--   To convert to degrees (0..360) as OpenCV fastAtan2 returns:
--     degrees = (angle_o * 360) / 2^(DATA_WIDTH-1)     (integer arithmetic)
--   then add 360 if the result is negative.
--   For the DATA_WIDTH=24 instance: degrees = angle * 360 / 2^23.
--   PS driver note: angle*360 peaks at ~1.51e9 -- it fits int32 with little
--   margin; use a 64-bit intermediate for the multiply to be safe.
--
-- Quadrant pre-rotation
--   CORDIC converges only for |degree| ≤ PI/2 (i.e. x > 0).
--   Inputs in Q2/Q3 are pre-rotated by +-90° and the offset is added back at
--   the output stage.
--
-- Latency	: ITERATIONS + 2 clock cycles (fully pipelined, 1 result/clk)
-- Resources  : ~3 × ITERATIONS DSP-free adders at INT_WIDTH bits
--
-- Generics
--   DATA_WIDTH  Input/output bit width			   (default 16)
--   ITERATIONS  CORDIC iterations = pipeline stages  (default 16)
--
-- The ATAN_LUT is computed at elaboration time from DATA_WIDTH and ITERATIONS
-- using ieee.math_real.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;   -- ARCTAN, MATH_PI

entity cordic_atan2 is
	generic (
		DATA_WIDTH : integer := 16;
		ITERATIONS : integer := 16
	);
	port (
		clk	 : in  std_logic;
		rst	 : in  std_logic;   -- synchronous active-high reset
		valid_i : in  std_logic;   -- pulse high when x_i/y_i are valid
		x_i	 : in  signed(DATA_WIDTH-1 downto 0);
		y_i	 : in  signed(DATA_WIDTH-1 downto 0);
		angle_o : out signed(DATA_WIDTH-1 downto 0);  -- see format above
		valid_o : out std_logic	-- high when angle_o is valid
	);
end entity cordic_atan2;

architecture rtl of cordic_atan2 is

	-- Guard bits: absorb CORDIC gain (~1.647) + pre-rotation negation
	constant INT_WIDTH : integer := DATA_WIDTH + 4;

	-- -------------------------------------------------------------------------
	-- CORDIC atan look-up table — computed at elaboration time
	--
	-- LUT[i] = round( arctan(2^−i) · 2^(DATA_WIDTH−2) / PI )
	--
	-- The scale factor 2^(DATA_WIDTH−2)/PI maps +-PI to +-2^(DATA_WIDTH−2),
	-- i.e. it fills the signed range of (DATA_WIDTH−1) bits.
	--
	-- ieee.math_real.ARCTAN and MATH_PI are evaluated by the tool during
	-- elaboration and folded into plain integer literals in the netlist.
	-- They produce zero hardware.
	-- -------------------------------------------------------------------------
	type atan_lut_t is array(0 to ITERATIONS-1) of integer;

	-- Impure function: reads the generics DATA_WIDTH and ITERATIONS from the
	-- enclosing architecture scope
	impure function build_atan_lut return atan_lut_t is
		variable lut   : atan_lut_t;
		variable scale : real;
	begin
		-- scale: maps a radian angle to the fixed-point integer domain
		scale := real(2**(DATA_WIDTH-2)) / MATH_PI;
		for i in 0 to ITERATIONS-1 loop
			-- ARCTAN(x) from math_real returns arctan(x) in radians
			lut(i) := integer(round(ARCTAN(2.0**(-i)) * scale));
		end loop;
		return lut;
	end function build_atan_lut;

	constant ATAN_LUT : atan_lut_t := build_atan_lut;

	-- PI/2 in the same fixed-point scale = 2^(DATA_WIDTH−3) = 8192 for W=16
	constant HALF_PI_FP : integer := 2**(DATA_WIDTH-3);

	-- -------------------------------------------------------------------------
	-- Pipeline arrays  (index 0 = output of pre-rotation stage)
	-- -------------------------------------------------------------------------
	type pipe_t  is array(0 to ITERATIONS) of signed(INT_WIDTH-1 downto 0);
	type valid_t is array(0 to ITERATIONS) of std_logic;
	type quad_t  is array(0 to ITERATIONS) of std_logic_vector(1 downto 0);

	signal x_pipe	  : pipe_t;
	signal y_pipe	  : pipe_t;
	signal z_pipe	  : pipe_t;   -- accumulated angle
	signal valid_pipe : valid_t;
	-- Quadrant tag propagated through pipeline for output correction:
	--   "00" -> no correction		(Q1 / Q4)
	--   "01" -> add		PI/2	(Q2: x<0, y>=0)
	--   "10" -> subtract	PI/2	(Q3: x<0, y<0)
	signal quad_pipe  : quad_t;
	signal zero_pipe  : valid_t;   -- '1' when this entry's input was (0,0):
								   -- atan2 is undefined there, output forced to 0

begin

	-- =========================================================================
	-- Stage 0 (registered): quadrant detection + pre-rotation
	--
	-- CORDIC converges for |initial_angle| < PI/2, which requires x > 0.
	-- Map Q2/Q3 inputs into Q1/Q4 before entering the CORDIC pipeline.
	--
	--  Q1/Q4 (x >= 0) : (x, y) unchanged,		correction = 0
	--  Q2	(x < 0, y >= 0) : ( y, −x),		correction = +PI/2
	--  Q3	(x < 0, y <  0) : (−y,  x),		correction = −PI/2
	-- =========================================================================
	p_prerotate : process(clk)
		variable xv : signed(INT_WIDTH-1 downto 0);
		variable yv : signed(INT_WIDTH-1 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				x_pipe(0)	  <= (others => '0');
				y_pipe(0)	  <= (others => '0');
				z_pipe(0)	  <= (others => '0');
				valid_pipe(0) <= '0';
				quad_pipe(0)  <= "00";
				zero_pipe(0)  <= '0';
			else
				valid_pipe(0) <= valid_i;
				z_pipe(0)	  <= (others => '0');

				xv := resize(x_i, INT_WIDTH);   -- sign-extend inputs
				yv := resize(y_i, INT_WIDTH);

				if xv = 0 and yv = 0 then
					zero_pipe(0) <= '1';
				else
					zero_pipe(0) <= '0';
				end if;

				if xv(INT_WIDTH-1) = '0' then		  -- x >= 0 : Q1 or Q4
					x_pipe(0)	 <= xv;
					y_pipe(0)	 <= yv;
					quad_pipe(0) <= "00";

				elsif yv(INT_WIDTH-1) = '0' then		-- x < 0, y >= 0 : Q2
					x_pipe(0)	 <=  yv;
					y_pipe(0)	 <= -xv;
					quad_pipe(0) <= "01";

				else									 -- x < 0, y < 0 : Q3
					x_pipe(0)	 <= -yv;
					y_pipe(0)	 <=  xv;
					quad_pipe(0) <= "10";
				end if;
			end if;
		end if;
	end process p_prerotate;

	-- =========================================================================
	-- Stages 1..ITERATIONS (generated, registered): CORDIC vectoring iterations
	--
	-- Iteration i:
	--   if y >= 0 (sign bit = 0):  rotate clockwise
	--	 x <- x + (y >> i)
	--	 y <- y − (x >> i)	  <- uses x BEFORE update (pipelined: prev stage x)
	--	 z <- z + atan(2^−i)
	--
	--   if y < 0 (sign bit = 1):  rotate counter-clockwise
	--	 x <- x − (y >> i)
	--	 y <- y + (x >> i)
	--	 z <- z − atan(2^−i)
	--
	-- shift_right on signed performs arithmetic (sign-extending) shift.
	-- =========================================================================
	gen_cordic : for i in 0 to ITERATIONS-1 generate
		p_iter : process(clk)
		begin
			if rising_edge(clk) then
				if rst = '1' then
					x_pipe(i+1)		<= (others => '0');
					y_pipe(i+1)		<= (others => '0');
					z_pipe(i+1)		<= (others => '0');
					valid_pipe(i+1)	<= '0';
					quad_pipe(i+1)	<= "00";
					zero_pipe(i+1)	<= '0';
				else
					valid_pipe(i+1)	<= valid_pipe(i);
					quad_pipe(i+1)	<= quad_pipe(i);
					zero_pipe(i+1)	<= zero_pipe(i);

					if y_pipe(i)(INT_WIDTH-1) = '0' then	 -- y >= 0 : rotate CW
						x_pipe(i+1)	<= x_pipe(i) + shift_right(y_pipe(i), i);
						y_pipe(i+1)	<= y_pipe(i) - shift_right(x_pipe(i), i);
						z_pipe(i+1)	<= z_pipe(i) + to_signed(ATAN_LUT(i), INT_WIDTH);
					else									  -- y < 0 : rotate CCW
						x_pipe(i+1)	<= x_pipe(i) - shift_right(y_pipe(i), i);
						y_pipe(i+1)	<= y_pipe(i) + shift_right(x_pipe(i), i);
						z_pipe(i+1)	<= z_pipe(i) - to_signed(ATAN_LUT(i), INT_WIDTH);
					end if;
				end if;
			end if;
		end process p_iter;
	end generate gen_cordic;

	-- =========================================================================
	-- Output stage (registered): apply quadrant correction
	-- =========================================================================
	p_output : process(clk)
		variable angle_v : signed(INT_WIDTH-1 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				angle_o <= (others => '0');
				valid_o <= '0';
			else
				valid_o <= valid_pipe(ITERATIONS);

				if zero_pipe(ITERATIONS) = '1' then
					-- (0,0) input -> zero angle, matching cv::fastAtan2(0,0) = 0
					-- and the Python golden model
					angle_o <= (others => '0');
				else
					case quad_pipe(ITERATIONS) is
						-- Q2: add PI/2
						when "01"	=>	angle_v := z_pipe(ITERATIONS) + to_signed(HALF_PI_FP, INT_WIDTH);
						-- Q3: subtract PI/2
						when "10"	=>	angle_v := z_pipe(ITERATIONS) - to_signed(HALF_PI_FP, INT_WIDTH);
						-- Q1/Q4: no change
						when others	=>	angle_v	:= z_pipe(ITERATIONS);
					end case;
					-- Truncate guard bits; angle is bounded to +-PI = +-2^(W-2) << 2^(W-1)
					angle_o <= resize(angle_v, DATA_WIDTH);
				end if;


			end if;
		end if;
	end process p_output;

end architecture rtl;
