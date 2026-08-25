library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.feature_pkg.all;
use work.umax_pkg.all;

-- Harris corner response for ORB keypoints (OpenCV ORB "HARRIS_SCORE" scoring).
--
-- Computes the Harris response over the BLOCK_SIZE x BLOCK_SIZE (7x7) block
-- centred on the patch centre, exactly as OpenCV's HarrisResponses() in
-- modules/features2d/src/orb.cpp (called with blockSize = 7, k = HARRIS_K =
-- 0.04). Gradients are the same 3x3 Sobel forms OpenCV uses:
--
--   Ix = 2*(P[r][c+1]-P[r][c-1]) + (P[r-1][c+1]-P[r-1][c-1]) + (P[r+1][c+1]-P[r+1][c-1])
--   Iy = 2*(P[r+1][c]-P[r-1][c]) + (P[r+1][c-1]-P[r-1][c-1]) + (P[r+1][c+1]-P[r-1][c+1])
--
-- with a = sum(Ix*Ix), b = sum(Iy*Iy), c = sum(Ix*Iy) over the block. OpenCV
-- then computes  response = (a*b - c*c - k*(a+b)^2) * scale^4  in float. Since
-- k = 0.04 = 1/25 exactly and scale^4 > 0 is a constant, this module emits the
-- ORDER-ISOMORPHIC integer transform (x25, drop the positive constant factor):
--
--   response_full = 25*(a*b - c*c) - (a+b)^2        -- = 25*(det - k*tr^2)
--   o_response    = response_full >>> OUT_SHIFT     -- arithmetic shift, to 32b
--
-- Ranking under o_response equals the algebraic Harris ranking exactly (no
-- fixed-point approximation of k; ties only within one 2**OUT_SHIFT bucket,
-- below OpenCV's own float rounding). The response is used purely for ranking
-- (DistributeOctTree max-per-node), so the monotone transform is sufficient.
--
-- Value bounds (drive every width below; BLOCK_PIX = 49 block pixels):
--   |Ix|,|Iy| <= 4*255                = 1020            -> 11b signed
--   |Ix*Iy|   <= 1020^2               = 1,040,400       -> 21b signed (+1 slack)
--   |a|,|b|,|c| <= 49*1020^2          = 50,979,600      -> 27b signed
--   a+b       <= 2*50,979,600         = 101,959,200     -> 28b signed
--   a*b, c*c  <= (49*1020^2)^2        ~= 2.599e15       -> 53b signed (kept 54)
--   |response_full| <= 25*2.599e15 + 1.040e16 ~= 7.537e16 < 2**57 -> 58b (kept 60)
--   OUT_SHIFT = 26 is the minimal shift making the result fit int32:
--   7.537e16 / 2**26 ~= 1.124e9 < 2**31 - 1 (shift 25 would overflow).
--
-- The 7x7 block plus its 1-pixel gradient halo spans centre +/-4, which lies
-- entirely inside the orientation circle (U_MAX(4) = 14 >= 4), so the circle-
-- masked corner_fifo patch materializes every pixel this module reads --
-- checked by the elaboration assert below.
--
-- Handshake and style mirror moment_processor: i_start latches i_patch, one
-- block row enters the pipeline per cycle (7 rows), balanced adder trees,
-- registered DSP stages, then a short finalize pipeline. Latency i_start ->
-- o_done is 19 cycles -- always shorter than moment_processor's 23, so running
-- both in parallel on the same corner leaves orientation_top's stage-M
-- initiation interval unchanged. o_ready re-asserts with o_done, like the
-- moment processor.

entity harris_response is
	port (
		clk        : in  std_logic;
		rst        : in  std_logic;

		i_start    : in  std_logic;
		i_patch    : in  patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

		o_done     : out std_logic;
		o_response : out signed(31 downto 0);

		o_ready    : out std_logic
	);
end entity;

architecture rtl of harris_response is

	constant CR          : natural := HALF_PATCH_SIZE;      -- patch centre (15)
	constant BLOCK_HALF  : natural := 3;                    -- OpenCV harrisBlockSize = 7
	constant BLOCK_SIZE  : natural := 2*BLOCK_HALF + 1;     -- 7

	-- Widths from the value bounds in the header. Slack bits are free in
	-- fabric adders; the OUT_SHIFT no-overflow argument uses the tight VALUE
	-- bounds, not these signal widths.
	constant GRAD_W      : natural := 11;                   -- |grad| <= 1020
	constant PROD_W      : natural := 2*GRAD_W;             -- 22 (21 needed)
	constant ROWSUM_W    : natural := PROD_W + 3;           -- 25 (7 products)
	constant ACC_W       : natural := 27;                   -- |a|,|b|,|c| < 2**26
	constant TR_W        : natural := ACC_W + 1;            -- 28
	constant DET_W       : natural := 2*ACC_W;              -- 54
	constant TR2_W       : natural := 2*TR_W;               -- 56
	constant RESP_W      : natural := DET_W + 6;            -- 60 (|resp| < 2**57)
	constant OUT_SHIFT   : natural := 26;                   -- minimal int32-safe shift
	constant OUT_W       : natural := 32;

	-- Control FSM: 7 compute cycles, drain the 3-stage row pipeline, then a
	-- 6-stage finalize (tr, DSP mults + copy, det, response, output/done).
	type state_t is (S_IDLE, S_SETUP, S_COMPUTE, S_DRAIN,
	                 S_TR, S_MUL, S_MULR, S_DET, S_RESP, S_OUT);
	signal state    : state_t := S_IDLE;
	signal br_count : natural range 0 to BLOCK_SIZE-1 := 0;

	signal patch_r  : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);

	-- Row pipeline. One lane per block column offset (7 lanes).
	type grad_arr_t is array (0 to BLOCK_SIZE-1) of signed(GRAD_W-1 downto 0);
	type prod_arr_t is array (0 to BLOCK_SIZE-1) of signed(PROD_W-1 downto 0);

	-- Stage 1: registered gradients per lane
	signal s1_valid : std_logic := '0';
	signal s1_last  : std_logic := '0';
	signal s1_gx    : grad_arr_t := (others => (others => '0'));
	signal s1_gy    : grad_arr_t := (others => (others => '0'));

	-- Stage 2: registered gradient products per lane (DSP)
	signal s2_valid : std_logic := '0';
	signal s2_last  : std_logic := '0';
	signal s2_pxx   : prod_arr_t := (others => (others => '0'));
	signal s2_pyy   : prod_arr_t := (others => (others => '0'));
	signal s2_pxy   : prod_arr_t := (others => (others => '0'));

	-- Stage 3: registered row sums (balanced 7-input trees)
	signal s3_valid : std_logic := '0';
	signal s3_last  : std_logic := '0';
	signal s3_rxx   : signed(ROWSUM_W-1 downto 0) := (others => '0');
	signal s3_ryy   : signed(ROWSUM_W-1 downto 0) := (others => '0');
	signal s3_rxy   : signed(ROWSUM_W-1 downto 0) := (others => '0');

	-- Stage 4: block accumulators (a, b, c)
	signal a_acc    : signed(ACC_W-1 downto 0) := (others => '0');
	signal b_acc    : signed(ACC_W-1 downto 0) := (others => '0');
	signal c_acc    : signed(ACC_W-1 downto 0) := (others => '0');
	signal acc_last : std_logic := '0';   -- last row landed in the accumulators

	-- Finalize pipeline registers
	signal tr_r     : signed(TR_W-1 downto 0)   := (others => '0');
	signal ab_p     : signed(DET_W-1 downto 0)  := (others => '0');   -- DSP MREG
	signal cc_p     : signed(DET_W-1 downto 0)  := (others => '0');
	signal tr2_p    : signed(TR2_W-1 downto 0)  := (others => '0');
	signal ab_r     : signed(DET_W-1 downto 0)  := (others => '0');   -- DSP PREG
	signal cc_r     : signed(DET_W-1 downto 0)  := (others => '0');
	signal tr2_r    : signed(TR2_W-1 downto 0)  := (others => '0');
	signal det_r    : signed(DET_W-1 downto 0)  := (others => '0');
	signal resp_r   : signed(RESP_W-1 downto 0) := (others => '0');

	-- Output registers
	signal out_resp : signed(OUT_W-1 downto 0) := (others => '0');
	signal done_r   : std_logic := '0';

	-- Unsigned pixel to 9-bit nonnegative signed (same helper as moment_processor)
	function pix_s(p : pixel_t) return signed is
	begin
		return signed('0' & p);
	end function;

begin

	-- The block + gradient halo must stay inside the patch AND inside the
	-- orientation circle (the corner_fifo stores only in-circle pixels; pixels
	-- outside unpack as zeros, which would silently corrupt the gradients).
	-- The circle worst case is the halo corner at offset (+/-4, +/-4).
	assert CR >= BLOCK_HALF + 1 and CR + BLOCK_HALF + 1 <= PATCH_SIZE - 1
		report "harris_response: block + gradient halo leaves the patch (CR="
			 & to_string(CR) & ", BLOCK_HALF=" & to_string(BLOCK_HALF)
			 & ", PATCH_SIZE=" & to_string(PATCH_SIZE) & ")"
		severity failure;
	assert in_circle(CR - BLOCK_HALF - 1, CR - BLOCK_HALF - 1)
		report "harris_response: gradient halo corner ("
			 & to_string(CR - BLOCK_HALF - 1) & "," & to_string(CR - BLOCK_HALF - 1)
			 & ") lies outside the orientation circle -- the circle-masked "
			 & "corner_fifo patch would feed zeros"
		severity failure;

	o_done     <= done_r;
	o_response <= out_resp;
	o_ready    <= '1' when state = S_IDLE else '0';

	-- ================================================================
	-- Control FSM. S_COMPUTE feeds block rows 0..6, one per cycle; S_DRAIN
	-- waits until the last row lands in the accumulators; the finalize
	-- states then walk the response through the wide multiplies.
	-- ================================================================
	p_fsm : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				state    <= S_IDLE;
				br_count <= 0;
			else
				case state is
					when S_IDLE =>
						br_count <= 0;
						if i_start = '1' then
							state <= S_SETUP;
						end if;

					when S_SETUP =>
						br_count <= 0;
						state    <= S_COMPUTE;

					when S_COMPUTE =>
						if br_count = BLOCK_SIZE-1 then
							state <= S_DRAIN;
						else
							br_count <= br_count + 1;
						end if;

					when S_DRAIN =>
						-- acc_last pulses the cycle the last row is accumulated;
						-- a/b/c are final from the next cycle on.
						if acc_last = '1' then
							state <= S_TR;
						end if;

					when S_TR   => state <= S_MUL;
					when S_MUL  => state <= S_MULR;
					when S_MULR => state <= S_DET;
					when S_DET  => state <= S_RESP;
					when S_RESP => state <= S_OUT;
					when S_OUT  => state <= S_IDLE;
				end case;
			end if;
		end if;
	end process;

	-- Latch the patch during S_SETUP; stable for the whole corner (the FSM
	-- returns to S_IDLE before the next i_start can arrive).
	p_patch : process(clk)
	begin
		if rising_edge(clk) then
			if state = S_SETUP then
				patch_r <= i_patch;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 1: Sobel gradients for the 7 lanes of block row br_count.
	-- Datapath free-runs (no reset); only the valid chain carries reset,
	-- same policy as fast_response / moment_processor.
	-- ================================================================
	p_stage1 : process(clk)
		variable r  : natural range 0 to PATCH_SIZE-1;
		variable c  : natural range 0 to PATCH_SIZE-1;
		variable gx : signed(GRAD_W-1 downto 0);
		variable gy : signed(GRAD_W-1 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s1_valid <= '0';
				s1_last  <= '0';
			else
				s1_valid <= '0';
				s1_last  <= '0';

				if state = S_COMPUTE then
					s1_valid <= '1';
					if br_count = BLOCK_SIZE-1 then
						s1_last <= '1';
					end if;

					r := CR - BLOCK_HALF + br_count;
					for l in 0 to BLOCK_SIZE-1 loop
						c := CR - BLOCK_HALF + l;

						-- 2*(E-W) + (NE-NW) + (SE-SW); |gx| <= 4*255 = 1020
						gx := shift_left(resize(pix_s(patch_r(r,   c+1)), GRAD_W)
						               - resize(pix_s(patch_r(r,   c-1)), GRAD_W), 1)
						    + (resize(pix_s(patch_r(r-1, c+1)), GRAD_W)
						     - resize(pix_s(patch_r(r-1, c-1)), GRAD_W))
						    + (resize(pix_s(patch_r(r+1, c+1)), GRAD_W)
						     - resize(pix_s(patch_r(r+1, c-1)), GRAD_W));

						-- 2*(S-N) + (SW-NW) + (SE-NE); |gy| <= 4*255 = 1020
						gy := shift_left(resize(pix_s(patch_r(r+1, c  )), GRAD_W)
						               - resize(pix_s(patch_r(r-1, c  )), GRAD_W), 1)
						    + (resize(pix_s(patch_r(r+1, c-1)), GRAD_W)
						     - resize(pix_s(patch_r(r-1, c-1)), GRAD_W))
						    + (resize(pix_s(patch_r(r+1, c+1)), GRAD_W)
						     - resize(pix_s(patch_r(r-1, c+1)), GRAD_W));

						s1_gx(l) <= gx;
						s1_gy(l) <= gy;
					end loop;
				end if;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 2: per-lane gradient products (21 multiplies, DSP-mapped:
	-- operands come from the stage-1 registers, products land here).
	-- ================================================================
	p_stage2 : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s2_valid <= '0';
				s2_last  <= '0';
			else
				s2_valid <= s1_valid;
				s2_last  <= s1_last;

				for l in 0 to BLOCK_SIZE-1 loop
					s2_pxx(l) <= s1_gx(l) * s1_gx(l);
					s2_pyy(l) <= s1_gy(l) * s1_gy(l);
					s2_pxy(l) <= s1_gx(l) * s1_gy(l);
				end loop;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 3: balanced 7-input row sums. Explicit parentheses force the
	-- tree shape (depth 3) instead of a ripple.
	-- ================================================================
	p_stage3 : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				s3_valid <= '0';
				s3_last  <= '0';
			else
				s3_valid <= s2_valid;
				s3_last  <= s2_last;

				s3_rxx <= ((resize(s2_pxx(0), ROWSUM_W) + resize(s2_pxx(1), ROWSUM_W))
				         + (resize(s2_pxx(2), ROWSUM_W) + resize(s2_pxx(3), ROWSUM_W)))
				        + ((resize(s2_pxx(4), ROWSUM_W) + resize(s2_pxx(5), ROWSUM_W))
				         +  resize(s2_pxx(6), ROWSUM_W));
				s3_ryy <= ((resize(s2_pyy(0), ROWSUM_W) + resize(s2_pyy(1), ROWSUM_W))
				         + (resize(s2_pyy(2), ROWSUM_W) + resize(s2_pyy(3), ROWSUM_W)))
				        + ((resize(s2_pyy(4), ROWSUM_W) + resize(s2_pyy(5), ROWSUM_W))
				         +  resize(s2_pyy(6), ROWSUM_W));
				s3_rxy <= ((resize(s2_pxy(0), ROWSUM_W) + resize(s2_pxy(1), ROWSUM_W))
				         + (resize(s2_pxy(2), ROWSUM_W) + resize(s2_pxy(3), ROWSUM_W)))
				        + ((resize(s2_pxy(4), ROWSUM_W) + resize(s2_pxy(5), ROWSUM_W))
				         +  resize(s2_pxy(6), ROWSUM_W));
			end if;
		end if;
	end process;

	-- ================================================================
	-- Stage 4: block accumulators. Cleared in S_SETUP (pipeline empty);
	-- acc_last pulses when the last row's sums are accumulated.
	-- ================================================================
	p_accumulate : process(clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				a_acc    <= (others => '0');
				b_acc    <= (others => '0');
				c_acc    <= (others => '0');
				acc_last <= '0';
			else
				acc_last <= '0';

				if state = S_SETUP then
					a_acc <= (others => '0');
					b_acc <= (others => '0');
					c_acc <= (others => '0');
				elsif s3_valid = '1' then
					a_acc <= a_acc + resize(s3_rxx, ACC_W);
					b_acc <= b_acc + resize(s3_ryy, ACC_W);
					c_acc <= c_acc + resize(s3_rxy, ACC_W);
					if s3_last = '1' then
						acc_last <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;

	-- ================================================================
	-- Finalize: tr = a+b, then the three wide products (registered twice,
	-- S_MUL/S_MULR = DSP MREG/PREG), then det, response, output. One state
	-- per register keeps every cycle a plain register-to-register hop.
	-- ================================================================
	p_finalize : process(clk)
		variable det_ext : signed(RESP_W-1 downto 0);
	begin
		if rising_edge(clk) then
			if rst = '1' then
				done_r <= '0';
			else
				done_r <= '0';

				case state is
					when S_TR =>
						tr_r <= resize(a_acc, TR_W) + resize(b_acc, TR_W);

					when S_MUL =>
						ab_p  <= a_acc * b_acc;
						cc_p  <= c_acc * c_acc;
						tr2_p <= tr_r * tr_r;

					when S_MULR =>
						ab_r  <= ab_p;
						cc_r  <= cc_p;
						tr2_r <= tr2_p;

					when S_DET =>
						det_r <= ab_r - cc_r;

					when S_RESP =>
						-- 25*det = (det<<4) + (det<<3) + det (three-term add; no DSP)
						det_ext := resize(det_r, RESP_W);
						resp_r  <= (shift_left(det_ext, 4) + shift_left(det_ext, 3) + det_ext)
						         - resize(tr2_r, RESP_W);

					when S_OUT =>
						-- Loss-free by the value bound in the header:
						-- |resp_r| / 2**OUT_SHIFT < 2**31. Simulation cross-check below.
						-- to_hstring, not integer'image: resp_r is 60 bits, so a
						-- to_integer detour would itself overflow VHDL's integer.
						assert resize(resize(shift_right(resp_r, OUT_SHIFT), OUT_W), RESP_W)
						     = shift_right(resp_r, OUT_SHIFT)
							report "harris_response: response overflows int32 -- OUT_SHIFT "
								 & "bound violated (resp_full = 0x" & to_hstring(resp_r)
								 & ", shift " & to_string(OUT_SHIFT) & ")"
							severity failure;
						out_resp <= resize(shift_right(resp_r, OUT_SHIFT), OUT_W);
						done_r   <= '1';

					when others =>
						null;
				end case;
			end if;
		end if;
	end process;

end architecture;
