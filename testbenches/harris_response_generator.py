"""Test-vector generator for harris_response.vhd.

Reference model: OpenCV's HarrisResponses() in modules/features2d/src/orb.cpp,
called by ORB with blockSize = 7 and k = HARRIS_K = 0.04. The hardware emits
the ORDER-ISOMORPHIC integer transform (k = 0.04 = 1/25 exactly, the positive
constant scale^4 factor dropped, x25 applied):

    response_full = 25*(a*b - c*c) - (a+b)^2      # = 25*(det - k*tr^2)
    out           = response_full >> 26           # arithmetic shift to int32

The integer golden here reproduces that spec exactly (Python's >> on negative
ints floors toward -inf, matching VHDL numeric_std shift_right on signed).
The generator also computes OpenCV's float32 response for the same patches and
reports the rank agreement between the two (thesis number: the only ranking
differences possible are ties inside one 2**26 truncation bucket vs float
rounding inside OpenCV itself).

Outputs:
  - harris_vectors.hex           : per line 961 patch bytes (31x31 row-major,
                                   2 hex chars each) then the expected response
                                   as 8 hex chars (32-bit two's complement)
  - harris_test_params_pkg.vhd   : TEST_N_VECTORS for the TB
"""

import argparse
import numpy as np


PATCH_SIZE = 31
CR = 15                 # patch centre
BLOCK_HALF = 3          # OpenCV harrisBlockSize = 7
HARRIS_K = np.float32(0.04)
OUT_SHIFT = 26


def gradients(patch: np.ndarray, r: int, c: int):
    """OpenCV HarrisResponses 3x3 Sobel gradients at patch[r, c] (int math)."""
    p = patch.astype(np.int64)
    ix = (2 * (p[r, c + 1] - p[r, c - 1])
          + (p[r - 1, c + 1] - p[r - 1, c - 1])
          + (p[r + 1, c + 1] - p[r + 1, c - 1]))
    iy = (2 * (p[r + 1, c] - p[r - 1, c])
          + (p[r + 1, c - 1] - p[r - 1, c - 1])
          + (p[r + 1, c + 1] - p[r - 1, c + 1]))
    return int(ix), int(iy)


def harris_abc(patch: np.ndarray):
    """a = sum(Ix^2), b = sum(Iy^2), c = sum(Ix*Iy) over the 7x7 centre block."""
    assert patch.shape == (PATCH_SIZE, PATCH_SIZE)
    a = b = c = 0
    for r in range(CR - BLOCK_HALF, CR + BLOCK_HALF + 1):
        for cc in range(CR - BLOCK_HALF, CR + BLOCK_HALF + 1):
            ix, iy = gradients(patch, r, cc)
            a += ix * ix
            b += iy * iy
            c += ix * iy
    return a, b, c


def harris_hw(patch: np.ndarray) -> int:
    """Bit-exact integer reference for harris_response.vhd's o_response."""
    a, b, c = harris_abc(patch)
    response_full = 25 * (a * b - c * c) - (a + b) * (a + b)
    out = response_full >> OUT_SHIFT          # floor shift == VHDL shift_right
    assert -2**31 <= out <= 2**31 - 1, \
        f"int32 overflow: response_full = {response_full}"
    return out


def harris_opencv_float(patch: np.ndarray) -> float:
    """OpenCV's float32 response (for rank-agreement reporting only)."""
    a, b, c = harris_abc(patch)
    scale = np.float32(1.0) / np.float32((1 << 2) * (2 * BLOCK_HALF + 1) * 255.0)
    scale_sq_sq = scale * scale * scale * scale
    af, bf, cf = np.float32(a), np.float32(b), np.float32(c)
    resp = (af * bf - cf * cf - HARRIS_K * (af + bf) * (af + bf)) * scale_sq_sq
    return float(resp)


def gen_edge_cases(rng):
    """Directed patches with predictable qualitative responses."""
    cases = []

    # 1. All zeros: a=b=c=0 -> response exactly 0
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    cases.append(("all_zeros", p))

    # 2. Uniform 128: all gradients 0 -> response exactly 0
    p = np.full((PATCH_SIZE, PATCH_SIZE), 128, dtype=np.uint8)
    cases.append(("uniform_128", p))

    # 3. Vertical step edge through the centre: pure edge -> negative response
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    p[:, CR:] = 255
    cases.append(("edge_vertical", p))

    # 4. Horizontal step edge: negative response
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    p[CR:, :] = 255
    cases.append(("edge_horizontal", p))

    # 5. Diagonal edge: negative response
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    for r in range(PATCH_SIZE):
        p[r, : max(0, r)] = 255
    cases.append(("edge_diagonal", p))

    # 6. Quadrant corner (checker quadrants): strong corner -> positive response
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    p[:CR + 1, CR + 1:] = 255
    p[CR + 1:, :CR + 1] = 255
    cases.append(("corner_quadrants", p))

    # 7. Halo isolation: garbage OUTSIDE the 9x9 gradient reach (centre +/-4),
    #    constant inside -> response exactly 0. Proves the module reads only
    #    the block + halo (and therefore only in-circle FIFO pixels).
    p = rng.integers(0, 256, size=(PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    p[CR - BLOCK_HALF - 1: CR + BLOCK_HALF + 2,
      CR - BLOCK_HALF - 1: CR + BLOCK_HALF + 2] = 77
    cases.append(("halo_isolation", p))

    # 8. Data exactly ON the halo ring (centre +/-4), zeros elsewhere:
    #    exercises the outermost pixels the gradients may read.
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    ring = np.zeros_like(p, dtype=bool)
    ring[CR - 4: CR + 5, CR - 4: CR + 5] = True
    ring[CR - 3: CR + 4, CR - 3: CR + 4] = False
    p[ring] = 200
    cases.append(("halo_ring_only", p))

    # 9./10. Alternating 0/255 column / row pairs: large gradients of both
    #    signs across the block (width stress in both a and b).
    p = np.zeros((PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
    p[:, (np.arange(PATCH_SIZE) // 2) % 2 == 1] = 255
    cases.append(("stripe_pairs_vertical", p))
    cases.append(("stripe_pairs_horizontal", p.T.copy()))

    return cases


def gen_extreme_binary(rng, n, keep):
    """Random 0/255 patches; keep the `keep` largest |response_full| as width
    stress vectors and report the observed maximum against the 2**57 bound."""
    scored = []
    for _ in range(n):
        p = (rng.integers(0, 2, size=(PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)) * 255
        a, b, c = harris_abc(p)
        rf = 25 * (a * b - c * c) - (a + b) * (a + b)
        scored.append((abs(rf), rf, p))
    scored.sort(key=lambda t: t[0], reverse=True)
    print(f"  extreme-binary search: max |response_full| = {scored[0][0]:.3e} "
          f"(bound 2**57 = {2**57:.3e}, "
          f"headroom x{2**57 / max(scored[0][0], 1):.1f})")
    return [p for _, _, p in scored[:keep]]


def rank_agreement(hw_vals, float_vals):
    """Pairwise ordering agreement between the integer HW response and the
    float32 OpenCV response (discordant pairs / total comparable pairs)."""
    hw = np.asarray(hw_vals, dtype=np.int64)
    fl = np.asarray(float_vals, dtype=np.float64)
    n = len(hw)
    dh = np.sign(hw[:, None] - hw[None, :])
    df = np.sign(fl[:, None] - fl[None, :])
    iu = np.triu_indices(n, k=1)
    dh, df = dh[iu], df[iu]
    strict = (dh != 0) & (df != 0)
    discordant = int(np.sum(dh[strict] != df[strict]))
    ties_hw_only = int(np.sum((dh == 0) & (df != 0)))
    total = int(strict.sum())
    return discordant, ties_hw_only, total


def write_vector_file(path, patches_with_resp):
    with open(path, "w") as f:
        for patch, resp in patches_with_resp:
            parts = [f"{int(patch[row, col]):02x}"
                     for row in range(PATCH_SIZE)
                     for col in range(PATCH_SIZE)]
            parts.append(f"{resp & 0xFFFFFFFF:08x}")   # 32-bit two's complement
            f.write(" ".join(parts) + "\n")


def write_params_pkg(path, n_vectors):
    content = f"""-- AUTO-GENERATED by harris_response_generator.py. DO NOT EDIT BY HAND.

library ieee;
use ieee.std_logic_1164.all;

package harris_test_params_pkg is
    constant TEST_N_VECTORS : natural := {n_vectors};
end package;
"""
    with open(path, "w") as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(
        description="Generate test vectors for harris_response.vhd"
    )
    parser.add_argument("-n", "--num-random", type=int, default=1000,
                        help="Number of random test vectors (default: 1000)")
    parser.add_argument("-s", "--seed", type=int, default=0,
                        help="RNG seed (default: 0)")
    parser.add_argument("-o", "--output", type=str, default="harris_vectors.hex",
                        help="Output vector file (default: harris_vectors.hex)")
    parser.add_argument("--params-file", type=str,
                        default="harris_test_params_pkg.vhd",
                        help="Auto-generated VHDL params package")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    patches = []
    print("Edge cases:")
    for name, patch in gen_edge_cases(rng):
        resp = harris_hw(patch)
        patches.append((patch, resp))
        print(f"  {name:25s}: response = {resp:12d}   "
              f"(OpenCV float {harris_opencv_float(patch):+.6e})")

    print("Width-stress vectors:")
    for patch in gen_extreme_binary(rng, n=2000, keep=10):
        patches.append((patch, harris_hw(patch)))

    rand_hw, rand_fl = [], []
    for _ in range(args.num_random):
        patch = rng.integers(0, 256, size=(PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
        resp = harris_hw(patch)
        patches.append((patch, resp))
        rand_hw.append(resp)
        rand_fl.append(harris_opencv_float(patch))

    discordant, hw_ties, total = rank_agreement(rand_hw, rand_fl)
    print(f"\nRank agreement vs OpenCV float32 over {args.num_random} random patches:")
    print(f"  strictly ordered pairs : {total}")
    print(f"  discordant pairs       : {discordant}")
    print(f"  HW-tie-only pairs      : {hw_ties} "
          f"(ties inside one 2**{OUT_SHIFT} truncation bucket)")

    write_vector_file(args.output, patches)
    write_params_pkg(args.params_file, len(patches))
    print(f"\nWrote {len(patches)} vectors to {args.output}")
    print(f"Wrote test params to {args.params_file}")


if __name__ == "__main__":
    main()
