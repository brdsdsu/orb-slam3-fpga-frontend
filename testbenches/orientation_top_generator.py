import argparse
import math
import numpy as np

HALF_PATCH_SIZE = 15
PATCH_SIZE = 31
CORDIC_WIDTH = 24


def build_umax():
    hp2 = HALF_PATCH_SIZE ** 2
    vmax = int(np.floor(HALF_PATCH_SIZE * np.sqrt(2.0) / 2.0 + 1.0))
    vmin = int(np.ceil(HALF_PATCH_SIZE * np.sqrt(2.0) / 2.0))
    umax = [0] * (HALF_PATCH_SIZE + 1)
    for v in range(vmax + 1):
        umax[v] = int(round(np.sqrt(hp2 - v * v)))
    v0 = 0
    for v in range(HALF_PATCH_SIZE, vmin - 1, -1):
        while umax[v0] == umax[v0 + 1]:
            v0 += 1
        umax[v] = v0
        v0 += 1
    return umax


U_MAX = build_umax()


def compute_moments(patch):
    """Bit-exact match to moment_processor / ORB-SLAM3 IC_Angle."""
    cr = HALF_PATCH_SIZE
    m_10 = 0
    m_01 = 0
    for u in range(-HALF_PATCH_SIZE, HALF_PATCH_SIZE + 1):
        m_10 += u * int(patch[cr, cr + u])
    for v in range(1, HALF_PATCH_SIZE + 1):
        d = U_MAX[v]
        v_sum = 0
        for u in range(-d, d + 1):
            vp = int(patch[cr + v, cr + u])
            vm = int(patch[cr - v, cr + u])
            v_sum += (vp - vm)
            m_10 += u * (vp + vm)
        m_01 += v * v_sum
    return m_10, m_01


def ideal_angle(m_01, m_10, data_width=CORDIC_WIDTH):
    """The mathematically ideal CORDIC-convention angle (what CORDIC approximates)."""
    angle_rad = math.atan2(m_01, m_10)
    scale = (1 << (data_width - 2)) / math.pi
    scaled = round(angle_rad * scale)
    max_val = (1 << (data_width - 1)) - 1
    min_val = -(1 << (data_width - 1))
    return max(min_val, min(max_val, scaled))


def per_vector_tolerance(m_10, m_01, data_width=CORDIC_WIDTH):
    """CORDIC angle error is magnitude-dependent: with input magnitude M,
    only ~log2(M) iterations are 'useful' before the shifts produce zero.
    The residual angle is ~2^-log2(M) rad. We convert that to output units
    and add generous margin for accumulated per-stage truncation.

    This is a loose bound on purpose: the goal here is to confirm the right
    moments reached CORDIC (a wrong-moment angle would be off by a totally
    different direction, far beyond this), not to re-validate CORDIC precision
    (already done in the cordic testbench). Metadata is checked exactly."""
    mag = math.hypot(m_10, m_01)
    if mag < 1.0:
        return 4   # (0,0) -> angle 0 exactly via zero-detection
    residual = (1 << (data_width - 2)) / (math.pi * mag)
    return int(4 * residual) + 512


# Integer Harris response (spec shared with harris_response.vhd; see
# harris_response_generator.py for the full derivation). Computed on the same
# pushed patch, checked on o_hresponse in the USE_HARRIS TB config.
HARRIS_BLOCK_HALF = 3
HARRIS_OUT_SHIFT = 26


def harris_response_hw(patch):
    p = patch.astype(np.int64)
    cr = HALF_PATCH_SIZE
    a = b = c = 0
    for r in range(cr - HARRIS_BLOCK_HALF, cr + HARRIS_BLOCK_HALF + 1):
        for cc in range(cr - HARRIS_BLOCK_HALF, cr + HARRIS_BLOCK_HALF + 1):
            ix = (2 * (p[r, cc + 1] - p[r, cc - 1])
                  + (p[r - 1, cc + 1] - p[r - 1, cc - 1])
                  + (p[r + 1, cc + 1] - p[r + 1, cc - 1]))
            iy = (2 * (p[r + 1, cc] - p[r - 1, cc])
                  + (p[r + 1, cc - 1] - p[r - 1, cc - 1])
                  + (p[r + 1, cc + 1] - p[r - 1, cc + 1]))
            a += int(ix) * int(ix)
            b += int(iy) * int(iy)
            c += int(ix) * int(iy)
    resp = (25 * (a * b - c * c) - (a + b) * (a + b)) >> HARRIS_OUT_SHIFT
    assert -2**31 <= resp <= 2**31 - 1
    return resp


# Distinct metadata per corner so any (angle <-> metadata) swap is caught
def meta_x(i):       return i % 2048
def meta_y(i):       return (i * 7) % 2048
def meta_score(i):   return (i * 13) % 256
def meta_brighter(i): return i % 2


def to_hex(val, width_bits):
    mask = (1 << width_bits) - 1
    chars = (width_bits + 3) // 4
    return f"{val & mask:0{chars}x}"


def main():
    p = argparse.ArgumentParser(description="Vectors for orientation_top_tb")
    p.add_argument("-n", "--num", type=int, default=64,
                   help="Number of corners (keep < FIFO_DEPTH to avoid drops)")
    p.add_argument("-s", "--seed", type=int, default=0)
    p.add_argument("--push-file", type=str, default="orientation_push.hex")
    p.add_argument("--check-file", type=str, default="orientation_check.hex")
    p.add_argument("--params-file", type=str,
                   default="orientation_test_params_pkg.vhd")
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)

    push_lines = []
    check_lines = []

    for i in range(args.num):
        patch = rng.integers(0, 256, size=(PATCH_SIZE, PATCH_SIZE), dtype=np.uint8)
        m10, m01 = compute_moments(patch)
        angle = ideal_angle(m01, m10)
        tol = per_vector_tolerance(m10, m01)
        x, y, sc, br = meta_x(i), meta_y(i), meta_score(i), meta_brighter(i)

        # Push line: metadata (decimal) + patch bytes (hex, row-major)
        parts = [str(x), str(y), str(sc), str(br)]
        for row in range(PATCH_SIZE):
            for col in range(PATCH_SIZE):
                parts.append(f"{int(patch[row, col]):02x}")
        push_lines.append(" ".join(parts))

        # Check line: metadata (decimal) + angle (hex24) + tol (decimal)
        #             + m10/m01 (hex24, for debug printing)
        #             + Harris response (hex32; expected on o_hresponse only in
        #               the USE_HARRIS config -- the FAST config expects 0)
        check_lines.append(
            f"{x} {y} {sc} {br} "
            f"{to_hex(angle, 24)} {tol} "
            f"{to_hex(m10, 24)} {to_hex(m01, 24)} "
            f"{to_hex(harris_response_hw(patch), 32)}"
        )

    with open(args.push_file, "w") as f:
        f.write("\n".join(push_lines) + "\n")
    with open(args.check_file, "w") as f:
        f.write("\n".join(check_lines) + "\n")

    with open(args.params_file, "w") as f:
        f.write(f"""-- AUTO-GENERATED by orientation_top_generator.py. DO NOT EDIT.

library ieee;
use ieee.std_logic_1164.all;

package orientation_test_params_pkg is
    constant TEST_N_VECTORS : natural := {args.num};
end package;
""")

    print(f"Generated {args.num} corners")
    print(f"  Push file : {args.push_file}")
    print(f"  Check file: {args.check_file}")
    print(f"  Params    : {args.params_file}")


if __name__ == "__main__":
    main()