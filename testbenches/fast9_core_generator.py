import argparse
import numpy as np


# The output file generated has the following structure:
# 80  80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80  14  00 00
# |   |_______________ 16 circle pixels ____________|   |   |  |
# center             c[0]..c[15]                       thr  |  is_brighter
#                                                        is_corner
#
# All numbers in HEX-Format, so 14 base 16 = 20 base 10

def detect_fast9(center: int, circle: np.ndarray, threshold: int) -> tuple[int, int]:
    """Bit-exact reference for fast9_core.vhd.

    Returns (is_corner, is_brighter), each 0 or 1.
    Matches OpenCV's FAST-9 contiguous-arc test semantics:
      - brighter pixel: circle[k] > center + threshold (strict)
      - darker pixel:   circle[k] < center - threshold (strict)
      - is_corner: any 9-contiguous arc is all-brighter OR all-darker
    """
    assert circle.shape == (16,)

    upper = center + threshold
    lower = center - threshold
    brighter = circle.astype(np.int32) > upper
    darker   = circle.astype(np.int32) < lower

    is_brighter = any(
        all(brighter[(s + i) % 16] for i in range(9))
        for s in range(16)
    )
    is_darker = any(
        all(darker[(s + i) % 16] for i in range(9))
        for s in range(16)
    )

    return (int(is_brighter or is_darker), int(is_brighter))


def gen_vector_file(path: str, n_random: int, seed: int):
    rng = np.random.default_rng(seed)

    with open(path, 'w') as f:
        # ----- Targeted edge cases -----
        # Format: (center, circle, threshold, comment)
        edge_cases = [
            # Flat regions: never a corner regardless of threshold
            (128, np.full(16, 128, dtype=np.uint8), 20),
            (0,   np.zeros(16, dtype=np.uint8),     20),
            (255, np.full(16, 255, dtype=np.uint8), 20),

            # Maximum-contrast brighter corner: both corners
            (0, np.full(16, 255, dtype=np.uint8), 20),
            # Maximum-contrast darker corner
            (255, np.zeros(16, dtype=np.uint8), 20),

            # Exact 9-arc brighter: should detect
            (128, np.array([255]*9 + [128]*7, dtype=np.uint8), 20),
            # Exact 8-arc brighter: should NOT detect (one short)
            (128, np.array([255]*8 + [128]*8, dtype=np.uint8), 20),

            # 9-arc that wraps the seam (starts at position 10)
            (128,
             np.concatenate([np.full(3, 255, dtype=np.uint8),  # positions 0-2
                             np.full(7, 128, dtype=np.uint8),  # positions 3-9
                             np.full(6, 255, dtype=np.uint8)]), # positions 10-15
             20),

            # Threshold-boundary case: pixel exactly at center+threshold
            # (strict inequality means this should NOT detect)
            (100, np.array([120]*9 + [100]*7, dtype=np.uint8), 20),
            # Just one above the boundary: should detect
            (100, np.array([121]*9 + [100]*7, dtype=np.uint8), 20),

            # Both polarities present but neither has 9-contiguous:
            # 8 brighter, then 8 darker → no corner
            (128,
             np.concatenate([np.full(8, 255, dtype=np.uint8),
                             np.zeros(8, dtype=np.uint8)]),
             20),

            # Threshold = 0: any strict variation counts
            (128, np.array([129]*9 + [128]*7, dtype=np.uint8), 0),

            # Threshold = 255: nothing can pass (no pixel > 255 + center)
            (0, np.full(16, 255, dtype=np.uint8), 255),
        ]

        for center, circle, threshold in edge_cases:
            is_corner, is_brighter = detect_fast9(int(center), circle, threshold)
            write_line(f, center, circle, threshold, is_corner, is_brighter)

        # ----- Random vectors for general coverage -----
        for _ in range(n_random):
            center    = rng.integers(0, 256)
            circle    = rng.integers(0, 256, size=16, dtype=np.uint8)
            threshold = rng.integers(0, 256)
            is_corner, is_brighter = detect_fast9(int(center), circle, int(threshold))
            write_line(f, center, circle, threshold, is_corner, is_brighter)


def write_line(f, center, circle, threshold, is_corner, is_brighter):
    # Hex format: center c0..c15 threshold is_corner is_brighter
    # 16 circle pixels + 3 single-byte fields = 20 hex bytes per line
    parts = [f"{int(center):02x}"]
    parts += [f"{int(p):02x}" for p in circle]
    parts += [f"{int(threshold):02x}"]
    parts += [f"{int(is_corner):02x}"]
    parts += [f"{int(is_brighter):02x}"]
    f.write(" ".join(parts) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Generate test vectors for fast9_core.vhd"
    )
    parser.add_argument("-n", "--num-vectors", type=int, default=100_000,
                        help="Number of random test vectors (default: 100000)")
    parser.add_argument("-s", "--seed", type=int, default=0,
                        help="RNG seed (default: 0)")
    parser.add_argument("-o", "--output", type=str, default="vectors_fast9.hex",
                        help="Output file path (default: vectors_fast9.hex)")
    args = parser.parse_args()

    gen_vector_file(args.output, n_random=args.num_vectors, seed=args.seed)
    print(f"Wrote {args.output} with {args.num_vectors} random vectors "
          f"+ edge cases (seed={args.seed})")


if __name__ == "__main__":
    main()