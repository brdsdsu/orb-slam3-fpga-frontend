import argparse
import numpy as np

# Bresenham circle offsets are baked into the hardware via the window mapping,
# so the reference only needs to operate on (center, circle[0..15]) tuples.
# This matches exactly what fast_response.vhd sees at its inputs.
#
# The output file generated has the following structure:
# 80        80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80  00
#  |        |_______________ 16 circle pixels ____________|   |
# center                        c[0]..c[15]                   expected score

def corner_score_simd(center: int, circle: np.ndarray) -> int:
    """Bit-exact reference matching OpenCV's CV_SIMD128 path in cornerScore<16>.
    
    Returns the score that fast_response.vhd should produce, including the
    saturate-to-uint8 step. Operates on raw windows without pre-filtering by
    detection — matches the SIMD path's "compute for every pixel" semantics.
    """
    assert circle.shape == (16,)
    d = np.int32(center) - circle.astype(np.int32)  # 16 signed differences

    # For each of 16 starting positions, the 9-arc min and 9-arc max
    arc_min = np.empty(16, dtype=np.int32)
    arc_max = np.empty(16, dtype=np.int32)
    for s in range(16):
        arc = np.array([d[(s + i) % 16] for i in range(9)], dtype=np.int32)
        arc_min[s] = arc.min()
        arc_max[s] = arc.max()

    brighter = arc_min.max()   # strongest "brighter" arc
    darker   = arc_max.min()   # strongest "darker"  arc (negative-valued)

    best = max(brighter, -darker)
    score = best - 1

    # Saturate to uint8 (matches the (uchar) cast in OpenCV + your VHDL S4)
    return max(0, min(255, score))


def gen_vector_file(path: str, n_random: int, seed: int):
    rng = np.random.default_rng(seed)

    with open(path, 'w') as f:
        # Targeted edge cases first
        edge_cases = [
            (128, np.full(16, 128, dtype=np.uint8)),         # flat: score = 0
            (255, np.zeros(16, dtype=np.uint8)),             # max brighter
            (0,   np.full(16, 255, dtype=np.uint8)),         # max darker
            (128, np.array([255]*9 + [128]*7, dtype=np.uint8)),  # exact 9-arc
            (128, np.array([255]*8 + [128]*8, dtype=np.uint8)),  # 8-arc (no corner)
        ]
        for center, circle in edge_cases:
            score = corner_score_simd(int(center), circle)
            write_line(f, center, circle, score)

        # Random vectors for coverage
        for _ in range(n_random):
            center = rng.integers(0, 256)
            circle = rng.integers(0, 256, size=16, dtype=np.uint8)
            score = corner_score_simd(int(center), circle)
            write_line(f, center, circle, score)


def write_line(f, center, circle, score):
    # Hex format, space-separated: center c0 c1 ... c15 score
    parts = [f"{int(center):02x}"]
    parts += [f"{int(p):02x}" for p in circle]
    parts += [f"{int(score):02x}"]
    f.write(" ".join(parts) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Generate test vectors for fast_response.vhd"
    )
    parser.add_argument(
        "-n", "--num-vectors",
        type=int,
        default=100_000,
        help="Number of random test vectors (default: 100000)",
    )
    parser.add_argument(
        "-s", "--seed",
        type=int,
        default=0,
        help="RNG seed for reproducibility (default: 0)",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="vectors.hex",
        help="Output file path (default: vectors.hex)",
    )
    args = parser.parse_args()

    gen_vector_file(args.output, n_random=args.num_vectors, seed=args.seed)
    print(f"Wrote {args.output} with {args.num_vectors} random vectors "
          f"+ 5 edge cases (seed={args.seed})")


if __name__ == "__main__":
    main()