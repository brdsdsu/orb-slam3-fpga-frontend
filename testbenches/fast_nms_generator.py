import argparse
import numpy as np

# OpenCV offsets16[] in (row, col), clockwise from 3 o'clock -- identical to
# extract_fast_circle / fast_pkg.
OFF16 = [(0, 3), (1, 3), (2, 2), (3, 1), (3, 0), (3, -1), (2, -2), (1, -3),
         (0, -3), (-1, -3), (-2, -2), (-3, -1), (-3, 0), (-3, 1), (-2, 2), (-1, 3)]

# Neighbour offsets (dr, dc); index 0 = centre, matching fast_nms.OFFS.
NB = [(0, 0), (-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def corner_score_simd(center: int, circle: np.ndarray) -> int:
    """Bit-exact reference matching OpenCV cornerScore<16> / fast_response.vhd."""
    assert circle.shape == (16,)
    d = np.int32(center) - circle.astype(np.int32)
    arc_min = np.empty(16, dtype=np.int32)
    arc_max = np.empty(16, dtype=np.int32)
    for s in range(16):
        arc = np.array([d[(s + i) % 16] for i in range(9)], dtype=np.int32)
        arc_min[s] = arc.min()
        arc_max[s] = arc.max()
    brighter = arc_min.max()
    darker   = arc_max.min()
    best = max(brighter, -darker)
    score = best - 1
    return max(0, min(255, score))


def patch_scores(patch: np.ndarray):
    """9 FAST scores [centre, 8 neighbours] computed from the SAME patch."""
    cr = patch.shape[0] // 2   # 15
    cc = patch.shape[1] // 2
    scores = []
    for (dr, dc) in NB:
        r0, c0 = cr + dr, cc + dc
        circ = np.array([patch[r0 + ro, c0 + co] for (ro, co) in OFF16], dtype=np.uint8)
        scores.append(corner_score_simd(int(patch[r0, c0]), circ))
    return scores


def is_local_max(scores) -> bool:
    # '>=' (plateau-keeping) rule -- MUST match fast_nms.vhd.
    #
    # PINNED 2026-08-11, AND IT DIFFERS FROM OpenCV: cv2 FAST_t requires
    # "score > n" for all 8 neighbours (opencv 4.5.4 fast.cpp), i.e. it DROPS
    # ties. On a real MH01 frame the '>' rule reproduces cv2 exactly (9857,
    # sym diff 0) while this '>=' rule keeps 11926. The deviation is KNOWN and
    # deliberately NOT fixed: changing it means resynthesis + redoing the whole
    # on-board campaign. See progress.md [F.24]. Keep this in sync with
    # fast_nms.vhd -- if the RTL is ever switched to '>', switch here too.
    return all(scores[0] >= scores[k] for k in range(1, 9))


def write_line(f, patch: np.ndarray, score: int, lmax: bool):
    parts  = [f"{int(p):02x}" for p in patch.flatten()]   # row-major
    parts.append(f"{int(score):02x}")
    parts.append("01" if lmax else "00")
    f.write(" ".join(parts) + "\n")


def gen(path: str, n_random: int, seed: int, N: int = 31):
    rng = np.random.default_rng(seed)
    cr = N // 2
    with open(path, 'w') as f:
        # Targeted edge cases (both verdicts)
        edges = []
        edges.append(np.zeros((N, N), dtype=np.uint8))                 # flat 0   -> lmax (tie at 0)
        edges.append(np.full((N, N), 128, dtype=np.uint8))             # flat 128 -> lmax (tie at 0)
        p = np.zeros((N, N), dtype=np.uint8); p[cr, cr] = 255          # bright centre -> lmax
        edges.append(p)
        p = np.zeros((N, N), dtype=np.uint8); p[cr + 1, cr + 1] = 255  # bright neighbour -> not lmax
        edges.append(p)
        for patch in edges:
            s = patch_scores(patch)
            write_line(f, patch, s[0], is_local_max(s))

        # Random patches (full 31x31; only the central 9x9-ish region affects scores)
        for _ in range(n_random):
            patch = rng.integers(0, 256, size=(N, N), dtype=np.uint8)
            s = patch_scores(patch)
            write_line(f, patch, s[0], is_local_max(s))


def main():
    ap = argparse.ArgumentParser(description="Generate test vectors for fast_nms.vhd")
    ap.add_argument("-n", "--num-vectors", type=int, default=2000)
    ap.add_argument("-s", "--seed", type=int, default=0)
    ap.add_argument("-o", "--output", type=str, default="fast_nms_vectors.hex")
    args = ap.parse_args()
    gen(args.output, n_random=args.num_vectors, seed=args.seed)
    print(f"Wrote {args.output}: {args.num_vectors} random + 4 edge cases (seed={args.seed})")


if __name__ == "__main__":
    main()