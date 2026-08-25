"""Pin down the exact relationship between FAST-9 detection at threshold T
and a comparison against the FAST score.

We want to know which of these identities holds universally:
    is_corner(T)  ==  (score >= T)
    is_corner(T)  ==  (score >= T+1)
    is_corner(T)  ==  (score >  T)
so that extractor_top can compute `passed_strict` as a single comparison
of the already-computed fast_response score against the strict threshold.

Both reference functions are copied verbatim from the validated generators
(fast9_core_generator.py and fast_response generator), so this check uses
exactly the semantics the hardware was validated against.
"""

import numpy as np
from tqdm import tqdm


# ---- Reference: FAST-9 detection (matches fast9_core.vhd) ----
def detect_fast9(center: int, circle: np.ndarray, threshold: int):
    """Returns (is_corner, is_brighter), each 0/1."""
    upper = center + threshold
    lower = center - threshold
    brighter = circle.astype(np.int32) > upper      # strict >
    darker = circle.astype(np.int32) < lower        # strict <

    is_brighter = any(
        all(brighter[(s + i) % 16] for i in range(9))
        for s in range(16)
    )
    is_darker = any(
        all(darker[(s + i) % 16] for i in range(9))
        for s in range(16)
    )
    return (int(is_brighter or is_darker), int(is_brighter))


# ---- Reference: FAST score (matches fast_response.vhd, OpenCV SIMD path) ----
def corner_score_simd(center: int, circle: np.ndarray) -> int:
    d = np.int32(center) - circle.astype(np.int32)   # center - circle

    arc_min = np.empty(16, dtype=np.int32)
    arc_max = np.empty(16, dtype=np.int32)
    for s in range(16):
        arc = np.array([d[(s + i) % 16] for i in range(9)], dtype=np.int32)
        arc_min[s] = arc.min()
        arc_max[s] = arc.max()

    brighter = arc_min.max()
    darker = arc_max.min()

    best = max(brighter, -darker)
    score = best - 1
    return max(0, min(255, score))   # saturate to uint8


def main():
    rng = np.random.default_rng(0)
    N = 200_000

    # Candidate relationships to test, keyed by name -> predicate(score, T)
    relations = {
        "score >= T":   lambda s, T: s >= T,
        "score >= T+1": lambda s, T: s >= T + 1,
        "score >  T":   lambda s, T: s > T,
    }

    # Thresholds of interest plus a broad sweep
    thresholds = [0, 1, 2, 5, 7, 10, 15, 20, 30, 50, 100, 200, 254, 255]

    # Track, per relation, the set of thresholds where it ever disagrees
    # with detection, and a sample mismatch.
    broken = {name: {} for name in relations}   # name -> {T: (center, circle, score, det)}

    # Also bias some vectors toward real corners so we exercise the
    # corner-positive path heavily, not just the common no-corner case.
    for trial in tqdm(range(N)):
        center = int(rng.integers(0, 256))
        if trial % 3 == 0:
            # crafted-ish: a run of brighter or darker pixels around the ring
            circle = np.full(16, center, dtype=np.uint8)
            run_len = int(rng.integers(7, 13))
            start = int(rng.integers(0, 16))
            delta = int(rng.integers(1, 120))
            sign = 1 if rng.integers(0, 2) == 0 else -1
            for i in range(run_len):
                val = np.clip(center + sign * delta, 0, 255)
                circle[(start + i) % 16] = val
        else:
            circle = rng.integers(0, 256, size=16, dtype=np.uint8)

        score = corner_score_simd(center, circle)

        for T in thresholds:
            det = detect_fast9(center, circle, T)[0]
            for name, pred in relations.items():
                if int(pred(score, T)) != det:
                    if T not in broken[name]:
                        broken[name][T] = (center, circle.tolist(), score, det)

    print(f"Tested {N} vectors across thresholds {thresholds}\n")
    for name in relations:
        bad_Ts = sorted(broken[name].keys())
        if not bad_Ts:
            print(f"  '{name}':  HOLDS for all tested thresholds")
        else:
            print(f"  '{name}':  FAILS at T = {bad_Ts}")
            # show one example at the smallest failing T
            T0 = bad_Ts[0]
            c, circ, sc, det = broken[name][T0]
            print(f"        e.g. T={T0}: center={c}, score={sc}, "
                  f"detection={det}, predicate={int(relations[name](sc, T0))}")
    print()

    # Specifically confirm the two thresholds we care about for extractor_top
    print("Focused check on the thresholds extractor_top uses:")
    for T in (7, 20):
        rel = "score >= T"
        ok = T not in broken[rel]
        print(f"  detection(T={T}) == (score >= {T}):  "
              f"{'OK' if ok else 'MISMATCH'}")


if __name__ == "__main__":
    main()
