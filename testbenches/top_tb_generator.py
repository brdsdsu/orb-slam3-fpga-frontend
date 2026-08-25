"""Test vector generator for TOP_tb (the AXI wrapper testbench).

Produces TWO frame vector sets with DIFFERENT dimensions, plus a shared
params package:
  - top_pixels_f1.hex / top_expected_f1.txt : frame 1 (full size = G_MAX_WIDTH)
  - top_pixels_f2.hex / top_expected_f2.txt : frame 2 (smaller -- exercises the
        runtime img_width/img_height path, i.e. the per-pyramid-level pattern)
  - top_test_params_pkg.vhd

All golden-model functions are IMPORTED from extractor_top_generator.py --
the exact references the per-module and extractor_top validation used. This
script adds no new model code, so the golden provenance is unchanged.

Run from testbenches/ (same convention as extractor_top_generator.py):
    py top_tb_generator.py
"""

import numpy as np

from extractor_top_generator import (
    make_image, golden_keypoints, hx,
    compute_cell_grid, apply_strict_gate, select_per_cell,
)

# Frame 1: identical parameters to the validated extractor_top run.
F1 = dict(width=256, height=224, seed=2)
# Frame 2: smaller in BOTH dimensions, different seed. Must satisfy
# PATCH_SIZE(31) <= width <= F1 width (= G_MAX_WIDTH in the TB).
F2 = dict(width=192, height=160, seed=7)  # seed 7: balanced strict/perm + bright/dark
# Frame 3 (T7, production config only): DENSE grid, > 1024 corners, so both
# FIFO pointer sets wrap at the SHIPPED depths (corner FIFO 512: twice;
# output FIFO 1024: once) while burst occupancy stays ~50 -- still drop-free.
F3 = dict(width=256, height=448, seed=12)

TH_PERM = 7
TH_STRICT = 20


def make_dense_image(W, H, seed, step=40, margin=24):
    """Tighter square grid than make_image (step 40 vs 70, sizes 20-28).
    Same flat-background-plus-squares construction; only DENSITY differs.
    The golden-model scan functions are unchanged (imported)."""
    rng = np.random.default_rng(seed)
    img = np.full((H, W), 128, dtype=np.uint8)
    for gy in range(margin, H - margin - 30, step):
        for gx in range(margin, W - margin - 30, step):
            ox = gx + int(rng.integers(0, 6))
            oy = gy + int(rng.integers(0, 6))
            size = int(rng.integers(20, 28))
            mode = rng.integers(0, 3)
            if mode == 0:
                val = int(rng.integers(175, 256))     # strong bright
            elif mode == 1:
                val = int(rng.integers(0, 80))        # strong dark
            else:
                delta = int(rng.integers(9, 19)) * (1 if rng.integers(0, 2) else -1)
                val = 128 + delta                     # weak (perm-only)
            img[oy:oy + size, ox:ox + size] = val
    return img


def emit_frame(tag, cfg, dense=False):
    W, H = cfg["width"], cfg["height"]
    if dense:
        img = make_dense_image(W, H, cfg["seed"])
    else:
        img = make_image(W, H, cfg["seed"])
    grid = compute_cell_grid(W, H)
    kps_full, _ = golden_keypoints(img, TH_PERM, TH_STRICT)

    # PL-side strict-cell gate: TOP writes this frame's grid to CELLDIM/CELLNUM, so the
    # DUT emits the gated stream. Verify the gate preserves the final SelectPerCellHW set.
    kps, n_supp = apply_strict_gate(kps_full, grid)
    ref  = select_per_cell(kps_full, W, H, grid)
    test = select_per_cell(kps,      W, H, grid)
    assert ref == test, f"frame {tag}: gate changed the post-SelectPerCellHW set"

    pix_name = f"top_pixels_{tag}.hex"

    with open(pix_name, "w") as f:
        f.write("\n".join(f"{int(v):02x}" for v in img.reshape(-1)) + "\n")

    # One expected file per score type (same corner set, different 32-bit
    # response column: FAST score zero-extended vs integer Harris response).
    for suffix, key in (("", "score"), ("_harris", "hresp")):
        with open(f"top_expected_{tag}{suffix}.txt", "w") as f:
            for k in kps:
                f.write(f"{k['x']} {k['y']} {k['score']} {k['is_brighter']} "
                        f"{k['passed_strict']} {hx(k['angle'], 24)} {k['tol']} "
                        f"{hx(k['m10'], 24)} {hx(k['m01'], 24)} {hx(k[key], 32)}\n")

    strict = sum(k['passed_strict'] for k in kps)
    bright = sum(k['is_brighter'] for k in kps)
    print(f"Frame {tag}: {W}x{H} seed {cfg['seed']} -> {len(kps)} corners "
          f"(strict={strict}, brighter={bright}, gate-suppressed={n_supp})")
    return dict(n=len(kps), n_supp=n_supp, grid=grid)


def main():
    assert 31 <= F2["width"] <= F1["width"], \
        "F2 width must be in [PATCH_SIZE, F1 width] (F1 width = G_MAX_WIDTH)"

    r1 = emit_frame("f1", F1)
    r2 = emit_frame("f2", F2)
    r3 = emit_frame("f3", F3, dense=True)
    assert r1['n'] > 0 and r2['n'] > 0, "a frame produced zero corners -- useless vectors"
    assert r3['n'] > 1024, "F3 must exceed the output-FIFO depth (1024) for T7 to wrap it"

    def grid_consts(tag, r):
        g = r['grid']
        return (f"    constant {tag}_WCELL        : natural  := {g['wCell']};\n"
                f"    constant {tag}_HCELL        : natural  := {g['hCell']};\n"
                f"    constant {tag}_NCOLS        : natural  := {g['nCols']};\n"
                f"    constant {tag}_NROWS        : natural  := {g['nRows']};\n"
                f"    constant {tag}_N_SUPPRESSED : natural  := {r['n_supp']};\n")

    with open("top_test_params_pkg.vhd", "w") as f:
        f.write(f"""-- AUTO-GENERATED by top_tb_generator.py. DO NOT EDIT.

package top_test_params_pkg is
    constant TOP_TB_TH_PERM   : natural  := {TH_PERM};
    constant TOP_TB_TH_STRICT : natural  := {TH_STRICT};

    -- Frame 1: full size (defines G_MAX_WIDTH in the TB)
    constant F1_WIDTH         : positive := {F1['width']};
    constant F1_HEIGHT        : positive := {F1['height']};
    constant F1_N_EXPECTED    : natural  := {r1['n']};
{grid_consts("F1", r1)}
    -- Frame 2: smaller, exercises runtime resize between frames
    constant F2_WIDTH         : positive := {F2['width']};
    constant F2_HEIGHT        : positive := {F2['height']};
    constant F2_N_EXPECTED    : natural  := {r2['n']};
{grid_consts("F2", r2)}
    -- Frame 3: dense grid for T7 (production config only) -- pointer wrap
    -- at the shipped FIFO depths (corner 512, output 1024)
    constant F3_WIDTH         : positive := {F3['width']};
    constant F3_HEIGHT        : positive := {F3['height']};
    constant F3_N_EXPECTED    : natural  := {r3['n']};
{grid_consts("F3", r3)}
    constant F1_PIXELS_FILE   : string := "testbenches/top_pixels_f1.hex";
    constant F1_EXPECTED_FILE : string := "testbenches/top_expected_f1.txt";
    constant F2_PIXELS_FILE   : string := "testbenches/top_pixels_f2.hex";
    constant F2_EXPECTED_FILE : string := "testbenches/top_expected_f2.txt";
    constant F3_PIXELS_FILE   : string := "testbenches/top_pixels_f3.hex";
    constant F3_EXPECTED_FILE : string := "testbenches/top_expected_f3.txt";
    -- Harris-build goldens (G_SCORE_TYPE = 0): same corner set, Harris response column
    constant F1_EXPECTED_HARRIS_FILE : string := "testbenches/top_expected_f1_harris.txt";
    constant F2_EXPECTED_HARRIS_FILE : string := "testbenches/top_expected_f2_harris.txt";
    constant F3_EXPECTED_HARRIS_FILE : string := "testbenches/top_expected_f3_harris.txt";
end package;
""")
    print("Params  : top_test_params_pkg.vhd")


if __name__ == "__main__":
    main()
