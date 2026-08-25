"""End-to-end test vector generator for extractor_top.

Produces, for a synthetic image:
  - extractor_pixels.hex   : W*H pixel bytes, raster order, one hex byte/line
  - extractor_expected.txt : golden keypoints in RASTER order of center position
                             (matches the hardware's emission order), one/line:
                             x y score is_brighter passed_strict angle_hex tol m10_hex m01_hex
  - extractor_test_params_pkg.vhd : IMG_WIDTH/HEIGHT, thresholds, N_EXPECTED

The golden model chains the SAME validated per-module references the hardware
was checked against: 16-point FAST circle -> FAST-9 detection (permissive
threshold) + SIMD score -> 3x3 non-max suppression (matches fast_nms.vhd, but
NOT OpenCV -- see the tie-rule note below) -> IC_Angle moments -> ideal
CORDIC-convention angle.

NMS: a detected corner is kept only when its FAST score is a 3x3 local maximum
of the score field. Neighbour scores are computed from the same physical pixels
the hardware's fast_nms sees in the patch, so the golden corner set matches the
NMS-gated hardware. The '>=' (plateau-keeping) tie rule MUST stay in sync with
fast_nms.vhd.

TIE RULE, PINNED 2026-08-11: the '>=' rule here and in fast_nms.vhd does NOT
match OpenCV, which drops ties ("score > n" for all 8, opencv 4.5.4 fast.cpp).
detect_fast9 and corner_score_simd ARE bit-exact vs cv2 (9857/9857 responses on
a real MH01 frame); the tie rule is the only divergence. KNOWN and deliberately
not fixed (resynthesis + full re-benchmark). See progress.md [F.24].

Image: flat background with sparse, large, well-separated squares. Large
squares fire FAST only at their four corners, keeping the corner count well
under FIFO_DEPTH so no drops can occur. Mixed intensities exercise
strict/permissive and brighter/darker; the four corners give varied orientations.
"""

import argparse
import math
import numpy as np

HALF_PATCH_SIZE = 15
PATCH_SIZE = 31
CORDIC_WIDTH = 24
HALF = HALF_PATCH_SIZE

OFFSETS16 = [
    ( 0,-3),( 1,-3),( 2,-2),( 3,-1),( 3, 0),( 3, 1),( 2, 2),( 1, 3),
    ( 0, 3),(-1, 3),(-2, 2),(-3, 1),(-3, 0),(-3,-1),(-2,-2),(-1,-3),
]

# 8-neighbourhood offsets (dr, dc) for 3x3 NMS; matches fast_nms.OFFS (minus centre).
NB8 = [(-1,-1),(-1, 0),(-1, 1),( 0,-1),( 0, 1),( 1,-1),( 1, 0),( 1, 1)]

# ---------------- strict-cell gate model (lockstep with extractor_top + SelectPerCellHW) ----
# The PL gate suppresses a permissive corner in a safe interior cell that already holds
# a strict corner (== SelectPerCellHW would drop it anyway). Both the RTL counter and
# the C++ helper realise the integer math below, so all three stay bit-exact.
EDGE_THRESHOLD = 19
GRID_ORIGIN = (EDGE_THRESHOLD - 3) + 3   # minBorderX(16) + FAST_BORDER(3) = 19
GUARD = 3                                # FAST_BORDER seam margin (== VHDL GUARD)
W_CELL_NOM = 35                          # ORB-SLAM3 grid cell size W


def compute_cell_grid(W, H, w=W_CELL_NOM):
    """Byte-identical to ORBextractor.cc ComputeCellGrid / SelectPerCellHW grid."""
    border = 2 * (EDGE_THRESHOLD - 3)
    width, height = W - border, H - border
    nCols = width // w
    nRows = height // w
    wCell = (width + nCols - 1) // nCols if nCols > 0 else width    # ceil
    hCell = (height + nRows - 1) // nRows if nRows > 0 else height
    return dict(nCols=nCols, nRows=nRows, wCell=wCell, hCell=hCell)


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


# ---------------- validated module references ----------------
def get_circle(img, cx, cy):
    return np.array([int(img[cy + dy, cx + dx]) for (dx, dy) in OFFSETS16],
                    dtype=np.int32)


def detect_fast9(center, circle, threshold):
    upper = center + threshold
    lower = center - threshold
    brighter = circle > upper
    darker = circle < lower
    is_b = any(all(brighter[(s + i) % 16] for i in range(9)) for s in range(16))
    is_d = any(all(darker[(s + i) % 16] for i in range(9)) for s in range(16))
    return (int(is_b or is_d), int(is_b))


def corner_score_simd(center, circle):
    d = np.int32(center) - circle
    arc_min = np.empty(16, dtype=np.int32)
    arc_max = np.empty(16, dtype=np.int32)
    for s in range(16):
        arc = np.array([d[(s + i) % 16] for i in range(9)], dtype=np.int32)
        arc_min[s] = arc.min()
        arc_max[s] = arc.max()
    brighter = arc_min.max()
    darker = arc_max.min()
    best = max(brighter, -darker)
    return max(0, min(255, best - 1))


def score_at(img, cx, cy):
    """FAST score at an arbitrary centre -- same value fast_response gives, and
    (since it uses the same radius-3 circle of physical pixels) the same value
    fast_nms computes for that position from the patch."""
    return corner_score_simd(int(img[cy, cx]), get_circle(img, cx, cy))


def is_local_max_at(img, cx, cy, center_score):
    """3x3 non-max suppression. Centre survives iff no neighbour STRICTLY exceeds
    its score ('>=' rule == fast_nms.vhd). Non-corner neighbours never suppress:
    their score < threshold <= the centre's (the centre is a detected corner)."""
    for (dr, dc) in NB8:
        if center_score < score_at(img, cx + dc, cy + dr):
            return False
    return True


# ---------------- Harris response (integer spec == harris_response.vhd) -----
# OpenCV HarrisResponses (orb.cpp, blockSize=7, k=0.04) as the order-isomorphic
# integer transform: k = 1/25 exactly, positive scale^4 factor dropped, x25.
HARRIS_BLOCK_HALF = 3
HARRIS_OUT_SHIFT = 26


def harris_response_hw(img64, cx, cy):
    """Integer Harris response at (cx, cy); img64 = image as int64 ndarray.
    Bit-exact spec shared with harris_response.vhd / harris_response_generator.py."""
    a = b = c = 0
    for r in range(cy - HARRIS_BLOCK_HALF, cy + HARRIS_BLOCK_HALF + 1):
        for cc in range(cx - HARRIS_BLOCK_HALF, cx + HARRIS_BLOCK_HALF + 1):
            ix = (2 * (img64[r, cc + 1] - img64[r, cc - 1])
                  + (img64[r - 1, cc + 1] - img64[r - 1, cc - 1])
                  + (img64[r + 1, cc + 1] - img64[r + 1, cc - 1]))
            iy = (2 * (img64[r + 1, cc] - img64[r - 1, cc])
                  + (img64[r + 1, cc - 1] - img64[r - 1, cc - 1])
                  + (img64[r + 1, cc + 1] - img64[r - 1, cc + 1]))
            a += int(ix) * int(ix)
            b += int(iy) * int(iy)
            c += int(ix) * int(iy)
    resp = (25 * (a * b - c * c) - (a + b) * (a + b)) >> HARRIS_OUT_SHIFT
    assert -2**31 <= resp <= 2**31 - 1
    return resp


def compute_moments(patch):
    cr = HALF
    m10 = 0
    m01 = 0
    for u in range(-HALF, HALF + 1):
        m10 += u * int(patch[cr, cr + u])
    for v in range(1, HALF + 1):
        d = U_MAX[v]
        vsum = 0
        for u in range(-d, d + 1):
            vp = int(patch[cr + v, cr + u])
            vm = int(patch[cr - v, cr + u])
            vsum += (vp - vm)
            m10 += u * (vp + vm)
        m01 += v * vsum
    return m10, m01


def ideal_angle(m01, m10, dw=CORDIC_WIDTH):
    scaled = round(math.atan2(m01, m10) * ((1 << (dw - 2)) / math.pi))
    return max(-(1 << (dw - 1)), min((1 << (dw - 1)) - 1, scaled))


def per_vector_tolerance(m10, m01, dw=CORDIC_WIDTH):
    mag = math.hypot(m10, m01)
    if mag < 1.0:
        return 4
    residual = (1 << (dw - 2)) / (math.pi * mag)
    return int(4 * residual) + 512


# ---------------- synthetic image ----------------
def make_image(W, H, seed):
    rng = np.random.default_rng(seed)
    img = np.full((H, W), 128, dtype=np.uint8)
    margin = 24
    step = 70
    ys = list(range(margin, H - margin - 30, step))
    xs = list(range(margin, W - margin - 30, step))
    for gy in ys:
        for gx in xs:
            ox = gx + int(rng.integers(0, 8))
            oy = gy + int(rng.integers(0, 8))
            size = int(rng.integers(20, 30))
            mode = rng.integers(0, 3)
            if mode == 0:
                val = int(rng.integers(175, 256))     # strong bright
            elif mode == 1:
                val = int(rng.integers(0, 80))        # strong dark
            else:
                delta = int(rng.integers(9, 19)) * (1 if rng.integers(0, 2) else -1)
                val = 128 + delta                     # weak (perm-only)
            img[oy:oy + size, ox:ox + size] = val
    add_gate_cluster(img)
    return img


def add_gate_cluster(img):
    """Dense mixed-strength cluster in a safe interior cell, so the strict-cell gate
    actually fires: a strong square (strict corners, scanned first) then a weak square
    (permissive corners) in the SAME cell -> the permissive corners get suppressed.
    Placed via the shared grid so it lands squarely in the safe interior. The cell is
    cleared to background first so the cluster is self-contained."""
    H, W = img.shape
    grid = compute_cell_grid(W, H)
    j = min(3, grid['nCols'] - 2)
    i = min(2, grid['nRows'] - 2)
    if j < 1 or i < 1:
        return  # image too small for an interior cell
    x0 = GRID_ORIGIN + j * grid['wCell']
    y0 = GRID_ORIGIN + i * grid['hCell']
    y1 = min(H, y0 + grid['hCell'])
    x1 = min(W, x0 + grid['wCell'])
    img[y0:y1, x0:x1] = 128                       # clear the cell to background
    # strong square -> strict corners, top-left of the cell (past the GUARD margin)
    img[y0 + 6:y0 + 17, x0 + 6:x0 + 17] = 230
    # weak square -> permissive corners, lower-right, same cell (delta +14 -> perm)
    img[y0 + 22:y0 + 29, x0 + 8:x0 + 15] = 142


# ---------------- golden scan (raster order) ----------------
def golden_keypoints(img, th_perm, th_strict):
    H, W = img.shape
    img64 = img.astype(np.int64)
    kps = []
    n_detected = 0
    for cy in range(HALF, H - HALF):
        for cx in range(HALF, W - HALF):
            center = int(img[cy, cx])
            circle = get_circle(img, cx, cy)
            is_corner, is_brighter = detect_fast9(center, circle, th_perm)
            if not is_corner:
                continue
            n_detected += 1
            score = corner_score_simd(center, circle)
            # 3x3 non-max suppression -- keep only local-maximum corners.
            if not is_local_max_at(img, cx, cy, score):
                continue
            patch = img[cy - HALF:cy + HALF + 1, cx - HALF:cx + HALF + 1].astype(np.int32)
            m10, m01 = compute_moments(patch)
            kps.append(dict(
                x=cx, y=cy, score=score, is_brighter=is_brighter,
                passed_strict=int(score >= th_strict),
                angle=ideal_angle(m01, m10), tol=per_vector_tolerance(m10, m01),
                m10=m10, m01=m01, hresp=harris_response_hw(img64, cx, cy)))
    return kps, n_detected


def apply_strict_gate(kps, grid):
    """Causal PL-side gate == extractor_top p_cellgrid + gate. Consumes the post-NMS
    corner stream in raster (emission) order; drops permissive corners in a cell that
    already holds a strict corner, but only in the conservative safe interior (inside
    the outer ring, GUARD px off the seams) where (x-19)//wCell provably equals
    SelectPerCellHW's cell_of. Returns (kept, n_suppressed)."""
    nCols, nRows = grid['nCols'], grid['nRows']
    wCell, hCell = grid['wCell'], grid['hCell']
    strict_seen, cur_band = set(), None
    kept, n_supp = [], 0
    for k in kps:
        x, y = k['x'], k['y']
        if x >= GRID_ORIGIN:
            cx = x - GRID_ORIGIN
            cellcol = cx // wCell
            phase_x = cx - cellcol * wCell
        else:
            cellcol, phase_x = -1, -1
        if y >= GRID_ORIGIN:
            cyy = y - GRID_ORIGIN
            band = cyy // hCell
            phase_y = cyy - band * hCell
        else:
            band, phase_y = 0, -1

        if band != cur_band:            # band change -> clear strict-seen (== band_q change)
            strict_seen.clear()
            cur_band = band

        safe = (1 <= cellcol <= nCols - 2 and 1 <= band <= nRows - 2
                and GUARD <= phase_x <= wCell - 1 - GUARD
                and GUARD <= phase_y <= hCell - 1 - GUARD)

        if k['passed_strict']:
            if safe:
                strict_seen.add(cellcol)
            kept.append(k)
        elif safe and cellcol in strict_seen:
            n_supp += 1                 # SUPPRESS (== gate drops the FIFO push)
        else:
            kept.append(k)
    return kept, n_supp


def select_per_cell(kps, W, H, grid):
    """Faithful port of SelectPerCellHW (ORBextractor.cc): per W=35 cell, if any strict
    corner keep only strict, else keep all permissive. Returns kept corners as (x,y)
    absolute-coord tuples in cell-row-major / raster-within-cell emission order. Used
    ONLY to assert the gate preserves the final set: select_per_cell(full) == (gated)."""
    FAST_BORDER = 3
    nCols, nRows = grid['nCols'], grid['nRows']
    wCell, hCell = grid['wCell'], grid['hCell']
    if nCols <= 0 or nRows <= 0:
        return []
    minBorderX = EDGE_THRESHOLD - 3            # 16
    minBorderY = minBorderX
    maxBorderX = W - EDGE_THRESHOLD + 3        # cols - 16
    maxBorderY = H - EDGE_THRESHOLD + 3

    def cdiv(a, b):                            # C++ integer division: truncate toward 0
        q = abs(a) // abs(b)
        return q if (a < 0) == (b < 0) else -q

    def cell_of(x, y):                         # == the C++ lambda, exactly
        j = cdiv(x - minBorderX - FAST_BORDER, wCell)
        i = cdiv(y - minBorderY - FAST_BORDER, hCell)
        if i < 0 or i >= nRows or j < 0 or j >= nCols:
            return -1
        iniX = minBorderX + j * wCell
        iniY = minBorderY + i * hCell
        if iniY >= maxBorderY - 3:  return -1  # thin bottom band, skipped
        if iniX >= maxBorderX - 6:  return -1  # thin right col, skipped
        maxX = min(iniX + wCell + 6, maxBorderX)
        maxY = min(iniY + hCell + 6, maxBorderY)
        if x < iniX + FAST_BORDER or x > maxX - FAST_BORDER - 1:  return -1
        if y < iniY + FAST_BORDER or y > maxY - FAST_BORDER - 1:  return -1
        return i * nCols + j

    nCells = nRows * nCols
    cellId = [cell_of(k['x'], k['y']) for k in kps]
    anyStrict = [False] * nCells
    start = [0] * (nCells + 1)
    for k, cid in zip(kps, cellId):
        if cid >= 0:
            start[cid + 1] += 1
            if k['passed_strict']:
                anyStrict[cid] = True
    for c in range(nCells):
        start[c + 1] += start[c]
    order = [0] * start[nCells]
    pos = start[:nCells]
    for idx, cid in enumerate(cellId):
        if cid >= 0:
            order[pos[cid]] = idx
            pos[cid] += 1
    out = []
    for idx in order:
        k = kps[idx]
        if anyStrict[cellId[idx]] and not k['passed_strict']:
            continue
        out.append((k['x'], k['y']))
    return out


def hx(val, width_bits):
    mask = (1 << width_bits) - 1
    chars = (width_bits + 3) // 4
    return f"{int(val) & mask:0{chars}x}"   # int(): numpy ints choke on 32-bit masks


def main():
    p = argparse.ArgumentParser(description="extractor_top end-to-end vectors")
    p.add_argument("-W", "--width", type=int, default=256)
    p.add_argument("-H", "--height", type=int, default=224)
    p.add_argument("-s", "--seed", type=int, default=2)
    p.add_argument("--th-perm", type=int, default=7)
    p.add_argument("--th-strict", type=int, default=20)
    p.add_argument("--score-type", choices=("fast", "harris"), default="fast",
                   help="response column content (corner set is identical: "
                        "detection/NMS/gate are FAST-based in both modes)")
    p.add_argument("--pixels-file", default="extractor_pixels.hex")
    p.add_argument("--expected-file", default="extractor_expected.txt")
    p.add_argument("--params-file", default="extractor_test_params_pkg.vhd")
    args = p.parse_args()

    img = make_image(args.width, args.height, args.seed)
    grid = compute_cell_grid(args.width, args.height)
    kps_full, n_detected = golden_keypoints(img, args.th_perm, args.th_strict)

    # PL-side strict-cell gate: the HW emits this reduced stream (permissive corners in
    # a safe strict cell are dropped before the FIFO). The gated stream is the golden
    # for the gate-on TB run; the ungated stream is what a cfg=0 (inert) run emits.
    kps, n_supp = apply_strict_gate(kps_full, grid)

    # Invariant guard: the gate must not change the post-SelectPerCellHW keypoint set.
    ref  = select_per_cell(kps_full, args.width, args.height, grid)
    test = select_per_cell(kps,      args.width, args.height, grid)
    if ref != test:
        only_ref  = sorted(set(ref)  - set(test))
        only_test = sorted(set(test) - set(ref))
        raise AssertionError("gate changed the post-SelectPerCellHW keypoint set: "
                             f"lost {only_ref}, gained {only_test}")

    # Pixel stream, raster order, one hex byte per line.
    with open(args.pixels_file, "w") as f:
        f.write("\n".join(f"{int(v):02x}" for v in img.reshape(-1)) + "\n")

    # Expected keypoints in raster order (matches gated hardware emission order).
    # Last column = the 32-bit response the DUT must emit: the FAST score
    # zero-extended (score-type fast) or the integer Harris response.
    with open(args.expected_file, "w") as f:
        for k in kps:
            resp = k['score'] if args.score_type == "fast" else k['hresp']
            f.write(f"{k['x']} {k['y']} {k['score']} {k['is_brighter']} "
                    f"{k['passed_strict']} {hx(k['angle'], 24)} {k['tol']} "
                    f"{hx(k['m10'], 24)} {hx(k['m01'], 24)} {hx(resp, 32)}\n")

    with open(args.params_file, "w") as f:
        f.write(f"""-- AUTO-GENERATED by extractor_top_generator.py. DO NOT EDIT.

library ieee;
use ieee.std_logic_1164.all;

package extractor_test_params_pkg is
    constant TEST_IMG_WIDTH    : positive := {args.width};
    constant TEST_IMG_HEIGHT   : positive := {args.height};
    constant TEST_TH_PERM      : natural  := {args.th_perm};
    constant TEST_TH_STRICT    : natural  := {args.th_strict};
    constant TEST_N_EXPECTED   : natural  := {len(kps)};
    -- Strict-cell gate: per-level grid geometry + suppressed-corner count.
    constant TEST_WCELL        : natural  := {grid['wCell']};
    constant TEST_HCELL        : natural  := {grid['hCell']};
    constant TEST_NCOLS        : natural  := {grid['nCols']};
    constant TEST_NROWS        : natural  := {grid['nRows']};
    constant TEST_N_SUPPRESSED : natural  := {n_supp};
end package;
""")

    strict = sum(k['passed_strict'] for k in kps)
    bright = sum(k['is_brighter'] for k in kps)
    print(f"Image {args.width}x{args.height} seed {args.seed}")
    print(f"  grid: wCell={grid['wCell']} hCell={grid['hCell']} "
          f"nCols={grid['nCols']} nRows={grid['nRows']}")
    print(f"  Detected (pre-NMS): {n_detected}")
    print(f"  Post-NMS          : {len(kps_full)}")
    print(f"  Gate-suppressed   : {n_supp}")
    print(f"  Kept (HW output)  : {len(kps)}  strict={strict}, perm-only={len(kps)-strict}, "
          f"brighter={bright}, darker={len(kps)-bright}")
    print(f"  select_per_cell equivalence: OK ({len(ref)} final keypoints)")
    print(f"  Pixels  : {args.pixels_file}  ({args.width*args.height} bytes)")
    print(f"  Expected: {args.expected_file}")
    print(f"  Params  : {args.params_file}")
    if n_supp == 0:
        print("  WARNING: gate suppressed 0 corners -- vector does not exercise the gate.")
    if len(kps) >= 256:
        print("  WARNING: corner count >= 256; FIFO_DEPTH=256 could drop. "
              "Reduce squares or raise FIFO_DEPTH.")


if __name__ == "__main__":
    main()