# ORB Feature-Extraction Accelerator (VHDL)

Synthesizable VHDL for the ORB-SLAM3 feature-extraction front end, targeting the
programmable logic of an AMD Kria KR260 (Zynq UltraScale+ MPSoC, `xck26-sfvc784-2LV-c`).
The accelerator takes over FAST detection, non-maximum suppression, corner scoring and
keypoint orientation from `ORBextractor`. The processing system streams one image-pyramid
level at a time into it over AXI DMA and reads back packed 128-bit keypoints.

Written for a Master's thesis at the University of the Bundeswehr Munich.
The work was carried out during a research visit to San Diego State University.

This repository holds the RTL, its testbenches, the non-regenerable Vivado project sources,
and the hand-written files the Kria needs to load the accelerator. The generated bitstreams
are not here, and neither is the software side.

The matching ORB-SLAM3 fork lives on the `kr260_hw_accelerator` branch of
[brdsdsu/ORB_SLAM3](https://github.com/brdsdsu/ORB_SLAM3/tree/kr260_hw_accelerator). It
holds the userspace driver (`src/ORBHwAccelerator.cc`), the `ORBextractor` changes that call
into the hardware, and the stereo two-instance path. What it needs from this design is the
interface contract documented below: the AXI4-Lite register map, the 128-bit keypoint
layout, and the build ID that couples them.

## Design at a glance

| | |
| --- | --- |
| Target | AMD Kria KR260, `xck26-sfvc784-2LV-c` |
| PL clock | 125 MHz, single clock domain (stream clock = fabric clock = AXI-Lite clock) |
| Interfaces | AXI4-Stream slave (8-bit pixels), AXI4-Stream master (128-bit keypoints), AXI4-Lite slave (control and status) |
| Language | VHDL-2008, plus a VHDL-93/2002 wrapper required by the Vivado IP packager |
| Detection | FAST-9 at the permissive threshold, 3x3 non-maximum suppression on the FAST score |
| Scoring | FAST score or Harris response, selected at elaboration via `G_SCORE_TYPE` |
| Orientation | intensity centroid over a 31x31 circular patch, CORDIC atan2 |
| Build ID | `0xC0DE0003` (readable at AXI-Lite offset `0x1C`) |

Generic defaults are sized so that two instances fit on a K26, which is what a stereo
configuration needs.

The design tracks OpenCV and ORB-SLAM3 semantics closely enough that the hardware keypoint
set can be compared against the software one directly: FAST-9 with `nonmaxSuppression=true`,
`HARRIS_SCORE` and `FAST_SCORE` as in `ORBextractor.h`, `IC_Angle()` moments with the
ORB-SLAM3 `u_max` table, and the `EDGE_THRESHOLD` and 35-pixel cell grid from
`ORBextractor.cc`.

## Dataflow

```
        AXI4-Stream slave  (8-bit pixels, raster order, from AXI DMA MM2S)
                 |
                 v
          +--------------+
          |  window_NxN  |  31x31 sliding window, 30 line buffers, 1-cycle latency
          +--------------+
             |        |
             |        +--------------------+
             v                             v
      +--------------+             +-----------------+
      |  fast9_core  | latency 3   |     fast_nms    | latency 5
      |  detect only |             |  fast_response  |
      +--------------+             | + 3x3 local max |
             |                     +-----------------+
             |                             |
             +--------> push gate <--------+   corner AND local max AND
                            |                  not suppressed by the cell gate
                            v
                  +---------------------+
                  |   orientation_top   |
                  |  +---------------+  |
                  |  |  corner_fifo  |  |  31x31 patch + coords, URAM + BRAM,
                  |  +---------------+  |  zero-drop via backpressure
                  |  +---------------+  |
                  |  | moment_proc.  |  |  m10 / m01, 23 cycles
                  |  | harris_resp.  |  |  7x7 Sobel Harris, 19 cycles (G_SCORE_TYPE=0)
                  |  +---------------+  |
                  |  +---------------+  |
                  |  | cordic_atan2  |  |  22 iterations, 24-cycle latency
                  |  +---------------+  |
                  +---------------------+
                            |
                            v
                     +-------------+
                     |     TOP     |  AXI4-Lite registers, keypoint FIFO,
                     +-------------+  128-bit packing, EOF sentinel
                            |
                            v
        AXI4-Stream master (128-bit keypoints, to AXI DMA S2MM)
```

Everything inside `extractor_top` is aligned to `T + ALIGN_LAT` (5 cycles after the window
output). The sub-module latency constants are cross-checked by static asserts and by a
simulation-time self-check against the actual valid chains, so a latency change in one
module fails the next testbench run instead of silently misaligning the pipeline.

`window_NxN` receives the image dimensions on ports rather than generics, so a single
instance serves every pyramid level. Only the line-buffer depth is fixed at elaboration,
via `MAX_WIDTH`, sized for level 0.

Two mechanisms keep the corner FIFO from overflowing:

* **Backpressure.** `o_prog_full` is wired into the pixel-stream `tready` gate in `TOP`, so
  the input stalls before the FIFO can fill. Tail drops became impossible. The cost is
  measured in `STALLCNT` (`0x2C`), and `DROPCNT` (`0x18`) should stay at zero.
* **Strict-cell gate.** A permissive corner is discarded when the ORB-SLAM3 selection cell
  it falls into already holds a strict corner, because the per-cell selection on the
  software side would discard it anyway. The grid math replicates `ORBextractor.cc`
  (`EDGE_THRESHOLD = 19`, nominal cell width 35). Suppressed corners are counted in
  `SUPPCNT` (`0x28`). Writing zero to `CELLDIM` and `CELLNUM` leaves the gate inert.

## Repository layout

```
src/            synthesizable RTL
testbenches/    self-checking testbenches and their Python vector generators
vivado/         block design and packaged IP definitions
deploy/         device-tree overlay source and the xmutil app descriptor
```

### `src/`

| File | Role |
| --- | --- |
| `TOP.vhd` | AXI wrapper: AXI4-Lite register file, per-frame control FSM, output keypoint FIFO, 128-bit packing, EOF sentinel |
| `TOP_vivado_wrapper_vhdl_2002.vhd` | VHDL-93/2002 entity `orb_feature_top`, forwards generics and ports 1:1. UG1118 requires the designated top-level IP file to be plain VHDL |
| `extractor_top.vhd` | Level-3 extractor: window, detection, NMS, strict-cell gate, orientation |
| `window_NxN.vhd` | Sliding NxN window from a raster stream, (N-1) line buffers plus an NxN register array |
| `fast9_core.vhd` | FAST-9 decision on the 16-pixel Bresenham circle, 3-stage pipeline |
| `fast_response.vhd` | FAST score, equivalent to OpenCV `cornerScore`, 4-stage pipeline |
| `fast_nms.vhd` | 3x3 non-maximum suppression on the FAST score field, reusing the same 31x31 patch, so it costs no line buffers |
| `harris_response.vhd` | Harris response over the 7x7 block, matching OpenCV `HarrisResponses()` with `k = 0.04` |
| `corner_fifo.vhd` | Synchronous FIFO carrying the full orientation patch, circle-masked storage, UltraRAM plus BRAM |
| `moment_processor.vhd` | Pipelined `m10` and `m01` for the intensity centroid, bit-exact against the sequential reference |
| `orientation_top.vhd` | Drains the corner FIFO, runs moments and Harris in parallel, emits enriched keypoints |
| `cordic_atan2.vhd` | Pipelined vectoring-mode CORDIC, four-quadrant, same sign convention as `cv::fastAtan2()` |
| `axi_axcache_override.vhd` | Combinational shim forcing `AxCACHE = 1111` and `AxPROT = 010` so DMA traffic through `S_AXI_HPC0_FPD` is snooped by the CCI |
| `feature_pkg.vhd` | Universal types: `pixel_t`, `coord_t`, `patch_arr_t`, `ceil_log2`. Kept VHDL-93/2002 clean because the packaging wrapper references it |
| `fast_pkg.vhd` | FAST-specific types and circle-extraction helpers |
| `umax_pkg.vhd` | `u_max` half-width table for the circular orientation patch, built at elaboration exactly as ORB-SLAM3 builds it |

## AXI4-Lite register map

32-bit registers, byte offsets from the base address of the block.

| Offset | Name | Access | Contents |
| --- | --- | --- | --- |
| `0x00` | `CTRL` | RW | `[0]` enable, `[1]` soft reset |
| `0x04` | `STATUS` | RO | `[0]` busy, `[1]` done, `[2]` overflow, `[3]` cfg_error |
| `0x08` | `WIDTH` | RW | image width of the current pyramid level |
| `0x0C` | `HEIGHT` | RW | image height of the current pyramid level |
| `0x10` | `THRESH` | RO | `[7:0]` permissive threshold, `[15:8]` strict threshold, `[16]` score type, all as synthesized |
| `0x14` | `KPCOUNT` | RO | real keypoints emitted this frame |
| `0x18` | `DROPCNT` | RO | corner-FIFO drops, expected to stay 0 since the backpressure gate |
| `0x1C` | `ID` | RO | build ID, currently `0xC0DE0003` |
| `0x20` | `CELLDIM` | RW | `[15:0]` wCell, `[31:16]` hCell |
| `0x24` | `CELLNUM` | RW | `[15:0]` nCols, `[31:16]` nRows |
| `0x28` | `SUPPCNT` | RO | corners suppressed by the strict-cell gate |
| `0x2C` | `STALLCNT` | RO | cycles the pixel input stalled on corner-FIFO backpressure |

Per-frame sequence driven by the PS: write `WIDTH` and `HEIGHT` (and the cell grid) for the
level, set `CTRL[0]`, wait for `STATUS[1]`, clear `CTRL[0]`, repeat for the next level. The
control FSM holds the core in reset while idle, so every level starts from clean counters.

`ID` is the compatibility contract with the driver, and the keypoint layout below is coupled
to it. Bump the build ID whenever the layout changes. `THRESH[16]` lets the driver verify at
probe time that the loaded bitstream was built with the score type it expects.

## Keypoint word format (v3, 128 bit, LSB-first)

| Word | Bits | Field |
| --- | --- | --- |
| 0 | `[15:0]` | x (11 bits used) |
| 0 | `[31:16]` | y (11 bits used) |
| 1 | `[63:32]` | response, int32. FAST score 0..255 zero-extended when `G_SCORE_TYPE=1`, signed Harris response when `G_SCORE_TYPE=0` |
| 2 | `[87:64]` | angle, 24-bit two-complement, `+2^22` corresponds to `+pi` |
| 2 | `[88]` | is_brighter |
| 2 | `[89]` | passed_strict |
| 2 | `[95:90]` | reserved, zero |
| 3 | `[127:96]` | `0x00000000` for a real keypoint, `0xFFFFFFFF` for the end-of-frame sentinel |

The dual-threshold scheme mirrors the `iniThFAST` and `minThFAST` adaptation in ORB-SLAM3.
Detection always runs at the permissive threshold, and each keypoint carries `passed_strict`,
so the software reproduces the two-pass behavior without a second hardware pass. Detection
and NMS are FAST-based in both scoring modes, so the corner *set* never depends on
`G_SCORE_TYPE`. Only the ranking value does.

## Simulation

The testbenches are self-checking and were developed under ModelSim and QuestaSim. They also
analyze and run under GHDL. Compile the packages first (`feature_pkg`, `fast_pkg`,
`umax_pkg`), then the module under test, then its testbench.

The testbenches open their vector files by paths relative to the repository root, for example
`testbenches/cordic_vectors.hex`, so **run the simulator from the repository root** and the
generators from inside `testbenches/`.

The stimulus and golden-output files (`*.hex`, `*expected*.txt`) are **not** in the
repository. They amount to roughly 78 MB of generated data and are reproduced from the
Python generators that are tracked here. Each generator is the golden model for its module,
so generator and testbench always belong together.

```bash
python -m venv .venv
```

```bash
pip install -r testbenches/0_requirements.txt
```

The generators are golden models, so `0_requirements.txt` pins exact versions rather than
leaving them open: a change in numpy integer or rounding behavior would alter the golden
data silently instead of failing. The thesis vectors were produced with numpy 2.4.6 and
tqdm 4.67.3 on CPython 3.14.5. Only numpy and tqdm are listed, because the remaining
imports are standard library modules that pip has nothing to install for. The leading `0`
in the filename exists purely to sort it above the files it describes.

```bash
cd testbenches && python window_NxN_generator.py && python fast9_core_generator.py && python fast_response_generator.py && python fast_nms_generator.py && python harris_response_generator.py && python moment_processor_generator.py && python cordic_atan2_generator.py && python orientation_top_generator.py && python extractor_top_generator.py && python pyramid_tb_generator.py && python top_tb_generator.py
```

Every generator takes `-h`. Image size, seed, thresholds and score type are parameters, for
example `python extractor_top_generator.py -W 256 -H 224 --score-type harris`. Each one also
writes a `*_test_params_pkg.vhd`, which *is* tracked, so the parameters a given run used
remain visible in the repository even without the vectors.

| Testbench | Covers |
| --- | --- |
| `window_NxN_tb.vhd` | window and line-buffer behavior, including line wraps |
| `fast9_core_tb.vhd` | FAST-9 decision against the software oracle |
| `fast_response_tb.vhd` | FAST score against OpenCV `cornerScore` |
| `fast_nms_tb.vhd` | 3x3 suppression, including tie handling |
| `harris_response_tb.vhd` | Harris response against `HarrisResponses()` |
| `moment_processor_tb.vhd` | `m10` and `m01` bit-exactness |
| `cordic_atan2_tb.vhd` | angle error against `fastAtan2` over all four quadrants |
| `orientation_top_tb.vhd` | FIFO drain, moment and CORDIC handshakes, keypoint output |
| `corner_fifo_tb.vhd` | fill, drain, programmable-full and drop behavior |
| `extractor_top_tb.vhd` | end-to-end extractor on a synthetic image |
| `extractor_pyramid_tb.vhd` | one instance driven across all pyramid levels |
| `TOP_tb.vhd` | AXI4-Lite transactions, per-frame FSM, packing and EOF sentinel |
| `umax_pkg_tb.vhd` | elaboration-time `u_max` table |

`extractor_top_tb` and `extractor_pyramid_tb` take the expected-output file as a generic, so
the same testbench covers both scoring modes:

```bash
-gG_USE_HARRIS=true -gG_EXPECTED_FILE=testbenches/extractor_expected_harris.txt
```

`fast_score_consistency.py` cross-checks the FAST score model against the detection model
and is not tied to a single testbench.

## Synthesis

The RTL is added to the Vivado block design by path as a module reference, so `src/` stays
the single source of truth and there is no mirrored copy inside the Vivado project.

When packaging as a custom IP the file types matter. UG1118 requires the designated
top-level file to be plain Verilog or VHDL-93/2002, which is why
`TOP_vivado_wrapper_vhdl_2002.vhd` (entity `orb_feature_top`) exists. Mark that file and
`feature_pkg.vhd` as VHDL, everything else as VHDL 2008.

`G_CORE_URAM_COLS = 32` gives 32 URAM plus 52 BRAM36 per instance, sized so two instances
fit the K26. `G_KP_FIFO_DEPTH` must be a power of two, because the pointer wrap relies on it,
and `G_PROG_FULL_GAP` must be at least `G_CORE_FIFO_DEPTH` plus the pipeline tail. Both
constraints are checked by asserts at elaboration.

### `vivado/`

Only the parts of the Vivado project that a build cannot regenerate. Synthesis runs,
implementation checkpoints, the IP cache and the generated output products are excluded.

| File | Role |
| --- | --- |
| `create_project.tcl` | recreates the project: part `xck26-sfvc784-2LV-c`, board `xilinx.com:kr260_som:part0:1.1`, filesets, run settings. Written with `write_project_tcl -no_copy_sources -use_bd_files`, so it references `../src` and `../testbenches` rather than copying them |
| `HW_Acc.bd` | the block design: two accelerator instances, their AXI DMAs on separate PS HP ports, the SmartConnects, the AxCACHE shim and the PS configuration. Authoritative |
| `HW_Acc_bd.tcl` | the same block design exported as Tcl by `write_bd_tcl`. Not used by the flow, kept so that block-design changes are reviewable as a text diff. Re-export it whenever the block design changes |
| `reports_post_route.tcl` | post-route report hook: timing, utilization and the DSP count that distinguishes a FAST build from a Harris one. Also runs standalone against an archived routed checkpoint |

To rebuild:

```bash
vivado -mode batch -source vivado/create_project.tcl
```

Two environment prerequisites, neither of which is a file in this repository. The project
was written by Vivado 2025.2.1 and uses the `Vivado Synthesis 2025` and
`Vivado Implementation 2025` flows, and `HW_Acc_bd.tcl` refuses to run on anything whose
version string does not contain `2025.2`. The board part `xilinx.com:kr260_som:part0:1.1`
requires the Kria board files to be installed.

Nothing else is needed. The constraint set is empty, since the design is entirely
PS-clocked with no PL pins of its own, and every IP in the block design comes from the
standard Vivado catalog, so no external IP repository has to be registered.

### A rebuild writes back into this folder

The project references `HW_Acc.bd` in place rather than copying it, so Vivado treats this
folder as its working directory for the block design. After a run you will find:

| Path | What |
| --- | --- |
| `vivado/HW_Acc.bd` | rewritten with the elaborated state, roughly 146 KB to 205 KB |
| `vivado/HW_Acc.bda` | block-design layout file, ignored by git |
| `vivado/ip/` | regenerated module-reference IP definitions, ignored by git |
| `vivado/Vivado-HW-Acc.gen/` | generated output products, ignored by git |

Only the first one is tracked, so the others stay invisible. Discard the rewritten block
design unless you actually changed it, which keeps the committed file the small, readable
one rather than a Vivado re-serialization:

```bash
git checkout -- vivado/HW_Acc.bd
```

This matters most after a *failed* run, which can leave the file half-written.

The block design instantiates `orb_feature_top` and `axi_axcache_override` as module
references, so Vivado re-infers their IP definitions from `src/` when the block design is
generated. `src/` stays the single source of truth and the project holds no copy of it.

`reports_post_route.tcl` runs automatically at the end of `route_design`. To run it
standalone against an already routed checkpoint, pass the configuration and the checkpoint:

```bash
vivado -mode batch -source vivado/reports_post_route.tcl -tclargs HARRIS /path/to/HW_Acc_wrapper_routed_HARRIS.dcp
```

Checkpoints live outside this repository, so the script has no default location for them.
Either give the `.dcp` path as the second argument, as above, or set `ARCHIVE_ROOT` in the
environment and the script derives `$ARCHIVE_ROOT/$CFG/HW_Acc_wrapper_routed_$CFG.dcp`.
Verify the configuration afterwards from the DSP count in the hierarchical utilization
report: 2 for a FAST build, 60 for Harris.

## `deploy/` : loading the accelerator on the KR260

`xmutil loadapp` reads one firmware directory, `/lib/firmware/xilinx/HW_Acc/`, and expects
three files in it:

| File | Origin |
| --- | --- |
| `shell.json` | `deploy/shell.json`, tracked here |
| `HW_Acc.dtbo` | compiled from `deploy/HW_Acc.dts`, tracked here |
| `HW_Acc_wrapper.bit.bin` | produced from the Vivado bitstream, see below |

The bitstream itself is a build product and is not in this repository, but the step that
converts it is. Copy `HW_Acc_wrapper.bit` out of the implementation run into `deploy/`,
then run bootgen against the tracked image description `HW_Acc_wrapper.bif`:

```bash
bootgen -image HW_Acc_wrapper.bif -arch zynqmp -process_bitstream bin -w -o HW_Acc_wrapper.bit.bin
```

The output name matters: it is what the `firmware-name` property in `HW_Acc.dts` looks for.

`shell.json` is the app descriptor. `XRT_FLAT` with one slot tells `xmutil` this is a flat
(non-DFX) design occupying the whole PL.

`HW_Acc.dts` is the device-tree overlay. It loads the bitstream named by its `firmware-name`
property, sets the PL clock to 125 MHz, and exposes the memory-mapped register files to
userspace as `generic-uio` nodes. It is written for the dual-accelerator build, and because
the second instance is simply appended, the same overlay also serves a single-instance
bitstream. The unused instance-1 nodes just sit idle.

```bash
dtc -@ -O dtb -o HW_Acc.dtbo HW_Acc.dts
```

`-@` keeps the symbol table the overlay needs to resolve `&fpga_full` and `&amba`.

Expected UIO probe order, assuming the base platform occupies `uio0` through `uio3`:

| Node | Address | Purpose |
| --- | --- | --- |
| `/dev/uio4` | `0xA000_0000` | instance 0, AXI DMA registers |
| `/dev/uio5` | `0xA001_0000` | instance 0, `orb_feature_top` registers |
| `/dev/uio6`, `/dev/uio7` | SPI 89, 90 | instance 0 DMA interrupts, registered only so they stay masked |
| `/dev/uio8` | `0xB000_0000` | instance 1, AXI DMA registers |
| `/dev/uio9` | `0xB001_0000` | instance 1, `orb_feature_top` registers |
| `/dev/uio10`, `/dev/uio11` | SPI 91, 92 | instance 1 DMA interrupts, masked |

Verify the numbering on the board before relying on it:

```bash
for d in /sys/class/uio/uio*; do echo "$d -> $(cat $d/name)"; done
```

The driver polls `DMASR` and never opens the interrupt nodes. They exist only so that
`uio_pdrv_genirq` registers and masks the wired PL-to-PS lines, because an unhandled
level-high SPI would otherwise storm.

The pixel and keypoint buffers are not device-tree nodes. They are created by loading the
out-of-tree `u-dma-buf` module, one input and one output buffer per instance:

```bash
sudo insmod u-dma-buf.ko udmabuf0=1048576 udmabuf1=1048576 udmabuf2=1048576 udmabuf3=1048576
```

## Build ID history

| ID | Change |
| --- | --- |
| `0xC0DE0003` | keypoint packing v3 (32-bit response in word 1, 24-bit angle plus flags in word 2), `G_SCORE_TYPE` generic, `THRESH[16]` readback |
| `0xC0DE0002` | `STALLCNT` register at `0x2C`, corner-FIFO backpressure and the UltraRAM split |
