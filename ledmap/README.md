# ledmap

Identify each LED on the SMI strips (strip index + position along the strip)
and recover its 3D position, so shaders can work in real space instead of
strip/index space.

See [PLAN.md](PLAN.md) for the full design. In short: every LED blinks a
unique, randomized color-coded ID; short iPhone clips from several stationary
viewpoints are decoded to label each visible LED; the labeled 2D observations
are triangulated into 3D positions. Anything ambiguous is dropped.

## Layout

- `sequence/`  — `idseq`: generates the seeded random codes (min Hamming
  distance) and plays preamble/marker/payload/check/gap (C++, built like
  `../ledtest`)
- `detect/`    — video decoder: blob detection, tracking, color classification,
  majority vote, ID lookup (Python + OpenCV); `treebox.py`, `process_clips.sh` run extract+decode per clip
- `solve/`     — `layout2d.py` (single view 2D -> hween layout), `sfm.py` (joint camera + 3D light
  solve from several views), `fillsym.py` / `fillstruct.py` / `fill3d.py` (fill unidentified LEDs),
  `unseen.py` (LEDs missing from a layout, for `idseq --only`)
- `data/`      — clips, frame caches, code tables (not committed)
- `results/`   — one directory per run: positions, hween layout, how it was made

## First milestone

One strand of 50-100 LEDs, one viewpoint, full encode-record-decode loop;
measure the decode rate and tune color set, symbol duration and code length.

## Status (2026-10-31)

Done:
- `sequence/idseq` encodes and plays the ID sequence (mode A: 8 colors, and
  mode B: 4 colors; Manchester mode C from the plan is not implemented).
  Seeded random codes with a minimum Hamming distance, plus check symbols.
  `--dump FILE` writes the code table for the decoder.
- Runs on ho.local on all 16 strips. Options: `--fps` (default 60),
  `--slow` (symbol time multiplier), `--scale` (color value multiplier),
  `--test-colors`, `--print`.
- Frames are only sent to the strips when their data changes. Re-sending the
  same frame every tick showed intermittent corruption at the same LED index
  on every strand (a shared DMA/SMI glitch); sending on change made it go away.

Working capture setup: `sudo ./idseq16 -n 300 --slow 4 --scale 0.25`, phone at
lowest ISO and 1/500 s shutter. Blobs have proper colors and all 8 are easy to
tell apart. Higher `--scale` or slower shutters clip the LED cores to white.

Things to know:
- Stop `hween` before running; two programs driving the SMI output fight.
- The code table is regenerated from `--seed`, `--mode`, `--payload`,
  `--check` and `--min-dist`, so keep those the same between a capture and its
  decode (or save the `--dump` file with the clip).

Decoding (first clip, `results/5053`): `detect/extract.py` caches a clip's
frames, `detect/decode.py` registers them (the phone drifts), finds the LED
blobs, reads each LED's color per slot from its core and a ring around it, and
matches the 12 symbols to the code table, refitting the color classes from
confident matches. 185 of 322 blobs decoded, and neighbouring ids land next to
each other, which the decoder never used. The rest are limited by clipped cores,
a too-blue white balance and having only one cycle (no voting).

Using a result in hween: `solve/layout2d.py positions2d.csv -o layout.csv`, then
`./hween --layout layout.csv`. Pixels not in the layout stay dark, so a dead data
line just has no rows. A new run is a new `results/<name>/` and a new file.

Next:
- A better clip (neutral white balance, lower exposure, no HDR, 3+ cycles, tripod).
- Several views and `solve/`: calibration, triangulation (z is 0 for now).

## Status (2026-11-11): the Starkville tree is mapped

`results/starkville-sfm2/layout_sym.csv` maps all 12 strips (3600 LEDs); see its README. Workflow:
1. `idseq16 -n 300 --slow 1 --scale 0.25` (all lights, or `--only FILE` with `strip,led` lines to light a subset);
   record ~10 s clips (3+ cycles) from several angles, whole tree in frame, phone steady.
2. In `data/`: `../detect/process_clips.sh DIR clip.mov ...` (crop box, frame caches, decode).
3. `solve/sfm.py -o OUT out_cNNN:cache_cNNN ...` (no assumed view order; needs shared LEDs between views, so lit
   subsets must overlap earlier views), then `solve/fillsym.py OUT/points3d_all.csv -o layout.csv`.
4. `./hween --layout layout.csv` (`--unmapped-red PCT` shows unmapped pixels as dim red).

Findings: the strands are folded in half (LED i and 299-i share a position, facing opposite ways); the tree is
12 identical meridian strips 30 deg apart. Known problem: about two thirds of clips decode to nearly nothing
(0-10 ids), cause not found; the ones that work give 300-850 ids each. Keep earlier layouts under `results/`.
