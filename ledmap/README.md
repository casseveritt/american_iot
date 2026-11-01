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
  majority vote, ID lookup (Python + OpenCV)
- `solve/`     — `layout2d.py` (2D positions -> hween layout); later calibration,
  triangulation, bundle adjustment, validation
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
