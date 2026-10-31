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
- `solve/`     — calibration, triangulation, bundle adjustment, validation
- `data/`      — clips and results (not committed if large)

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

Next:
- `detect/`: sync from the preamble/marker, find and track blobs, classify
  colors from a ring around the blob peak (the center clips), vote across
  cycles, verify check symbols, look up the id.
- A short test clip in `data/` to develop against.
- `solve/`: calibration, triangulation, validation.
