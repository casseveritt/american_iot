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
