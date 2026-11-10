# Run 5053

First decoded run, 2026-10-31. Single view, 8 strands (data line 6 was dead and
left out, so strip 6 has no LEDs).

- Sequence: `sudo ./idseq16 -n 300 --slow 4 --scale 0.25` (mode A, seed 1,
  payload 10, check 2, min distance 4; 400 ms symbols), driver brightness 25.
- Clip: iPhone, 4K HLG, handheld, 17 s. One complete cycle at t0 = 4.5 s; the
  slots change length after ~15 s, so only that first cycle is used.
- Code table: `idseq_host -s 16 -n 300 -P 1 -D codes.csv`.
- Decode: `extract.py clip.MOV cache`, then
  `decode.py cache codes.csv --t0 4.5` -> `positions2d.csv` (185 of 322 blobs).
- Layout: `solve/layout2d.py positions2d.csv -o layout.csv --z-frac 0.25` (plane offset in z by a quarter of the x width) -> 304 pixels
  (decoded ones kept, gaps interpolated, ends extrapolated), z = 0.

Use it: `./hween --layout ../ledmap/results/5053/layout.csv`

`overlay.png` shows the detected blobs: green = decoded, red = not.
