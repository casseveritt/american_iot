# starkville tree, view C016 (2026-11-10)

One 9.3 s clip (4K, 60 fps) of the built tree, one strand not installed yet; sequence
`idseq16 -n 300 --slow 1 --scale 0.25` (100 ms symbols, 2.5 s cycle, 3 cycles in the clip).

- Extract: `extract.py A001_11101647_C016.mov cache_c016 --crop 1450,220,920,1740` (full-res tree
  crop) and `extract.py ... lowres_c016 --width 960` (whole frame, for registration).
- Decode: `decode.py cache_c016 codes_mA_s1_n4800.csv --symbol-ms 100 --lowres lowres_c016
  --bg 20,150,380,540 --cycles 3 --core-r 1.5 --ring 3,5 --dog 1.5,4 --peak-win 5 --blob-thresh 0.1
  --norm-white --features ring --kmeans-init --gap-min 6 --max-dist 3` -> 734 of 1171 blobs identified,
  strips 0,2,3,4,5,8,10-13 (strip 14: 4 stray hits, dropped).
- Layout: `layout2d.py positions2d.csv -o layout.csv --fold --leds 300 --max-gap 8 --no-extrap
  --tol-px 10 --z-frac 0.25 --min-leds 10 --axis 417.8,300,466.2,1700` -> 1820 pixels.
  `--axis` is two points on the trunk (the lit pole, fitted from the all-white average; it leans 2 deg in
  the image): the layout's y axis runs along it and x is distance from it, so the trunk is x = 0.

The strands are folded in half, so LED i and LED 299-i are at the same place facing opposite
ways; a view sees one of each pair, and both get that position (`source` = mirror). Gaps of up to
8 positions are interpolated; nothing is extrapolated, so everything else stays dark in hween.
Single view: z is a constant plane at a quarter of the x width.
