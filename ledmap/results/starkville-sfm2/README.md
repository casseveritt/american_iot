# starkville tree, joint multi-view solve + symmetry fill (2026-11-11)

The tree has 12 strips (outputs 0-5, 8, 10-14); outputs 6, 7, 9 and 15 are not on it. 4K 60 fps
clips of the full pattern (`idseq16 -n 300 --slow 1 --scale 0.25`) from six stationary viewpoints.

**Result: `layout_sym.csv`, 3600 LEDs (12 strips x 300), the one to use in hween.**

## How it was made

1. Decode each clip (`detect/process_clips.sh`, run in `data/`; same settings as `../starkville-c016`).
   Of the 14 clips, six decoded (C016 735 ids, C018 310, C019 841, C021 335, C029 478, C034 656).
   The rest give 0-10 ids; the cause is unknown (C017's colour readings don't repeat between cycles).
2. `solve/sfm.py -o sfm out_c016:cache_c016 out_c018:cache_c018 out_c019:cache_c019 out_c021:cache_c021
   out_c029:cache_c029 out_c034:cache_c034` solves camera poses and 3D LED positions together: no
   assumed order, shared focal length (2736 px), essential-matrix start pair, PnP, bundle adjustment.
   Scale from the 1/60 m LED pitch, y axis (trunk) from the tree's rotational symmetry.
   970 fold positions triangulated, 1.5 px RMS (0.76 median); tree 1.58 m high, 0.37 m radius.
   Also ray-casts LEDs seen in only one view onto the surface of revolution (`points3d_all.csv`, source `ray`).
   `cameras.csv` gives camera positions in the tree frame (azimuth is arbitrary: not comparable between solves).
3. `solve/fillsym.py points3d_all.csv -o layout_sym.csv` fills the rest from symmetry (below).

## The tree's structure

Each strip is a vertical meridian. All strips follow the same (distance from trunk r, height y) profile
along the strand (k = 0..149 = fold position, LED i and 299-i share k = min(i, 299-i); k = 0 is the
bottom, y = 0), rotated about the trunk by a multiple of 30 degrees. Azimuths (degrees, strip):
`0:0  2:30  3:60  4:90  5:120  8:150  10:180  13:210  11:240  12:270  14:300  1:330` (measured within 3 deg
of the grid). `fillsym.py` keeps triangulated points as measured and gives every other position
(median profile over strips, rotated to the strip's azimuth).
Leave-one-strip-out error of the profile: median 0.8 cm, 90th percentile 1.5 cm.

## Files

- `layout_sym.csv`: final layout (source `sfmN` = triangulated from N views, `sym` = symmetry)
- `layout_struct.csv`: older fill (`solve/fillstruct.py`: per-strip curve bridging; 0.4-1 cm median but 5-10 cm
  at the 90th percentile)
- `layout.csv`: triangulated points only (1940 LEDs); `points3d.csv`, `points3d_all.csv` (with ray-cast), `cameras.csv`
- Earlier versions are kept: `../starkville-c016` (single view, 1820 LEDs) and `../starkville-sfm` (4 views).

## Using it

`./hween --layout ledmap/results/starkville-sfm2/layout_sym.csv --prog twist` (stop idseq first).
`--unmapped-red 10` shows every unmapped pixel as 10% red, to see where they are.
