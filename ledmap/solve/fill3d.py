#!/usr/bin/env python3
"""Fill gaps in an sfm.py layout: fold positions between two solved ones on a strip
are interpolated in 3D (up to --max-gap). Writes a new layout.csv."""
import argparse
import csv
from collections import defaultdict

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("points3d")
ap.add_argument("-o", "--out", required=True)
ap.add_argument("--leds", type=int, default=300)
ap.add_argument("--max-gap", type=int, default=8)
a = ap.parse_args()

by = defaultdict(dict)
for r in csv.DictReader(open(a.points3d)):
    by[int(r["strip"])][int(r["k"])] = (float(r["x"]), float(r["y"]), float(r["z"]), ("sfm" + r["nviews"]) if r["nviews"].isdigit() else r["nviews"])
pos = {}
for s, d in by.items():
    ks = sorted(d)
    for k in ks:
        pos[(s, k)] = d[k]
    for p, q in zip(ks, ks[1:]):
        if 1 < q - p <= a.max_gap + 1:
            for k in range(p + 1, q):
                f = (k - p) / (q - p)
                pos[(s, k)] = tuple(np.add((1 - f) * np.array(d[p][:3]), f * np.array(d[q][:3]))) + ("interp",)
n = 0
with open(a.out, "w") as fh:
    fh.write("# hween layout: sfm.py points + 3D gap interpolation (max-gap %d)\nstrip,led,x,y,z,source\n" % a.max_gap)
    for (s, k), p in sorted(pos.items()):
        for i in sorted({k, a.leds - 1 - k}):
            fh.write("%d,%d,%.4f,%.4f,%.4f,%s\n" % (s, i, *p)); n += 1
print(len(pos), "positions,", n, "LEDs")
