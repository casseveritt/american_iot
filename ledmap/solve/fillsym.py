#!/usr/bin/env python3
"""Fill every unidentified position using the tree's symmetry.

    fillsym.py points3d_all.csv -o layout.csv [--spokes 12]

The measurements show that each strip is a vertical meridian: all strips follow the same
(distance from trunk r, height y) profile along the strand, rotated about the trunk by a
multiple of 360/spokes degrees (the mean azimuth of every strip is within a few degrees of that grid).
So the master profile (median over strips, per fold position k) plus the strip's azimuth predicts
any position. Triangulated positions are kept as measured; everything else (ray-cast or never
seen) comes from the master profile. Strips with too few measurements for an azimuth are left out.
Single-view ray-cast points are not used.
"""
import argparse
import csv
from collections import defaultdict

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("points")
ap.add_argument("-o", "--out", required=True)
ap.add_argument("--leds", type=int, default=300)
ap.add_argument("--spokes", type=int, default=12)
ap.add_argument("--min-pts", type=int, default=40, help="triangulated positions a strip needs to define the profile")
ap.add_argument("--min-az", type=int, default=5, help="triangulated positions (beyond 12 cm from the trunk) to fix a strip's azimuth")
a = ap.parse_args()
nk = a.leds // 2

T = defaultdict(dict)  # triangulated: strip -> k -> (x, y, z, views)
for r in csv.DictReader(open(a.points)):
    if r["nviews"] != "ray":
        T[int(r["strip"])][int(r["k"])] = (float(r["x"]), float(r["y"]), float(r["z"]), int(r["nviews"]))

# azimuth of each strip, snapped to the grid whose offset fits best
az = {}
for s, kn in T.items():
    P = np.array([v[:3] for v in kn.values()])
    m = np.hypot(P[:, 0], P[:, 2]) > 0.12
    if m.sum() >= a.min_az:
        Q = P[m]
        az[s] = np.angle(np.sum(np.exp(1j * np.arctan2(Q[:, 0], Q[:, 2])) * np.hypot(Q[:, 0], Q[:, 2])))
step = 2 * np.pi / a.spokes
offs = np.linspace(0, step, 721)
best = min(offs, key=lambda o: sum(abs(np.angle(np.exp(1j * (v - o - step * round((v - o) / step))))) ** 2 for v in az.values()))
snap = {s: best + step * round((v - best) / step) for s, v in az.items()}
used = {}
for s, v in sorted(snap.items()):
    used.setdefault(round(np.degrees(v)) % 360, []).append(s)
print("strip azimuths (deg, snapped):", {k: v for k, v in sorted(used.items())})
print("measured vs grid, max error %.1f deg" % max(abs(np.degrees(np.angle(np.exp(1j * (az[s] - snap[s]))))) for s in az))

# master (r, y) profile per k, from strips with enough measurements
prof = defaultdict(list)
for s, kn in T.items():
    if s in snap and len(kn) >= a.min_pts:
        for k, (x, y, z, _) in kn.items():
            prof[k].append((np.hypot(x, z), y))
ks = sorted(k for k, v in prof.items() if len(v) >= 3)
M = np.array([np.median(prof[k], 0) for k in ks])
ky = np.arange(nk)
Mr, My = np.interp(ky, ks, M[:, 0]), np.interp(ky, ks, M[:, 1])   # k without a profile: interpolate
print("profile from %d strips; %d of %d k interpolated" % (len({s for s in T if s in snap and len(T[s]) >= a.min_pts}), nk - len(ks), nk))

n = counts = 0
src = defaultdict(int)
with open(a.out, "w") as fh:
    fh.write("# hween layout: measured points + symmetry (fillsym.py, %d spokes)\nstrip,led,x,y,z,source\n" % a.spokes)
    for s in sorted(snap):
        for k in range(nk):
            if k in T[s]:
                x, y, z, v = T[s][k]; tag = "sfm%d" % v
            else:
                x, y, z = Mr[k] * np.sin(snap[s]), My[k], Mr[k] * np.cos(snap[s]); tag = "sym"
            src[tag[:3]] += 1
            for i in sorted({k, a.leds - 1 - k}):
                fh.write("%d,%d,%.4f,%.4f,%.4f,%s\n" % (s, i, x, y, z, tag)); n += 1
print(n, "LEDs written;", dict(src), "fold positions")
