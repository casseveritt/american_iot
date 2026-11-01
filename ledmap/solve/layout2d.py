#!/usr/bin/env python3
"""Turn decoded 2D LED positions into a layout file for hween.

    layout2d.py positions2d.csv -o layout.csv [--leds N] [--pitch-m 0.016667]

positions2d.csv (from detect/decode.py) has one row per decoded LED with its
pixel position in one view. The strands are close to straight, but perspective
changes the pixel pitch along them, so each strip is cleaned of outliers
against a quadratic fit. Then every LED index 0..N-1 gets a position: decoded
LEDs keep theirs, gaps are interpolated and the ends are extrapolated along the
strand. Strips with no decoded LEDs (dead or not
connected) are left out, and hween does not light pixels that are not in the
layout.

Output is meters, y up, with x centered on the mapped LEDs and y = 0 at the
lowest one. z is 0: a single view has no depth. The `source` column says where
each position came from (decoded, interp or extrap).
"""
import argparse
import csv
import sys
from collections import defaultdict

import numpy as np


def fit(leds, xy):
    """Quadratic in the LED index for x and y: strands bend a little and the
    LED pitch changes along them with perspective."""
    deg = 2 if len(leds) >= 5 else 1
    return np.polyfit(leds, xy[:, 0], deg), np.polyfit(leds, xy[:, 1], deg)


def clean(leds, xy, tol_px):
    """Drop points that are far from a robust fit through the strip."""
    keep = np.ones(len(leds), bool)
    for _ in range(4):
        if keep.sum() < 3:
            break
        px, py = fit(leds[keep], xy[keep])
        res = np.hypot(np.polyval(px, leds) - xy[:, 0], np.polyval(py, leds) - xy[:, 1])
        keep = res < tol_px
    return keep


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("positions")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--leds", type=int, default=0,
                    help="LEDs per strip to fill (default: highest decoded index + 1)")
    ap.add_argument("--pitch-m", type=float, default=0.016667,
                    help="distance between neighboring LEDs in meters (default 1/60 m)")
    ap.add_argument("--tol-px", type=float, default=8.0,
                    help="outlier distance from a strip's fitted curve (default 8 px)")
    args = ap.parse_args()

    by = defaultdict(list)
    for r in csv.DictReader(open(args.positions)):
        by[int(r["strip"])].append((int(r["led"]), float(r["x"]), float(r["y"])))
    if not by:
        sys.exit("no decoded LEDs in " + args.positions)
    nled = args.leds or max(l for v in by.values() for l, _, _ in v) + 1

    pos = {}  # (strip, led) -> (x, y, source) in pixels
    pitches = []
    for strip, pts in sorted(by.items()):
        pts = np.array(sorted(pts), float)
        leds, xy = pts[:, 0], pts[:, 1:]
        keep = clean(leds, xy, args.tol_px)
        dropped = int((~keep).sum())
        leds, xy = leds[keep], xy[keep]
        if len(leds) < 2:
            print("strip %d: only %d usable LEDs, skipped" % (strip, len(leds)))
            continue
        px, py = fit(leds, xy)
        mid = (leds.min() + leds.max()) / 2
        pitches.append(np.hypot(np.polyval(np.polyder(px), mid), np.polyval(np.polyder(py), mid)))
        counts = {"decoded": 0, "interp": 0, "extrap": 0}
        lo, hi = leds.min(), leds.max()
        for i in range(nled):
            if i in leds:
                j = int(np.nonzero(leds == i)[0][0])
                pos[(strip, i)] = (xy[j, 0], xy[j, 1], "decoded")
                counts["decoded"] += 1
            elif lo < i < hi:
                pos[(strip, i)] = (np.interp(i, leds, xy[:, 0]), np.interp(i, leds, xy[:, 1]), "interp")
                counts["interp"] += 1
            else:
                pos[(strip, i)] = (np.polyval(px, i), np.polyval(py, i), "extrap")
                counts["extrap"] += 1
        print("strip %d: %d decoded, %d outliers dropped, %d interpolated, %d extrapolated (leds %d..%d)" %
              (strip, counts["decoded"], dropped, counts["interp"], counts["extrap"], lo, hi))

    px_per_led = float(np.median(pitches))
    m_per_px = args.pitch_m / px_per_led
    xs = np.array([p[0] for p in pos.values()])
    ys = np.array([p[1] for p in pos.values()])
    cx0, ybot = (xs.min() + xs.max()) / 2, ys.max()
    print("LED pitch %.1f px -> %.3f mm/px; %d pixels in %d strips" %
          (px_per_led, m_per_px * 1e3, len(pos), len({k[0] for k in pos})))

    with open(args.out, "w") as f:
        f.write("# hween layout: strip,led,x,y,z in meters (y up, z unknown = 0)\n")
        f.write("# from %s; %d LEDs/strip, pitch %.4f m = %.1f px\n" %
                (args.positions, nled, args.pitch_m, px_per_led))
        f.write("strip,led,x,y,z,source\n")
        for (strip, led), (x, y, src) in sorted(pos.items()):
            f.write("%d,%d,%.4f,%.4f,%.4f,%s\n" %
                    (strip, led, (x - cx0) * m_per_px, (ybot - y) * m_per_px, 0.0, src))


if __name__ == "__main__":
    main()
