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

With --fold, each strip is a strand doubled back on itself: LED i and LED
leds-1-i sit at the same spot, facing opposite ways, so one view usually sees
only one of the pair. Both LEDs get the position found for either. Gaps are
filled only up to --max-gap positions and nothing is extrapolated with
--no-extrap, so LEDs far from any identified one stay out of the layout and dark.

Output is meters, y up, with x centered on the mapped LEDs and y = 0 at the
lowest one. z is constant (0, or --z-frac times the x width): a single view has no depth. The `source` column says where
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


def clean_local(k, xy, tol_px, reach=6):
    """Fold mode: strands are not a single smooth curve (they bend at the hoops), so
    judge each point against a line through its neighbors within `reach` positions."""
    keep = np.ones(len(k), bool)
    while True:
        worst, wi = 0.0, -1
        for i in np.nonzero(keep)[0]:
            nb = keep & (np.abs(k - k[i]) <= reach) & (np.arange(len(k)) != i)
            if nb.sum() < 3 or np.ptp(k[nb]) == 0:
                continue
            e = [abs(np.polyval(np.polyfit(k[nb], xy[nb, a], 1), k[i]) - xy[i, a]) for a in (0, 1)]
            r = float(np.hypot(*e))
            if r > worst:
                worst, wi = r, i
        if worst <= tol_px:
            return keep
        keep[wi] = False


def fold_strip(strip, pts, nled, args, pos, pitches):
    """Fold mode: one position per fold position k = min(i, nled-1-i), given to both LEDs."""
    pts = np.array(sorted(pts), float)
    k = np.minimum(pts[:, 0], nled - 1 - pts[:, 0])
    xy, led = pts[:, 1:], pts[:, 0]
    if len(k) < args.min_leds:
        print("strip %d: only %d identified LEDs, skipped" % (strip, len(k)))
        return
    keep = clean_local(k, xy, args.tol_px)
    dropped = int((~keep).sum())
    k, xy, led = k[keep], xy[keep], led[keep]
    if len(k) < args.min_leds:
        print("strip %d: only %d usable LEDs, skipped" % (strip, len(k)))
        return
    uk = np.unique(k)
    ux = np.array([xy[k == v].mean(0) for v in uk])  # both faces identified: average
    for a in range(len(uk) - 1):
        if uk[a + 1] - uk[a] == 1:
            pitches.append(float(np.hypot(*(ux[a + 1] - ux[a]))))
    place = {int(v): (p[0], p[1], "decoded") for v, p in zip(uk, ux)}
    ninterp = 0
    for a in range(len(uk) - 1):
        gap = int(uk[a + 1] - uk[a]) - 1
        if gap > 0 and (args.max_gap == 0 or gap <= args.max_gap):
            for v in range(int(uk[a]) + 1, int(uk[a + 1])):
                p = ux[a] + (v - uk[a]) / (uk[a + 1] - uk[a]) * (ux[a + 1] - ux[a])
                place[v] = (p[0], p[1], "interp")
                ninterp += 1
    if not args.no_extrap:
        print("strip %d: --fold extends nothing past the identified LEDs" % strip)
    seen = set(int(i) for i in led)
    n = 0
    for v, (x, y, src) in place.items():
        for i in {v, nled - 1 - v}:
            pos[(strip, i)] = (x, y, "interp" if src == "interp" else "decoded" if i in seen else "mirror")
            n += 1
    print("strip %d: %d identified, %d outliers dropped, %d interpolated, %d LEDs placed" %
          (strip, len(uk), dropped, ninterp, n))


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
    ap.add_argument("--z-frac", type=float, default=0.0,
                    help="put the whole plane at z = this fraction of the x width, so it "
                         "does not pass through the y axis (radial effects look odd there)")
    ap.add_argument("--fold", action="store_true",
                    help="strands are folded in half: LED i and leds-1-i share a position")
    ap.add_argument("--max-gap", type=int, default=0,
                    help="fill gaps between identified LEDs only up to this many positions (0: any)")
    ap.add_argument("--no-extrap", action="store_true", help="do not extend past the identified LEDs")
    ap.add_argument("--axis", help="x0,y0,x1,y1: two image points on the tree's trunk. The layout's y "
                    "axis is made to run along it (x = distance from the trunk, any camera roll removed)")
    ap.add_argument("--min-leds", type=int, default=3,
                    help="skip strips with fewer identified LEDs than this")
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
        if args.fold:
            fold_strip(strip, pts, nled, args, pos, pitches)
            continue
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
    if args.axis:
        # trunk-aligned frame, still in pixels: x across the trunk, y up along it
        ax = [float(v) for v in args.axis.split(",")]
        p0, u = np.array(ax[:2]), np.array([ax[2] - ax[0], ax[3] - ax[1]])
        u /= np.hypot(*u)
        nrm = np.array([u[1], -u[0]])   # image-right when u points down
        if nrm[0] < 0:
            nrm = -nrm
        for key, (x, y, src) in pos.items():
            d = np.array([x, y]) - p0
            pos[key] = (d @ nrm, -(d @ u), src)
        xs = np.array([p[0] for p in pos.values()])
        ys = np.array([p[1] for p in pos.values()])
        cx0 = 0.0
        ybot = ys.min()
        print("trunk axis tilted %.1f deg in the image; x = 0 on the trunk" %
              np.degrees(np.arctan2(u[0], u[1])))
    print("LED pitch %.1f px -> %.3f mm/px; %d pixels in %d strips" %
          (px_per_led, m_per_px * 1e3, len(pos), len({k[0] for k in pos})))

    zbias = args.z_frac * (xs.max() - xs.min()) * m_per_px
    with open(args.out, "w") as f:
        f.write("# hween layout: strip,led,x,y,z in meters (y up, z = constant plane offset %.4f m)\n" % zbias)
        f.write("# from %s; %d LEDs/strip, pitch %.4f m = %.1f px\n" %
                (args.positions, nled, args.pitch_m, px_per_led))
        f.write("strip,led,x,y,z,source\n")
        for (strip, led), (x, y, src) in sorted(pos.items()):
            f.write("%d,%d,%.4f,%.4f,%.4f,%s\n" %
                    (strip, led, (x - cx0) * m_per_px, ((y - ybot) if args.axis else (ybot - y)) * m_per_px, zbias, src))


if __name__ == "__main__":
    main()
