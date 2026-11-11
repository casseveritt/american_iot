#!/usr/bin/env python3
"""Guess positions of fold positions nobody identified, from what is known about the strands:
each strip is one continuous path with LEDs every 1/60 m (so the path length between two
identified positions is known), the path stays on the tree surface and k=0 is the bottom.

    fillstruct.py points3d_all.csv -o layout.csv [--validate]

Gaps are bridged with a Hermite curve whose tangents come from the neighbouring identified
points, then re-sampled at equal arc length. Ends are extended along the end tangent with the
same pitch, turning with the average curvature of the last few positions, until the next position would
leave the tree's surface of revolution (r(y)) or its height range.
--validate hides random identified (triangulated) positions, predicts them and reports the error.
"""
import argparse
import csv
from collections import defaultdict

import numpy as np

PITCH = 1 / 60


def load(path):
    d = defaultdict(dict)
    for r in csv.DictReader(open(path)):
        d[int(r["strip"])][int(r["k"])] = (np.array([float(r["x"]), float(r["y"]), float(r["z"])]),
                                           "sfm" + r["nviews"] if r["nviews"].isdigit() else r["nviews"])
    return d


def tangent(pts, ks, i, side, n=4):
    """Unit direction of travel (increasing k) near known index i from up to n neighbours."""
    lo, hi = max(0, i - n), min(len(ks) - 1, i + n)
    if side < 0:
        hi = i
    elif side > 0:
        lo = i
    if hi <= lo:
        lo, hi = max(0, i - 1), min(len(ks) - 1, i + 1)
    v = pts[hi] - pts[lo]
    return v / (np.linalg.norm(v) + 1e-9)


def bridge(p0, p1, t0, t1, n):
    """n-1 points between p0 and p1 (n steps of PITCH along a Hermite curve)."""
    L = n * PITCH
    c = np.linalg.norm(p1 - p0)
    m = max(L, c)
    u = np.linspace(0, 1, 400)[:, None]
    h = np.hstack
    curve = ((2 * u**3 - 3 * u**2 + 1) * p0 + (u**3 - 2 * u**2 + u) * t0 * m +
             (-2 * u**3 + 3 * u**2) * p1 + (u**3 - u**2) * t1 * m)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
    want = np.linspace(0, s[-1], n + 1)[1:-1]
    return [np.array([np.interp(w, s, curve[:, a]) for a in range(3)]) for w in want]


def extend(pts, ks, step, rprof, ymax, maxn, bend):
    """Extend past the last identified position in direction `step` (+1 up in k, -1 down)."""
    sel = slice(-5, None) if step > 0 else slice(0, 5)
    P = np.array(pts)[sel]
    d = (P[-1] - P[0]) if step > 0 else (P[0] - P[-1])
    d = d / (np.linalg.norm(d) + 1e-9)
    p = np.array(pts[-1] if step > 0 else pts[0])
    out = []
    for _ in range(maxn):
        # steer back toward the tree surface when drifting off it
        q = p + d * PITCH
        r = np.hypot(q[0], q[2])
        if q[1] < -0.02 or q[1] > ymax + 0.02 or r > np.interp(max(q[1], 0), *rprof) + 0.12:
            break
        out.append(q)
        p = q
    return out


def predict(known, nled, ymax, rprof, maxext, maxgap):
    ks = sorted(known)
    pts = [known[k] for k in ks]
    res = {}
    for a in range(len(ks) - 1):
        n = ks[a + 1] - ks[a]
        if 1 < n <= maxgap:
            for j, q in enumerate(bridge(pts[a], pts[a + 1], tangent(pts, ks, a, +1), tangent(pts, ks, a + 1, -1), n), 1):
                res[ks[a] + j] = q
    top = nled - 1
    up = extend(pts, ks, +1, rprof, ymax, min(maxext, top - ks[-1]), 0)
    for j, q in enumerate(up, 1):
        res[ks[-1] + j] = q
    dn = extend(pts, ks, -1, rprof, ymax, min(maxext, ks[0]), 0)
    for j, q in enumerate(dn, 1):
        res[ks[0] - j] = q
    return res


def profile(d):
    P = np.array([v[0] for s in d.values() for v in s.values()])
    y, r = P[:, 1], np.hypot(P[:, 0], P[:, 2])
    e = np.linspace(0, y.max() + 1e-6, 12)
    cy, cr = [], []
    for a in range(11):
        m = (y >= e[a]) & (y < e[a + 1])
        if m.sum() >= 5:
            cy.append(y[m].mean()); cr.append(np.percentile(r[m], 90))
    return (cy, cr), y.max()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("points")
    ap.add_argument("-o", "--out")
    ap.add_argument("--leds", type=int, default=300)
    ap.add_argument("--max-ext", type=int, default=15, help="most positions to add past each end")
    ap.add_argument("--max-gap", type=int, default=40, help="largest gap (positions) to bridge")
    ap.add_argument("--validate", action="store_true")
    a = ap.parse_args()
    d = load(a.points)
    rprof, ymax = profile(d)
    nk = a.leds // 2

    if a.validate:
        rng = np.random.default_rng(1)
        errs = defaultdict(list)
        for s, kn in d.items():
            tri = [k for k, v in kn.items() if v[1].startswith("sfm")]
            if len(kn) < 30:
                continue
            for trial in range(6):
                hide = set(rng.choice(tri, size=max(1, len(tri) // 6), replace=False).tolist()) if trial < 3 else \
                    set(k for k in tri if (k // 8) % 6 == trial - 3)       # also hide whole 8-blocks
                known = {k: v[0] for k, v in kn.items() if k not in hide}
                pred = predict(known, nk, ymax, rprof, a.max_ext, a.max_gap)
                for k in hide:
                    if k in pred:
                        gap = min(abs(k - q) for q in known)
                        errs[min(gap, 12)].append(np.linalg.norm(pred[k] - kn[k][0]))
        for g in sorted(errs):
            e = np.array(errs[g]) * 100
            print("distance to nearest identified %2d%s: n=%4d  median err %.1f cm  90%% %.1f cm" %
                  (g, "+" if g == 12 else " ", len(e), np.median(e), np.percentile(e, 90)))
        return

    n = 0
    with open(a.out, "w") as fh:
        fh.write("# hween layout: sfm points + structural guesses (fillstruct.py)\nstrip,led,x,y,z,source\n")
        for s, kn in sorted(d.items()):
            if len(kn) < 20:
                continue
            known = {k: v[0] for k, v in kn.items()}
            pred = predict(known, nk, ymax, rprof, a.max_ext, a.max_gap)
            allp = {k: (v[0], v[1]) for k, v in kn.items()}
            allp.update({k: (p, "guess") for k, p in pred.items()})
            for k, (p, src) in sorted(allp.items()):
                for i in sorted({k, a.leds - 1 - k}):
                    fh.write("%d,%d,%.4f,%.4f,%.4f,%s\n" % (s, i, *p, src)); n += 1
    print(n, "LEDs written")


if __name__ == "__main__":
    main()
