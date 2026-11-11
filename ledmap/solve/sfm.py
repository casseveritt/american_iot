#!/usr/bin/env python3
"""Solve camera poses and 3D LED positions together from several decoded views.

    sfm.py -o out_dir [--leds 300] [--f0 2700] VIEW [VIEW ...]

VIEW is a decode output directory (positions2d.csv) followed by its frame cache
(crop.txt gives where the cache sits in the 3840x2160 source frame), written as
`out_c016:cache_c016`.

The views can be in any order and from anywhere: every identified LED is a point seen by
name in each view that saw it. Strands are folded in half, so LED i and LED leds-1-i are the
same spot; one point per (strip, min(i, leds-1-i)). Pinhole cameras with one shared focal
length (principal point at the image center, no distortion), estimated by bundle adjustment.

Steps: pick the pair of views with the most shared points and a real baseline, relative
pose from the essential matrix, triangulate; add the other views one by one with PnP;
re-triangulate and adjust all cameras and points after each; drop observations that do not
fit; then fix the gauge: scale from the LED spacing (1/60 m), up from the tree's rotational
symmetry (at each height the lights sit at one distance from the trunk).

Writes out_dir/points3d.csv (strip,k,x,y,z,nviews,rms_px), cameras.csv, layout.csv (hween).
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import cv2
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.sparse import lil_matrix

W, H = 3840, 2160


def load_view(spec, nled):
    out, cache = spec.split(":")
    x0, y0 = [float(v) for v in open(os.path.join(cache, "crop.txt")).read().split()[0].split(",")]
    sc = float(open(os.path.join(cache, "crop.txt")).read().split()[1])
    seen = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(out, "positions2d.csv"))):
        s, i = int(r["strip"]), int(r["led"])
        seen[(s, min(i, nled - 1 - i))].append((x0 + sc * float(r["x"]), y0 + sc * float(r["y"]), i))
    obs = {}
    for pid, v in seen.items():
        if len(v) == 1:
            obs[pid] = np.array(v[0][:2])
        elif len({p[2] for p in v}) == 2 and len(v) == 2:      # both faces: should coincide
            a, b = np.array(v[0][:2]), np.array(v[1][:2])
            if np.hypot(*(a - b)) < 12:
                obs[pid] = (a + b) / 2
        # more than one reading of the same LED: ambiguous, skip
    return os.path.basename(out.rstrip("/")), obs


def K_of(f):
    return np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])


def project(rv, t, X, f):
    P = cv2.Rodrigues(rv)[0] @ X.T + t[:, None]
    return (f * P[:2] / P[2] + np.array([[W / 2], [H / 2]])).T, P[2]


def triangulate(cams, uv, f):
    """Linear (DLT) triangulation from normalized coordinates. cams: list of (rv, t)."""
    A = []
    for (rv, t), (u, v) in zip(cams, uv):
        R = cv2.Rodrigues(rv)[0]
        P = np.hstack([R, t[:, None]])
        x, y = (u - W / 2) / f, (v - H / 2) / f
        A += [x * P[2] - P[0], y * P[2] - P[1]]
    X = np.linalg.svd(np.array(A))[2][-1]
    return X[:3] / X[3]


def bundle(cam_ids, cams, f, pts, obs_list, fix_first=True):
    """cams: {view: (rv, t)}, pts: {pid: X}, obs_list: [(view, pid, u, v)] -> refined."""
    cids = [c for c in cam_ids]
    pids = sorted({o[1] for o in obs_list})
    ci = {c: i for i, c in enumerate(cids)}
    pi = {p: i for i, p in enumerate(pids)}
    nfree = len(cids) - (1 if fix_first else 0)
    off = lambda c: (ci[c] - (1 if fix_first else 0)) * 6
    x0 = np.concatenate([np.concatenate(cams[c]) for c in cids[1 if fix_first else 0:]] +
                        [[f]] + [pts[p] for p in pids])
    nc6 = nfree * 6
    nO = len(obs_list)
    S = lil_matrix((2 * nO, len(x0)), dtype=int)
    cidx = np.array([ci[o[0]] for o in obs_list]); pidx = np.array([pi[o[1]] for o in obs_list])
    for k in range(nO):
        c = cids[cidx[k]]
        if not (fix_first and cidx[k] == 0):
            S[2 * k:2 * k + 2, off(c):off(c) + 6] = 1
        S[2 * k:2 * k + 2, nc6] = 1
        S[2 * k:2 * k + 2, nc6 + 1 + 3 * pidx[k]:nc6 + 4 + 3 * pidx[k]] = 1
    uv = np.array([[o[2], o[3]] for o in obs_list])

    def unpack(x):
        cc = {}
        for c in cids:
            if fix_first and c == cids[0]:
                cc[c] = (np.zeros(3), np.zeros(3))
            else:
                cc[c] = (x[off(c):off(c) + 3], x[off(c) + 3:off(c) + 6])
        return cc, x[nc6], x[nc6 + 1:].reshape(-1, 3)

    def resid(x):
        cc, ff, P = unpack(x)
        r = np.empty((nO, 2))
        for c in cids:
            m = cidx == ci[c]
            if m.any():
                pr, z = project(cc[c][0], cc[c][1], P[pidx[m]], ff)
                r[m] = pr - uv[m]
                r[m] += np.where(z[:, None] < 0.01, 1e4, 0)          # behind the camera
        return r.ravel()

    sol = least_squares(resid, x0, jac_sparsity=S, loss="soft_l1", f_scale=3.0, x_scale="jac",
                        method="trf", max_nfev=40)
    cc, ff, P = unpack(sol.x)
    res = resid(sol.x).reshape(-1, 2)
    return cc, ff, {p: P[pi[p]] for p in pids}, np.hypot(res[:, 0], res[:, 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("views", nargs="+")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--leds", type=int, default=300)
    ap.add_argument("--f0", type=float, default=2700, help="initial focal length, pixels of the 4K frame")
    ap.add_argument("--pitch-m", type=float, default=1 / 60, help="LED spacing along a strand")
    ap.add_argument("--max-px", type=float, default=6.0, help="drop observations that miss by more than this")
    ap.add_argument("--z-frac", type=float, default=0.0,
                    help="shift everything toward +z by this fraction of the tree's width (hween "
                         "layouts with the trunk at x=z=0 normally want 0)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    views = dict(load_view(v, args.leds) for v in args.views)
    for n, o in views.items():
        print("view %s: %d points" % (n, len(o)))
    names = list(views)
    f = args.f0

    # ---- initial pair
    best = None
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            common = sorted(set(views[a]) & set(views[b]))
            if len(common) < 40:
                continue
            p1 = np.array([views[a][c] for c in common]); p2 = np.array([views[b][c] for c in common])
            E, m = cv2.findEssentialMat(p1, p2, K_of(f), cv2.RANSAC, 0.999, 3.0)
            if E is None:
                continue
            n_in, R, t, m2 = cv2.recoverPose(E, p1, p2, K_of(f), mask=m)
            spread = np.median(np.hypot(*(p1 - p2).T))
            score = n_in * min(spread, 400)
            if best is None or score > best[0]:
                best = (score, a, b, common, R, t, n_in, spread)
    if best is None:
        sys.exit("no pair of views shares enough points (need 40)")
    _, a, b, common, R, t, n_in, spread = best
    print("initial pair %s + %s: %d shared, %d inliers, median parallax %.0f px" % (a, b, len(common), n_in, spread))
    cams = {a: (np.zeros(3), np.zeros(3)), b: (cv2.Rodrigues(R)[0].ravel(), t.ravel())}
    pts = {}
    for c in common:
        X = triangulate([cams[a], cams[b]], [views[a][c], views[b][c]], f)
        if np.isfinite(X).all():
            pts[c] = X

    def obs_for(cams_, pts_):
        return [(v, p, *views[v][p]) for v in cams_ for p in views[v] if p in pts_]

    def retriangulate():
        by = defaultdict(list)
        for v in cams:
            for p, uv in views[v].items():
                by[p].append((v, uv))
        for p, lst in by.items():
            if p not in pts and len(lst) >= 2:
                X = triangulate([cams[v] for v, _ in lst], [uv for _, uv in lst], f)
                if np.isfinite(X).all() and np.abs(X).max() < 1e4:
                    ok = 0
                    for v, uv in lst:
                        pr, z = project(*cams[v], X[None], f)
                        ok += z[0] > 0 and np.hypot(*(pr[0] - uv)) < 15
                    if ok >= 2:
                        pts[p] = X

    def adjust():
        nonlocal f, pts
        cc, ff, P, err = bundle(list(cams), cams, f, pts, obs_for(cams, pts))
        cams.update(cc); f = ff
        pts = P
        return err

    def prune(thresh):
        ol = obs_for(cams, pts)
        err = adjust()
        bad = defaultdict(int)
        for o, e in zip(ol, err):
            if e > thresh:
                bad[(o[0], o[1])] += 1
        for (v, p) in bad:
            views[v].pop(p, None)
        for p in list(pts):
            if sum(p in views[v] for v in cams) < 2:
                del pts[p]
        return len(bad)

    for _ in range(3):
        adjust(); prune(max(args.max_px, 10))
    todo = [n for n in names if n not in cams]
    while todo:
        cand = []
        for n in todo:
            c = [p for p in views[n] if p in pts]
            cand.append((len(c), n))
        cand.sort(reverse=True)
        k, n = cand[0]
        todo.remove(n)
        if k < 12:
            print("view %s: only %d known points, left out" % (n, k)); continue
        pl = [p for p in views[n] if p in pts]
        ok, rv, tv, inl = cv2.solvePnPRansac(np.array([pts[p] for p in pl]), np.array([views[n][p] for p in pl]),
                                             K_of(f), None, reprojectionError=10, iterationsCount=500,
                                             flags=cv2.SOLVEPNP_SQPNP)
        if not ok or inl is None or len(inl) < 10:
            print("view %s: PnP failed (%d known points), left out" % (n, k)); continue
        cams[n] = (rv.ravel(), tv.ravel())
        retriangulate()
        nb = prune(max(args.max_px, 10))
        print("added %s: %d points seen, %d PnP inliers, %d points total, f=%.0f, dropped %d" %
              (n, k, len(inl), len(pts), f, nb))
    for _ in range(2):
        retriangulate(); prune(args.max_px)
    err = adjust()
    ol = obs_for(cams, pts)
    print("final: %d cameras, %d points, f=%.0f px (%.1f deg wide), rms %.2f px, median %.2f px" %
          (len(cams), len(pts), f, 2 * np.degrees(np.arctan(W / 2 / f)), np.sqrt((err ** 2).mean()), np.median(err)))

    # ---- gauge: scale from the LED spacing along strands
    d = []
    for (s, k), X in pts.items():
        if (s, k + 1) in pts:
            d.append(np.linalg.norm(pts[(s, k + 1)] - X))
    scale = args.pitch_m / np.median(d)
    print("median neighbor spacing %.4f units -> scale %.4f m/unit (%d pairs)" % (np.median(d), scale, len(d)))
    keys = sorted(pts)
    X = np.array([pts[k] for k in keys]) * scale
    camsM = {v: (r, t * scale) for v, (r, t) in cams.items()}

    # ---- up direction and trunk from rotational symmetry
    c0 = X.mean(0)
    # camera +y points down in the image, so the world direction of "up" is -R^T e_y
    cam_up = np.mean([-(cv2.Rodrigues(r)[0].T[:, 1]) for r, t in camsM.values()], 0)
    cam_up /= np.linalg.norm(cam_up)

    def frame(u):
        u = u / np.linalg.norm(u)
        a = np.cross(u, [1, 0, 0] if abs(u[0]) < .9 else [0, 1, 0]); a /= np.linalg.norm(a)
        return u, a, np.cross(u, a)

    def cost(p):
        th, ph, ox, oz = p
        u = cam_up + th * frame(cam_up)[1] + ph * frame(cam_up)[2]
        u, a, b = frame(u)
        Y = (X - c0) @ u
        r = np.hypot((X - c0) @ a - ox, (X - c0) @ b - oz)
        bins = np.floor(Y / 0.04).astype(int)
        return sum(np.var(r[bins == q]) * (bins == q).sum() for q in np.unique(bins) if (bins == q).sum() > 3)

    sol = min((minimize(cost, [t0, p0, 0, 0], method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-8, maxiter=800))
               for t0 in (-.1, 0, .1) for p0 in (-.1, 0, .1)), key=lambda s: s.fun)
    th, ph, ox, oz = sol.x
    u, a, b = frame(cam_up + th * frame(cam_up)[1] + ph * frame(cam_up)[2])
    n = (X - c0).shape[0]
    print("symmetry fit: axis %.1f deg from the mean camera up; radius spread %.3f m rms" %
          (np.degrees(np.arccos(np.clip(u @ cam_up, -1, 1))), np.sqrt(sol.fun / n)))
    Y = (X - c0) @ u
    A = (X - c0) @ a - ox
    B = (X - c0) @ b - oz
    Y -= Y.min()
    R_world = np.array([a, u, b])          # rows: new x, y(up), z  (right-handed: a x u = -b?)
    if np.linalg.det(R_world) < 0:
        B = -B
        b = -b
        R_world = np.array([a, u, b])
    zshift = args.z_frac * (A.max() - A.min())
    Xn = np.c_[A, Y, B + zshift]

    # ---- cameras in the new frame
    with open(os.path.join(args.out, "cameras.csv"), "w") as fh:
        fh.write("view,x,y,z,distance_to_trunk_m,azimuth_deg,height_m,f_px\n")
        for v, (r, t) in camsM.items():
            C = -cv2.Rodrigues(r)[0].T @ t
            Cn = np.array([(C - c0) @ a - ox, ((C - c0) @ u) - ((X - c0) @ u).min(), (C - c0) @ b - oz])
            fh.write("%s,%.3f,%.3f,%.3f,%.2f,%.0f,%.2f,%.0f\n" % (
                v, *Cn, np.hypot(Cn[0], Cn[2]), np.degrees(np.arctan2(Cn[0], Cn[2])), Cn[1], f))
    cams_txt = open(os.path.join(args.out, "cameras.csv")).read()
    print(cams_txt)

    nviews = {k: sum(k in views[v] for v in cams) for k in keys}
    with open(os.path.join(args.out, "points3d.csv"), "w") as fh:
        fh.write("strip,k,x,y,z,nviews\n")
        for k, p in zip(keys, Xn):
            fh.write("%d,%d,%.4f,%.4f,%.4f,%d\n" % (k[0], k[1], *p, nviews[k]))
    with open(os.path.join(args.out, "layout.csv"), "w") as fh:
        fh.write("# hween layout from sfm.py: strip,led,x,y,z in meters, y up along the trunk, trunk at x=z=0\n")
        fh.write("strip,led,x,y,z,source\n")
        for (s, k), p in zip(keys, Xn):
            for i in sorted({k, args.leds - 1 - k}):
                fh.write("%d,%d,%.4f,%.4f,%.4f,sfm%d\n" % (s, i, *p, nviews[(s, k)]))
    print("wrote points3d.csv, cameras.csv, layout.csv in %s (%d fold positions, %d LEDs)" %
          (args.out, len(keys), sum(2 if k[1] != args.leds - 1 - k[1] else 1 for k in keys)))

    # ---- LEDs seen by too few views to triangulate: cast their ray onto the tree's surface
    # of revolution r(y) (from the solved points) and take the near (visible) hit
    rad = np.hypot(Xn[:, 0], Xn[:, 2])
    edges = np.linspace(0, Xn[:, 1].max() + 1e-6, 16)
    cy, cr = [], []
    for q in range(15):
        m = (Xn[:, 1] >= edges[q]) & (Xn[:, 1] < edges[q + 1])
        if m.sum() >= 5:
            cy.append(Xn[m, 1].mean()); cr.append(np.median(rad[m]))
    solved = set(keys)
    cast = defaultdict(list)
    for v, (r, t) in camsM.items():
        Rn = cv2.Rodrigues(r)[0] @ R_world.T
        C = -cv2.Rodrigues(r)[0].T @ t
        Cn = np.array([(C - c0) @ a - ox, ((C - c0) @ u) - ((X - c0) @ u).min(), (C - c0) @ b - oz])
        for pid, uv in views[v].items():
            if pid in solved:
                continue
            d = Rn.T @ np.array([(uv[0] - W / 2) / f, (uv[1] - H / 2) / f, 1.0])
            d /= np.linalg.norm(d)
            sv = np.linspace(0.3, 8, 3000)
            P = Cn + sv[:, None] * d
            g = np.hypot(P[:, 0], P[:, 2]) - np.interp(P[:, 1], cy, cr)
            g[(P[:, 1] < 0) | (P[:, 1] > Xn[:, 1].max())] = np.nan
            hit = np.nonzero((g[:-1] > 0) & (g[1:] <= 0))[0]
            if len(hit):
                cast[pid].append(P[hit[0]])
    with open(os.path.join(args.out, "points3d_all.csv"), "w") as fh:
        fh.write("strip,k,x,y,z,nviews\n")
        for k, p in zip(keys, Xn):
            fh.write("%d,%d,%.4f,%.4f,%.4f,%d\n" % (k[0], k[1], *p, nviews[k]))
        for k, ps in cast.items():
            fh.write("%d,%d,%.4f,%.4f,%.4f,ray\n" % (k[0], k[1], *np.mean(ps, 0)))
    print("ray-cast %d more LEDs seen in too few views" % len(cast))
    print("tree: height %.2f m, radius %.2f m" % (Xn[:, 1].max(), np.hypot(Xn[:, 0], Xn[:, 2]).max()))


if __name__ == "__main__":
    main()
