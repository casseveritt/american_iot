#!/usr/bin/env python3
"""Decode the LED ID sequence from an extracted clip into 2D LED positions.

    extract.py clip.MOV cache_dir
    decode.py cache_dir codes.csv [--symbol-ms 400] [--bg x0,y0,x1,y1] -o out_dir

Steps:
  1. Find the cycle start (t0) by matching the preamble/marker against the
     overall brightness.
  2. Register every frame to a reference frame with a phase-correlation shift
     measured on a static background patch (the phone is not on a tripod).
  3. Average the frames where every LED is white and find the LED blobs.
  4. For each blob and symbol slot, measure the color of the core and of a ring
     around it (the core clips).
  5. Score every code word in the table against each blob's 12 symbols, refit
     the color classes from the confident matches, and repeat.

Writes out_dir/positions2d.csv (id,strip,led,x,y,gap,dist), overlay.png.
x,y are pixel coordinates in the reference frame of the cache.
"""
import argparse
import collections
import os

import cv2
import numpy as np

CHARS = "KRGYBMCW"  # symbol value -> name (bit0 R, bit1 G, bit2 B)
PREAMBLE_W = [0, 2, 4, 6, 8, 9]  # slots known to be white on every LED
PREAMBLE_K = [1, 3, 5, 7, 10]    # slots known to be dark on every LED
N_PRE, N_MARK, N_GAP = 8, 3, 2

# Starting guesses for (core RGB, ring RGB) per symbol; refit by self-training.
PROTO = np.array([
    [175, 168, 136, 163, 157, 129],
    [255, 110, 110, 180, 127, 170],
    [57, 252, 241, 48, 206, 232],
    [251, 251, 238, 173, 201, 220],
    [17, 133, 255, 138, 172, 239],
    [249, 225, 254, 150, 160, 240],
    [47, 243, 255, 107, 201, 254],
    [254, 254, 254, 160, 214, 255]], float)


# Same, for features divided by each blob's own all-white features: an LED is
# then 1 in every channel it has on and about 0 in the others.
BITS = np.array([[(c >> b) & 1 for b in range(3)] for c in range(8)], float)
PROTO_NORM = np.hstack([BITS * .9, BITS * .4])


def read_codes(path):
    rows = [l.strip().split(",") for l in open(path) if l[:1].isdigit()]
    ids = np.array([int(r[0]) for r in rows])
    strips = np.array([int(r[1]) for r in rows])
    leds = np.array([int(r[2]) for r in rows])
    table = np.array([[CHARS.index(c) for c in r[3]] for r in rows])
    return ids, strips, leds, table


def find_t0(t, lum, slot_s):
    """Cycle start: best match of the preamble+marker square wave."""
    pat = [1, -1] * 4 + [1, 1, -1]
    lum = lum - lum.mean()
    best = (-1e18, 0.0)
    for t0 in np.arange(0, t[-1], 0.01):
        score = 0.0
        for k, s in enumerate(pat):
            m = (t >= t0 + slot_s * (k + .3)) & (t <= t0 + slot_s * (k + .8))
            if not m.any():
                score = -1e18
                break
            score += s * lum[m].mean()
        if score > best[0]:
            best = (score, t0)
    return best[1]


def refine_t0(t, lum, slot_s, t0):
    """find_t0 only says which slot the preamble is in (any start inside a slot scores
    about the same); lock onto the sharp brightness edges of the preamble instead."""
    mid = (t[1:] + t[:-1]) / 2
    g = np.abs(np.diff(lum))
    best = (-1.0, t0)
    for c in np.arange(t0 - slot_s, t0 + slot_s, 0.002):
        sc = sum(np.interp(c + k * slot_s, mid, g) for k in range(0, 11) if k != 9)
        if sc > best[0]:
            best = (sc, c)
    return best[1]


def register(F, bg, ref):
    x0, y0, x1, y1 = bg
    def g(i):
        return cv2.cvtColor(np.ascontiguousarray(F[i, y0:y1, x0:x1]),
                            cv2.COLOR_RGB2GRAY).astype(np.float32)
    r = g(ref)
    win = cv2.createHanningWindow(r.shape[::-1], cv2.CV_32F)
    return np.array([cv2.phaseCorrelate(r, g(i), win)[0] for i in range(len(F))])


def register_robust(L, bg, ref, scale, min_resp=0.5):
    """Per-frame shift from a low-res copy of the clip (whole frame, so there is a
    static background), scaled to the full-resolution cache. Frames where the
    correlation is weak (the scene is dark) get an interpolated shift."""
    x0, y0, x1, y1 = bg
    def g(i):
        p = cv2.cvtColor(np.ascontiguousarray(L[i, y0:y1, x0:x1]), cv2.COLOR_RGB2GRAY)
        return cv2.GaussianBlur(p.astype(np.float32), (0, 0), 1.0)
    r = g(ref)
    win = cv2.createHanningWindow(r.shape[::-1], cv2.CV_32F)
    res = [cv2.phaseCorrelate(r, g(i), win) for i in range(len(L))]
    sh = np.array([a for a, _ in res]) * scale
    ok = np.array([resp >= min_resp for _, resp in res])
    # a wild jump is as bad as a weak peak
    d = np.abs(sh - np.median(sh, 0)).max(1)
    ok &= d < 5 * max(np.median(d), 1.0) + 10 * scale
    idx = np.arange(len(L))
    for a in range(2):
        sh[:, a] = np.interp(idx, idx[ok], sh[ok, a])
    print("registration: %d / %d frames confident; shift range x %.1f..%.1f y %.1f..%.1f" %
          (ok.sum(), len(ok), sh[:, 0].min(), sh[:, 0].max(), sh[:, 1].min(), sh[:, 1].max()))
    return sh


def slot_frames(t, t0, slot_s, k):
    lo, hi = t0 + slot_s * (k + .3), t0 + slot_s * (k + .8)
    return np.nonzero((t >= lo) & (t <= hi))[0]


def find_blobs(F, t, sh, t0s, slot_s, dog=(3, 9), peak_win=21, thresh=.35, rel=.8):
    white = []
    for t0 in t0s:
        for k in PREAMBLE_W:
            white += list(slot_frames(t, t0, slot_s, k))
    acc = np.zeros(F.shape[1:], np.float32)
    for i in white:
        M = np.float32([[1, 0, -sh[i, 0]], [0, 1, -sh[i, 1]]])
        acc += cv2.warpAffine(np.ascontiguousarray(F[i]), M,
                              (F.shape[2], F.shape[1])).astype(np.float32)
    acc /= len(white)
    gray = acc.mean(2)
    dog = cv2.GaussianBlur(gray, (0, 0), dog[0]) - cv2.GaussianBlur(gray, (0, 0), dog[1])
    peak = (dog == cv2.dilate(dog, np.ones((peak_win, peak_win), np.uint8))) & (dog > thresh * dog.max())
    ys, xs = np.nonzero(peak)
    g = gray[ys, xs]
    keep = g > rel * np.median(g)
    return acc, xs[keep], ys[keep]


def slot_features(F, t, sh, xy, t0, slot_s, nslots, core=3, ring=(9, 12), half=14):
    yy, xx = np.mgrid[-half:half + 1, -half:half + 1]
    r = np.hypot(xx, yy)
    mc, mr = r <= core, (r >= ring[0]) & (r <= ring[1])
    H, W = F.shape[1:3]
    feat = np.full((len(xy), nslots, 6), np.nan)
    for k in range(nslots):
        idx = slot_frames(t, t0, slot_s, k)
        acc = np.zeros((len(xy), 6))
        n = np.zeros(len(xy))
        for i in idx:
            for b, (x, y) in enumerate(xy):
                px, py = int(round(x + sh[i, 0])), int(round(y + sh[i, 1]))
                if px < half or py < half or px >= W - half or py >= H - half:
                    continue
                p = np.asarray(F[i, py - half:py + half + 1, px - half:px + half + 1], np.float32)
                acc[b] += np.r_[p[mc].mean(0), p[mr].mean(0)]
                n[b] += 1
        with np.errstate(invalid="ignore"):
            feat[:, k] = acc / n[:, None]
    return feat


def kmeans_classes(X, floor):
    """Start the 8 color classes from k-means of the payload observations, named
    by matching the (per channel rescaled) centers to the corners of the RGB cube.
    Needed when the camera response is far from any fixed prototype."""
    from scipy.cluster.vq import kmeans2
    from scipy.optimize import linear_sum_assignment
    X = X[~np.isnan(X).any(1)]
    C, lab = kmeans2(X, 8, minit="++", seed=1)
    z = (C - C.min(0)) / np.maximum(C.max(0) - C.min(0), 1e-6)
    corners = BITS if X.shape[1] == 3 else np.hstack([BITS, BITS])
    row, col = linear_sum_assignment(((z[:, None, :] - corners[None]) ** 2).sum(-1))
    mu, sig = np.zeros((8, X.shape[1])), np.zeros((8, X.shape[1]))
    for r, c in zip(row, col):
        sel = X[lab == r]
        mu[c], sig[c] = sel.mean(0), sel.std(0) + floor
    return mu, sig


def classify(feats, table, nsym, iters=6, gap_min=40, max_dist=2, proto=None, sig0=30.0, floor=4.0, kmeans_init=False):
    """feats: (cycles, blobs, slots, 6). The cycles are repeats of the same code, so
    their log-likelihoods add."""
    nc, nb = feats.shape[:2]
    nd = feats.shape[-1]
    if kmeans_init:
        mu, sig = kmeans_classes(feats[:, :, N_PRE + N_MARK:N_PRE + N_MARK + nsym].reshape(-1, nd), floor)
    else:
        mu, sig = (PROTO if proto is None else proto).copy(), np.full((8, nd), sig0)
        Xw = feats[:, :, PREAMBLE_W].reshape(-1, nd)
        Xk = feats[:, :, PREAMBLE_K].reshape(-1, nd)
        mu[0], mu[7] = np.nanmean(Xk, 0), np.nanmean(Xw, 0)
        sig[0], sig[7] = np.nanstd(Xk, 0) + floor, np.nanstd(Xw, 0) + floor
    X = feats[:, :, N_PRE + N_MARK:N_PRE + N_MARK + nsym]          # (nc, nb, nsym, 6)
    seen = ~np.isnan(X).any(-1)
    X = np.nan_to_num(X, nan=0.0)
    for it in range(iters):
        d = (X[..., None, :] - mu) / sig
        LL = -0.5 * (d ** 2).sum(-1) - np.log(sig).sum(-1)       # (nc, nb, nsym, 8)
        LL = np.where(seen[..., None], LL, 0.0)
        S = np.zeros((nb, len(table)))
        for c in range(nc):
            for k in range(nsym):
                S += LL[c, :, k, :][:, table[:, k]]
        order = np.argsort(-S, 1)
        best = order[:, 0]
        gap = S[np.arange(nb), best] - S[np.arange(nb), order[:, 1]]
        dist = (LL.argmax(3) != table[best]).sum(2).min(0)
        good = (gap > gap_min) & (dist <= max_dist)
        print("  pass %d: %d / %d confident" % (it, good.sum(), nb))
        if good.sum() < 10:
            break
        lab = np.tile(table[best[good]].reshape(-1), nc)
        Xg = X[:, good].reshape(-1, nd)
        ok = seen[:, good].reshape(-1)
        for c in range(8):
            sel = Xg[(lab == c) & ok]
            if len(sel) > 5:
                mu[c], sig[c] = sel.mean(0), sel.std(0) + floor
    return best, gap, dist, good


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cache")
    ap.add_argument("codes")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--symbol-ms", type=float, default=400)
    ap.add_argument("--t0", type=float, default=None, help="cycle start (s); default auto")
    ap.add_argument("--bg", default="0,300,300,560",
                    help="static background patch x0,y0,x1,y1 for registration")
    ap.add_argument("--lowres", help="low-res whole-frame cache of the same clip, used for "
                    "registration (a crop has no static background); with --lowres-scale")
    ap.add_argument("--lowres-scale", type=float, default=4.0, help="cache pixels per low-res pixel")
    ap.add_argument("--cycles", type=int, default=1, help="use this many full cycles from t0")
    ap.add_argument("--core-r", type=float, default=3, help="LED core radius, pixels")
    ap.add_argument("--ring", default="9,12", help="inner,outer radius of the ring around the core")
    ap.add_argument("--dog", default="3,9", help="DoG sigmas for blob detection")
    ap.add_argument("--peak-win", type=int, default=21, help="peak suppression window, pixels")
    ap.add_argument("--blob-thresh", type=float, default=.35, help="DoG threshold, fraction of max")
    ap.add_argument("--norm-white", action="store_true",
                    help="divide each blob's features by its own all-white features")
    ap.add_argument("--features", choices=["both", "ring"], default="both",
                    help="classify on core+ring or on the ring alone (the core can be washed out)")
    ap.add_argument("--kmeans-init", action="store_true", help="start color classes from k-means")
    ap.add_argument("--gap-min", type=float, default=40)
    ap.add_argument("--max-dist", type=int, default=2)
    args = ap.parse_args()
    out = args.out or args.cache
    os.makedirs(out, exist_ok=True)
    slot_s = args.symbol_ms / 1e3

    F = np.load(os.path.join(args.cache, "frames.npy"), mmap_mode="r")
    t = np.load(os.path.join(args.cache, "pts.npy"))
    ids, strips, leds, table = read_codes(args.codes)
    nsym = table.shape[1]
    nslots = N_PRE + N_MARK + nsym + N_GAP

    lum = np.array([F[i, ::8, ::8].mean() for i in range(len(F))])
    t0 = args.t0 if args.t0 is not None else find_t0(t, lum, slot_s)
    if args.t0 is None:
        t0 = refine_t0(t, lum, slot_s, t0)
    print("cycle start t0 = %.3f s" % t0)
    cyc = nslots * slot_s
    while t0 - cyc >= 0 and args.t0 is None:   # earliest full cycle
        t0 -= cyc
    t0s = [t0 + k * cyc for k in range(args.cycles) if t0 + (k + 1) * cyc <= t[-1]]
    if not t0s:
        raise SystemExit("clip does not contain a complete cycle after t0")
    print("cycles starting at", ", ".join("%.2f" % x for x in t0s))

    bg = [int(v) for v in args.bg.split(",")]
    if args.lowres:
        L = np.load(os.path.join(args.lowres, "frames.npy"), mmap_mode="r")
        sh = register_robust(L, bg, len(F) // 2, args.lowres_scale)
    else:
        sh = register(F, bg, len(F) // 2)
        print("registration shift range x %.1f..%.1f  y %.1f..%.1f" %
              (sh[:, 0].min(), sh[:, 0].max(), sh[:, 1].min(), sh[:, 1].max()))
    dog = [float(v) for v in args.dog.split(",")]
    white, xs, ys = find_blobs(F, t, sh, t0s, slot_s, dog, args.peak_win, args.blob_thresh)
    xy = np.c_[xs, ys]
    print("blobs: %d" % len(xy))
    ring = [float(v) for v in args.ring.split(",")]
    half = int(ring[1]) + 2
    feats = np.stack([slot_features(F, t, sh, xy, x, slot_s, nslots, args.core_r, ring, half)
                      for x in t0s])
    kw = {}
    if args.norm_white:
        w = np.nanmean(feats[:, :, PREAMBLE_W], 2, keepdims=True)           # (cycles, blobs, 1, 6)
        feats = feats / np.maximum(w, 5.0)
        kw = dict(proto=PROTO_NORM, sig0=.3, floor=.05)
    if args.features == "ring":
        feats = feats[..., 3:6]
    kw["kmeans_init"] = args.kmeans_init
    best, gap, dist, good = classify(feats, table, nsym, gap_min=args.gap_min, max_dist=args.max_dist, **kw)

    rows = [(ids[best[b]], strips[best[b]], leds[best[b]], xy[b, 0], xy[b, 1], gap[b], dist[b])
            for b in np.nonzero(good)[0]]
    rows.sort()
    dup = [i for i, c in collections.Counter(r[0] for r in rows).items() if c > 1]
    with open(os.path.join(out, "positions2d.csv"), "w") as f:
        f.write("id,strip,led,x,y,gap,dist\n")
        for r in rows:
            f.write("%d,%d,%d,%d,%d,%.0f,%d\n" % r)
    vis = cv2.cvtColor(white.astype(np.uint8), cv2.COLOR_RGB2BGR)
    for b in range(len(xy)):
        c = (0, 255, 0) if good[b] else (0, 0, 255)
        cv2.circle(vis, (int(xy[b, 0]), int(xy[b, 1])), 12, c, 2)
    cv2.imwrite(os.path.join(out, "overlay.png"), vis)
    print("decoded %d of %d blobs; %d ids seen more than once" % (len(rows), len(xy), len(dup)))
    print("per strip:", sorted(collections.Counter(r[1] for r in rows).items()))


if __name__ == "__main__":
    main()
