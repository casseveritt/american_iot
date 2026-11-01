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


def register(F, bg, ref):
    x0, y0, x1, y1 = bg
    def g(i):
        return cv2.cvtColor(np.ascontiguousarray(F[i, y0:y1, x0:x1]),
                            cv2.COLOR_RGB2GRAY).astype(np.float32)
    r = g(ref)
    win = cv2.createHanningWindow(r.shape[::-1], cv2.CV_32F)
    return np.array([cv2.phaseCorrelate(r, g(i), win)[0] for i in range(len(F))])


def slot_frames(t, t0, slot_s, k):
    lo, hi = t0 + slot_s * (k + .3), t0 + slot_s * (k + .8)
    return np.nonzero((t >= lo) & (t <= hi))[0]


def find_blobs(F, t, sh, t0, slot_s):
    white = []
    for k in PREAMBLE_W:
        white += list(slot_frames(t, t0, slot_s, k))
    acc = np.zeros(F.shape[1:], np.float32)
    for i in white:
        M = np.float32([[1, 0, -sh[i, 0]], [0, 1, -sh[i, 1]]])
        acc += cv2.warpAffine(np.ascontiguousarray(F[i]), M,
                              (F.shape[2], F.shape[1])).astype(np.float32)
    acc /= len(white)
    gray = acc.mean(2)
    dog = cv2.GaussianBlur(gray, (0, 0), 3) - cv2.GaussianBlur(gray, (0, 0), 9)
    peak = (dog == cv2.dilate(dog, np.ones((21, 21), np.uint8))) & (dog > .35 * dog.max())
    ys, xs = np.nonzero(peak)
    g = gray[ys, xs]
    keep = g > .8 * np.median(g)
    return acc, xs[keep], ys[keep]


def slot_features(F, t, sh, xy, t0, slot_s, nslots):
    yy, xx = np.mgrid[-14:15, -14:15]
    r = np.hypot(xx, yy)
    mc, mr = r <= 3, (r >= 9) & (r <= 12)
    H, W = F.shape[1:3]
    feat = np.full((len(xy), nslots, 6), np.nan)
    for k in range(nslots):
        idx = slot_frames(t, t0, slot_s, k)
        acc = np.zeros((len(xy), 6))
        n = np.zeros(len(xy))
        for i in idx:
            for b, (x, y) in enumerate(xy):
                px, py = int(round(x + sh[i, 0])), int(round(y + sh[i, 1]))
                if px < 14 or py < 14 or px >= W - 14 or py >= H - 14:
                    continue
                p = np.asarray(F[i, py - 14:py + 15, px - 14:px + 15], np.float32)
                acc[b] += np.r_[p[mc].mean(0), p[mr].mean(0)]
                n[b] += 1
        with np.errstate(invalid="ignore"):
            feat[:, k] = acc / n[:, None]
    return feat


def classify(feat, table, nsym, iters=6):
    nb = len(feat)
    mu, sig = PROTO.copy(), np.full((8, 6), 30.0)
    Xw = feat[:, PREAMBLE_W].reshape(-1, 6)
    Xk = feat[:, PREAMBLE_K].reshape(-1, 6)
    mu[0], mu[7] = np.nanmean(Xk, 0), np.nanmean(Xw, 0)
    sig[0], sig[7] = np.nanstd(Xk, 0) + 5, np.nanstd(Xw, 0) + 5
    X = np.nan_to_num(feat[:, N_PRE + N_MARK:N_PRE + N_MARK + nsym], nan=0.0)
    for it in range(iters):
        d = (X[..., None, :] - mu) / sig
        LL = -0.5 * (d ** 2).sum(-1) - np.log(sig).sum(-1)       # (nb, nsym, 8)
        S = np.zeros((nb, len(table)))
        for k in range(nsym):
            S += LL[:, k, table[:, k]]
        order = np.argsort(-S, 1)
        best = order[:, 0]
        gap = S[np.arange(nb), best] - S[np.arange(nb), order[:, 1]]
        dist = (LL.argmax(2) != table[best]).sum(1)
        good = (gap > 40) & (dist <= 2)
        print("  pass %d: %d / %d confident" % (it, good.sum(), nb))
        if good.sum() < 10:
            break
        lab = table[best[good]].reshape(-1)
        Xg = X[good].reshape(-1, 6)
        for c in range(8):
            sel = Xg[lab == c]
            if len(sel) > 5:
                mu[c], sig[c] = sel.mean(0), sel.std(0) + 4
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
    print("cycle start t0 = %.2f s" % t0)
    if t0 + nslots * slot_s > t[-1]:
        raise SystemExit("clip does not contain a complete cycle after t0")

    bg = [int(v) for v in args.bg.split(",")]
    sh = register(F, bg, len(F) // 2)
    print("registration shift range x %.1f..%.1f  y %.1f..%.1f" %
          (sh[:, 0].min(), sh[:, 0].max(), sh[:, 1].min(), sh[:, 1].max()))
    white, xs, ys = find_blobs(F, t, sh, t0, slot_s)
    xy = np.c_[xs, ys]
    print("blobs: %d" % len(xy))
    feat = slot_features(F, t, sh, xy, t0, slot_s, nslots)
    best, gap, dist, good = classify(feat, table, nsym)

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
