#!/usr/bin/env python3
"""Print a crop box "x,y,w,h" (source pixels) around the lit LEDs of a clip, for extract.py --crop.

    treebox.py clip.mov [--pad 120]

Takes the per-pixel maximum over every 10th frame at 1/4 size and boxes the largest bright cluster.
"""
import argparse
import subprocess

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clip")
    ap.add_argument("--pad", type=int, default=120)
    args = ap.parse_args()
    w, h = 960, 540
    p = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", args.clip, "-vf",
         "select='not(mod(n\\,10))',scale=%d:%d,format=gray" % (w, h), "-vsync", "0", "-f", "rawvideo", "-"],
        stdout=subprocess.PIPE)
    mx = np.zeros((h, w), np.uint8)
    while True:
        b = p.stdout.read(w * h)
        if len(b) < w * h:
            break
        mx = np.maximum(mx, np.frombuffer(b, np.uint8).reshape(h, w))
    base = cv2.GaussianBlur(mx, (0, 0), 2)
    m = (base > max(60, np.percentile(base, 99.5) * 0.5)).astype(np.uint8)
    m = cv2.dilate(m, np.ones((15, 15), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    x, y, ww, hh, _ = st[1 + np.argmax(st[1:, 4])]
    x0, y0 = max(0, 4 * x - args.pad), max(0, 4 * y - args.pad)
    x1, y1 = min(3840, 4 * (x + ww) + args.pad), min(2160, 4 * (y + hh) + args.pad)
    print("%d,%d,%d,%d" % (x0, y0, x1 - x0, y1 - y0))


if __name__ == "__main__":
    main()
