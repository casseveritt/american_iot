#!/usr/bin/env python3
"""Decode a clip once into a frame cache the other detect tools can use.

Writes <out>/frames.npy (N, H, W, 3 uint8 RGB, memory-mappable) and
<out>/pts.npy (N presentation times in seconds). Frames are taken as they
are in the file (no frame-rate conversion), so dropped frames show up as
gaps in pts. HLG/HDR clips are converted to SDR BT.709.

    extract.py clip.MOV cache_dir [--width 1080]
"""
import argparse
import os
import subprocess

import numpy as np


def probe(path):
    out = subprocess.check_output(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height,color_transfer:stream_side_data=rotation",
         "-of", "default=nw=1", path], text=True)
    info = dict(l.split("=", 1) for l in out.split() if "=" in l)
    return info


def frame_times(path):
    out = subprocess.check_output(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "frame=pts_time", "-of", "csv=p=0", path])
    # side data adds trailing commas to some rows
    return np.array([float(x.split(b",")[0]) for x in out.split() if x.strip(b",")])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clip")
    ap.add_argument("out")
    ap.add_argument("--width", type=int, default=1080,
                    help="output width after rotation (default 1080)")
    args = ap.parse_args()

    info = probe(args.clip)
    w, h = int(info["width"]), int(info["height"])
    if info.get("rotation", "0") in ("90", "-90", "270", "-270"):
        w, h = h, w
    ow = args.width
    oh = round(h * ow / w / 2) * 2

    pts = frame_times(args.clip)
    hlg = info.get("color_transfer") == "arib-std-b67"
    if hlg:
        vf = ("zscale=w=%d:h=%d:tin=arib-std-b67:pin=bt2020:min=bt2020nc:rin=tv:"
              "t=bt709:p=bt709:m=bt709:r=pc,format=rgb24" % (ow, oh))
    else:
        vf = "scale=%d:%d,format=rgb24" % (ow, oh)

    os.makedirs(args.out, exist_ok=True)
    frames = np.lib.format.open_memmap(
        os.path.join(args.out, "frames.npy"), mode="w+", dtype=np.uint8,
        shape=(len(pts), oh, ow, 3))
    p = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", args.clip, "-fps_mode", "passthrough",
         "-vf", vf, "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
        stdout=subprocess.PIPE)
    size = ow * oh * 3
    n = 0
    while n < len(pts):
        b = p.stdout.read(size)
        if len(b) < size:
            break
        frames[n] = np.frombuffer(b, np.uint8).reshape(oh, ow, 3)
        n += 1
    p.stdout.close()
    p.wait()
    frames.flush()
    np.save(os.path.join(args.out, "pts.npy"), pts[:n])
    print("%s: %d frames, %dx%d, %.2f..%.2f s%s" %
          (args.clip, n, ow, oh, pts[0], pts[n - 1], " (HLG->SDR)" if hlg else ""))
    if n != len(pts):
        print("warning: ffprobe saw %d frames, decoded %d" % (len(pts), n))


if __name__ == "__main__":
    main()
