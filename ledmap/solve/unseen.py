#!/usr/bin/env python3
"""List the LEDs a layout does not have, for `idseq16 --only`.

    unseen.py layout.csv -o unseen.txt [--strips 16] [--leds 300] [--skip-strips 6]
              [--sources decoded]

Writes one "strip,led" per line. By default an LED counts as seen if the layout
has a position for it, however it got one. With --sources decoded only LEDs the
decoder actually identified count (mirror, interp and extrap entries are listed
again, which checks the fold assumption and the interpolation).
"""
import argparse
import csv


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("layout")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--strips", type=int, default=16)
    ap.add_argument("--leds", type=int, default=300)
    ap.add_argument("--skip-strips", default="", help="comma-separated strips to leave out (dead, not connected)")
    ap.add_argument("--sources", help="comma-separated layout sources that count as seen (default: all)")
    args = ap.parse_args()

    skip = {int(v) for v in args.skip_strips.split(",") if v}
    want = set(args.sources.split(",")) if args.sources else None
    seen = set()
    for r in csv.DictReader(l for l in open(args.layout) if not l.startswith("#")):
        if want is None or r.get("source") in want:
            seen.add((int(r["strip"]), int(r["led"])))
    todo = [(s, l) for s in range(args.strips) if s not in skip
            for l in range(args.leds) if (s, l) not in seen]
    with open(args.out, "w") as f:
        f.write("# LEDs not seen in %s\n" % args.layout)
        for s, l in todo:
            f.write("%d,%d\n" % (s, l))
    print("%d LEDs listed (%d seen, %d strips skipped) -> %s" %
          (len(todo), len(seen), len(skip), args.out))


if __name__ == "__main__":
    main()
