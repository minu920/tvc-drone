"""Check the gimbal through its whole travel, not just at rest, offline.

assembly.py checks interference with every part at its nominal position. A gimbal that
passes that test can still be a solid block, because the one pose it never checks is the
one the mechanism exists to reach. Parts that clear each other at zero can foul at 12
degrees and nothing would say so until it is printed and assembled.

So the two moving members are swept through their commanded travel and intersected at each
pose:

    outer ring   fixed, bolted to a bulkhead
    inner ring   rotates about Y inside the outer ring
    cradle       rotates about X inside the inner ring, and is carried by the inner ring

Pairs are checked only against the angles they actually depend on, so the outer/inner pair
is swept in one variable and only outer/cradle needs the full grid.
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import cadquery as cq


def rot(shape, axis, deg):
    if abs(deg) < 1e-9:
        return shape
    v = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    return shape.rotate((0, 0, 0), v, deg)


def clash(a, b, tol):
    hit = cq.Workplane().add(a).intersect(cq.Workplane().add(b))
    if not hit.val().Solids():
        return 0.0
    v = hit.val().Volume()
    return v if v > tol else 0.0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--tilt", type=float, default=15.0,
                   help="Commanded travel per axis, degrees")
    p.add_argument("--steps", type=int, default=5,
                   help="Poses sampled per axis across the full travel")
    p.add_argument("--tol", type=float, default=1.0)
    p.add_argument("--json", type=Path, default=None)
    args = p.parse_args(argv)

    parts = {}
    for n in ("gimbal-outer-ring", "gimbal-inner-ring", "gimbal-cradle"):
        parts[n] = cq.importers.importStep(str(args.input / f"{n}.step")).val().Solids()[0]
    outer, inner, cradle = (parts["gimbal-outer-ring"], parts["gimbal-inner-ring"],
                            parts["gimbal-cradle"])

    n = max(2, args.steps)
    angles = [-args.tilt + 2 * args.tilt * i / (n - 1) for i in range(n)]
    print(f"Sweeping the gimbal +-{args.tilt:.0f} deg per axis, {n} poses per axis "
          f"({n} + {n} + {n*n} intersections)")

    bad, rows = [], []
    # outer vs inner: depends on the inner ring's angle alone.
    for b in angles:
        v = clash(outer, rot(inner, "Y", b), args.tol)
        rows.append({"pair": "outer/inner", "beta": b, "alpha": None, "mm3": v})
        if v:
            bad.append(f"outer/inner at beta={b:+.1f} deg: {v:.1f} mm3")
    # inner vs cradle: depends on the cradle's angle relative to the inner ring.
    for a in angles:
        v = clash(inner, rot(cradle, "X", a), args.tol)
        rows.append({"pair": "inner/cradle", "beta": None, "alpha": a, "mm3": v})
        if v:
            bad.append(f"inner/cradle at alpha={a:+.1f} deg: {v:.1f} mm3")
    # outer vs cradle: the cradle carries both rotations.
    for b, a in itertools.product(angles, angles):
        moved = rot(rot(cradle, "X", a), "Y", b)
        v = clash(outer, moved, args.tol)
        rows.append({"pair": "outer/cradle", "beta": b, "alpha": a, "mm3": v})
        if v:
            bad.append(f"outer/cradle at beta={b:+.1f}, alpha={a:+.1f} deg: {v:.1f} mm3")

    worst = max((r["mm3"] for r in rows), default=0.0)
    for pair in ("outer/inner", "inner/cradle", "outer/cradle"):
        sub = [r for r in rows if r["pair"] == pair]
        hits = [r for r in sub if r["mm3"]]
        print(f"  {pair:14s} {len(sub):3d} poses, {len(hits)} fouling"
              + (f", worst {max(r['mm3'] for r in hits):.1f} mm3" if hits else ""))
    if args.json:
        args.json.write_text(json.dumps(rows, indent=1), encoding="utf-8")
    if bad:
        print(f"  THE GIMBAL CANNOT REACH ITS COMMANDED TRAVEL, {len(bad)} fouling pose(s):")
        for b in bad[:12]:
            print(f"    {b}")
        if len(bad) > 12:
            print(f"    ... and {len(bad) - 12} more")
        return 2
    print(f"  clear through the full +-{args.tilt:.0f} deg on both axes "
          f"(worst intersection {worst:.1f} mm3, tolerance {args.tol} mm3)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
