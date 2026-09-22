"""Audit every heat-set insert pocket in every part, offline.

A pocket is only as good as the wall around it. Measuring air along the pocket axis is not
enough: a pocket that opens into a bore, a slot or the far face is full of air for its
whole length and still has nothing for the insert to grip. Two of the four pockets in this
design failed that way, one with 1.0 mm of real wall and one with 5.06 mm, while an
axis-only probe reported both as a full 6.7 mm.

So every Ø(pilot) cylindrical hole is found in the solid, and the wall is probed around the
pocket at each station along it. The supported depth ends where the wall does.

Run it after any geometry change. It exits non-zero if any pocket is short.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType

import features as F

PARTS = ["gimbal-outer-ring", "gimbal-inner-ring", "gimbal-cradle", "bulkhead",
         "battery-tray", "gimbal-servo-bracket", "leg-bracket", "leg-foot", "fit-coupon"]


def _perp(d):
    """Two unit vectors spanning the plane normal to d."""
    a = (1.0, 0.0, 0.0) if abs(d[2]) > 0.9 else (0.0, 0.0, 1.0)
    u = (a[1]*d[2] - a[2]*d[1], a[2]*d[0] - a[0]*d[2], a[0]*d[1] - a[1]*d[0])
    n = math.sqrt(sum(c*c for c in u)) or 1.0
    u = tuple(c/n for c in u)
    v = (d[1]*u[2] - d[2]*u[1], d[2]*u[0] - d[0]*u[2], d[0]*u[1] - d[1]*u[0])
    return u, v


def supported_depth(solid, mouth, direction, spec=F.INSERT_M3, step=0.1, probes=8):
    """How far a continuous wall follows the pocket from its mouth inward."""
    u, v = _perp(direction)
    r = spec["hole_dia"] / 2 + 0.3
    depth = 0.0
    while depth < spec["depth"] + step:
        d = depth + step / 2
        c = [mouth[i] + direction[i] * d for i in range(3)]
        for k in range(probes):
            th = 2 * math.pi * k / probes
            pt = cq.Vector(*[c[i] + r * (math.cos(th) * u[i] + math.sin(th) * v[i])
                             for i in range(3)])
            if not solid.isInside(pt, 1e-6):
                return depth
        depth += step
    return depth


def pockets(solid, spec=F.INSERT_M3):
    """Every pilot-diameter hole, as (mouth point, inward direction) pairs."""
    found, seen = [], set()
    for f in solid.Faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() != GeomAbs_SurfaceType.GeomAbs_Cylinder:
            continue
        cyl = s.Cylinder()
        if abs(2 * cyl.Radius() - spec["hole_dia"]) > 0.05:
            continue
        ax = cyl.Axis()
        d = (ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z())
        bb = f.BoundingBox()
        ends = [(bb.xmin, bb.ymin, bb.zmin), (bb.xmax, bb.ymax, bb.zmax)]
        # The mouth is whichever end of the hole is NOT buried: probe just outside each.
        for sign in (+1, -1):
            dd = tuple(sign * c for c in d)
            lo = [bb.xmin, bb.ymin, bb.zmin]
            hi = [bb.xmax, bb.ymax, bb.zmax]
            mouth = [(hi[i] if dd[i] < 0 else lo[i]) if abs(d[i]) > 0.5
                     else (lo[i] + hi[i]) / 2 for i in range(3)]
            key = (tuple(round(c, 1) for c in mouth), tuple(round(c, 1) for c in dd))
            if key in seen:
                continue
            outside = cq.Vector(*[mouth[i] - dd[i] * 0.4 for i in range(3)])
            if solid.isInside(outside, 1e-6):
                continue          # buried end, not the mouth
            seen.add(key)
            found.append((tuple(mouth), dd))
    return found


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--json", type=Path, default=None)
    args = p.parse_args(argv)

    need = F.INSERT_M3["depth"]
    report, short = {}, 0
    print(f"M3 heat-set insert, {F.INSERT_M3['length']} mm long, needs a {need} mm pocket "
          f"with a continuous wall.")
    for name in PARTS:
        f = args.input / f"{name}.step"
        if not f.exists():
            continue
        solid = cq.importers.importStep(str(f)).val().Solids()[0]
        rows = []
        for mouth, d in pockets(solid):
            got = supported_depth(solid, mouth, d)
            ok = got >= need - 0.05
            short += 0 if ok else 1
            rows.append({"mouth_mm": [round(c, 1) for c in mouth],
                         "direction": [round(c, 2) for c in d],
                         "supported_mm": round(got, 1), "ok": ok})
        report[name] = rows
        if rows:
            print(f"  {name}")
            for r in rows:
                flag = "ok" if r["ok"] else f"SHORT by {need - r['supported_mm']:.1f} mm"
                print(f"    at {str(r['mouth_mm']):>22s} dir {str(r['direction']):>16s} "
                      f"{r['supported_mm']:4.1f} mm  {flag}")
    total = sum(len(v) for v in report.values())
    print(f"  {total} pocket(s) across {sum(1 for v in report.values() if v)} part(s), "
          f"{short} short")
    if args.json:
        args.json.write_text(json.dumps(report, indent=1), encoding="utf-8")
    if short:
        print("An insert in a pocket shorter than itself sits proud, or spins because the "
              "wall it should bite into is not there. Fix the geometry before printing.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
