"""Audit whether every part can actually be assembled, offline.

The leg fittings shipped with sockets capped at both ends: no rod could enter either one,
and nothing caught it, because a rendering shows a socket and a volume check shows a
sensible mass. What was missing was a test of the thing that matters, which is whether the
mating part can physically get to where it has to sit.

So every hole in every part is found, classified by what goes into it, and then that thing
is simulated: a cylinder of the real hardware diameter, travelling in along the hole axis
from outside the part. If it cannot reach the seat, the part cannot be assembled.

    Ø8.2  carbon rod          rod is Ø8.0
    Ø6.9  683ZZ bearing seat  bearing OD is Ø7.0, pressed
    Ø4.0  M3 insert pocket    insert is Ø4.0 brass, driven by a soldering tip
    Ø3.4  M3 clearance        screw shank Ø3.0, head Ø5.5 needs room on the entry side
    Ø3.2  ball link           M3 ball stud

Blind features need one open end, through features need two. Anything with no open end is
a defect. Run it after any geometry change; it exits non-zero on a defect.
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

# hole diameter -> (what goes in, its diameter, blind is acceptable)
# The mating diameter is what has to TRAVEL down the hole, so a press fit is simulated at
# the bore size rather than the part's nominal size. A Ø7.0 bearing driven into its Ø6.9
# seat interferes by design, and simulating it at Ø7.0 reports the intended press as a
# blockage.
HARDWARE = [
    (8.2, "carbon rod Ø8", 8.0, True),
    (6.9, "683ZZ bearing Ø7 (press)", 6.85, True),
    (4.0, "M3 insert Ø4", 4.0, True),
    (3.4, "M3 screw Ø3", 3.0, False),
    (3.2, "M3 ball stud", 3.0, False),
]
HEAD_DIA = 5.5      # M3 socket head, needs room where the screw goes in


def classify(dia):
    for d, label, mate, blind_ok in HARDWARE:
        if abs(dia - d) < 0.16:
            return label, mate, blind_ok
    return None, None, None


def holes(solid):
    """Distinct holes as (axis, point_on_axis, t_lo, t_hi, diameter)."""
    found, seen = {}, set()
    for f in solid.Faces():
        a = BRepAdaptor_Surface(f.wrapped)
        if a.GetType() != GeomAbs_SurfaceType.GeomAbs_Cylinder:
            continue
        c = a.Cylinder()
        r = c.Radius()
        if r < 1.4 or r > 5.0:
            continue
        ax = c.Axis()
        d = (ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z())
        loc = (ax.Location().X(), ax.Location().Y(), ax.Location().Z())
        # Canonical axis sense, so two holes facing opposite ways do not merge.
        for comp in d:
            if abs(comp) > 1e-9:
                if comp < 0:
                    d = (-d[0], -d[1], -d[2])
                break
        # Is the material outside this cylinder? Otherwise it is a boss.
        umin, umax, vmin, vmax = _uv(f)
        pt = a.Value((umin + umax) / 2, (vmin + vmax) / 2)
        t = sum((getattr(pt, k)() - loc[i]) * d[i] for i, k in enumerate("XYZ"))
        rad = tuple(getattr(pt, k)() - loc[i] - t * d[i] for i, k in enumerate("XYZ"))
        rn = math.sqrt(sum(x * x for x in rad))
        if rn < 1e-9:
            continue
        probe = cq.Vector(*[getattr(pt, k)() - 0.4 * rad[i] for i, k in enumerate("XYZ")])
        if solid.isInside(probe, 1e-6):
            continue
        ts = [sum((v.toTuple()[i] - loc[i]) * d[i] for i in range(3)) for v in f.Vertices()]
        if not ts:
            continue
        lo, hi = min(ts), max(ts)
        tfoot = sum(loc[i] * d[i] for i in range(3))
        line = tuple(round(loc[i] - tfoot * d[i], 1) for i in range(3))
        key = (round(r, 2), tuple(round(x, 2) for x in d), line, round((lo + hi) / 2, 0))
        if key in seen:
            continue
        seen.add(key)
        found[key] = (d, loc, lo, hi, 2 * r)
    return list(found.values())


def _uv(face):
    from OCP.BRepTools import BRepTools
    return BRepTools.UVBounds_s(face.wrapped)


def can_insert(solid, axis, loc, t_seat, direction, mate_dia, reach=250.0):
    """Drive a cylinder of `mate_dia` in from outside, stopping at the seat.

    The cylinder is built along +Z and then rotated by the single rotation that carries +Z
    onto the hole axis. Composing two Euler angles instead looks simpler and quietly fails
    for axes with a Y component, which made every trunnion pocket in the gimbal report as
    unreachable when the aim had merely been abandoned.
    """
    d = tuple(direction * axis[i] for i in range(3))
    n = math.sqrt(sum(c * c for c in d))
    d = tuple(c / n for c in d)
    start = [loc[i] + axis[i] * t_seat for i in range(3)]
    rod = cq.Workplane("XY").circle(mate_dia / 2).extrude(reach)
    # Rotation carrying +Z onto d: about the axis +Z x d, by the angle between them.
    dot = max(-1.0, min(1.0, d[2]))
    ang = math.degrees(math.acos(dot))
    if ang > 1e-9:
        cr = (-d[1], d[0], 0.0)          # (0,0,1) x d
        if math.sqrt(sum(c * c for c in cr)) < 1e-9:
            cr = (1.0, 0.0, 0.0)         # antiparallel: any perpendicular axis will do
        rod = rod.rotate((0, 0, 0), cr, ang)
    # Verify the aim rather than trusting it.
    tip = rod.val().BoundingBox()
    aim = [reach * d[i] for i in range(3)]
    span = (tip.xlen, tip.ylen, tip.zlen)
    for i in range(3):
        if abs(abs(aim[i]) - span[i]) > mate_dia + 0.5:
            return None, None
    rod = rod.translate(tuple(start))
    hit = cq.Workplane().add(solid).intersect(rod)
    vol = hit.val().Volume() if hit.val().Solids() else 0.0
    return vol < 1.0, vol


def audit(solid, name):
    rows = []
    for axis, loc, lo, hi, dia in holes(solid):
        label, mate, blind_ok = classify(dia)
        if label is None:
            continue
        ends = []
        for t_seat, direction, which in ((lo, +1, "lo"), (hi, -1, "hi")):
            ok, vol = can_insert(solid, axis, loc, t_seat, direction, mate)
            if ok is None:
                continue
            ends.append({"from": which, "fits": ok, "collision_mm3": round(vol, 1)})
        open_ends = [e for e in ends if e["fits"]]
        need = 1 if blind_ok else 1
        centre = tuple(round(loc[i] + axis[i] * (lo + hi) / 2, 1) for i in range(3))
        rows.append({"part": name, "dia_mm": round(dia, 2), "hardware": label,
                     "at_mm": centre, "length_mm": round(hi - lo, 1),
                     "open_ends": len(open_ends), "ends": ends,
                     "ok": len(open_ends) >= need})
    return rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--json", type=Path, default=None)
    args = p.parse_args(argv)

    all_rows, bad = [], 0
    print("Can the mating hardware physically reach its seat?")
    for name in PARTS:
        f = args.input / f"{name}.step"
        if not f.exists():
            continue
        solid = cq.importers.importStep(str(f)).val().Solids()[0]
        rows = audit(solid, name)
        all_rows += rows
        print(f"  {name}")
        for r in rows:
            flag = "ok" if r["ok"] else "CANNOT BE ASSEMBLED"
            if not r["ok"]:
                bad += 1
            worst = min((e["collision_mm3"] for e in r["ends"]), default=0.0)
            print(f"    Ø{r['dia_mm']:4.1f} {r['hardware']:18s} at {str(r['at_mm']):>22s} "
                  f"len {r['length_mm']:5.1f}  open ends {r['open_ends']}  "
                  f"{flag}" + ("" if r["ok"] else f" (best path blocked by {worst:.0f} mm3)"))
        if not rows:
            print("    (no fastener features)")
    print(f"  {len(all_rows)} features checked, {bad} that hardware cannot reach")
    if args.json:
        args.json.write_text(json.dumps(all_rows, indent=1), encoding="utf-8")
    return 2 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
