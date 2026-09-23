"""Check that every bolt has something to bolt into, offline.

A clearance hole is easy to draw on a plate and means nothing on its own: the bolt has to
pass through it and land on a matching hole or a threaded insert in the part underneath.
Nothing in this repo checked that the two patterns agree, and a bolt circle can be moved
on one part without the other noticing.

Each part's holes are transformed into the assembly frame using the placements recorded by
assembly.py, so this uses the layout that was actually built rather than re-deriving it.
Then every M3 clearance hole is matched against the features of every other placed part:

    another clearance hole   a through bolt with a nut on the far side
    an insert pocket         a bolt threaded into brass

A hole with no mate is a bolt with nothing to grip. Exits non-zero if any exist.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType

# Holes whose mating part is bought, not printed, so having no mate in the assembly is
# correct rather than a defect. Keyed by part, taking the hole's position and axis in the
# PART's own frame.
BOUGHT = {
    # The motor bolts to the underside of the cradle plate: rectangular 16x19 pattern.
    "gimbal-cradle": lambda at, d: abs(d[2]) > 0.9 and at[2] < -80.0,
    # The MG996R's own flange bolts, on the plate. The two holes through the wall are what
    # actually holds the bracket to the airframe, and those must mate.
    "gimbal-servo-bracket": lambda at, d: abs(d[2]) > 0.9,
}

CLEAR_DIA = 3.4
INSERT_DIA = 4.0
POS_TOL = 0.8      # mm between axis lines
ANG_TOL = 2.0      # degrees between axes


def features(solid, dia):
    """Distinct holes of one diameter as (axis, point_on_axis, t_lo, t_hi)."""
    out, seen = [], set()
    for f in solid.Faces():
        a = BRepAdaptor_Surface(f.wrapped)
        if a.GetType() != GeomAbs_SurfaceType.GeomAbs_Cylinder:
            continue
        c = a.Cylinder()
        if abs(2 * c.Radius() - dia) > 0.06:
            continue
        ax = c.Axis()
        d = (ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z())
        loc = (ax.Location().X(), ax.Location().Y(), ax.Location().Z())
        for comp in d:
            if abs(comp) > 1e-9:
                if comp < 0:
                    d = (-d[0], -d[1], -d[2])
                break
        ts = [sum((v.toTuple()[i] - loc[i]) * d[i] for i in range(3)) for v in f.Vertices()]
        if not ts:
            continue
        lo, hi = min(ts), max(ts)
        tfoot = sum(loc[i] * d[i] for i in range(3))
        line = tuple(round(loc[i] - tfoot * d[i], 1) for i in range(3))
        key = (tuple(round(x, 2) for x in d), line, round((lo + hi) / 2, 0))
        if key in seen:
            continue
        seen.add(key)
        out.append((d, loc, lo, hi))
    return out


def place(d, loc, rz_deg, xyz):
    """Rotate about Z then translate, matching assembly.py."""
    a = math.radians(rz_deg)
    ca, sa = math.cos(a), math.sin(a)
    rd = (d[0] * ca - d[1] * sa, d[0] * sa + d[1] * ca, d[2])
    rl = (loc[0] * ca - loc[1] * sa, loc[0] * sa + loc[1] * ca, loc[2])
    return rd, tuple(rl[i] + xyz[i] for i in range(3))


def axes_match(d1, p1, d2, p2):
    dot = abs(sum(d1[i] * d2[i] for i in range(3)))
    if math.degrees(math.acos(max(-1.0, min(1.0, dot)))) > ANG_TOL:
        return None
    w = tuple(p2[i] - p1[i] for i in range(3))
    t = sum(w[i] * d1[i] for i in range(3))
    perp = math.sqrt(max(0.0, sum(w[i] * w[i] for i in range(3)) - t * t))
    return perp if perp <= POS_TOL else None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--json", type=Path, default=None)
    args = p.parse_args(argv)

    report = json.loads((args.input / "vehicle-assembly.json").read_text(encoding="utf-8"))
    placements = report["placements"]

    cache = {}
    for name in placements:
        f = args.input / f"{name}.step"
        if not f.exists():
            continue
        s = cq.importers.importStep(str(f)).val().Solids()[0]
        cache[name] = {"clear": features(s, CLEAR_DIA), "insert": features(s, INSERT_DIA)}

    # Everything, placed.
    world = []
    for name, places in placements.items():
        if name not in cache:
            continue
        for idx, pl in enumerate(places):
            xyz, rz = pl["xyz_mm"], pl["rz_deg"]
            for kind in ("clear", "insert"):
                for (d, loc, lo, hi) in cache[name][kind]:
                    wd, wl = place(d, loc, rz, xyz)
                    world.append({"part": name, "inst": idx, "kind": kind,
                                  "d": wd, "p": wl, "d_local": d,
                                  "at_local": tuple(round(loc[i] + d[i] * (lo + hi) / 2, 1)
                                                    for i in range(3)),
                                  "at": tuple(round(wl[i] + wd[i] * (lo + hi) / 2, 1)
                                              for i in range(3))})

    for w in world:
        f = BOUGHT.get(w["part"])
        w["bought"] = bool(f and f(w["at_local"], w["d_local"]))
    holes = [w for w in world if w["kind"] == "clear" and not w["bought"]]
    bought_n = sum(1 for w in world if w["kind"] == "clear" and w["bought"])
    print(f"  {bought_n} clearance holes take bought hardware (motor, servo) and are "
          f"not expected to mate with a printed part")
    print(f"Matching {len(holes)} M3 clearance holes against "
          f"{len(world) - len(holes)} insert pockets and each other")
    orphans, matched = [], 0
    for h in holes:
        mates = []
        for o in world:
            if o["part"] == h["part"] and o["inst"] == h["inst"]:
                continue
            g = axes_match(h["d"], h["p"], o["d"], o["p"])
            if g is not None:
                mates.append((o["part"], o["kind"], round(g, 2)))
        if mates:
            matched += 1
        else:
            orphans.append(h)
    by_part = {}
    for h in holes:
        by_part.setdefault(h["part"], [0, 0])[0] += 1
    for h in orphans:
        by_part[h["part"]][1] += 1
    for name, (tot, orph) in sorted(by_part.items()):
        flag = "ok" if not orph else f"{orph} with NOTHING TO BOLT INTO"
        print(f"  {name:24s} {tot:3d} clearance holes  {flag}")
    if orphans:
        print(f"  {len(orphans)} orphaned bolt hole(s):")
        shown = {}
        for h in orphans:
            k = (h["part"], h["inst"])
            shown.setdefault(k, []).append(h["at"])
        for (name, inst), spots in sorted(shown.items()):
            print(f"    {name} #{inst}: " + ", ".join(str(s) for s in spots[:4])
                  + (" ..." if len(spots) > 4 else ""))
    if args.json:
        args.json.write_text(json.dumps(
            {"matched": matched, "orphans": [{"part": o["part"], "at": o["at"]}
                                             for o in orphans]}, indent=1), encoding="utf-8")
    print(f"  {matched}/{len(holes)} matched")
    return 2 if orphans else 0


if __name__ == "__main__":
    raise SystemExit(main())
