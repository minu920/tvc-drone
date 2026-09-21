"""Place every printed part into one assembly, offline.

The generators each emit their parts about a convenient origin. This puts them where they
actually sit, in the shared frame the gimbal defines: z = 0 at the gimbal pivot, +Z up.
The result is one STEP that opens in CATIA or any other CAD in assembled position, plus a
bill of printed parts.

Struts are carbon rod and are drawn as plain cylinders on the verified centrelines. They
are bought stock, not printed, and are included so the assembly reads as a vehicle rather
than a pile of fittings.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

DENSITY_ASA = 1.07

# Printed parts: file stem -> (count, placements). A placement is (x, y, z, rot_z_deg).
# Parts already generated in vehicle coordinates get a single identity placement.
IDENTITY = [(0.0, 0.0, 0.0, 0.0)]


def ring_placements(count, radius, z, start_deg=0.0):
    return [(radius * math.cos(math.radians(start_deg + 360 * i / count)),
             radius * math.sin(math.radians(start_deg + 360 * i / count)),
             z, start_deg + 360 * i / count) for i in range(count)]


def rod(p0_rz, p1_rz, azimuth_deg, dia):
    """Carbon strut between two (radius, z) points at a given azimuth.

    makeCylinder takes a base point and a direction, which avoids building a rotation from
    an axis that degenerates when the strut happens to be vertical.
    """
    a = math.radians(azimuth_deg)
    p0 = cq.Vector(p0_rz[0] * math.cos(a), p0_rz[0] * math.sin(a), p0_rz[1])
    p1 = cq.Vector(p1_rz[0] * math.cos(a), p1_rz[0] * math.sin(a), p1_rz[1])
    direction = p1.sub(p0)
    return cq.Solid.makeCylinder(dia / 2, direction.Length, p0, direction.normalized())


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--parts", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--output", type=Path, default=Path("../../artifacts/cad"))
    p.add_argument("--body-z", type=float, default=50.0)
    p.add_argument("--body-length", type=float, default=320.0)
    p.add_argument("--legs", type=int, default=3)
    p.add_argument("--knee-r", type=float, default=215.0)
    p.add_argument("--knee-z", type=float, default=15.0)
    p.add_argument("--foot-z", type=float, default=-215.0)
    p.add_argument("--attach-r", type=float, default=43.0)
    p.add_argument("--attach-a-z", type=float, default=140.0)
    p.add_argument("--attach-c-z", type=float, default=30.0)
    p.add_argument("--rod-dia", type=float, default=8.0)
    args = p.parse_args(argv)

    stations = [args.body_z + args.body_length * f for f in (0.04, 0.34, 0.64, 0.93)]
    leg_az = [360.0 * i / args.legs for i in range(args.legs)]

    layout = {
        # Already in vehicle coordinates.
        "gimbal-outer-ring": IDENTITY,
        "gimbal-inner-ring": IDENTITY,
        "gimbal-cradle": IDENTITY,
        "rocket-shell-skirt": IDENTITY,
        "rocket-shell-body-half-a": IDENTITY,
        "rocket-shell-body-half-b": IDENTITY,
        "rocket-shell-nose": IDENTITY,
        "rocket-shell-fin": [(0.0, 0.0, 0.0, 90.0 * i) for i in range(4)],
        # Placed here.
        "bulkhead": [(0.0, 0.0, z, 0.0) for z in stations],
        "battery-tray": [(0.0, 0.0, stations[0] + 14.0, 0.0)],
        # Servos sit just above the gimbal, inside the bay. The linkage offset is axial,
        # not radial: 60 mm radially would put the servo outside the 43 mm body.
        "gimbal-servo-bracket": [(0.0, 0.0, args.body_z + 20.0, 0.0),
                                 (0.0, 0.0, args.body_z + 20.0, 90.0)],
        "leg-bracket-upper": [(args.attach_r * math.cos(math.radians(a)),
                               args.attach_r * math.sin(math.radians(a)),
                               args.attach_a_z, a) for a in leg_az],
        "leg-bracket-lower": [(args.attach_r * math.cos(math.radians(a)),
                               args.attach_r * math.sin(math.radians(a)),
                               args.attach_c_z, a) for a in leg_az],
        "leg-knee": [(args.knee_r * math.cos(math.radians(a)),
                      args.knee_r * math.sin(math.radians(a)),
                      args.knee_z, a) for a in leg_az],
        "leg-foot": [(args.knee_r * math.cos(math.radians(a)),
                      args.knee_r * math.sin(math.radians(a)),
                      args.foot_z, a) for a in leg_az],
    }

    assembly, bom, missing = None, [], []
    for stem, places in layout.items():
        step = args.parts / f"{stem}.step"
        if not step.exists():
            missing.append(stem)
            continue
        shape = cq.importers.importStep(str(step))
        volume = shape.val().Volume()
        for x, y, z, rz in places:
            moved = shape.rotate((0, 0, 0), (0, 0, 1), rz).translate((x, y, z))
            assembly = moved if assembly is None else assembly.union(moved)
        bom.append({"part": stem, "count": len(places),
                    "volume_cm3": volume / 1000.0,
                    "mass_asa_g": volume / 1000.0 * DENSITY_ASA * len(places)})
    if missing:
        raise SystemExit(f"Missing exported parts: {', '.join(missing)}. "
                         "Run the generators first.")

    # Carbon struts, drawn for context only.
    struts = [((args.attach_r, args.attach_a_z), (args.knee_r, args.knee_z)),
              ((args.attach_r, args.attach_c_z), (args.knee_r, args.knee_z)),
              ((args.knee_r, args.knee_z), (args.knee_r, args.foot_z))]
    rod_len_mm = 0.0
    for a in leg_az:
        for p0, p1 in struts:
            assembly = assembly.union(cq.Workplane(obj=rod(p0, p1, a, args.rod_dia)))
            rod_len_mm += math.dist(p0, p1)

    args.output.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(assembly, str(args.output / "vehicle-assembly.step"))
    cq.exporters.export(assembly, str(args.output / "vehicle-assembly.stl"))
    bb = assembly.val().BoundingBox()

    printed_total = sum(b["mass_asa_g"] for b in bom)
    report = {"frame": "z = 0 at the gimbal pivot, +Z up",
              "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
              "z_range_mm": [round(bb.zmin, 1), round(bb.zmax, 1)],
              "printed_parts": bom,
              "printed_kinds": len(bom),
              "printed_pieces": sum(b["count"] for b in bom),
              "printed_mass_asa_g": printed_total,
              "carbon_rod_total_mm": rod_len_mm,
              "not_in_this_assembly": [
                  "Motors, propellers, ESCs, battery, flight controller, servos, bearings, "
                  "pushrods, ball links and fasteners.",
                  "Pushrod routing is not laid out. The linkage was verified kinematically, "
                  "and its 60 mm offset has to be taken axially rather than radially, but "
                  "the rod paths and horn positions are not drawn.",
              ], "offline_only": True}
    (args.output / "vehicle-assembly.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Vehicle assembly  {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, "
          f"z {bb.zmin:.0f} to {bb.zmax:.0f}")
    print(f"  {'part':28s} {'qty':>4s} {'ASA g':>8s}")
    for b in bom:
        print(f"  {b['part']:28s} {b['count']:4d} {b['mass_asa_g']:8.1f}")
    print(f"  {'':28s} {report['printed_pieces']:4d} {printed_total:8.1f}")
    print(f"  {report['printed_kinds']} kinds, {report['printed_pieces']} pieces printed; "
          f"carbon rod {rod_len_mm / 1000:.2f} m")
    print(f"  wrote vehicle-assembly.step / .stl / .json to {args.output.resolve()}")
    print("Printed parts and carbon struts only. Bought hardware is not placed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
