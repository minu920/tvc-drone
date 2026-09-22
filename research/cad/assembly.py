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
    p.add_argument("--no-shell", action="store_true",
                   help="Bare carbon frame: skip the skirt, body halves, nose and fins")
    p.add_argument("--body-z", type=float, default=50.0)
    p.add_argument("--body-length", type=float, default=320.0)
    p.add_argument("--legs", type=int, default=3)
    p.add_argument("--legs-style", choices=("straight", "truss"), default="straight")
    p.add_argument("--knee-r", type=float, default=215.0)
    p.add_argument("--knee-z", type=float, default=15.0)
    p.add_argument("--foot-z", type=float, default=-215.0)
    p.add_argument("--attach-r", type=float, default=43.0)
    p.add_argument("--attach-a-z", type=float, default=140.0)
    p.add_argument("--attach-c-z", type=float, default=30.0)
    p.add_argument("--stations", type=str, default="",
                   help="Comma-separated bulkhead z positions; blank uses the shell layout")
    p.add_argument("--straight-attach-z", type=float, default=290.0)
    p.add_argument("--straight-foot-r", type=float, default=290.0)
    p.add_argument("--straight-foot-z", type=float, default=-185.0)
    p.add_argument("--rod-dia", type=float, default=8.0)
    p.add_argument("--spine-count", type=int, default=4)
    p.add_argument("--spine-bcd", type=float, default=70.0)
    p.add_argument("--spine-dia", type=float, default=8.0)
    p.add_argument("--spine-overhang", type=float, default=12.0,
                   help="How far the spine runs past the end stations, mm")
    args = p.parse_args(argv)

    stations = ([float(v) for v in args.stations.split(",")] if args.stations
                else [args.body_z + args.body_length * f
                      for f in (0.04, 0.34, 0.64, 0.93)])
    leg_az = [360.0 * i / args.legs for i in range(args.legs)]

    def on_ring(radius, z):
        return [(radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a)),
                 z, a) for a in leg_az]

    layout = {
        # Already in vehicle coordinates.
        "gimbal-outer-ring": IDENTITY,
        "gimbal-inner-ring": IDENTITY,
        "gimbal-cradle": IDENTITY,
        # Placed here.
        "bulkhead": [(0.0, 0.0, z, 0.0) for z in stations],
        "battery-tray": [(0.0, 0.0, stations[0] + 14.0, 0.0)],
        # Servos sit just above the gimbal. The linkage offset is axial, not radial:
        # 60 mm radially would put the servo outside the body.
        "gimbal-servo-bracket": [(0.0, 0.0, args.body_z + 20.0, 0.0),
                                 (0.0, 0.0, args.body_z + 20.0, 90.0)],
    }
    if not args.no_shell:
        layout.update({
            "rocket-shell-skirt": IDENTITY,
            "rocket-shell-body-half-a": IDENTITY,
            "rocket-shell-body-half-b": IDENTITY,
            "rocket-shell-nose": IDENTITY,
            "rocket-shell-fin": [(0.0, 0.0, 0.0, 90.0 * i) for i in range(4)],
        })
    if args.legs_style == "straight":
        layout["leg-bracket"] = on_ring(args.attach_r, args.straight_attach_z)
        layout["leg-foot"] = on_ring(args.straight_foot_r, args.straight_foot_z)
    else:
        layout["leg-bracket-upper"] = on_ring(args.attach_r, args.attach_a_z)
        layout["leg-bracket-lower"] = on_ring(args.attach_r, args.attach_c_z)
        layout["leg-knee"] = on_ring(args.knee_r, args.knee_z)
        layout["leg-foot"] = on_ring(args.knee_r, args.foot_z)

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
    if args.legs_style == "straight":
        struts = [((args.attach_r, args.straight_attach_z),
                   (args.straight_foot_r, args.straight_foot_z))]
    else:
        struts = [((args.attach_r, args.attach_a_z), (args.knee_r, args.knee_z)),
                  ((args.attach_r, args.attach_c_z), (args.knee_r, args.knee_z)),
                  ((args.knee_r, args.knee_z), (args.knee_r, args.foot_z))]
    rod_len_mm = 0.0
    for a in leg_az:
        for p0, p1 in struts:
            assembly = assembly.union(cq.Workplane(obj=rod(p0, p1, a, args.rod_dia)))
            rod_len_mm += math.dist(p0, p1)

    # Carbon spine. This is the main structural member: the bulkheads are threaded onto it
    # and the gimbal hangs off the bottom station. Leaving it out makes the rings look like
    # they float.
    spine_r = args.spine_bcd / 2.0
    spine_len_mm = 0.0
    for i in range(args.spine_count):
        az = 360.0 * i / args.spine_count + 180.0 / args.spine_count
        p0 = (spine_r, min(stations) - args.spine_overhang)
        p1 = (spine_r, max(stations) + args.spine_overhang)
        assembly = assembly.union(cq.Workplane(obj=rod(p0, p1, az, args.spine_dia)))
        spine_len_mm += p1[1] - p0[1]

    args.output.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(assembly, str(args.output / "vehicle-assembly.step"))
    cq.exporters.export(assembly, str(args.output / "vehicle-assembly.stl"))
    bb = assembly.val().BoundingBox()

    printed_total = sum(b["mass_asa_g"] for b in bom)
    report = {"frame": "z = 0 at the gimbal pivot, +Z up",
              "configuration": {"shell": not args.no_shell, "legs": args.legs_style,
                                "leg_count": args.legs,
                                "bulkhead_stations_mm": stations},
              "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
              "z_range_mm": [round(bb.zmin, 1), round(bb.zmax, 1)],
              "printed_parts": bom,
              "printed_kinds": len(bom),
              "printed_pieces": sum(b["count"] for b in bom),
              "printed_mass_asa_g": printed_total,
              "carbon_leg_rod_total_mm": rod_len_mm,
              "carbon_spine_total_mm": spine_len_mm,
              "carbon_rod_total_mm": rod_len_mm + spine_len_mm,
              "not_in_this_assembly": [
                  "Motors, propellers, ESCs, battery, flight controller, servos, bearings, "
                  "pushrods, ball links and fasteners.",
                  "Pushrod routing is not laid out. The linkage was verified kinematically, "
                  "and its 60 mm offset has to be taken axially rather than radially, but "
                  "the rod paths and horn positions are not drawn.",
                  "With no shell the electronics have nothing to mount against except the "
                  "bulkheads and the carbon spine; those brackets are not designed.",
              ], "offline_only": True}
    (args.output / "vehicle-assembly.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Vehicle assembly  {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, "
          f"z {bb.zmin:.0f} to {bb.zmax:.0f}")
    print(f"  {'part':28s} {'qty':>4s} {'ASA g':>8s}")
    for b in bom:
        print(f"  {b['part']:28s} {b['count']:4d} {b['mass_asa_g']:8.1f}")
    print(f"  {'':28s} {report['printed_pieces']:4d} {printed_total:8.1f}")
    print(f"  {report['printed_kinds']} kinds, {report['printed_pieces']} pieces printed")
    print(f"  carbon: legs {rod_len_mm / 1000:.2f} m + spine {spine_len_mm / 1000:.2f} m "
          f"= {(rod_len_mm + spine_len_mm) / 1000:.2f} m of {args.rod_dia:.0f} mm rod")
    print(f"  wrote vehicle-assembly.step / .stl / .json to {args.output.resolve()}")
    print("Printed parts and carbon struts only. Bought hardware is not placed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
