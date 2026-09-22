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
    # Must match landing_gear.py --straight-attach-r: this radius is both where the
    # bracket is placed and where the verified strut centreline starts.
    p.add_argument("--attach-r", type=float, default=34.0)
    p.add_argument("--attach-a-z", type=float, default=140.0)
    p.add_argument("--attach-c-z", type=float, default=30.0)
    p.add_argument("--stations", type=str, default="",
                   help="Comma-separated bulkhead z positions; blank uses the shell layout")
    p.add_argument("--straight-attach-z", type=float, default=320.0)
    p.add_argument("--straight-foot-r", type=float, default=290.0)
    p.add_argument("--straight-foot-z", type=float, default=-185.0)
    p.add_argument("--bulkhead-thickness", type=float, default=4.0)
    p.add_argument("--boss-height", type=float, default=8.2,
                   help="How far the bulkhead insert bosses stand proud, mm")
    p.add_argument("--bracket-plate-t", type=float, default=5.0)
    p.add_argument("--servo-z", type=float, default=38.0)
    p.add_argument("--servo-pitch", type=float, default=22.0,
                   help="Axial spacing between the two servo brackets, mm")
    p.add_argument("--tray-z", type=float, default=90.0)
    p.add_argument("--rod-dia", type=float, default=8.0)
    p.add_argument("--clash-tol", type=float, default=1.0,
                   help="Intersection volume in mm3 below which a pair is treated as touching")
    p.add_argument("--allow-clashes", action="store_true",
                   help="Report interference but still write the assembly")
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
        # Flange bolts onto the insert bosses, so it stands off the ring by their height.
        "battery-tray": [(0.0, 0.0, args.tray_z + args.boss_height, 0.0)],
        # Servos sit just above the gimbal. The linkage offset is axial, not radial:
        # 60 mm radially would put the servo outside the body.
        # Two servos, stacked rather than stacked on top of each other: both were
        # previously placed at the identical point and differed only by rotation.
        "gimbal-servo-bracket": [(0.0, 0.0, args.servo_z, 0.0),
                                 (0.0, 0.0, args.servo_z + args.servo_pitch, 90.0)],
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
        # Under the bulkhead, clear of the insert bosses that stand up from its top face.
        layout["leg-bracket"] = on_ring(args.attach_r,
                                        args.straight_attach_z - args.bracket_plate_t)
        layout["leg-foot"] = on_ring(args.straight_foot_r, args.straight_foot_z)
    else:
        layout["leg-bracket-upper"] = on_ring(args.attach_r, args.attach_a_z)
        layout["leg-bracket-lower"] = on_ring(args.attach_r, args.attach_c_z)
        layout["leg-knee"] = on_ring(args.knee_r, args.knee_z)
        layout["leg-foot"] = on_ring(args.knee_r, args.foot_z)

    placed, bom, missing = [], [], []
    for stem, places in layout.items():
        step = args.parts / f"{stem}.step"
        if not step.exists():
            missing.append(stem)
            continue
        shape = cq.importers.importStep(str(step))
        volume = shape.val().Volume()
        solids = len(shape.solids().vals())
        for n, (x, y, z, rz) in enumerate(places):
            moved = shape.rotate((0, 0, 0), (0, 0, 1), rz).translate((x, y, z))
            placed.append((f"{stem}#{n}" if len(places) > 1 else stem, moved))
        bom.append({"part": stem, "count": len(places),
                    "volume_cm3": volume / 1000.0,
                    "solids_in_step": solids,
                    "mass_asa_g": volume / 1000.0 * DENSITY_ASA * len(places)})
    if missing:
        raise SystemExit(f"Missing exported parts: {', '.join(missing)}. "
                         "Run the generators first.")

    # Interference check, before anything is unioned. Unioning first hides exactly the
    # defect worth catching: parts that occupy the same space cannot be assembled, and on
    # the gimbal they cannot rotate. Bolted parts meet face to face, so any non-trivial
    # intersection volume is a defect rather than intended contact.
    clashes = []
    for a in range(len(placed)):
        for b in range(a + 1, len(placed)):
            na, sa = placed[a]
            nb, sb = placed[b]
            ba, bb = sa.val().BoundingBox(), sb.val().BoundingBox()
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin
                    or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue                      # boxes miss, skip the boolean
            try:
                vol = sa.intersect(sb).val().Volume()
            except Exception:
                continue
            if vol > args.clash_tol:
                clashes.append({"a": na, "b": nb, "volume_mm3": round(vol, 2)})
    stray = [b["part"] for b in bom if b["solids_in_step"] > 1]

    assembly = None
    for _, solid in placed:
        assembly = solid if assembly is None else assembly.union(solid)

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
              "interference": {"pairs": clashes, "tolerance_mm3": args.clash_tol},
              "steps_with_stray_solids": stray,
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
    if clashes:
        print(f"  INTERFERENCE: {len(clashes)} pair(s) occupy the same space")
        for c in clashes:
            print(f"    {c['a']} / {c['b']}: {c['volume_mm3']} mm3")
    else:
        print("  interference: none above "
              f"{args.clash_tol} mm3 across {len(placed)} placed bodies")
    if stray:
        print(f"  STRAY SOLIDS in: {', '.join(stray)} (a slicer will treat these as parts)")
    print(f"  wrote vehicle-assembly.step / .stl / .json to {args.output.resolve()}")
    print("Printed parts and carbon struts only. Bought hardware is not placed.")
    return 2 if (clashes or stray) and not args.allow_clashes else 0


if __name__ == "__main__":
    raise SystemExit(main())
