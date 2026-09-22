"""Generate the airframe internals and landing gear, offline.

Three groups of parts:

  bulkhead    rings that carry the carbon spine, locate the gimbal and mount electronics
  battery tray a cradle for the selected pack
  landing gear brackets and feet for carbon strut legs

The legs are printed end fittings on bought carbon rod rather than printed struts. A strut
long enough to clear the rotors would be a 260 mm printed beam loaded in bending, which is
the worst case for layer adhesion, and the baseline specification already puts structural
loads in carbon rather than in printed parts.

Leg geometry is checked against the propeller sweep envelope rather than against the
propeller diameter. The swept volume is wider than the rotor disc, so clearing 305 mm is
not enough: the strut has to stay outside the swept profile at every height it passes.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

import features as F
import prop_sweep_envelope as env

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}
M3_CLEARANCE = 3.4


def envelope_radius_at(profile, z):
    """Largest envelope radius at a height, or 0 below or above the swept volume."""
    best = 0.0
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if min(z0, z1) <= z <= max(z0, z1) and abs(z1 - z0) > 1e-9:
            f = (z - z0) / (z1 - z0)
            best = max(best, r0 + f * (r1 - r0))
    return best


def check_leg(attach, foot, spacing, thickness, tilt_deg, pivot_offset, samples=240):
    """Minimum radial gap between the strut centreline and the swept rotors.

    Uses the exact sweep test rather than the revolved keep-out solid. That solid fills the
    notch near the axis on purpose, which would report a foul for anything close in, the
    airframe included.
    """
    worst, worst_z = float("inf"), None
    radius = env.PROP_DIAMETER_MM / 2
    for i in range(samples + 1):
        f = i / samples
        r = attach[0] + f * (foot[0] - attach[0])
        z = attach[1] + f * (foot[1] - attach[1])
        gap = env.sweep_clearance((r, z), radius, spacing, thickness, tilt_deg, pivot_offset)
        if gap < worst:
            worst, worst_z = gap, z
    return worst, worst_z


def bulkhead(outer_dia, thickness, bore, spine_bcd, spine_holes, spine_dia,
             mount_bcd, mount_holes, insert_bcd, insert_count, flange_width,
             flange_thickness, wire_dia, wire_count):
    """Universal bay ring: carries the spine, takes the gimbal and leg brackets, passes wires.

    One part serves all four stations. Inserts rather than tapped plastic, because the
    gimbal and legs come off repeatedly. Notches at the split plane clear the shell's
    internal bolting flanges, without which the bulkhead cannot drop into a closed half.
    """
    ring = cq.Workplane("XY").circle(outer_dia / 2).circle(bore / 2).extrude(thickness)

    # Insert bosses stand proud of the ring so there is wall around each insert.
    for i in range(insert_count):
        a = 2 * math.pi * i / insert_count + math.pi / insert_count
        ring = ring.union(F.insert_boss(height=F.INSERT_M3["depth"] + 1.5)
                          .translate((insert_bcd / 2 * math.cos(a),
                                      insert_bcd / 2 * math.sin(a), 0)))

    cut = cq.Workplane("XY")
    for i in range(spine_holes):
        a = 2 * math.pi * i / spine_holes + math.pi / spine_holes
        cut = cut.moveTo(spine_bcd / 2 * math.cos(a),
                         spine_bcd / 2 * math.sin(a)).circle(spine_dia / 2)
    for i in range(mount_holes):
        a = 2 * math.pi * i / mount_holes
        cut = cut.moveTo(mount_bcd / 2 * math.cos(a),
                         mount_bcd / 2 * math.sin(a)).circle(M3_CLEARANCE / 2)
    for i in range(wire_count):
        a = 2 * math.pi * i / wire_count + 0.4
        cut = cut.moveTo((bore / 2 + outer_dia / 2) / 2 * math.cos(a),
                         (bore / 2 + outer_dia / 2) / 2 * math.sin(a)).circle(wire_dia / 2)
    ring = ring.cut(cut.extrude(thickness * 6, both=True))

    # Clearance for the shell split flanges at the split plane. Skipped when there is
    # no shell to clear.
    if flange_width <= 0 or flange_thickness <= 0:
        return ring
    notch = (cq.Workplane("XY")
             .rect(outer_dia, 2 * flange_thickness + 0.6)
             .extrude(thickness * 6, both=True)
             .intersect(cq.Workplane("XY").circle(outer_dia / 2)
                        .circle(outer_dia / 2 - flange_width - 0.4)
                        .extrude(thickness * 6, both=True)))
    return ring.cut(notch)


def battery_tray(pack_l, pack_w, pack_h, wall, body_bore, strap_width):
    """U-channel sized to the pack, with a curved back that sits against the body wall."""
    outer = (cq.Workplane("XY")
             .box(pack_l, pack_w + 2 * wall, pack_h / 2 + wall, centered=(True, True, False)))
    pocket = (cq.Workplane("XY").workplane(offset=wall)
              .box(pack_l + 2, pack_w, pack_h, centered=(True, True, False)))
    tray = outer.cut(pocket)
    # Trim anything outside the body bore so it drops in.
    tray = tray.intersect(cq.Workplane("XY").circle(body_bore / 2)
                          .extrude(pack_h * 3, both=True))
    # Strap slots near each end.
    slots = cq.Workplane("XY")
    for s in (-1, 1):
        slots = slots.moveTo(s * pack_l * 0.32, 0).rect(strap_width, pack_w + 2 * wall + 10)
    tray = tray.cut(slots.extrude(wall, both=False).translate((0, 0, -0.1)))
    return tray


def leg_bracket(rod_dia, angle_deg, plate_l, plate_w, plate_t, clamp_len, wall):
    """Bolts flat to a bulkhead face; holds the carbon rod at the leg angle."""
    plate = cq.Workplane("XY").box(plate_l, plate_w, plate_t, centered=(True, True, False))
    cut = cq.Workplane("XY")
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut = cut.moveTo(sx * plate_l * 0.3, sy * plate_w * 0.28).circle(M3_CLEARANCE / 2)
    plate = plate.cut(cut.extrude(plate_t * 3, both=True))

    a = math.radians(angle_deg)
    boss_od = rod_dia + 2 * wall
    boss = (cq.Workplane("XZ").workplane(offset=-boss_od / 2)
            .circle(boss_od / 2).extrude(boss_od)
            .rotate((0, 0, 0), (0, 1, 0), 0))
    boss = (cq.Workplane("XY").transformed(rotate=(0, angle_deg, 0))
            .circle(boss_od / 2).extrude(clamp_len))
    bore = (cq.Workplane("XY").transformed(rotate=(0, angle_deg, 0))
            .circle(rod_dia / 2).extrude(clamp_len * 1.2))
    return plate.union(boss.translate((0, 0, plate_t - 0.1))).cut(
        bore.translate((0, 0, plate_t - 0.1)))


def leg_foot(rod_dia, socket_len, wall, pad_dia, pad_t, angle_deg):
    a = math.radians(angle_deg)
    pad = cq.Workplane("XY").circle(pad_dia / 2).extrude(pad_t)
    socket = (cq.Workplane("XY").workplane(offset=pad_t)
              .transformed(rotate=(0, -angle_deg, 0))
              .circle((rod_dia + 2 * wall) / 2).extrude(socket_len))
    bore = (cq.Workplane("XY").workplane(offset=pad_t)
            .transformed(rotate=(0, -angle_deg, 0))
            .circle(rod_dia / 2).extrude(socket_len - wall))
    return pad.union(socket).cut(bore)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--body-od", type=float, default=90.0)
    p.add_argument("--no-shell", action="store_true",
                   help="Bare carbon frame: no split-flange notches, own ring diameter")
    p.add_argument("--bulkhead-od", type=float, default=84.0,
                   help="Ring outer diameter when --no-shell is set, mm")
    p.add_argument("--body-wall", type=float, default=1.2)
    p.add_argument("--bulkhead-thickness", type=float, default=4.0)
    p.add_argument("--bulkhead-bore", type=float, default=52.0)
    p.add_argument("--spine-holes", type=int, default=4)
    p.add_argument("--spine-dia", type=float, default=8.2, help="Carbon spine tube, 8 mm nominal")
    p.add_argument("--spine-bcd", type=float, default=70.0)
    p.add_argument("--gimbal-bcd", type=float, default=80.0)
    p.add_argument("--pack", type=str, default="137x44x33")
    p.add_argument("--tray-wall", type=float, default=2.4)
    p.add_argument("--strap-width", type=float, default=16.0)
    p.add_argument("--insert-bcd", type=float, default=62.0,
                   help="Bolt circle for the heat-set inserts the legs and gimbal use")
    p.add_argument("--insert-count", type=int, default=6)
    p.add_argument("--flange-width", type=float, default=8.0)
    p.add_argument("--flange-thickness", type=float, default=3.0)
    p.add_argument("--wire-dia", type=float, default=9.0)
    p.add_argument("--wire-count", type=int, default=3)
    p.add_argument("--leg-count", type=int, default=4)
    p.add_argument("--leg-rod-dia", type=float, default=8.2)
    p.add_argument("--leg-attach-r", type=float, default=43.0)
    p.add_argument("--leg-attach-z", type=float, default=-15.0)
    p.add_argument("--leg-foot-r", type=float, default=215.0)
    p.add_argument("--leg-foot-z", type=float, default=-215.0)
    p.add_argument("--rotor-offset", type=float, default=58.0)
    p.add_argument("--rotor-spacing", type=float, default=64.0)
    p.add_argument("--per-axis-deg", type=float, default=15.0)
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)

    body_bore = args.body_od - 2 * args.body_wall
    L, W, H = (float(v) for v in args.pack.lower().split("x"))

    # Envelope for the final gimbal geometry, used to check the legs.
    pivot_offset = args.rotor_offset + args.rotor_spacing / 2.0
    tilt = env.combined_tilt_deg(args.per_axis_deg)
    profile = env.envelope_profile(env.PROP_DIAMETER_MM / 2, args.rotor_spacing, 20.0,
                                   tilt, pivot_offset, 10.0)
    attach = (args.leg_attach_r, args.leg_attach_z)
    foot = (args.leg_foot_r, args.leg_foot_z)
    gap, gap_z = check_leg(attach, foot, args.rotor_spacing, 20.0, tilt, pivot_offset)
    env_bottom = min(z for r, z in profile if r > 0)
    leg_angle = math.degrees(math.atan2(foot[0] - attach[0], attach[1] - foot[1]))

    # With no shell the ring sets its own diameter and needs no flange relief, because
    # there is no split shell for it to drop into.
    ring_od = args.bulkhead_od if args.no_shell else body_bore - 0.4
    flange_w = 0.0 if args.no_shell else args.flange_width
    flange_t = 0.0 if args.no_shell else args.flange_thickness
    parts = {
        "bulkhead": bulkhead(ring_od, args.bulkhead_thickness, args.bulkhead_bore,
                             args.spine_bcd, args.spine_holes, args.spine_dia,
                             args.gimbal_bcd, 6, args.insert_bcd, args.insert_count,
                             flange_w, flange_t,
                             args.wire_dia, args.wire_count),
        "battery-tray": battery_tray(L, W, H, args.tray_wall,
                                     (ring_od if args.no_shell else body_bore) - 0.6,
                                     args.strap_width),
    }
    counts = {"bulkhead": 4, "battery-tray": 1}

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"landing_gear": {
                  "leg_count": args.leg_count, "leg_angle_from_vertical_deg": leg_angle,
                  "attach_rz_mm": list(attach), "foot_rz_mm": list(foot),
                  "strut_length_mm": math.dist(attach, foot),
                  "stance_diameter_mm": 2 * args.leg_foot_r,
                  "envelope_bottom_z_mm": env_bottom,
                  "ground_clearance_mm": env_bottom - args.leg_foot_z,
                  "min_strut_to_envelope_mm": gap, "worst_case_z_mm": gap_z,
                  "clears_envelope": gap > 0},
              "fasteners": {
                  "insert": "M3 heat-set, 4.0 mm pilot hole, 6.7 mm deep, 8 mm boss",
                  "shell_interface": "notched at the split plane to clear the shell flanges",
                  "wire": "round pass-throughs; a slot would concentrate stress"},
              "parts": {}, "assumptions": [
                  "Strut clearance is measured to the rod centreline, so half the rod "
                  "diameter plus any ball-link hardware still has to come off it.",
                  "Leg stiffness and buckling are not analysed; rod size is a starting pick.",
                  "Landing loads are not analysed.",
              ], "offline_only": True}

    total = 0.0
    for name, wp in parts.items():
        vol = wp.val().Volume()
        total += vol * counts[name]
        bb = wp.val().BoundingBox()
        cq.exporters.export(wp, str(args.output / f"{name}.step"))
        cq.exporters.export(wp, str(args.output / f"{name}.stl"))
        report["parts"][name] = {
            "count": counts[name], "volume_cm3": vol / 1000,
            "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
            "mass_each_g": {m: round(vol / 1000 * d, 1) for m, d in DENSITY.items()},
            "mass_total_g": {m: round(vol / 1000 * d * counts[name], 1) for m, d in DENSITY.items()}}
    report["total"] = {"volume_cm3": total / 1000,
                       "mass_g": {m: round(total / 1000 * d, 1) for m, d in DENSITY.items()}}
    (args.output / "airframe-parts.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    lg = report["landing_gear"]
    print(f"Airframe parts  body bore {body_bore:.1f} mm, pack {L:.0f}x{W:.0f}x{H:.0f} mm")
    print(f"  landing gear: {args.leg_count} legs at {leg_angle:.1f} deg from vertical, "
          f"strut {lg['strut_length_mm']:.0f} mm, stance {lg['stance_diameter_mm']:.0f} mm")
    print(f"    envelope bottom {env_bottom:.1f} mm, feet at {args.leg_foot_z:.0f} mm "
          f"-> ground clearance {lg['ground_clearance_mm']:.1f} mm")
    print(f"    strut to envelope: {gap:+.1f} mm at z={gap_z:.0f}  "
          f"({'clears' if gap > 0 else 'FOULS'})")
    print(f"  {'part':14s} {'qty':>4s} {'ea cm3':>8s} {'ASA ea':>7s} {'ASA tot':>8s}   bbox")
    for name, d in report["parts"].items():
        print(f"  {name:14s} {d['count']:4d} {d['volume_cm3']:8.1f} "
              f"{d['mass_each_g']['ASA']:7.1f} {d['mass_total_g']['ASA']:8.1f}   {d['bbox_mm']}")
    print(f"  {'TOTAL':14s} {'':4s} {'':8s} {'':7s} {report['total']['mass_g']['ASA']:8.1f}")
    print(f"  wrote STEP/STL to {args.output.resolve()}")
    print("Carbon rod, bearings, fasteners and straps are bought parts and are not in this mass.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
