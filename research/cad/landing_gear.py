"""Generate the landing gear, offline.

A straight leg from the airframe to the ground is not possible on this vehicle. The rotor
rim swings upward when the gimbal tilts, and the swept band reaches inward to about 63 mm
radius near the pivot plane while extending out to 199 mm. Any straight strut from the body
to a foot outside the rotors has to cross that band. Every straight geometry tried fouls,
by 36 to 100 mm depending on the attachment height.

So each leg is a truss that leaves the airframe above the swept volume, reaches outward
clear of it, and only then turns down:

    strut A  upper diagonal, body to knee            carries compression
    strut C  lower tie, body to knee                 triangulates the knee
    strut B  vertical leg, knee to foot              carries the landing load

Struts are bought carbon rod. Only the end fittings are printed, which follows the baseline
specification's rule that thrust and landing loads stay in carbon rather than in printed
parts. Every strut is verified against the exact rotor sweep, not against the propeller
diameter and not against the revolved keep-out solid, which is deliberately conservative
near the axis and would reject the airframe itself.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

import prop_sweep_envelope as env

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}
M3_CLEARANCE = 3.4


def strut_clearance(p0, p1, spacing, thickness, tilt_deg, pivot_offset, samples=200):
    worst, worst_z = float("inf"), None
    radius = env.PROP_DIAMETER_MM / 2
    for i in range(samples + 1):
        f = i / samples
        r = p0[0] + f * (p1[0] - p0[0])
        z = p0[1] + f * (p1[1] - p0[1])
        g = env.sweep_clearance((r, z), radius, spacing, thickness, tilt_deg, pivot_offset)
        if g < worst:
            worst, worst_z = g, z
    return worst, worst_z


def _rod_socket(rod_dia, wall, length, angle_deg):
    """Blind socket for a carbon rod, rising from the XY plane at an angle from vertical."""
    od = rod_dia + 2 * wall
    boss = (cq.Workplane("XY").transformed(rotate=(0, angle_deg, 0))
            .circle(od / 2).extrude(length))
    bore = (cq.Workplane("XY").transformed(rotate=(0, angle_deg, 0))
            .circle(rod_dia / 2).extrude(length - wall))
    return boss, bore


def body_bracket(rod_dia, wall, socket_len, angle_deg, plate_l, plate_w, plate_t, bolt_bcd):
    """Bolts flat against a bulkhead, holding one rod at the strut angle."""
    plate = (cq.Workplane("XY").box(plate_l, plate_w, plate_t, centered=(True, True, False))
             .edges("|Z").fillet(4.0))
    cut = cq.Workplane("XY")
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut = cut.moveTo(sx * bolt_bcd / 2, sy * plate_w * 0.28).circle(M3_CLEARANCE / 2)
    plate = plate.cut(cut.extrude(plate_t * 4, both=True))
    boss, bore = _rod_socket(rod_dia, wall, socket_len, angle_deg)
    return plate.union(boss.translate((0, 0, plate_t - 0.1))).cut(
        bore.translate((0, 0, plate_t - 0.1)))


def knee(rod_dia, wall, socket_len, angle_a, angle_c, hub_dia, web_t=5.0, fillet=2.0):
    """Joins the two inboard struts to the vertical leg, with a web across the truss plane.

    This joint reacts the moment from the vertical leg, so three sockets meeting at a ball
    is the wrong shape for it: the load path runs through the thinnest section. A web filling
    the triangle between the three rod axes carries that moment in shear instead, and all
    three axes are coplanar here so one flat web does it.
    """
    angles = sorted([180 - angle_a, 180 - angle_c, 0.0])
    hub = cq.Workplane("XY").sphere(hub_dia / 2)
    solid = hub
    bores, tips = [], []
    for ang in angles:
        b, r = _rod_socket(rod_dia, wall, socket_len, ang)
        solid = solid.union(b)
        bores.append(r)
        a = math.radians(ang)
        tips.append((socket_len * math.sin(a), socket_len * math.cos(a)))

    # Web across the plane containing all three rod axes, built as a fan from the hub out
    # to each socket tip. The tips are visited in order of socket angle, not in order of
    # their polar position: the three axes span less than a half turn, so the triangle of
    # tips does not contain the hub and joining them directly leaves the middle hollow.
    web = (cq.Workplane("XZ").polyline([(0.0, 0.0)] + tips).close()
           .extrude(web_t / 2, both=True))
    solid = solid.union(web)
    for r in bores:
        solid = solid.cut(r)
    before = solid.val().Volume()
    try:
        filleted = solid.edges("|Y").fillet(fillet)
        # A fillet that removes a large fraction has selected the wrong edges.
        if filleted.val().Volume() > before * 0.85:
            solid = filleted
    except Exception:
        pass
    return solid


def foot(rod_dia, wall, socket_len, pad_dia, pad_t):
    pad = cq.Workplane("XY").circle(pad_dia / 2).extrude(pad_t).edges(">Z").fillet(2.0)
    boss = (cq.Workplane("XY").workplane(offset=pad_t)
            .circle((rod_dia + 2 * wall) / 2).extrude(socket_len))
    bore = (cq.Workplane("XY").workplane(offset=pad_t)
            .circle(rod_dia / 2).extrude(socket_len - wall))
    return pad.union(boss).cut(bore)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--legs", type=int, default=4)
    p.add_argument("--body-r", type=float, default=43.0)
    p.add_argument("--attach-a-z", type=float, default=140.0)
    p.add_argument("--attach-c-z", type=float, default=30.0)
    p.add_argument("--knee-r", type=float, default=215.0)
    p.add_argument("--knee-z", type=float, default=15.0)
    p.add_argument("--foot-z", type=float, default=-215.0)
    p.add_argument("--rod-dia", type=float, default=8.2)
    p.add_argument("--wall", type=float, default=3.0)
    p.add_argument("--socket-len", type=float, default=26.0)
    p.add_argument("--rotor-spacing", type=float, default=64.0)
    p.add_argument("--rotor-offset", type=float, default=58.0)
    p.add_argument("--disc-thickness", type=float, default=20.0)
    p.add_argument("--per-axis-deg", type=float, default=15.0)
    p.add_argument("--min-margin", type=float, default=10.0)
    p.add_argument("--knee-web-t", type=float, default=5.0,
                   help="Gusset thickness across the truss plane, mm")
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)

    pivot_offset = args.rotor_offset + args.rotor_spacing / 2.0
    tilt = env.combined_tilt_deg(args.per_axis_deg)
    knee_pt = (args.knee_r, args.knee_z)
    struts = {
        "A upper diagonal": ((args.body_r, args.attach_a_z), knee_pt),
        "C lower tie": ((args.body_r, args.attach_c_z), knee_pt),
        "B vertical leg": (knee_pt, (args.knee_r, args.foot_z)),
    }
    checks, fouled = {}, []
    for name, (p0, p1) in struts.items():
        gap, z = strut_clearance(p0, p1, args.rotor_spacing, args.disc_thickness,
                                 tilt, pivot_offset)
        checks[name] = {"from_rz": list(p0), "to_rz": list(p1),
                        "length_mm": math.dist(p0, p1),
                        "angle_from_vertical_deg": math.degrees(
                            math.atan2(p1[0] - p0[0], p0[1] - p1[1])),
                        "min_clearance_mm": gap, "at_z_mm": z,
                        "passes": gap >= args.min_margin}
        if not checks[name]["passes"]:
            fouled.append(name)

    angle_a = checks["A upper diagonal"]["angle_from_vertical_deg"]
    angle_c = checks["C lower tie"]["angle_from_vertical_deg"]
    parts = {
        "leg-bracket-upper": body_bracket(args.rod_dia, args.wall, args.socket_len,
                                          angle_a, 44.0, 26.0, 4.0, 30.0),
        "leg-bracket-lower": body_bracket(args.rod_dia, args.wall, args.socket_len,
                                          angle_c, 44.0, 26.0, 4.0, 30.0),
        "leg-knee": knee(args.rod_dia, args.wall, args.socket_len, angle_a, angle_c,
                         args.rod_dia + 2 * args.wall + 6.0, args.knee_web_t),
        "leg-foot": foot(args.rod_dia, args.wall, args.socket_len, 30.0, 5.0),
    }

    env_profile = env.envelope_profile(env.PROP_DIAMETER_MM / 2, args.rotor_spacing,
                                       args.disc_thickness, tilt, pivot_offset, 10.0)
    env_bottom = min(z for r, z in env_profile if r > 0)

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"geometry": {"legs": args.legs, "stance_diameter_mm": 2 * args.knee_r,
                           "overall_height_mm": args.attach_a_z - args.foot_z,
                           "ground_clearance_under_rotors_mm": env_bottom - args.foot_z,
                           "combined_tilt_deg": tilt},
              "strut_checks": checks, "all_struts_clear": not fouled,
              "rod_total_length_mm": args.legs * sum(c["length_mm"] for c in checks.values()),
              "parts": {}, "not_analysed": [
                  "Landing loads and rod buckling are not calculated. The knee now "
                  "has a web across the truss plane, but its size is not from a "
                  "stress result.",
                  "Clearance is to the rod centreline; half the rod diameter still comes off it.",
              ], "offline_only": True}

    total = 0.0
    for name, wp in parts.items():
        vol = wp.val().Volume()
        total += vol * args.legs
        bb = wp.val().BoundingBox()
        cq.exporters.export(wp, str(args.output / f"{name}.step"))
        cq.exporters.export(wp, str(args.output / f"{name}.stl"))
        report["parts"][name] = {
            "count": args.legs, "volume_cm3": vol / 1000,
            "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
            "mass_total_g": {m: round(vol / 1000 * d * args.legs, 1) for m, d in DENSITY.items()}}
    report["total"] = {"volume_cm3": total / 1000,
                       "mass_g": {m: round(total / 1000 * d, 1) for m, d in DENSITY.items()}}
    (args.output / "landing-gear.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Landing gear  {args.legs} legs, stance {2 * args.knee_r:.0f} mm, "
          f"rotor ground clearance {env_bottom - args.foot_z:.0f} mm")
    for name, c in checks.items():
        print(f"  {name:18s} {c['length_mm']:6.0f} mm at {c['angle_from_vertical_deg']:5.1f} deg  "
              f"clearance {c['min_clearance_mm']:+7.1f} mm at z={c['at_z_mm']:+6.0f}  "
              f"{'pass' if c['passes'] else 'FAIL'}")
    print(f"  carbon rod needed: {report['rod_total_length_mm'] / 1000:.2f} m total")
    print(f"  {'part':20s} {'qty':>4s} {'ea cm3':>8s} {'ASA tot':>8s}   bbox")
    for name, d in report["parts"].items():
        print(f"  {name:20s} {d['count']:4d} {d['volume_cm3']:8.1f} "
              f"{d['mass_total_g']['ASA']:8.1f}   {d['bbox_mm']}")
    print(f"  {'TOTAL printed':20s} {'':4s} {'':8s} {report['total']['mass_g']['ASA']:8.1f}")
    print(f"  wrote STEP/STL to {args.output.resolve()}")
    if fouled:
        print("  FAIL: " + ", ".join(fouled))
        return 2
    print("End fittings only. Carbon rod, fasteners and pads are bought parts.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
