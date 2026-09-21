"""Generate the two-axis TVC gimbal, offline.

These parts are a load path. Thrust passes from the motors through the cradle, the inner
ring and the outer ring into the airframe, so they are structural and should not be printed
in PLA.

Both rotors hang below the pivot. That is not a style choice. A 305 mm rotor tilted to the
hard stop swings its rim upward, and within the airframe radius the highest point it reaches
is

    z = r_frame * tan(theta) - h / cos(theta)

for a rotor h below the pivot at combined tilt theta. With the rotor at or above the pivot
that value is positive, meaning the rotor cuts through the skirt. The cradle exists to
provide the offset that makes it negative.

    outer ring   fixed to the airframe, carries the outer-axis bearings and servo 1
      inner ring pivots on the outer axis, carries the inner-axis bearings and servo 2
        cradle   pivots on the inner axis, reaches down to the motor plate
          plate  motors bolt to both faces, one rotor above it and one below

Each axis is driven by a pushrod from a servo horn to a lever on the moving member. Gears
were tried first and do not package: a module 1 sector at this reduction has a 17 mm tip
radius, which fouls the ring annulus on both axes. The lever ratio supplies the same
reduction, and reduction is what brings a large dead band servo inside the hysteresis
target at the gimbal output.

A pushrod ratio is not constant with angle the way a gear ratio is. Over the commanded
travel the variation is small, but it is real and belongs in the control allocation rather
than being assumed away.

Motor interface comes from the supplied D4215 STEP: 42.25 mm body, M3 on 16 mm and 19 mm
bolt circles. Third-party model values, not caliper measurements.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}

MOTOR_OD = 42.25
MOTOR_BCD = (16.0, 19.0)
M3_CLEARANCE = 3.4
BALL_LINK_BORE = 3.2


def min_rotor_offset(frame_radius, tilt_deg, skirt_below_pivot):
    """Smallest rotor-below-pivot distance that keeps the swept rim clear of the airframe."""
    t = math.radians(tilt_deg)
    return (frame_radius * math.tan(t) + skirt_below_pivot) * math.cos(t)


def combined_tilt_deg(per_axis_deg):
    c = math.cos(math.radians(per_axis_deg))
    return math.degrees(math.acos(c * c))


def _ring(inner_dia, outer_dia, height):
    return (cq.Workplane("XY").circle(outer_dia / 2).circle(inner_dia / 2)
            .extrude(height / 2, both=True))


def _rot_z(shape, axis):
    return shape.rotate((0, 0, 0), (0, 0, 1), 90) if axis == "Y" else shape


def _axis_boss(solid, axis, outer_dia, boss_dia):
    """Local thickening along one axis so a bearing pocket has material around it."""
    cyl = (cq.Workplane("YZ").workplane(offset=-outer_dia / 2)
           .circle(boss_dia / 2).extrude(outer_dia))
    return solid.union(_rot_z(cyl, axis))


def _bore_axis(solid, axis, bearing_od, bearing_width, pin_dia, outer_dia):
    """Bearing pockets from both outer faces of one axis, plus a through pin bore."""
    pin = (cq.Workplane("YZ").workplane(offset=-outer_dia)
           .circle(pin_dia / 2 + 0.25).extrude(2 * outer_dia))
    pocket = (cq.Workplane("YZ").workplane(offset=outer_dia / 2 - bearing_width)
              .circle(bearing_od / 2).extrude(bearing_width + 0.5))
    pocket = pocket.union(pocket.mirror("YZ"))
    return solid.cut(_rot_z(pin, axis)).cut(_rot_z(pocket, axis))


def _trunnions(solid, axis, start_dia, reach, boss_dia, pin_dia):
    cyl = (cq.Workplane("YZ").workplane(offset=start_dia / 2)
           .circle(boss_dia / 2).extrude(reach))
    cyl = cyl.union(cyl.mirror("YZ"))
    bore = (cq.Workplane("YZ").workplane(offset=-start_dia)
            .circle(pin_dia / 2).extrude(2 * start_dia))
    return solid.union(_rot_z(cyl, axis)).cut(_rot_z(bore, axis))


def _bolt_circle(solid, bcd, count, dia, height):
    cut = cq.Workplane("XY")
    for i in range(count):
        a = 2 * math.pi * i / count + math.pi / count
        cut = cut.moveTo(bcd / 2 * math.cos(a), bcd / 2 * math.sin(a)).circle(dia / 2)
    return solid.cut(cut.extrude(height * 3, both=True))


def _lever(axis, radius, length, width, thickness, ball_bore):
    """Arm hanging from a pivot axis, with a ball-link hole at its tip.

    Placed on the pivot axis at +radius, reaching down to -length.
    """
    prof = [(-width / 2, 0.0), (width / 2, 0.0),
            (width / 2 * 0.7, -length), (-width / 2 * 0.7, -length)]
    arm = (cq.Workplane("XZ").polyline(prof).close()
           .extrude(thickness / 2, both=True))
    arm = (arm.faces(">Y").workplane(centerOption="CenterOfBoundBox")
           .center(0, -length + width / 2).hole(ball_bore))
    arm = arm.rotate((0, 0, 0), (0, 0, 1), 90).translate((0, radius, 0))
    return _rot_z(arm, "X" if axis == "Y" else "Y")


def servo_bracket(body_l, body_w, flange_span, flange_pitch, plate_t, pad_w, wall_h):
    """Plate that traps an MG996R-class servo body, with a wall to bolt against a ring."""
    plate = (cq.Workplane("XY").box(flange_span + 10, pad_w, plate_t, centered=(True, True, False))
             .cut(cq.Workplane("XY").box(body_l + 0.6, body_w + 0.6, plate_t * 4,
                                         centered=(True, True, True))))
    cut = cq.Workplane("XY")
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut = cut.moveTo(sx * flange_span / 2, sy * flange_pitch / 2).circle(M3_CLEARANCE / 2)
    plate = plate.cut(cut.extrude(plate_t * 4, both=True))
    wall = (cq.Workplane("XY")
            .box(flange_span + 10, plate_t, wall_h, centered=(True, True, False))
            .translate((0, -pad_w / 2 + plate_t / 2, 0)))
    holes = cq.Workplane("XZ").workplane(offset=pad_w / 2)
    for sx in (-1, 1):
        holes = holes.moveTo(sx * (flange_span / 2 + 2), wall_h * 0.6).circle(M3_CLEARANCE / 2)
    return plate.union(wall).cut(holes.extrude(-pad_w * 2))


def build(a):
    parts, meta = {}, {}
    plate_z = -(a.rotor_offset + a.rotor_spacing / 2.0)

    outer = _ring(a.outer_id, a.outer_od, a.ring_height)
    outer = _axis_boss(outer, "Y", a.outer_od, a.boss_dia)
    outer = _bore_axis(outer, "Y", a.bearing_od, a.bearing_width, a.pin_dia, a.outer_od)
    outer = _bolt_circle(outer, a.airframe_bcd, 6, M3_CLEARANCE, a.ring_height)
    parts["outer-ring"] = outer

    inner = _ring(a.inner_id, a.inner_od, a.ring_height)
    inner = _axis_boss(inner, "X", a.inner_od, a.boss_dia)
    inner = _bore_axis(inner, "X", a.bearing_od, a.bearing_width, a.pin_dia, a.inner_od)
    inner = _trunnions(inner, "Y", a.inner_od, (a.outer_id - a.inner_od) / 2 + 3.0,
                       a.boss_dia, a.pin_dia)
    inner = inner.union(_lever("Y", a.inner_od / 2 - 2, a.gimbal_lever, a.lever_width,
                               a.lever_thickness, BALL_LINK_BORE))
    parts["inner-ring"] = inner

    # Cradle: arms on the inner axis reaching down to the motor plate.
    prof = [(a.arm_x_top - a.arm_thickness / 2, 0.0), (a.arm_x_top + a.arm_thickness / 2, 0.0),
            (a.arm_x_bot + a.arm_thickness / 2, plate_z), (a.arm_x_bot - a.arm_thickness / 2, plate_z)]
    arm = cq.Workplane("XZ").polyline(prof).close().extrude(a.arm_width / 2, both=True)
    cradle = arm.union(arm.mirror("YZ"))
    cradle = cradle.union(cq.Workplane("XY").workplane(offset=plate_z)
                          .circle(a.plate_dia / 2).extrude(a.plate_thickness / 2, both=True))
    cradle = _trunnions(cradle, "X", 2 * (a.arm_x_top + a.arm_thickness / 2) - 4,
                        a.trunnion_reach, a.boss_dia, a.pin_dia)
    # Motor bolt patterns through the plate, plus a cable pass-through.
    cut = cq.Workplane("XY")
    for bcd in MOTOR_BCD:
        for i in range(4):
            ang = math.radians(45 + 90 * i)
            cut = cut.moveTo(bcd / 2 * math.cos(ang), bcd / 2 * math.sin(ang)).circle(M3_CLEARANCE / 2)
    cut = cut.moveTo(0, 0).circle(a.cable_hole / 2)
    cradle = cradle.cut(cut.extrude(abs(plate_z) * 3, both=True))
    # Pushrod attachment through one arm. The cradle turns about X, so a point on the arm
    # below the pivot moves in Y; the hole must therefore run along Y, not along the arm.
    arm_x_at_lever = (a.arm_x_top
                      + (a.arm_x_bot - a.arm_x_top) * (a.gimbal_lever / abs(plate_z)))
    rod = (cq.Workplane("XZ").workplane(offset=-a.arm_width)
           .center(arm_x_at_lever, -a.gimbal_lever)
           .circle(BALL_LINK_BORE / 2).extrude(2 * a.arm_width))
    cradle = cradle.cut(rod)
    parts["cradle"] = cradle

    parts["servo-bracket"] = servo_bracket(a.servo_l, a.servo_w, a.servo_flange_span,
                                           a.servo_flange_pitch, 3.0, a.servo_w + 12, 14.0)

    meta["plate_z_mm"] = plate_z
    meta["rotor_z_mm"] = [-a.rotor_offset, -(a.rotor_offset + a.rotor_spacing)]
    return parts, meta


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--outer-od", type=float, default=86.0)
    p.add_argument("--outer-id", type=float, default=76.0)
    p.add_argument("--inner-od", type=float, default=72.0)
    p.add_argument("--inner-id", type=float, default=64.0)
    p.add_argument("--ring-height", type=float, default=10.0)
    p.add_argument("--arm-x-top", type=float, default=26.0)
    p.add_argument("--arm-x-bot", type=float, default=30.0)
    p.add_argument("--arm-thickness", type=float, default=8.0)
    p.add_argument("--arm-width", type=float, default=10.0)
    p.add_argument("--trunnion-reach", type=float, default=4.0)
    p.add_argument("--plate-dia", type=float, default=56.0)
    p.add_argument("--plate-thickness", type=float, default=6.0)
    p.add_argument("--cable-hole", type=float, default=12.0)
    p.add_argument("--rotor-offset", type=float, default=58.0)
    p.add_argument("--rotor-spacing", type=float, default=64.0)
    p.add_argument("--bearing-od", type=float, default=9.0)
    p.add_argument("--bearing-width", type=float, default=4.0)
    p.add_argument("--pin-dia", type=float, default=4.0)
    p.add_argument("--boss-dia", type=float, default=11.0)
    p.add_argument("--airframe-bcd", type=float, default=80.0)
    p.add_argument("--tilt-deg", type=float, default=15.0)
    p.add_argument("--frame-radius", type=float, default=45.0)
    p.add_argument("--skirt-below-pivot", type=float, default=30.0)
    p.add_argument("--gimbal-lever", type=float, default=30.0)
    p.add_argument("--servo-horn", type=float, default=15.0)
    p.add_argument("--lever-width", type=float, default=12.0)
    p.add_argument("--lever-thickness", type=float, default=6.0)
    p.add_argument("--servo-l", type=float, default=40.7)
    p.add_argument("--servo-w", type=float, default=19.7)
    p.add_argument("--servo-flange-span", type=float, default=49.0)
    p.add_argument("--servo-flange-pitch", type=float, default=10.0)
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)

    parts, meta = build(args)
    combined = combined_tilt_deg(args.tilt_deg)
    h_min = min_rotor_offset(args.frame_radius, combined, args.skirt_below_pivot)
    reduction = args.gimbal_lever / args.servo_horn

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"geometry": {**meta, "per_axis_tilt_deg": args.tilt_deg,
                           "combined_tilt_deg": combined,
                           "min_rotor_offset_mm": h_min,
                           "rotor_offset_mm": args.rotor_offset,
                           "rotor_offset_margin_mm": args.rotor_offset - h_min},
              "linkage": {"gimbal_lever_mm": args.gimbal_lever,
                          "servo_horn_mm": args.servo_horn, "reduction": reduction,
                          "mg996r_deadband_deg_at_servo": 0.9,
                          "deadband_deg_at_gimbal": 0.9 / reduction,
                          "hysteresis_target_deg": 0.5},
              "parts": {}, "not_yet_designed": [
                  "Pushrods and ball links are bought parts; only their holes are modelled.",
                  "Bearing retention and preload.",
                  "Hard stops at the travel limits.",
                  "Motor wire routing and strain relief.",
              ], "offline_only": True}

    total = 0.0
    for name, wp in parts.items():
        vol = wp.val().Volume(); total += vol
        bb = wp.val().BoundingBox()
        cq.exporters.export(wp, str(args.output / f"gimbal-{name}.step"))
        cq.exporters.export(wp, str(args.output / f"gimbal-{name}.stl"))
        report["parts"][name] = {"volume_cm3": vol / 1000,
                                 "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                                 "mass_g": {m: round(vol / 1000 * d, 1) for m, d in DENSITY.items()}}
    report["total"] = {"volume_cm3": total / 1000,
                       "mass_g": {m: round(total / 1000 * d, 1) for m, d in DENSITY.items()}}
    (args.output / "gimbal.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"TVC gimbal  per-axis {args.tilt_deg:.0f} deg -> combined {combined:.2f} deg")
    print(f"  rotors at z = {meta['rotor_z_mm'][0]:.0f} and {meta['rotor_z_mm'][1]:.0f} mm, "
          f"plate at {meta['plate_z_mm']:.0f} mm")
    print(f"  minimum rotor offset {h_min:.1f} mm, using {args.rotor_offset:.0f} mm "
          f"-> margin {args.rotor_offset - h_min:+.1f} mm")
    print(f"  linkage {args.gimbal_lever:.0f}/{args.servo_horn:.0f} = {reduction:.2f}:1, "
          f"dead band at gimbal {0.9 / reduction:.2f} deg (target 0.5)")
    print(f"  {'part':15s} {'vol cm3':>9s} {'ASA g':>7s} {'PA-CF g':>8s}   bbox")
    for name, d in report["parts"].items():
        print(f"  {name:15s} {d['volume_cm3']:9.1f} {d['mass_g']['ASA']:7.1f} "
              f"{d['mass_g']['PA-CF']:8.1f}   {d['bbox_mm']}")
    t = report["total"]
    print(f"  {'TOTAL':15s} {t['volume_cm3']:9.1f} {t['mass_g']['ASA']:7.1f} {t['mass_g']['PA-CF']:8.1f}")
    print(f"  wrote STEP/STL per part to {args.output.resolve()}")
    print("Structure only. Pushrods, bearings, retention and hard stops are bought or "
          "not designed, so this mass is a floor.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
