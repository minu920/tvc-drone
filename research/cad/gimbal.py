"""Generate the two-axis TVC gimbal structure, offline.

Unlike the outer shell, these parts are a load path. Thrust passes from the motors through
the plate, the inner ring and the outer ring into the airframe, so the printed geometry
here is structural and should be printed in a stiff material, not PLA.

Architecture follows the reference vehicle: the two motors are bolted back to back to a
central plate, one propeller above and one below, so the rotor midpoint coincides with the
gimbal pivot. That is the pivot assumption the sweep envelope already uses, which keeps the
two models consistent.

    outer ring   fixed to the airframe, carries the outer-axis bearings
      inner ring pivots on the outer axis, carries the inner-axis bearings
        plate    pivots on the inner axis, carries both motors

Motor interface comes from the supplied D4215 STEP: 42.25 mm body, M3 on 16 mm and 19 mm
bolt circles, 4 mm shaft. Those are third-party model values, not caliper measurements.

This produces the structure only. Servo mounting pads are placed, but the reduction linkage
and the bearing retention details are not designed yet, so the reported mass is a floor,
not the finished part.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}

MOTOR_OD = 42.25          # from the supplied D4215 STEP
MOTOR_BCD = (16.0, 19.0)  # M3, four holes each
M3_CLEARANCE = 3.4


def _bolt_holes(wp, bcd, count, diameter, depth, through=True):
    for i in range(count):
        a = math.radians(45 + 360 * i / count)
        wp = wp.moveTo(bcd / 2 * math.cos(a), bcd / 2 * math.sin(a)).circle(diameter / 2)
    return wp.cutThruAll() if through else wp.cutBlind(-depth)


def build_plate(diameter, thickness, trunnion_reach, trunnion_dia, pin_dia, cable_hole):
    """Central plate: motors bolt to both faces, trunnions run on the inner axis."""
    plate = cq.Workplane("XY").circle(diameter / 2).extrude(thickness, both=True)
    for x in (-1, 1):
        # Start at the plate rim and grow outward. The offset must not also shift by the
        # reach, or the negative side extends twice as far as the positive one.
        plate = plate.union(
            cq.Workplane("YZ").workplane(offset=x * diameter / 2)
            .circle(trunnion_dia / 2).extrude(x * trunnion_reach))
    plate = plate.faces(">Z").workplane().hole(cable_hole)
    for bcd in MOTOR_BCD:
        plate = _bolt_holes(plate.faces(">Z").workplane(), bcd, 4, M3_CLEARANCE, thickness)
    # Pin bores through both trunnions, along the inner axis.
    plate = (plate.faces(">X").workplane(centerOption="CenterOfBoundBox")
             .circle(pin_dia / 2).cutThruAll())
    return plate


def build_ring(inner_dia, outer_dia, height, boss_axis, boss_reach, boss_dia,
               bearing_od, bearing_width, pin_dia, mount_holes=0, mount_bcd=0.0):
    """A gimbal ring: bearing pockets on one axis, trunnions on the perpendicular axis."""
    ring = (cq.Workplane("XY").circle(outer_dia / 2).circle(inner_dia / 2)
            .extrude(height / 2, both=True))

    # Bearing pockets face inward on the bearing axis.
    plane = "YZ" if boss_axis == "Y" else "XZ"
    for s in (-1, 1):
        ring = ring.union(cq.Workplane(plane)
                          .workplane(offset=s * inner_dia / 2 if plane == "YZ" else -s * inner_dia / 2)
                          .circle(boss_dia / 2)
                          .extrude(s * (outer_dia - inner_dia) / 2 if plane == "YZ"
                                   else -s * (outer_dia - inner_dia) / 2))
    face = ">X" if boss_axis == "Y" else ">Y"
    ring = (ring.faces(face).workplane(centerOption="CenterOfBoundBox")
            .circle(bearing_od / 2).cutBlind(-bearing_width)
            .faces(face).workplane(centerOption="CenterOfBoundBox")
            .circle(pin_dia / 2 + 0.3).cutThruAll())

    # Trunnions on the perpendicular axis. The outer ring is fixed, so it gets none.
    if boss_reach > 0:
        tplane = "XZ" if boss_axis == "Y" else "YZ"
        for s in (-1, 1):
            off = s * outer_dia / 2
            ring = ring.union(cq.Workplane(tplane).workplane(offset=off if tplane == "YZ" else -off)
                              .circle(boss_dia / 2)
                              .extrude(s * boss_reach if tplane == "YZ" else -s * boss_reach))
        tface = ">Y" if boss_axis == "Y" else ">X"
        ring = (ring.faces(tface).workplane(centerOption="CenterOfBoundBox")
                .circle(pin_dia / 2).cutThruAll())

    if mount_holes:
        ring = _bolt_holes(ring.faces(">Z").workplane(), mount_bcd, mount_holes,
                           M3_CLEARANCE, height)
    return ring


def clearance_report(plate_dia, motor_reach, inner_id, outer_id, inner_od, ring_h, tilt_deg):
    """Analytic swing clearances. A tilted body reaches further than its static radius."""
    t = math.radians(tilt_deg)
    motor_swing = MOTOR_OD / 2 * math.cos(t) + motor_reach * math.sin(t)
    ring_corner = inner_od / 2 * math.cos(t) + ring_h / 2 * math.sin(t)
    return {"tilt_deg": tilt_deg,
            "motor_swept_radius_mm": motor_swing,
            "inner_ring_bore_radius_mm": inner_id / 2,
            "motor_to_inner_ring_mm": inner_id / 2 - motor_swing,
            "inner_ring_swept_radius_mm": ring_corner,
            "outer_ring_bore_radius_mm": outer_id / 2,
            "inner_to_outer_ring_mm": outer_id / 2 - ring_corner}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--plate-dia", type=float, default=56.0)
    p.add_argument("--plate-thickness", type=float, default=3.0, help="Half-thickness each side, mm")
    p.add_argument("--motor-reach", type=float, default=26.0,
                   help="How far one motor stands off the plate face, mm")
    p.add_argument("--inner-id", type=float, default=64.0)
    p.add_argument("--inner-od", type=float, default=74.0)
    p.add_argument("--outer-id", type=float, default=80.0)
    p.add_argument("--outer-od", type=float, default=90.0)
    p.add_argument("--ring-height", type=float, default=12.0)
    p.add_argument("--bearing-od", type=float, default=9.0, help="684ZZ outer diameter")
    p.add_argument("--bearing-width", type=float, default=4.0)
    p.add_argument("--pin-dia", type=float, default=4.0)
    p.add_argument("--boss-dia", type=float, default=11.0)
    p.add_argument("--cable-hole", type=float, default=12.0)
    p.add_argument("--tilt-deg", type=float, default=20.0, help="Per-axis hard stop")
    p.add_argument("--airframe-bcd", type=float, default=84.0)
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)

    # The trunnion stops short of the ring bore. A pin runs from outside, through the
    # bearing pressed into the ring, and into this boss, so the two must not touch.
    trunnion_gap = 1.0
    trunnion_reach = (args.inner_id - args.plate_dia) / 2 - trunnion_gap
    parts = {
        "plate": build_plate(args.plate_dia, args.plate_thickness, trunnion_reach,
                             args.boss_dia, args.pin_dia, args.cable_hole),
        "inner-ring": build_ring(args.inner_id, args.inner_od, args.ring_height, "Y",
                                 (args.outer_id - args.inner_od) / 2 + 2.0, args.boss_dia,
                                 args.bearing_od, args.bearing_width, args.pin_dia),
        "outer-ring": build_ring(args.outer_id, args.outer_od, args.ring_height, "X",
                                 0.0, args.boss_dia, args.bearing_od, args.bearing_width,
                                 args.pin_dia, mount_holes=6, mount_bcd=args.airframe_bcd),
    }

    clear = clearance_report(args.plate_dia, args.motor_reach, args.inner_id,
                             args.outer_id, args.inner_od, args.ring_height, args.tilt_deg)

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"parts": {}, "clearances_mm": clear,
              "motor_interface": {"body_od_mm": MOTOR_OD, "bolt_circles_mm": list(MOTOR_BCD),
                                  "screw": "M3", "source": "supplied D4215 STEP, not measured"},
              "pin_interface": {
                  "pin_dia_mm": args.pin_dia,
                  "bearing": f"{args.bearing_od} x {args.bearing_width} mm, pressed into the ring "
                             f"from the outside face",
                  "trunnion_gap_mm": trunnion_gap,
                  "note": "The trunnion boss stops short of the ring bore; the pin spans the gap "
                          "through the bearing. Boss and ring must never touch."},
              "not_yet_designed": ["Servo reduction linkage or gear train.",
                                   "Bearing retention, shoulders and preload.",
                                   "Motor wire routing and strain relief.",
                                   "Hard stops at the travel limits.",
                                   "Fastener bosses tying the outer ring to the airframe."],
              "offline_only": True}

    total_vol = 0.0
    for name, wp in parts.items():
        vol = wp.val().Volume()
        total_vol += vol
        bb = wp.val().BoundingBox()
        cq.exporters.export(wp, str(args.output / f"gimbal-{name}.step"))
        cq.exporters.export(wp, str(args.output / f"gimbal-{name}.stl"))
        report["parts"][name] = {"volume_cm3": vol / 1000.0,
                                 "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                                 "mass_g": {m: round(vol / 1000.0 * d, 1) for m, d in DENSITY.items()}}
    report["total"] = {"volume_cm3": total_vol / 1000.0,
                       "mass_g": {m: round(total_vol / 1000.0 * d, 1) for m, d in DENSITY.items()}}

    assembly = parts["plate"].union(parts["inner-ring"]).union(parts["outer-ring"])
    cq.exporters.export(assembly, str(args.output / "gimbal-assembly.step"))
    for view, d in (("iso", (1.0, -1.0, 0.5)), ("side", (0.0, -1.0, 0.0))):
        cq.exporters.export(assembly, str(args.output / f"gimbal-{view}.svg"),
                            opt={"projectionDir": d, "width": 700, "height": 520,
                                 "showAxes": False, "strokeWidth": 0.4,
                                 "strokeColor": (40, 40, 40), "showHidden": False})
    (args.output / "gimbal.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"TVC gimbal  rings {args.inner_id:.0f}/{args.inner_od:.0f} and "
          f"{args.outer_id:.0f}/{args.outer_od:.0f} mm, pin {args.pin_dia:.0f} mm, "
          f"bearing {args.bearing_od:.0f}x{args.bearing_width:.0f}")
    print(f"  {'part':12s} {'vol cm3':>9s} {'ASA g':>7s} {'PA-CF g':>8s}   bbox")
    for name, d in report["parts"].items():
        print(f"  {name:12s} {d['volume_cm3']:9.1f} {d['mass_g']['ASA']:7.1f} "
              f"{d['mass_g']['PA-CF']:8.1f}   {d['bbox_mm']}")
    t = report["total"]
    print(f"  {'TOTAL':12s} {t['volume_cm3']:9.1f} {t['mass_g']['ASA']:7.1f} {t['mass_g']['PA-CF']:8.1f}")
    print(f"  clearance at {clear['tilt_deg']:.0f} deg per axis:")
    print(f"    motor swept radius {clear['motor_swept_radius_mm']:6.1f} vs inner bore "
          f"{clear['inner_ring_bore_radius_mm']:5.1f}  ->  {clear['motor_to_inner_ring_mm']:+6.1f} mm")
    print(f"    inner ring swept   {clear['inner_ring_swept_radius_mm']:6.1f} vs outer bore "
          f"{clear['outer_ring_bore_radius_mm']:5.1f}  ->  {clear['inner_to_outer_ring_mm']:+6.1f} mm")
    print(f"  wrote STEP/STL per part plus assembly to {args.output.resolve()}")
    print("Structure only. Linkage, bearing retention and hard stops are not designed yet, "
          "so this mass is a floor.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
