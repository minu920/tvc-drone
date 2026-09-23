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

Motor interface comes from the supplied D4215 STEP: 42.25 mm body and a rectangular
16 x 19 mm four-screw M3 pattern. Third-party model values, not caliper measurements.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

import features as F

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}

MOTOR_OD = 42.25
# 16x19 is one rectangular pattern, not two bolt circles: the 16 mm pair sits on one
# axis and the 19 mm pair on the perpendicular one, four screws in total. Read off the
# supplied D4215 STEP, where the 16 mm holes are at 90/270 deg and the 19 mm at 0/180.
# Treating it as two full circles of four puts holes 1.5 mm apart with 3.4 mm
# clearance, so the cutters overlap and the boolean leaves plugs behind.
MOTOR_HOLES = ((0.0, 8.0), (0.0, -8.0), (9.5, 0.0), (-9.5, 0.0))
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


def _axis_boss(solid, axis, inner_dia, outer_dia, boss_dia, overhang=0.0):
    """Local thickening on one axis so a bearing seat has material around it.

    Only the annulus is thickened. Extruding a cylinder across the full diameter instead
    would leave a bar straight through the ring, blocking whatever is meant to pivot inside
    it. The overhang defaults to zero because the gap between a ring bore and the member
    turning inside it is only 2 mm: any overhang eats exactly that running clearance, and
    the parts come out interfering.
    """
    for s in (+1, -1):
        start = s * (inner_dia / 2 - overhang)
        length = s * ((outer_dia - inner_dia) / 2 + overhang)
        cyl = (cq.Workplane("YZ").workplane(offset=start)
               .circle(boss_dia / 2).extrude(length))
        solid = solid.union(_rot_z(cyl, axis))
    return solid


def _bore_axis(solid, axis, outer_dia, bearing=F.BEARING_683):
    """Bearing seats cut inward from each outer face, with a shoulder and a lead-in.

    The shoulder gives the outer race a face to sit against so the press fit is not the
    only thing locating it axially, and the chamfered mouth stops the bearing shaving the
    bore on the way in.
    """
    cutter = F.bearing_pocket_cutter(bearing)
    for s in (+1, -1):
        # The cutter eats material below its own -Z, so rotate that direction inward.
        c = cutter.rotate((0, 0, 0), (0, 1, 0), 90 * s).translate((s * outer_dia / 2, 0, 0))
        solid = solid.cut(_rot_z(c, axis))
    return solid


def _trunnions(solid, axis, start_dia, reach, boss_dia):
    """Stub shafts on one axis. The insert pockets are cut separately, see below."""
    cyl = (cq.Workplane("YZ").workplane(offset=start_dia / 2)
           .circle(boss_dia / 2).extrude(reach))
    cyl = cyl.union(cyl.mirror("YZ"))
    return solid.union(_rot_z(cyl, axis))


def _trunnion_backing(solid, axis, bore_r, tip, half_w, ring_height, spec=F.INSERT_M3):
    """Material behind a trunnion, on the bore side, so its insert pocket has a wall.

    A stub on a ring can only reach as far as the running clearance to the bore it turns
    inside, which here is 1 mm on a 4 mm annulus: 5 mm of material against the 6.7 mm the
    insert needs. Widening the annulus is not available either, because the bore has to
    stay large enough for the member turning inside it.

    So the wall is added inward instead, as a local pad at the two trunnion stations only,
    rather than by thickening the whole ring. It projects into the bore, which is free
    space on this axis: the member inside turns about the perpendicular axis, so it sweeps
    nowhere near these two spots.
    """
    short = max(0.0, spec["depth"] - (tip - bore_r)) + 1.0
    if short <= 1.0:
        return solid
    half_h = min(half_w, ring_height / 2.0)
    pad = (cq.Workplane("XY").workplane(offset=-half_h)
           .center(bore_r - short / 2.0, 0).rect(short, 2 * half_w)
           .extrude(2 * half_h))
    pad = pad.union(pad.mirror("YZ"))
    return solid.union(_rot_z(pad, axis))


def _trunnion_pockets(solid, axis, tip, spec=F.INSERT_M3):
    """Cut the heat-set insert pockets in the trunnion end faces. MUST BE CALLED LAST.

    An M3 screw runs from outside through the bearing bore and threads into the insert, so
    the same screw is the pivot pin and sets the axial preload on the inner race. Screwing
    into bare plastic here would strip, and this joint comes apart often.

    Two things made this pocket 3 mm deep instead of 6.7 when it was cut inside _trunnions.
    The hole ran the wrong way: insert_hole() rises along +Z and was rotated +90*s about Y,
    which sends it along +x*s, straight out of the tip into fresh air, so early versions cut
    nothing at all and both trunnions printed solid. Once that was negated it cut inward but
    bottomed out, because the stub is only 1 mm long on a 4 mm annulus, and because the
    lever is unioned onto this same axis afterwards and refilled what had been cut.

    Cutting last turns the second problem into the solution: the lever root is the backing
    material the stub does not have on its own, and the pocket reaches full depth without
    any part growing. Nothing may be unioned onto this axis after this call.
    """
    for s in (+1, -1):
        hole = (F.insert_hole(spec).rotate((0, 0, 0), (0, 1, 0), -90 * s)
                .translate((s * tip, 0, 0)))
        solid = solid.cut(_rot_z(hole, axis))
    return solid


def trunnion_pocket_depth(solid, axis, tip, spec=F.INSERT_M3, step=0.1, probes=8):
    """Measure how much SUPPORTED bore a pocket really has, by probing the finished solid.

    The depth is a consequence of several unrelated features meeting on one axis, so it is
    measured rather than declared. An insert pressed into a pocket shallower than its own
    length sits proud and the joint never closes.

    What has to be measured is the wall, not the void. Walking the axis and stopping at the
    first solid point measures air, and a ring's own open bore is air too: that test
    reported a full 6.7 mm for a pocket whose wall stopped after 5.06 mm and then opened
    into the bore, where an insert has nothing to grip and would push straight through.
    So each station is checked by probing a circle of points just outside the pilot radius.
    The supported depth ends at the first station where that wall is not there.
    """
    body = solid.val() if hasattr(solid, "val") else solid
    sol = body.Solids()[0]
    a = math.radians({"X": 0.0, "Y": 90.0}[axis])
    # Unit vectors: along the pocket axis, and the two directions across it.
    ax = (math.cos(a), math.sin(a), 0.0)
    u = (-math.sin(a), math.cos(a), 0.0)
    v = (0.0, 0.0, 1.0)
    r_wall = spec["hole_dia"] / 2 + 0.3
    depth = 0.0
    while depth < spec["depth"] + step:
        d = depth + step / 2
        c = [ax[i] * (tip - d) for i in range(3)]
        walled = True
        for k in range(probes):
            th = 2 * math.pi * k / probes
            pt = cq.Vector(*[c[i] + r_wall * (math.cos(th) * u[i] + math.sin(th) * v[i])
                             for i in range(3)])
            if not sol.isInside(pt, 1e-6):
                walled = False
                break
        if not walled:
            break
        depth += step
    return depth


def _bolt_circle(solid, bcd, count, dia, height):
    cut = cq.Workplane("XY")
    for i in range(count):
        a = 2 * math.pi * i / count + math.pi / count
        cut = cut.moveTo(bcd / 2 * math.cos(a), bcd / 2 * math.sin(a)).circle(dia / 2)
    return solid.cut(cut.extrude(height * 3, both=True))


def _stop_screws(solid, axis, ring_top, target_r, stop_z, riser_r, annulus_w,
                 spec=F.INSERT_M3):
    """Adjustable stop screws on the axis perpendicular to rotation.

    A moulded-in tab would have to hold a tolerance the printer does not, and any error
    there becomes travel error. A screw is adjustable, so the limit is set on the bench
    against a measured angle rather than trusted from the model.

    The stop has to act over the member that moves, which sits inside this ring's bore, so
    each one is an L: a riser off the annulus and an arm reaching inward, with the insert
    running vertically through the arm.

    The riser is only as wide radially as the annulus it stands on. A wider one hangs over
    the bore on the inside and past the rim on the outside, and on the inner ring that
    overhang reached into the outer ring and locked the gimbal.
    """
    arm_t = 5.0
    riser_radial = min(9.0, annulus_w)
    arm_top = stop_z + arm_t + spec["depth"]
    for s in (+1, -1):
        riser = (cq.Workplane("XY").workplane(offset=ring_top - 1.0)
                 .center(s * riser_r, 0).rect(riser_radial, 12.0)
                 .extrude(arm_top - ring_top + 1.0))
        # The arm runs from the riser's outer face inward to just past the insert, rather
        # than being centred between the two radii with slop added on both sides. Sized
        # the latter way it overshot the riser outward and reached into the ring turning
        # outside this one.
        arm_out = riser_r + riser_radial / 2.0
        arm_in = target_r - spec["boss_dia"] / 2.0
        arm = (cq.Workplane("XY").workplane(offset=stop_z + arm_t)
               .center(s * (arm_out + arm_in) / 2.0, 0)
               .rect(arm_out - arm_in, 12.0)
               .extrude(spec["depth"]))
        hole = (cq.Workplane("XY").workplane(offset=stop_z)
                .center(s * target_r, 0).circle(spec["hole_dia"] / 2)
                .extrude(arm_t + spec["depth"] + 1.0))
        feature = riser.union(arm).cut(hole)
        solid = solid.union(_rot_z(feature, "Y" if axis == "X" else "X"))
    return solid


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

    stop_r = (a.inner_od + a.inner_id) / 4.0        # mid-width of the inner ring top face
    stop_z = F.hard_stop_gap(stop_r, a.tilt_deg, a.ring_height / 2.0)
    arm_stop_z = -a.ring_height / 2.0 - a.stop_drop

    outer = _ring(a.outer_id, a.outer_od, a.ring_height)
    outer = _axis_boss(outer, "Y", a.outer_id, a.outer_od, a.boss_dia)
    outer = _bore_axis(outer, "Y", a.outer_od)
    outer = _bolt_circle(outer, a.airframe_bcd, 6, M3_CLEARANCE, a.ring_height)
    # Stop-screw bosses, on the axis the inner ring rotates across.
    outer = _stop_screws(outer, "Y", a.ring_height / 2.0, stop_r, stop_z,
                         (a.outer_id + a.outer_od) / 4.0,
                         (a.outer_od - a.outer_id) / 2.0)
    parts["outer-ring"] = outer

    inner = _ring(a.inner_id, a.inner_od, a.ring_height)
    inner = _axis_boss(inner, "X", a.inner_id, a.inner_od, a.boss_dia)
    inner = _bore_axis(inner, "X", a.inner_od)
    # The trunnion is a stub that must stop short of the ring turning around it: the pin
    # spans the rest through the bearing. Reaching into the mating bore instead makes the
    # two solids interfere and the gimbal cannot turn.
    inner_tip = a.inner_od / 2 + (a.outer_id - a.inner_od) / 2 - a.running_clearance
    inner = _trunnions(inner, "Y", a.inner_od,
                       (a.outer_id - a.inner_od) / 2 - a.running_clearance,
                       a.boss_dia)
    inner = _trunnion_backing(inner, "Y", a.inner_id / 2, inner_tip,
                              a.boss_dia / 2, a.ring_height)
    # The lever hangs from the pivot axis but must not reach past the bore of the ring
    # turning around it. Placed at inner_od/2 its outer edge sat 2 mm inside the outer
    # ring's annulus, so the two interfered and the gimbal could not turn.
    #
    # It also has to stay clear of the stop-screw insert, which sits just outside the bore
    # on this same axis. I once moved the lever out by the running clearance to make it
    # butt its own annulus, on the theory that its root would back the trunnion pocket.
    # It does not - the lever hangs below the pivot plane and contributes nothing there,
    # and the pocket is backed by _trunnion_backing instead. What that move did do was push
    # the lever's outer edge from y=31 to y=32, into the Ø4 insert pocket spanning
    # 31.2..35.2, so a soldering tip could no longer reach that one insert. The lever is on
    # +radius only, so the defect appeared on one stop screw and not its mirror.
    lever_r = a.inner_id / 2 - a.lever_width / 2 - a.running_clearance
    inner = inner.union(_lever("Y", lever_r, a.gimbal_lever, a.lever_width,
                               a.lever_thickness, BALL_LINK_BORE))
    # Stops for the cradle, reaching below the ring onto the arm shoulder pads.
    inner = _stop_screws(inner, "X", a.ring_height / 2.0,
                         a.arm_x_top + a.arm_thickness / 2 + 3.0, arm_stop_z,
                         (a.inner_id + a.inner_od) / 4.0,
                         (a.inner_od - a.inner_id) / 2.0)
    # Last, so the lever root backs the pocket instead of refilling it.
    inner = _trunnion_pockets(inner, "Y", inner_tip)
    parts["inner-ring"] = inner

    # Cradle: arms on the inner axis reaching down to the motor plate.
    # The arm has to rise ABOVE the pivot axis, not stop at it. The trunnion boss needs
    # material all round it to hold an insert, and with the profile topping out at z = 0
    # the upper half of the pocket opened straight into air: the supported bore measured
    # 1.0 mm against the 6.7 mm the insert needs. The rise is the boss radius plus the
    # minimum wall that insert wants.
    arm_rise = a.boss_dia / 2 + F.INSERT_M3["min_wall"]
    prof = [(a.arm_x_top - a.arm_thickness / 2, arm_rise),
            (a.arm_x_top + a.arm_thickness / 2, arm_rise),
            (a.arm_x_bot + a.arm_thickness / 2, plate_z), (a.arm_x_bot - a.arm_thickness / 2, plate_z)]
    arm = cq.Workplane("XZ").polyline(prof).close().extrude(a.arm_width / 2, both=True)
    cradle = arm.union(arm.mirror("YZ"))
    cradle = cradle.union(cq.Workplane("XY").workplane(offset=plate_z)
                          .circle(a.plate_dia / 2).extrude(a.plate_thickness / 2, both=True))
    arm_outer = a.arm_x_top + a.arm_thickness / 2
    cradle_tip = a.inner_id / 2 - a.running_clearance
    cradle = _trunnions(cradle, "X", 2 * arm_outer,
                        a.inner_id / 2 - arm_outer - a.running_clearance,
                        a.boss_dia)
    # Shoulder pads the inner-ring stop screws land on.
    for s in (+1, -1):
        pad = (cq.Workplane("XY").workplane(offset=arm_stop_z - a.stop_pad_t)
               .center(s * (a.arm_x_top + 1.0), 0)
               .rect(a.arm_thickness + 6.0, a.arm_width + 6.0)
               .extrude(a.stop_pad_t))
        cradle = cradle.union(pad)
    # Motor bolt patterns through the plate, plus a cable pass-through.
    cut = cq.Workplane("XY")
    for hx, hy in MOTOR_HOLES:
        cut = cut.moveTo(hx, hy).circle(M3_CLEARANCE / 2)
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
    # Last: the stop pads land on this same axis and would otherwise refill the pockets.
    cradle = _trunnion_pockets(cradle, "X", cradle_tip)
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
    p.add_argument("--running-clearance", type=float, default=1.0,
                   help="Gap a trunnion stops short of the bore it turns inside, mm")
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
    p.add_argument("--pushrod-len", type=float, default=60.0)
    p.add_argument("--linkage-offset", type=float, default=60.0)
    p.add_argument("--command-limit-deg", type=float, default=8.0)
    p.add_argument("--stop-drop", type=float, default=10.0,
                   help="How far below the inner ring the cradle stop pads sit, mm")
    p.add_argument("--stop-pad-t", type=float, default=4.0)
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
    sweep = F.linkage_travel(args.servo_horn, args.gimbal_lever, args.pushrod_len,
                             args.linkage_offset, 40.0)
    neutral = sweep.get("gimbal_at_neutral_deg", 0.0)
    cmd = [(s, g) for s, g in sweep.get("samples", [])
           if abs(g - neutral) <= args.command_limit_deg]
    ratios = [abs((g2 - g1) / (s2 - s1)) for (s1, g1), (s2, g2) in zip(cmd, cmd[1:])
              if abs(s2 - s1) > 1e-9] or [1.0 / reduction]
    link = {"gimbal_lever_mm": args.gimbal_lever, "servo_horn_mm": args.servo_horn,
            "pushrod_len_mm": args.pushrod_len, "linkage_offset_mm": args.linkage_offset,
            "nominal_reduction": reduction,
            "closes_over_full_servo_range": sweep.get("closes"),
            "gimbal_span_deg": sweep.get("gimbal_span_deg"),
            "ratio_over_command_range": [min(ratios), max(ratios)],
            "ratio_spread_pct": 100.0 * (max(ratios) - min(ratios)) / max(ratios),
            "mg996r_deadband_deg_at_servo": 0.9,
            "deadband_deg_at_gimbal": [0.9 * min(ratios), 0.9 * max(ratios)],
            "hysteresis_target_deg": 0.5,
            "deadband_within_target": 0.9 * max(ratios) <= 0.5}
    stop_r = (args.inner_od + args.inner_id) / 4.0
    stops = {"outer_axis_stop_radius_mm": stop_r,
             "outer_axis_stop_face_z_mm": F.hard_stop_gap(stop_r, args.tilt_deg,
                                                          args.ring_height / 2.0),
             "cradle_stop_pad_z_mm": -args.ring_height / 2.0 - args.stop_drop,
             "screw": "M3 into a heat-set insert, adjustable on the bench",
             "note": "Set each screw against a measured angle. The printed model does "
                     "not hold this tolerance well enough to trust it as drawn."}

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"geometry": {**meta, "per_axis_tilt_deg": args.tilt_deg,
                           "combined_tilt_deg": combined,
                           "min_rotor_offset_mm": h_min,
                           "rotor_offset_mm": args.rotor_offset,
                           "rotor_offset_margin_mm": args.rotor_offset - h_min},
              "linkage": link,
              "hard_stops": stops,
              "fasteners": {
                  "insert": "M3 heat-set, 4.0 mm pilot hole, 6.7 mm deep, 8 mm boss",
                  "bearing": "683ZZ 3x7x3, seat %.2f mm press with a %.1f mm lead-in"
                             % (F.BEARING_683["od"] + F.BEARING_PRESS, F.BEARING_LEADIN),
                  "pivot": "M3 screw from outside through the bearing bore into the "
                           "trunnion insert; a shim washer sets the preload"},
              "parts": {}, "not_yet_designed": [
                  "Pushrods and ball links are bought parts; only their holes are modelled.",
                  "Servo horn geometry is assumed, not taken from a measured horn.",
                  "Stop screw length is set on the bench, not derived here.",
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
    print(f"  linkage horn {args.servo_horn:.0f} / lever {args.gimbal_lever:.0f} / rod "
          f"{args.pushrod_len:.0f} / offset {args.linkage_offset:.0f} mm")
    print(f"    ratio over the +-{args.command_limit_deg:.0f} deg command range "
          f"{min(ratios):.3f}-{max(ratios):.3f} (spread {link['ratio_spread_pct']:.1f}%)")
    print(f"    dead band at gimbal {0.9 * min(ratios):.2f}-{0.9 * max(ratios):.2f} deg, "
          f"target 0.5 -> {'within' if link['deadband_within_target'] else 'OVER'}")
    print(f"  hard stops: M3 screws at r={stop_r:.1f} mm, outer-axis stop face "
          f"z={stops['outer_axis_stop_face_z_mm']:.2f} mm for {args.tilt_deg:.0f} deg")
    print(f"  {'part':15s} {'vol cm3':>9s} {'ASA g':>7s} {'PA-CF g':>8s}   bbox")
    for name, d in report["parts"].items():
        print(f"  {name:15s} {d['volume_cm3']:9.1f} {d['mass_g']['ASA']:7.1f} "
              f"{d['mass_g']['PA-CF']:8.1f}   {d['bbox_mm']}")
    t = report["total"]
    print(f"  {'TOTAL':15s} {t['volume_cm3']:9.1f} {t['mass_g']['ASA']:7.1f} {t['mass_g']['PA-CF']:8.1f}")
    print(f"  wrote STEP/STL per part to {args.output.resolve()}")
    shallow = 0
    # Same expressions the builder uses: both tips stop one running clearance short of the
    # bore they turn inside.
    for name, axis, tip in (("inner-ring", "Y", args.outer_id / 2 - args.running_clearance),
                            ("cradle", "X", args.inner_id / 2 - args.running_clearance)):
        got = trunnion_pocket_depth(parts[name], axis, tip)
        need = F.INSERT_M3["depth"]
        ok = got >= need - 0.05
        flag = "ok" if ok else f"TOO SHALLOW, insert needs {need} mm"
        print(f"  {name} insert pocket {got:.1f} mm deep at r={tip:.1f} mm -> {flag}")
        shallow += 0 if ok else 1
    print("Structure only. Pushrods, bearings, retention and hard stops are bought or "
          "not designed, so this mass is a floor.")
    if shallow:
        print(f"{shallow} insert pocket(s) too shallow. An insert pressed into a pocket "
              f"shorter than itself sits proud and the pivot never closes. Not printable.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
