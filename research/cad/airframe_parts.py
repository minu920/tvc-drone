"""Generate the airframe internals, offline.

  bulkhead      rings carrying the carbon spine, locating the gimbal, mounting the board
  battery tray  a cradle for the selected pack

Landing gear lives in landing_gear.py.

Ring diameters and bolt circles are checked against each other rather than trusted.
Shrinking the ring for the bare-frame build once left 0.3 mm of rim outside the gimbal
bolt circle, which prints as nothing, so check_walls() now refuses to emit a ring whose
holes leave under 2 mm anywhere.

The board mounting pattern is read off the flight controller rather than guessed: four M3
holes on a 62.04 mm circle at plus/minus 73.0 and plus/minus 107.2 degrees.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

import features as F

DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}
M3_CLEARANCE = 3.4




def check_walls(outer_dia, bore, features, minimum=2.0):
    """Reject a ring whose holes leave too little material to survive printing.

    Reducing the ring diameter without rechecking the bolt circles on it left 0.3 mm of rim
    outside the gimbal holes, which prints as nothing. A ring is cheap to resize and
    expensive to discover broken after assembly, so this is an error rather than a warning.
    """
    problems = []
    for name, bcd, hole_dia in features:
        outer_wall = outer_dia / 2 - (bcd / 2 + hole_dia / 2)
        inner_wall = (bcd / 2 - hole_dia / 2) - bore / 2
        if outer_wall < minimum:
            problems.append(f"{name}: {outer_wall:.2f} mm to the rim")
        if inner_wall < minimum:
            problems.append(f"{name}: {inner_wall:.2f} mm to the bore")
    if problems:
        raise ValueError(
            f"Ring {outer_dia:.1f} mm outer / {bore:.1f} mm bore leaves under "
            f"{minimum} mm of wall: " + "; ".join(problems))
    return [{"feature": n, "bcd_mm": b, "hole_dia_mm": d,
             "wall_to_rim_mm": outer_dia / 2 - (b / 2 + d / 2),
             "wall_to_bore_mm": (b / 2 - d / 2) - bore / 2} for n, b, d in features]


def check_hole_spacing(placed, minimum=2.0):
    """Reject a ring whose holes are too close to EACH OTHER.

    check_walls compares every bolt circle with the rim and the bore, and passing it says
    nothing about two features on different circles colliding. Three wiring holes placed at
    a fixed angular offset put one of them 6.35 mm from an insert boss whose radii sum to
    8.5 mm, so the Ø9 hole ate 2.15 mm into the boss and that insert had no wall left at
    all. The audit found it only because the pocket then measured 0.0 mm.
    """
    problems = []
    for i in range(len(placed)):
        ni, xi, yi, di = placed[i]
        for j in range(i + 1, len(placed)):
            nj, xj, yj, dj = placed[j]
            gap = math.hypot(xi - xj, yi - yj) - (di + dj) / 2
            if gap < minimum:
                problems.append(f"{ni} and {nj}: {gap:.2f} mm apart")
    if problems:
        raise ValueError(
            f"Features on this ring come within {minimum} mm of each other: "
            + "; ".join(problems))
    return True


def place_clear_holes(count, radius, hole_dia, fixed, minimum=2.0, step_deg=1.0):
    """Angles for `count` holes at `radius` that clear everything already on the ring.

    Placing them at an even spacing plus a constant offset is what collided with an insert
    boss. Candidate angles are scanned instead, the ones that clear every fixed feature are
    kept, and the requested number is chosen greedily to be as far apart as possible.
    """
    cand = []
    for k in range(int(360 / step_deg)):
        a = math.radians(k * step_deg)
        x, y = radius * math.cos(a), radius * math.sin(a)
        if all(math.hypot(x - fx, y - fy) - (hole_dia + fd) / 2 >= minimum
               for _, fx, fy, fd in fixed):
            cand.append(k * step_deg)
    if len(cand) < count:
        raise ValueError(
            f"Only {len(cand)} of {int(360/step_deg)} angles at r={radius:.1f} mm clear the "
            f"existing features by {minimum} mm; cannot place {count} holes of "
            f"Ø{hole_dia} mm. Move them to another radius or use fewer.")
    sep = lambda a, b: min(abs(a - b), 360 - abs(a - b))
    chosen = [cand[0]]
    while len(chosen) < count:
        chosen.append(max(cand, key=lambda c: min(sep(c, p) for p in chosen)))
    return chosen


def leg_bolt_positions(attach_r, bolt_radial, bolt_tang, azimuths):
    """Where the landing-gear brackets' bolts land on a bulkhead.

    The bracket plate is narrow radially and long tangentially, and its four bolts sit at
    +-bolt_radial radially and +-bolt_tang/2 tangentially about the attachment radius. The
    bulkhead had no holes at any of these points, so the brackets that carry the entire
    landing load had nothing to bolt into: the pattern existed on one part only.
    """
    pts = []
    for az in azimuths:
        a = math.radians(az)
        for dr in (-bolt_radial, bolt_radial):
            for dt in (-bolt_tang / 2, bolt_tang / 2):
                x, y = attach_r + dr, dt
                pts.append((x * math.cos(a) - y * math.sin(a),
                            x * math.sin(a) + y * math.cos(a)))
    return pts


def bulkhead(outer_dia, thickness, bore, spine_bcd, spine_holes, spine_dia,
             mount_bcd, mount_holes, insert_bcd, insert_angles, flange_width,
             flange_thickness, wire_dia, wire_count, leg_bolts=()):
    """Universal bay ring: carries the spine, takes the gimbal and leg brackets, passes wires.

    One part serves all four stations. Inserts rather than tapped plastic, because the
    gimbal and legs come off repeatedly. Notches at the split plane clear the shell's
    internal bolting flanges, without which the bulkhead cannot drop into a closed half.
    """
    walls = check_walls(outer_dia, bore, [
        ("spine", spine_bcd, spine_dia),
        ("gimbal mount", mount_bcd, M3_CLEARANCE),
        ("insert boss", insert_bcd, F.INSERT_M3["boss_dia"]),
    ])
    ring = cq.Workplane("XY").circle(outer_dia / 2).circle(bore / 2).extrude(thickness)

    # Insert bosses stand proud of the ring so there is wall around each insert. The
    # angles come from the flight controller's own mounting holes rather than an even
    # spacing, so the board bolts straight down onto a bulkhead.
    for a_deg in insert_angles:
        a = math.radians(a_deg)
        ring = ring.union(F.insert_boss(height=F.INSERT_M3["depth"] + 1.5)
                          .translate((insert_bcd / 2 * math.cos(a),
                                      insert_bcd / 2 * math.sin(a), 0)))

    # Everything already committed to this ring, as (name, x, y, diameter).
    placed = [(f"insert boss {a:.0f} deg",
               insert_bcd / 2 * math.cos(math.radians(a)),
               insert_bcd / 2 * math.sin(math.radians(a)),
               F.INSERT_M3["boss_dia"]) for a in insert_angles]
    cut = cq.Workplane("XY")
    for i in range(spine_holes):
        a = 2 * math.pi * i / spine_holes + math.pi / spine_holes
        x, y = spine_bcd / 2 * math.cos(a), spine_bcd / 2 * math.sin(a)
        placed.append((f"spine {i}", x, y, spine_dia))
        cut = cut.moveTo(x, y).circle(spine_dia / 2)
    for i in range(mount_holes):
        a = 2 * math.pi * i / mount_holes
        x, y = mount_bcd / 2 * math.cos(a), mount_bcd / 2 * math.sin(a)
        placed.append((f"gimbal mount {i}", x, y, M3_CLEARANCE))
        cut = cut.moveTo(x, y).circle(M3_CLEARANCE / 2)
    for i, (x, y) in enumerate(leg_bolts):
        placed.append((f"leg bolt {i}", x, y, M3_CLEARANCE))
        cut = cut.moveTo(x, y).circle(M3_CLEARANCE / 2)
    wire_r = (bore / 2 + outer_dia / 2) / 2
    wire_angles = place_clear_holes(wire_count, wire_r, wire_dia, placed)
    for i, a_deg in enumerate(wire_angles):
        a = math.radians(a_deg)
        x, y = wire_r * math.cos(a), wire_r * math.sin(a)
        placed.append((f"wire {i}", x, y, wire_dia))
        cut = cut.moveTo(x, y).circle(wire_dia / 2)
    check_hole_spacing(placed)
    ring = ring.cut(cut.extrude(thickness * 6, both=True))

    # Clearance for the shell split flanges at the split plane. Skipped when there is
    # no shell to clear.
    if flange_width <= 0 or flange_thickness <= 0:
        return ring, walls
    notch = (cq.Workplane("XY")
             .rect(outer_dia, 2 * flange_thickness + 0.6)
             .extrude(thickness * 6, both=True)
             .intersect(cq.Workplane("XY").circle(outer_dia / 2)
                        .circle(outer_dia / 2 - flange_width - 0.4)
                        .extrude(thickness * 6, both=True)))
    return ring.cut(notch), walls


def battery_tray(pack_l, pack_w, pack_h, wall, frame_dia, strap_width, insert_bcd,
                 insert_angles):
    """U-channel cradling the pack along the vehicle axis, bolting to a bulkhead.

    The pack is 137 mm long and no bulkhead bore it could pass through is wider than about
    48 mm, so it stands up the axis in the open bay rather than lying across it. An earlier
    version laid it along X and then trimmed the result to the bore, which quietly cut the
    137 mm cradle down to the bore diameter.

    Nothing here has to fit through a bore: the tray is bolted in from the side of an open
    frame. It only has to stay inside the frame envelope.
    """
    length = pack_l + 2 * wall
    outer = (cq.Workplane("XY")
             .box(pack_w + 2 * wall, pack_h + 2 * wall, length, centered=(True, True, False)))
    # Pocket for the pack, open at the top and along +Y so it can be dropped in.
    pocket = (cq.Workplane("XY").workplane(offset=wall)
              .box(pack_w, pack_h + 2 * wall, length, centered=(True, True, False))
              .translate((0, wall, 0)))
    tray = outer.cut(pocket)

    # Base flange bolting to the bulkhead inserts.
    flange = cq.Workplane("XY").box(insert_bcd + 14.0, insert_bcd + 14.0, wall,
                                    centered=(True, True, False))
    holes = cq.Workplane("XY")
    for a_deg in insert_angles:
        a = math.radians(a_deg)
        holes = holes.moveTo(insert_bcd / 2 * math.cos(a),
                             insert_bcd / 2 * math.sin(a)).circle(M3_CLEARANCE / 2)
    flange = flange.cut(holes.extrude(wall * 4, both=True))
    flange = flange.intersect(cq.Workplane("XY").circle(frame_dia / 2)
                              .extrude(wall * 4, both=True))
    tray = tray.union(flange)

    # Strap slots through the two side walls only. Cutting the full cross-section, as an
    # earlier version did, severed the cradle into three separate solids: a slicer then
    # treats each as its own part.
    for f in (0.25, 0.75):
        z = wall + pack_l * f
        for s in (-1, 1):
            slot = (cq.Workplane("XY")
                    .box(wall + 1.0, pack_h + 2 * wall + 2.0, strap_width,
                         centered=(True, True, False))
                    .translate((s * (pack_w + wall) / 2, 0, z - strap_width / 2)))
            tray = tray.cut(slot)
    return tray




def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--body-od", type=float, default=90.0)
    # The vehicle is a bare frame: the shell was dropped for mass, so that is the
    # default rather than an opt-in. Leaving it opt-out meant a plain run rebuilt the
    # shell, and in airframe_parts it also derived the bulkhead from the shell bore
    # and tripped check_walls.
    p.add_argument("--shell", action="store_true",
                   help="Rebuild the aerodynamic shell and size parts to its bore")
    p.add_argument("--bulkhead-od", type=float, default=88.0,
                   help="Ring outer diameter when --no-shell is set, mm")
    p.add_argument("--body-wall", type=float, default=1.2)
    p.add_argument("--bulkhead-thickness", type=float, default=4.0)
    # 48 mm keeps 2 mm of wall everywhere. The battery cannot pass this bore either
    # way (55 mm diagonal), so it lives in the bay between two stations instead.
    p.add_argument("--bulkhead-bore", type=float, default=48.0)
    p.add_argument("--spine-holes", type=int, default=4)
    p.add_argument("--spine-dia", type=float, default=8.2, help="Carbon spine tube, 8 mm nominal")
    p.add_argument("--spine-bcd", type=float, default=70.0)
    p.add_argument("--gimbal-bcd", type=float, default=80.0)
    p.add_argument("--pack", type=str, default="137x44x33")
    p.add_argument("--tray-wall", type=float, default=2.4)
    p.add_argument("--strap-width", type=float, default=16.0)
    # Read off the flight controller board: four M3 holes on a 62.04 mm circle at
    # plus/minus 73.0 and plus/minus 107.2 degrees.
    p.add_argument("--insert-bcd", type=float, default=62.04,
                   help="Bolt circle for the heat-set inserts, matching the FC board")
    p.add_argument("--insert-angles", type=str, default="73.0,107.2,253.0,287.0",
                   help="Insert angles in degrees, matching the FC mounting holes")
    p.add_argument("--flange-width", type=float, default=8.0)
    p.add_argument("--flange-thickness", type=float, default=3.0)
    # Must match landing_gear.py: --straight-attach-r, and the bracket bolt pattern
    # derived from --bracket-radial (0.28 of it) and --bracket-bolt-span.
    p.add_argument("--leg-attach-r", type=float, default=34.0)
    p.add_argument("--leg-bolt-radial", type=float, default=19.0 * 0.28)
    p.add_argument("--leg-bolt-span", type=float, default=30.0)
    p.add_argument("--leg-azimuths", type=str, default="0,120,240")
    p.add_argument("--wire-dia", type=float, default=9.0)
    p.add_argument("--wire-count", type=int, default=3)
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)
    args.no_shell = not args.shell

    body_bore = args.body_od - 2 * args.body_wall
    L, W, H = (float(v) for v in args.pack.lower().split("x"))

    # With no shell the ring sets its own diameter and needs no flange relief, because
    # there is no split shell for it to drop into.
    ring_od = args.bulkhead_od if args.no_shell else body_bore - 0.4
    flange_w = 0.0 if args.no_shell else args.flange_width
    flange_t = 0.0 if args.no_shell else args.flange_thickness

    insert_angles = [float(v) for v in args.insert_angles.split(",")]
    ring, walls = bulkhead(ring_od, args.bulkhead_thickness, args.bulkhead_bore,
                           args.spine_bcd, args.spine_holes, args.spine_dia,
                           args.gimbal_bcd, 6, args.insert_bcd, insert_angles,
                           flange_w, flange_t, args.wire_dia, args.wire_count,
                           leg_bolt_positions(
                               args.leg_attach_r, args.leg_bolt_radial,
                               args.leg_bolt_span,
                               [float(v) for v in args.leg_azimuths.split(",")]))
    parts = {
        "bulkhead": ring,
        "battery-tray": battery_tray(L, W, H, args.tray_wall,
                                     (ring_od if args.no_shell else body_bore) - 0.6,
                                     args.strap_width, args.insert_bcd, insert_angles),
    }
    counts = {"bulkhead": 4, "battery-tray": 1}

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"bulkhead_walls_mm": walls,
              "fc_mounting": {"bolt_circle_mm": args.insert_bcd,
                              "angles_deg": insert_angles,
                              "hole_dia_mm": 3.2,
                              "source": "read from resources/pcb/rera.kicad_pcb"},
              "battery_clearance": {
                  "pack_mm": [L, W, H], "bore_mm": args.bulkhead_bore,
                  "pack_diagonal_mm": (W ** 2 + H ** 2) ** 0.5,
                  "passes_through_bore": (W ** 2 + H ** 2) ** 0.5 <= args.bulkhead_bore,
                  "note": "The pack does not fit through a bulkhead, so it has to live in "
                          "a bay between two stations rather than being threaded through."},
              "parts": {}, "assumptions": [
                  "Ring diameters and bolt circles are checked against each other, but no "
                  "load case sizes them.",
                  "The board mounting pattern is read from the PCB file, not measured off "
                  "a physical board.",
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

    print(f"Airframe parts  ring {ring_od:.1f} mm outer, bore {args.bulkhead_bore:.0f} mm, "
          f"pack {L:.0f}x{W:.0f}x{H:.0f} mm")
    print("  wall check:")
    for w in walls:
        print(f"    {w['feature']:14s} BCD {w['bcd_mm']:6.2f}  rim {w['wall_to_rim_mm']:5.2f}  "
              f"bore {w['wall_to_bore_mm']:5.2f} mm")
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
