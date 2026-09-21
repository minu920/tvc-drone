"""Shared assembly features for the printed airframe parts.

The numbers here come from published practice for FDM parts rather than from guesses, and
they are kept in one place because several generators have to agree on them. Sources are
noted per constant.

Threaded joints use M3 brass heat-set inserts rather than screwing into plastic. A screw
cut directly into a printed boss strips after a few cycles, and this airframe is meant to
come apart repeatedly for sensor and gain changes.

Bearing seats are sized for a light press. FDM holes come out undersize, so the modelled
pocket is only slightly below the bearing outer diameter, and a lead-in chamfer is added
because a square-mouthed pocket shaves the bearing on the way in and cracks thin bosses.
"""
import math

import cadquery as cq

# M3 heat-set insert, Ruthex / CNC Kitchen pattern.
# Pilot hole 4.0 mm; depth is insert length plus about 1 mm so displaced plastic has
# somewhere to go in a blind hole; keep at least 1.6 mm of wall around the insert.
INSERT_M3 = {"hole_dia": 4.0, "length": 5.7, "depth": 6.7, "min_wall": 1.6,
             "boss_dia": 8.0, "screw": 3.0}
M3_CLEARANCE = 3.4
M3_HEAD_DIA = 6.0

# Ball bearing fits for FDM: press about -0.1 mm on the bore, snug +0.05, sliding +0.15.
BEARING_PRESS = -0.10
BEARING_LEADIN = 0.5      # 45 degree chamfer at the pocket mouth

# 683ZZ, 3 x 7 x 3 mm. A 3 mm bore lets an M3 screw double as the pivot pin, so the screw
# clamps the inner race and sets preload directly.
BEARING_683 = {"id": 3.0, "od": 7.0, "width": 3.0}


def insert_hole(spec=INSERT_M3, extra_depth=0.0):
    """Blind pilot hole body for a heat-set insert, rising along +Z from the origin."""
    return (cq.Workplane("XY").circle(spec["hole_dia"] / 2)
            .extrude(spec["depth"] + extra_depth))


def insert_boss(spec=INSERT_M3, height=None):
    """Boss with enough wall around the insert, plus the pilot hole already cut."""
    h = height if height is not None else spec["depth"] + 1.5
    boss = cq.Workplane("XY").circle(spec["boss_dia"] / 2).extrude(h)
    return boss.cut(cq.Workplane("XY").workplane(offset=h - spec["depth"])
                    .circle(spec["hole_dia"] / 2).extrude(spec["depth"]))


def bearing_pocket_cutter(bearing=BEARING_683, seat_depth=None, through_dia=None,
                          leadin=BEARING_LEADIN, press=BEARING_PRESS):
    """Cutter for a bearing seat: chamfered mouth, press-fit bore, then a shoulder.

    The shoulder sets how deep the bearing goes and gives the outer race something to sit
    against, so the press fit is not the only thing holding it axially. Built along +Z from
    the origin, cutting downward into material below z = 0.
    """
    od = bearing["od"] + press
    depth = seat_depth if seat_depth is not None else bearing["width"] + 0.2
    thru = through_dia if through_dia is not None else bearing["od"] - 1.6
    cutter = (cq.Workplane("XY").circle(od / 2).extrude(-depth)
              .union(cq.Workplane("XY").workplane(offset=-depth)
                     .circle(thru / 2).extrude(-20.0)))
    # 45 degree lead-in at the mouth.
    cone = (cq.Workplane("XY").circle(od / 2 + leadin)
            .workplane(offset=-leadin).circle(od / 2).loft())
    return cutter.union(cone)


def louver_slots(count, radius, z_centre, slot_w, slot_h, depth, rake_deg=25.0):
    """Ring of raked slots for cooling flow.

    Raked rather than square so the opening prints without support on a vertical wall and
    so the slot does not look straight down the airflow path.
    """
    cutter = None
    for i in range(count):
        a = 2 * math.pi * i / count
        slot = (cq.Workplane("XZ").rect(slot_w, slot_h).extrude(depth)
                .rotate((0, 0, 0), (1, 0, 0), rake_deg)
                .translate((0, radius - depth / 2, z_centre))
                .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)))
        cutter = slot if cutter is None else cutter.union(slot)
    return cutter


def grommet_hole(dia, count, radius, z_centre):
    """Ring of round wire pass-throughs. Round because a slot concentrates stress."""
    cutter = None
    for i in range(count):
        a = 2 * math.pi * i / count + math.pi / count
        h = (cq.Workplane("XZ").circle(dia / 2).extrude(20.0)
             .translate((0, radius - 10.0, z_centre))
             .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)))
        cutter = h if cutter is None else cutter.union(h)
    return cutter


def hard_stop_gap(radius, angle_deg, face_z):
    """Height at which a stop face must sit to arrest rotation at a given angle.

    A pad at (radius, face_z) on the rotating member swings up to this height, so an
    overhanging stop placed here makes contact exactly at the limit.
    """
    a = math.radians(angle_deg)
    return radius * math.sin(a) + face_z * math.cos(a)


def four_bar_gimbal_angle(servo_deg, horn_r, lever_r, rod_len, offset):
    """Gimbal angle for a servo angle in a planar pushrod linkage.

    Both cranks start perpendicular to the rod. Returns None when the geometry cannot close,
    which is what a too-short rod or too-large travel looks like.
    """
    hx = offset - horn_r * math.sin(math.radians(servo_deg))
    hy = horn_r * math.cos(math.radians(servo_deg))
    d = math.hypot(hx, hy)
    if d > rod_len + lever_r or d < abs(rod_len - lever_r):
        return None
    cos_t = (d * d + lever_r * lever_r - rod_len * rod_len) / (2 * d * lever_r)
    cos_t = max(-1.0, min(1.0, cos_t))
    return math.degrees(math.atan2(hy, hx) + math.acos(cos_t)) - 90.0


def linkage_travel(horn_r, lever_r, rod_len, offset, servo_range_deg, steps=24):
    """Sample the linkage across the servo range; reports gimbal travel and ratio spread."""
    rows = []
    for i in range(steps + 1):
        s = -servo_range_deg + 2 * servo_range_deg * i / steps
        g = four_bar_gimbal_angle(s, horn_r, lever_r, rod_len, offset)
        rows.append((s, g))
    valid = [(s, g) for s, g in rows if g is not None]
    if len(valid) < 2:
        return {"closes": False, "samples": rows}
    zero = [g for s, g in valid if abs(s) < 1e-9]
    base = zero[0] if zero else valid[len(valid) // 2][1]
    span = max(g for _, g in valid) - min(g for _, g in valid)
    ratios = [abs((g2 - g1) / (s2 - s1)) for (s1, g1), (s2, g2) in zip(valid, valid[1:])
              if abs(s2 - s1) > 1e-9]
    return {"closes": len(valid) == len(rows), "gimbal_span_deg": span,
            "gimbal_at_neutral_deg": base,
            "ratio_min": min(ratios), "ratio_max": max(ratios),
            "ratio_spread_pct": 100.0 * (max(ratios) - min(ratios)) / max(ratios),
            "samples": valid}
