"""Involute spur gear profiles for the TVC gimbal reduction.

A proper involute form is used rather than an approximated tooth, because the reduction
sits between the servo and the gimbal axis: any tooth-form error shows up directly as
backlash, and backlash is the quantity the first bench milestone has to measure.

Only what this project needs is implemented: external spur gears, standard 20 degree
pressure angle, and a sector gear for axes that travel a few tens of degrees rather than
turning continuously.
"""
import math

import cadquery as cq

PRESSURE_ANGLE = math.radians(20.0)


def _involute_point(base_radius, t):
    return (base_radius * (math.cos(t) + t * math.sin(t)),
            base_radius * (math.sin(t) - t * math.cos(t)))


def _polar(radius, angle):
    return (radius * math.cos(angle), radius * math.sin(angle))


def gear_profile(module, teeth, backlash=0.0, steps=10):
    """Closed 2D outline of an external involute spur gear, centred on the origin.

    Each tooth is built in polar form: a radial run from the root up to the base circle,
    the involute flank out to the tip, across the tip, back down the mirrored flank, and a
    root arc to the next tooth. Both flanks must converge towards the tip, so the involute
    is reflected on one side; taking it directly on both gives teeth that splay outward.

    backlash thins every tooth by that arc length at the pitch circle, split between the
    two flanks. Printed gears need it; a nominal profile will bind.
    """
    if teeth < 7:
        raise ValueError("Too few teeth for an unmodified involute profile.")
    pitch_r = module * teeth / 2.0
    base_r = pitch_r * math.cos(PRESSURE_ANGLE)
    tip_r = pitch_r + module
    root_r = max(pitch_r - 1.25 * module, base_r * 0.80)
    inv_a = math.tan(PRESSURE_ANGLE) - PRESSURE_ANGLE
    half_tooth = math.pi / (2 * teeth) - backlash / (2 * pitch_r)
    if half_tooth <= 0:
        raise ValueError("Backlash exceeds the tooth thickness.")

    # Involute in polar form: radius grows with t, polar angle is t - atan(t).
    t_tip = math.sqrt(max((tip_r / base_r) ** 2 - 1.0, 0.0))
    flank = []
    for i in range(steps + 1):
        t = t_tip * i / steps
        r = base_r * math.sqrt(1.0 + t * t)
        # Reflected, then shifted so the pitch point sits at +half_tooth.
        flank.append((r, half_tooth + inv_a - (t - math.atan(t))))

    pitch_angle = 2 * math.pi / teeth
    points = []
    for i in range(teeth):
        c = i * pitch_angle
        base_angle = flank[0][1]
        # Up the first flank: radial run to the base circle, then the involute.
        points.append(_polar(root_r, c - base_angle))
        for r, a in flank:
            points.append(_polar(r, c - a))
        # Across the tip, then back down the mirrored flank.
        for r, a in reversed(flank):
            points.append(_polar(r, c + a))
        points.append(_polar(root_r, c + base_angle))
        # Root arc through to the next tooth.
        span = pitch_angle - 2 * base_angle
        for k in range(1, 4):
            points.append(_polar(root_r, c + base_angle + span * k / 4.0))
    return points, {"pitch_radius": pitch_r, "base_radius": base_r,
                    "tip_radius": tip_r, "root_radius": root_r,
                    "inv_alpha": inv_a, "half_tooth_rad": half_tooth}


def spur_gear(module, teeth, width, bore=0.0, backlash=0.15):
    """A full external spur gear, extruded along Z and centred on the origin."""
    points, geom = gear_profile(module, teeth, backlash)
    gear = cq.Workplane("XY").polyline(points).close().extrude(width)
    if bore > 0:
        gear = gear.faces(">Z").workplane().hole(bore)
    return gear, geom


def sector_gear(module, teeth, width, sweep_deg, bore=0.0, backlash=0.15, hub_radius=None):
    """A gear cut back to the arc it actually needs, plus a hub.

    Axes that only travel a few tens of degrees do not need a full wheel, and the removed
    material is mass the gimbal would otherwise have to accelerate.
    """
    gear, geom = spur_gear(module, teeth, width, 0.0, backlash)
    hub_r = hub_radius if hub_radius is not None else geom["root_radius"] * 0.55
    half = math.radians(sweep_deg) / 2.0
    # Keep a wedge of the toothed rim, plus a full hub disc to carry the bore.
    reach = geom["tip_radius"] * 1.2
    wedge_pts = [(0.0, 0.0)]
    n = 24
    for i in range(n + 1):
        a = -half + 2 * half * i / n
        wedge_pts.append((reach * math.cos(a), reach * math.sin(a)))
    wedge = cq.Workplane("XY").polyline(wedge_pts).close().extrude(width)
    sector = gear.intersect(wedge).union(
        cq.Workplane("XY").circle(hub_r).extrude(width))
    if bore > 0:
        sector = sector.faces(">Z").workplane().hole(bore)
    return sector, {**geom, "hub_radius": hub_r, "sweep_deg": sweep_deg}


def centre_distance(module, teeth_a, teeth_b):
    return module * (teeth_a + teeth_b) / 2.0
