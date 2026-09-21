"""Generate the rocket-form outer shell for the research airframe, offline.

This is the exterior only: the aerodynamic and protective skin, not a load path. The
baseline specification keeps thrust loads in the carbon skeleton and gimbal bearing blocks,
so this shell carries its own weight and handling loads and nothing else.

Layout follows the reference vehicle, measured from resources/3d-models: propellers at the
bottom, a conical skirt enclosing the gimbal, a cylindrical electronics bay above it, and a
nose cone on top. The reference airframe is 84 mm across and about 585 mm long, built from
split half shells with four internal bulkhead rings.

Every dimension is a parameter. Defaults are stated design choices, not measurements: the
battery, flight controller and ESC dimensions that should drive the bay diameter and length
are not known yet, so the defaults leave room and will need revising. The geometry below
the skirt is deliberately left open, which keeps this file independent of where the gimbal
pivot ends up.

Print mass is reported per material so the shell can be checked against the mass budget
before anything is sliced.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

# Filament densities in g/cm^3, for the print-mass estimate only.
DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "ASA": 1.07, "PA-CF": 1.10}


def ogive_profile(base_radius, length, steps=120):
    """Tangent-ogive radius from tip to base. Returns (z_from_tip, radius) points."""
    if length <= base_radius:
        raise ValueError("Nose length must exceed the base radius for a tangent ogive.")
    rho = (base_radius ** 2 + length ** 2) / (2 * base_radius)
    points = []
    for i in range(steps + 1):
        x = length * i / steps
        r = math.sqrt(max(rho ** 2 - (length - x) ** 2, 0.0)) + base_radius - rho
        points.append((x, max(r, 0.0)))
    return points


def _revolve(profile):
    """Revolve an (r, z) profile about the Z axis.

    revolve() takes its axis in workplane-local coordinates. The default local axis is the
    workplane Y axis, which on "XZ" is the global Z axis, so no axis argument is passed.
    """
    points, tol = [], 1e-7
    for r, z in profile:
        p = (max(r, 0.0), z)
        if not points or math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) > tol:
            points.append(p)
    return cq.Workplane("XZ").polyline(points).close().revolve(360)


def build_nose(base_radius, length, wall, z_base):
    """Hollow tangent-ogive nose, open at its base."""
    outer = [(r, z_base + length - x) for x, r in ogive_profile(base_radius, length)]
    outer = [(0.0, z_base + length)] + [p for p in outer if p[0] > 1e-6] + [(base_radius, z_base)]
    inner_len = max(length - wall, wall * 2)
    inner = [(r, z_base + inner_len - x) for x, r in ogive_profile(max(base_radius - wall, wall), inner_len)]
    inner = [(0.0, z_base + inner_len)] + [p for p in inner if p[0] > 1e-6] + [(base_radius - wall, z_base)]
    return _revolve(outer).cut(_revolve(inner + [(0.0, z_base)]))


def build_body(radius, length, wall, z_base):
    return (cq.Workplane("XY").workplane(offset=z_base).circle(radius).extrude(length)
            .cut(cq.Workplane("XY").workplane(offset=z_base).circle(radius - wall).extrude(length)))


def build_skirt(top_radius, bottom_radius, height, wall, z_base):
    """Truncated cone narrowing downward, open at both ends, enclosing the gimbal."""
    outer = [(bottom_radius, z_base), (top_radius, z_base + height)]
    inner = [(bottom_radius - wall, z_base), (top_radius - wall, z_base + height)]
    shell = [(0.0, z_base)] + outer + [(top_radius, z_base + height)]
    profile = outer + list(reversed(inner))
    return _revolve(profile)


def build_fin(body_radius, root_chord, tip_chord, span, sweep, thickness, z_root):
    """One trapezoidal fin in the XZ plane, extruded across its thickness."""
    r0 = body_radius - 1.0          # start inside the skin so the union is manifold
    r1 = body_radius + span
    pts = [(r0, z_root), (r0, z_root + root_chord),
           (r1, z_root + sweep + tip_chord), (r1, z_root + sweep)]
    return (cq.Workplane("XZ").polyline(pts).close()
            .extrude(thickness, both=True).translate((0, 0, 0)))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--body-od", type=float, default=110.0, help="Electronics bay outer diameter, mm")
    p.add_argument("--body-length", type=float, default=380.0, help="Cylindrical bay length, mm")
    p.add_argument("--nose-length", type=float, default=150.0)
    p.add_argument("--skirt-height", type=float, default=70.0)
    p.add_argument("--skirt-bottom-od", type=float, default=72.0,
                   help="Opening the gimbal passes through, mm")
    p.add_argument("--wall", type=float, default=2.0)
    p.add_argument("--fins", type=int, default=4, help="Fin count; 0 disables fins")
    p.add_argument("--fin-span", type=float, default=55.0)
    p.add_argument("--fin-root-chord", type=float, default=120.0)
    p.add_argument("--fin-tip-chord", type=float, default=45.0)
    p.add_argument("--fin-sweep", type=float, default=60.0)
    p.add_argument("--fin-thickness", type=float, default=2.4)
    p.add_argument("--hatch-width", type=float, default=60.0,
                   help="Access opening width, mm; 0 disables it")
    p.add_argument("--hatch-height", type=float, default=150.0)
    p.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = p.parse_args(argv)

    br, wall = args.body_od / 2.0, args.wall
    z_skirt, z_body = 0.0, args.skirt_height
    z_nose = z_body + args.body_length
    total = args.skirt_height + args.body_length + args.nose_length

    parts = {
        "skirt": build_skirt(br, args.skirt_bottom_od / 2.0, args.skirt_height, wall, z_skirt),
        "body": build_body(br, args.body_length, wall, z_body),
        "nose": build_nose(br, args.nose_length, wall, z_nose),
    }
    if args.fins > 0:
        fin = build_fin(br, args.fin_root_chord, args.fin_tip_chord, args.fin_span,
                        args.fin_sweep, args.fin_thickness / 2.0, z_body + 10.0)
        parts["fin"] = fin

    if args.hatch_width > 0:
        hatch = (cq.Workplane("XY").workplane(offset=z_body + args.body_length * 0.35)
                 .box(args.body_od, args.hatch_width, args.hatch_height, centered=(True, True, False)))
        parts["body"] = parts["body"].cut(hatch)

    assembled = parts["skirt"].union(parts["body"]).union(parts["nose"])
    if args.fins > 0:
        for i in range(args.fins):
            assembled = assembled.union(parts["fin"].rotate((0, 0, 0), (0, 0, 1), 360.0 * i / args.fins))

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"layout": {"skirt_z": [z_skirt, z_body], "body_z": [z_body, z_nose],
                         "nose_z": [z_nose, z_nose + args.nose_length],
                         "total_length_mm": total, "body_od_mm": args.body_od,
                         "wall_mm": wall, "fin_count": args.fins},
              "parts": {}, "assumptions": [
                  "Body diameter and length are design choices, not derived from measured "
                  "battery, flight controller or ESC dimensions.",
                  "The shell is not a load path; thrust loads stay in the carbon skeleton.",
                  "No fastener bosses, bulkhead seats, vents or wiring pass-throughs yet.",
                  "Fin size is chosen for the rocket form, not from an aerodynamic requirement.",
              ], "offline_only": True}

    for name, wp in parts.items():
        vol = wp.val().Volume()
        cq.exporters.export(wp, str(args.output / f"rocket-shell-{name}.step"))
        cq.exporters.export(wp, str(args.output / f"rocket-shell-{name}.stl"))
        bb = wp.val().BoundingBox()
        report["parts"][name] = {
            "volume_cm3": vol / 1000.0,
            "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
            "mass_g": {m: round(vol / 1000.0 * d, 1) for m, d in DENSITY.items()}}

    cq.exporters.export(assembled, str(args.output / "rocket-shell-assembled.step"))
    cq.exporters.export(assembled, str(args.output / "rocket-shell-assembled.stl"))
    for view, direction in (("iso", (1.0, -1.0, 0.45)), ("side", (0.0, -1.0, 0.0))):
        cq.exporters.export(assembled, str(args.output / f"rocket-shell-{view}.svg"),
                            opt={"projectionDir": direction, "width": 620, "height": 900,
                                 "showAxes": False, "strokeWidth": 0.5,
                                 "strokeColor": (40, 40, 40), "hiddenColor": (200, 200, 200),
                                 "showHidden": False, "marginLeft": 20, "marginTop": 20})
    vol = assembled.val().Volume()
    bb = assembled.val().BoundingBox()
    report["assembled"] = {"volume_cm3": vol / 1000.0,
                           "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                           "mass_g": {m: round(vol / 1000.0 * d, 1) for m, d in DENSITY.items()}}
    (args.output / "rocket-shell.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Rocket shell  OD {args.body_od:.0f} mm, total length {total:.0f} mm, wall {wall:.1f} mm")
    print(f"  skirt {z_skirt:.0f}..{z_body:.0f}   body {z_body:.0f}..{z_nose:.0f}   "
          f"nose {z_nose:.0f}..{z_nose + args.nose_length:.0f}")
    print(f"  {'part':10s} {'vol cm3':>9s} {'PLA g':>7s} {'PETG g':>7s} {'ASA g':>7s} {'PA-CF g':>8s}   bbox")
    for name, d in list(report["parts"].items()) + [("ASSEMBLED", report["assembled"])]:
        m = d["mass_g"]
        print(f"  {name:10s} {d['volume_cm3']:9.1f} {m['PLA']:7.1f} {m['PETG']:7.1f} "
              f"{m['ASA']:7.1f} {m['PA-CF']:8.1f}   {d['bbox_mm']}")
    if args.fins > 0:
        print(f"  note: fin volume above is for ONE fin; {args.fins} are fitted.")
    print(f"  wrote STEP/STL per part plus assembled to {args.output.resolve()}")
    print("Exterior skin only. Solid wall, no infill assumption; slicer settings will change mass.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
