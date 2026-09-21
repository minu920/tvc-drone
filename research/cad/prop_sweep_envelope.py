"""Generate the propeller sweep envelope for the coaxial TVC gimbal, offline.

The envelope is the volume swept by the two rotor discs as the gimbal moves through its
travel. No external structure may enter it. This is a clearance body, not a part.

Only inputs already fixed by the baseline specification are required: propeller diameter,
rotor spacing range and gimbal travel. Nothing here depends on a part that has not been
measured yet, which is why this can be built before the motor, servo and frame are known.

Two inputs are explicit assumptions and are labelled as such in the output: the axial
thickness of the swept rotor disc, and the clearance margin. Replace them once the
propeller and the assembled gimbal are measured.

The gimbal axes are independent, so the combined tilt exceeds the per-axis limit. Using the
repository convention Rx(alpha) @ Ry(beta) @ [0, 0, -1], the angle from the nominal thrust
axis satisfies cos(theta) = cos(alpha) * cos(beta). Clearance is sized on the hard stop,
not the commanded limit, so that a runaway command cannot drive the rotors into structure.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

PROP_DIAMETER_MM = 304.8       # APC 12x4.5MR/MRP, 12 inch. Fixed by the baseline spec.
SPACING_MIN_MM = 0.15 * PROP_DIAMETER_MM
SPACING_MAX_MM = 0.35 * PROP_DIAMETER_MM


def combined_tilt_deg(per_axis_deg):
    """Worst-case tilt from the nominal thrust axis when both gimbal axes are at their limit."""
    c = math.cos(math.radians(per_axis_deg))
    return math.degrees(math.acos(c * c))


def _body_boundary(radius, spacing, thickness, pivot_offset, samples=400):
    """Sample the outline of the two rotor discs in the (r, z) half-plane, relative to the pivot."""
    points = []
    for sign in (+1, -1):
        centre = sign * spacing / 2.0 - pivot_offset
        z_lo, z_hi = centre - thickness / 2.0, centre + thickness / 2.0
        for i in range(samples + 1):
            f = i / samples
            points.append((radius * f, z_lo))          # bottom face, axis outward
            points.append((radius * f, z_hi))          # top face
            points.append((radius, z_lo + f * thickness))  # outer rim
    return points


def envelope_profile(radius, spacing, thickness, tilt_deg, pivot_offset, margin, steps=721):
    """Outer profile of the swept volume as (r, z) points, from +axis around to -axis.

    A body point at polar (rho, phi) about the pivot can reach any polar angle psi with
    |psi - phi| <= tilt. The outer envelope is the largest rho reaching each psi. Taking the
    maximum per direction fills the notch between the two discs, which is conservative and
    correct here: that space is occupied by the gimbal itself, so no external structure may
    use it either.
    """
    tilt = math.radians(tilt_deg)
    polar = []
    for r, z in _body_boundary(radius, spacing, thickness, pivot_offset):
        polar.append((math.hypot(r, z), math.atan2(r, z)))

    profile = []
    for i in range(steps):
        psi = math.pi * i / (steps - 1)
        reach = max((rho for rho, phi in polar if abs(psi - phi) <= tilt), default=0.0)
        if reach > 0.0:
            reach += margin
        profile.append((reach * math.sin(psi), reach * math.cos(psi)))
    return profile


def build_solid(profile, tol=1e-6):
    """Revolve the (r, z) profile about the Z axis, dropping the degenerate edges.

    The profile starts and ends on the rotation axis, so the closing segment runs straight
    down the axis. Consecutive duplicates there would make zero-length edges, which the
    kernel rejects.
    """
    points = []
    for r, z in profile:
        p = (max(r, 0.0), z)
        if not points or math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) > tol:
            points.append(p)
    if points[0][0] > tol:
        points.insert(0, (0.0, points[0][1]))
    if points[-1][0] > tol:
        points.append((0.0, points[-1][1]))
    # revolve() takes its axis in workplane-local coordinates, not world coordinates.
    # The default local axis is the workplane Y axis, which on "XZ" is the global Z axis.
    # Passing world (0,0,1) here revolves about the plane normal and yields an empty solid.
    return cq.Workplane("XZ").polyline(points).close().revolve(360)


def plot_profile(path, profile, radius, spacing, thickness, tilt_deg, pivot_offset):
    """Draw the swept profile with the rotor discs at rest and at full tilt, for inspection."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    mirrored = [(-r, z) for r, z in reversed(profile)]
    ax.fill([r for r, _ in profile + mirrored], [z for _, z in profile + mirrored],
            color="0.85", edgecolor="0.4", linewidth=1.0, label="swept envelope")

    for tilt, style, label in ((0.0, "-", "rotors at rest"),
                               (math.radians(tilt_deg), "--", f"rotors at {tilt_deg:.1f} deg")):
        for sign in (+1, -1):
            centre = sign * spacing / 2.0 - pivot_offset
            corners = [(-radius, centre - thickness / 2), (radius, centre - thickness / 2),
                       (radius, centre + thickness / 2), (-radius, centre + thickness / 2)]
            rot = [(r * math.cos(tilt) + z * math.sin(tilt),
                    -r * math.sin(tilt) + z * math.cos(tilt)) for r, z in corners]
            rot.append(rot[0])
            ax.plot([p[0] for p in rot], [p[1] for p in rot], style, color="C3" if tilt else "C0",
                    linewidth=1.2, label=label if sign > 0 else None)

    ax.plot(0, 0, "k+", markersize=10, label="gimbal pivot")
    ax.axhline(0, color="0.8", linewidth=0.5)
    ax.axvline(0, color="0.8", linewidth=0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("radial [mm]")
    ax.set_ylabel("axial [mm]")
    ax.set_title(f"Prop sweep envelope — spacing {spacing:.0f} mm, combined tilt {tilt_deg:.1f} deg")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def summarise(name, profile, solid, inputs):
    radii = [r for r, _ in profile]
    zs = [z for r, z in profile if r > 0.0]
    bb = solid.val().BoundingBox()
    return {"case": name, **inputs,
            "max_radius_mm": max(radii), "max_diameter_mm": 2 * max(radii),
            "axial_top_mm": max(zs), "axial_bottom_mm": min(zs),
            "axial_height_mm": max(zs) - min(zs),
            "bbox_diameter_mm": max(bb.xlen, bb.ylen), "bbox_height_mm": bb.zlen,
            "volume_mm3": solid.val().Volume(),
            "exceeds_x1c_bed": 2 * max(radii) > 256.0}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spacing", type=float, default=SPACING_MAX_MM,
                        help="Axial rotor spacing in mm. Default is the 0.35D end of the range.")
    parser.add_argument("--per-axis-deg", type=float, default=20.0,
                        help="Per-axis gimbal travel. Default 20 deg, the hard stop.")
    parser.add_argument("--disc-thickness", type=float, default=20.0,
                        help="ASSUMED axial thickness of one swept rotor disc, mm.")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="ASSUMED clearance margin added to the swept surface, mm.")
    parser.add_argument("--pivot-offset", type=float, default=0.0,
                        help="Pivot position along the thrust axis from the rotor midpoint, mm.")
    parser.add_argument("--output", type=Path, default=Path("artifacts/cad"))
    args = parser.parse_args(argv)

    if not SPACING_MIN_MM - 1e-9 <= args.spacing <= SPACING_MAX_MM + 1e-9:
        parser.error(f"--spacing {args.spacing} mm is outside the specified "
                     f"{SPACING_MIN_MM:.1f}-{SPACING_MAX_MM:.1f} mm range (0.15D-0.35D).")

    radius = PROP_DIAMETER_MM / 2.0
    tilt = combined_tilt_deg(args.per_axis_deg)
    inputs = {"prop_diameter_mm": PROP_DIAMETER_MM, "rotor_spacing_mm": args.spacing,
              "per_axis_travel_deg": args.per_axis_deg, "combined_tilt_deg": tilt,
              "assumed_disc_thickness_mm": args.disc_thickness,
              "assumed_clearance_margin_mm": args.margin,
              "pivot_offset_from_rotor_midpoint_mm": args.pivot_offset}

    profile = envelope_profile(radius, args.spacing, args.disc_thickness, tilt,
                               args.pivot_offset, args.margin)
    solid = build_solid(profile)
    summary = summarise("hard_stop", profile, solid, inputs)

    args.output.mkdir(parents=True, exist_ok=True)
    stem = f"prop-sweep-envelope_{args.spacing:.0f}mm_{args.per_axis_deg:.0f}deg"
    cq.exporters.export(solid, str(args.output / f"{stem}.step"))
    cq.exporters.export(solid, str(args.output / f"{stem}.stl"))
    plot_profile(args.output / f"{stem}.png", profile, radius, args.spacing,
                 args.disc_thickness, tilt, args.pivot_offset)

    # Reference cases, reported but not exported, so the effect of each input is visible.
    reference = []
    for label, spacing, per_axis in (("min spacing, hard stop", SPACING_MIN_MM, 20.0),
                                     ("max spacing, mechanical", args.spacing, 15.0),
                                     ("max spacing, commanded", args.spacing, 8.0)):
        t = combined_tilt_deg(per_axis)
        p = envelope_profile(radius, spacing, args.disc_thickness, t, args.pivot_offset, args.margin)
        rs = [r for r, _ in p]
        zs = [z for r, z in p if r > 0.0]
        reference.append({"case": label, "rotor_spacing_mm": spacing,
                          "per_axis_travel_deg": per_axis, "combined_tilt_deg": t,
                          "max_diameter_mm": 2 * max(rs),
                          "axial_height_mm": max(zs) - min(zs)})

    report = {"summary": summary, "reference_cases": reference,
              "assumptions": ["Swept disc thickness is assumed, not measured from the propeller.",
                              "Clearance margin is assumed; it does not cover deflection under load.",
                              "The pivot is placed on the thrust axis; a real gimbal has two "
                              "offset axes, which this single-pivot model does not represent.",
                              "Blade flapping, vibration and structural deflection are excluded."],
              "offline_only": True}
    (args.output / f"{stem}.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Prop sweep envelope  spacing {args.spacing:.1f} mm, "
          f"per-axis {args.per_axis_deg:.0f} deg -> combined {tilt:.2f} deg")
    print(f"  outer diameter   {summary['max_diameter_mm']:8.1f} mm")
    print(f"  axial extent     {summary['axial_bottom_mm']:+8.1f} .. {summary['axial_top_mm']:+.1f} mm "
          f"(height {summary['axial_height_mm']:.1f} mm)")
    print(f"  volume           {summary['volume_mm3'] / 1000:8.1f} cm^3")
    print(f"  exceeds X1C 256 mm bed: {summary['exceeds_x1c_bed']}")
    print("  reference cases:")
    for row in reference:
        print(f"    {row['case']:26s} spacing {row['rotor_spacing_mm']:6.1f}  "
              f"tilt {row['combined_tilt_deg']:5.2f} deg  "
              f"dia {row['max_diameter_mm']:7.1f}  height {row['axial_height_mm']:6.1f}")
    print(f"  wrote {stem}.step / .stl / .json to {args.output.resolve()}")
    print("Clearance body only. Assumed disc thickness and margin; not a measured envelope.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
