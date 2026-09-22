"""Generate a fit-test coupon, offline.

Every dimension in this airframe traces back to one third-party motor STEP and zero caliper
measurements. Printing all sixteen parts before checking that is a bet. This coupon carries
the three interfaces that would each waste a whole part if they are wrong, in something that
prints in about fifteen minutes:

  motor bolt pattern   M3 clearance on 16 and 19 mm bolt circles, plus the 42.25 mm body
                       outline scribed on the face so the motor can be laid against it
  heat-set insert      4.0 mm pilot, 6.7 mm deep, in a boss with the real wall thickness
  bearing seat         683ZZ pocket at the press fit, with the lead-in chamfer, and a
                       second pocket 0.1 mm larger so both can be tried

Print it in the material the structural parts will use, at the same layer height and wall
count. A coupon printed in PLA at different settings does not answer the question.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq

import features as F

MOTOR_OD = 42.25
# Rectangular 16x19, four screws. See the note in gimbal.py.
MOTOR_HOLES = ((0.0, 8.0), (0.0, -8.0), (9.5, 0.0), (-9.5, 0.0))


def coupon(length, width, thickness, boss_h, scribe_depth):
    plate = (cq.Workplane("XY").box(length, width, thickness, centered=(True, True, False))
             .edges("|Z").fillet(4.0))

    # --- station 1: motor bolt pattern, at -x ---
    x1 = -length / 4.0
    holes = cq.Workplane("XY")
    for hx, hy in MOTOR_HOLES:
        holes = holes.moveTo(x1 + hx, hy).circle(F.M3_CLEARANCE / 2)
    plate = plate.cut(holes.extrude(thickness * 3, both=True))
    # Scribe the motor outline so the real can be set against it.
    scribe = (cq.Workplane("XY").workplane(offset=thickness - scribe_depth)
              .center(x1, 0).circle(MOTOR_OD / 2).circle(MOTOR_OD / 2 - 0.8)
              .extrude(scribe_depth + 0.1))
    plate = plate.cut(scribe)

    # --- station 2: heat-set insert boss, at centre ---
    boss = F.insert_boss(height=boss_h)
    plate = plate.union(boss.translate((0, 0, thickness - 0.01)))

    # --- station 3: two bearing seats, at +x ---
    for i, extra in enumerate((0.0, 0.10)):
        x = length / 4.0 + (i - 0.5) * 22.0
        pocket = F.bearing_pocket_cutter(F.BEARING_683,
                                         press=F.BEARING_PRESS + extra)
        plate = plate.cut(pocket.translate((x, 0, thickness)))
    return plate


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--length", type=float, default=110.0)
    p.add_argument("--width", type=float, default=34.0)
    p.add_argument("--thickness", type=float, default=6.0)
    p.add_argument("--boss-height", type=float, default=8.2)
    p.add_argument("--scribe-depth", type=float, default=0.6)
    p.add_argument("--output", type=Path, default=Path("../../artifacts/cad"))
    args = p.parse_args(argv)

    part = coupon(args.length, args.width, args.thickness, args.boss_height,
                  args.scribe_depth)
    args.output.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(part, str(args.output / "fit-coupon.step"))
    cq.exporters.export(part, str(args.output / "fit-coupon.stl"))
    vol = part.val().Volume()
    bb = part.val().BoundingBox()

    checks = [
        {"station": "motor bolt pattern",
         "what": "M3 clearance, rectangular 16 x 19 mm pattern, 42.25 mm body scribed",
         "pass": "The motor drops onto either bolt circle and its body sits inside the "
                 "scribed ring with an even gap all round.",
         "if_wrong": "Measure the real bolt circle and body diameter and set MOTOR_BCD and "
                     "MOTOR_OD in gimbal.py, then regenerate."},
        {"station": "heat-set insert",
         "what": f"{F.INSERT_M3['hole_dia']} mm pilot, {F.INSERT_M3['depth']} mm deep, "
                 f"{F.INSERT_M3['boss_dia']} mm boss",
         "pass": "The insert sinks flush with light pressure at soldering temperature and "
                 "an M3 screw pulls tight without spinning it.",
         "if_wrong": "Too loose means the insert spins: reduce hole_dia in features.py. "
                     "Too tight cracks the boss: raise it by 0.1 mm at a time."},
        {"station": "bearing seat",
         "what": f"683ZZ ({F.BEARING_683['od']} mm) in two pockets: "
                 f"{F.BEARING_683['od'] + F.BEARING_PRESS:.2f} and "
                 f"{F.BEARING_683['od'] + F.BEARING_PRESS + 0.10:.2f} mm",
         "pass": "One pocket takes the bearing with firm thumb pressure and holds it. "
                 "Note which one.",
         "if_wrong": "If the tight one cracks and the loose one falls out, split the "
                     "difference in BEARING_PRESS in features.py."},
    ]
    report = {"size_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
              "volume_cm3": vol / 1000.0,
              "mass_asa_g": round(vol / 1000.0 * 1.07, 1),
              "print_with": "The same material, layer height and wall count as the "
                            "structural parts. A coupon printed differently does not "
                            "answer the question.",
              "checks": checks, "offline_only": True}
    (args.output / "fit-coupon.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")

    print(f"Fit coupon  {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.1f} mm, "
          f"{vol / 1000:.1f} cm3, about {vol / 1000 * 1.07:.0f} g in ASA")
    for c in checks:
        print(f"  {c['station']:22s} {c['what']}")
    print(f"  wrote fit-coupon.step / .stl / .json to {args.output.resolve()}")
    print("Print this before the other sixteen parts. Three interfaces, one print.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
