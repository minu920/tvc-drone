"""Screen a coaxial propulsion candidate against published propeller limits, offline.

No hardware is connected and no measured aircraft data is used. Thrust and shaft power
come from the manufacturer's published static tables at zero airspeed, so every derived
number inherits those conditions. Passing this check means the candidate is not already
excluded on paper. It is not a flight-readiness judgement, and it does not replace the
protected coaxial thrust stand.

Electrical current and battery endurance are deliberately not estimated: both need a
measured motor efficiency curve, which this repository does not have.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

LBF_TO_N = 4.4482216152605
HP_TO_W = 745.6998715822702
STANDARD_GRAVITY = 9.80665

DEFAULT_CANDIDATE = Path(__file__).resolve().parent / "parameters" / "propulsion_candidate.json"

# APC publishes a suggested maximum RPM per propeller series as a constant divided by
# the diameter in inches. https://www.apcprop.com/technical-information/rpm-limits/
SERIES_RPM_CONSTANT = {"SF": 65000.0, "MRF": 105000.0, "MR": 105000.0,
                       "F": 120000.0, "E": 150000.0}

# Static thrust and shaft power from the APC PER3 performance files, zero-airspeed rows.
# Retrieved 2026-09-20 from https://www.apcprop.com/files/PER3_<designation>.dat
PROPELLER_DATA = {
    "12x3.8SF": {"series": "SF", "diameter_in": 12.0, "file_version": "v2022-0915",
                 "rows": [(4000, 1.064, 0.045), (5000, 1.667, 0.087), (6000, 2.409, 0.150),
                          (7000, 3.292, 0.238), (8000, 4.321, 0.357)]},
    "12x4.5MR": {"series": "MR", "diameter_in": 12.0, "file_version": "v2022-0915",
                 "rows": [(5000, 1.492, 0.076), (6000, 2.156, 0.130), (7000, 2.946, 0.205),
                          (8000, 3.865, 0.304), (9000, 4.918, 0.433)]},
}

REQUIRED = ("battery_cells", "cell_voltage_nominal", "cell_voltage_min_operating", "motor_kv",
            "motor_count", "propeller", "loaded_rpm_fraction", "coaxial_thrust_loss",
            "target_mass", "thrust_to_weight_goal", "tvc_tilt_limit")
VALID_STATUS = ("selected", "convention", "assumed", "manufacturer", "measured", "unknown")


def candidate_record(path=DEFAULT_CANDIDATE):
    """Load and validate a candidate file, refusing to substitute anything for a missing value."""
    raw = Path(path).read_bytes()
    candidate = json.loads(raw.decode("utf-8"))
    if candidate.get("schema_version") != 1:
        raise ValueError("Unsupported schema_version; expected 1.")
    entries = candidate.get("propulsion")
    if not isinstance(entries, dict):
        raise ValueError("Missing 'propulsion' section.")
    for name in REQUIRED:
        entry = entries.get(name)
        if not isinstance(entry, dict) or not {"value", "unit", "status", "source"} <= set(entry):
            raise ValueError(f"'{name}' must define value, unit, status and source.")
        if entry["status"] not in VALID_STATUS:
            raise ValueError(f"'{name}' has unknown status '{entry['status']}'.")
        if (entry["value"] is None) != (entry["status"] == "unknown"):
            raise ValueError(f"'{name}' must be null exactly when its status is 'unknown'.")
        if entry["value"] is None:
            raise ValueError(f"'{name}' is unknown; this check does not invent inputs.")
    designation = entries["propeller"]["value"]
    if designation not in PROPELLER_DATA:
        known = ", ".join(sorted(PROPELLER_DATA))
        raise ValueError(f"No published data embedded for '{designation}'. Known: {known}.")
    return {"candidate_path": str(Path(path).resolve()), "candidate": candidate,
            "candidate_sha256": hashlib.sha256(raw).hexdigest(), "offline_only": True}


def _interpolate(rows, column, rpm):
    """Interpolate within the published range only; extrapolation is refused."""
    speeds = [row[0] for row in rows]
    if not speeds[0] <= rpm <= speeds[-1]:
        raise ValueError(f"{rpm:.0f} RPM is outside the published range "
                         f"{speeds[0]:.0f}-{speeds[-1]:.0f}; this check does not extrapolate.")
    for (lo_rpm, *lo), (hi_rpm, *hi) in zip(rows, rows[1:]):
        if lo_rpm <= rpm <= hi_rpm:
            span = (rpm - lo_rpm) / (hi_rpm - lo_rpm)
            return lo[column] + span * (hi[column] - lo[column])
    raise ValueError(f"Could not bracket {rpm:.0f} RPM.")


def thrust_newton(designation, rpm):
    return _interpolate(PROPELLER_DATA[designation]["rows"], 0, rpm) * LBF_TO_N


def shaft_power_watt(designation, rpm):
    return _interpolate(PROPELLER_DATA[designation]["rows"], 1, rpm) * HP_TO_W


def rpm_limit(designation):
    data = PROPELLER_DATA[designation]
    return SERIES_RPM_CONSTANT[data["series"]] / data["diameter_in"]


def _operating_point(entries, designation, pack_voltage, label):
    """Evaluate one battery voltage. Returns None when the speed leaves the published range."""
    no_load_rpm = entries["motor_kv"]["value"] * pack_voltage
    loaded_rpm = no_load_rpm * entries["loaded_rpm_fraction"]["value"]
    limit = rpm_limit(designation)
    point = {"label": label, "pack_voltage_v": pack_voltage, "no_load_rpm": no_load_rpm,
             "loaded_rpm": loaded_rpm, "propeller_rpm_limit": limit,
             "within_propeller_limit": loaded_rpm <= limit,
             "propeller_limit_usage": loaded_rpm / limit}
    try:
        per_motor = thrust_newton(designation, loaded_rpm)
        point["shaft_power_per_motor_w"] = shaft_power_watt(designation, loaded_rpm)
    except ValueError as exc:
        return {**point, "thrust_available": False, "reason": str(exc)}

    count = entries["motor_count"]["value"]
    isolated = per_motor * count
    coaxial = isolated * (1.0 - entries["coaxial_thrust_loss"]["value"])
    weight = entries["target_mass"]["value"] * STANDARD_GRAVITY
    tilt = entries["tvc_tilt_limit"]["value"]
    required = entries["thrust_to_weight_goal"]["value"] * weight / math.cos(tilt)
    return {**point, "thrust_available": True,
            "thrust_per_motor_isolated_n": per_motor,
            "thrust_pair_isolated_n": isolated,
            "thrust_coaxial_n": coaxial,
            "target_weight_n": weight,
            "required_thrust_n": required,
            "meets_goal": coaxial >= required,
            "thrust_to_weight": coaxial / weight,
            "hover_thrust_fraction": weight / coaxial,
            "mass_ceiling_at_goal_kg": coaxial * math.cos(tilt)
            / (entries["thrust_to_weight_goal"]["value"] * STANDARD_GRAVITY)}


def compare_propellers(entries, loaded_rpm):
    """Record why other embedded propellers were or were not viable at the same speed."""
    rows = []
    for designation, data in sorted(PROPELLER_DATA.items()):
        limit = SERIES_RPM_CONSTANT[data["series"]] / data["diameter_in"]
        row = {"propeller": designation, "series": data["series"], "rpm_limit": limit,
               "limit_usage": loaded_rpm / limit, "within_limit": loaded_rpm <= limit}
        try:
            row["thrust_per_motor_n"] = thrust_newton(designation, loaded_rpm)
        except ValueError as exc:
            row["thrust_per_motor_n"] = None
            row["note"] = str(exc)
        rows.append(row)
    return rows


def loss_sensitivity(entries, designation, loaded_rpm, losses=(0.10, 0.15, 0.20, 0.25)):
    """The coaxial loss is assumed, so show what the conclusion does across the plausible band."""
    weight = entries["target_mass"]["value"] * STANDARD_GRAVITY
    tilt = entries["tvc_tilt_limit"]["value"]
    goal = entries["thrust_to_weight_goal"]["value"]
    try:
        isolated = thrust_newton(designation, loaded_rpm) * entries["motor_count"]["value"]
    except ValueError:
        return []
    rows = []
    for loss in losses:
        coaxial = isolated * (1.0 - loss)
        rows.append({"assumed_loss": loss, "thrust_coaxial_n": coaxial,
                     "thrust_to_weight": coaxial / weight,
                     "mass_ceiling_at_goal_kg": coaxial * math.cos(tilt) / (goal * STANDARD_GRAVITY)})
    return rows


def analyse(record):
    entries = record["candidate"]["propulsion"]
    designation = entries["propeller"]["value"]
    cells = entries["battery_cells"]["value"]
    points = [_operating_point(entries, designation, cells * entries["cell_voltage_nominal"]["value"],
                               "nominal cell voltage"),
              _operating_point(entries, designation, cells * entries["cell_voltage_min_operating"]["value"],
                               "minimum operating cell voltage")]
    worst = points[-1]
    return {"propeller": designation,
            "propeller_source": {**PROPELLER_DATA[designation],
                                 "url": f"https://www.apcprop.com/files/PER3_{designation.replace('.', '')}.dat",
                                 "rpm_limit_url": "https://www.apcprop.com/technical-information/rpm-limits/",
                                 "retrieved": "2026-09-20"},
            "operating_points": points,
            "propeller_comparison": compare_propellers(entries, worst["loaded_rpm"]),
            "coaxial_loss_sensitivity": loss_sensitivity(entries, designation, worst["loaded_rpm"]),
            "verdict": {
                "within_propeller_limit": all(p["within_propeller_limit"] for p in points),
                "meets_goal_at_nominal": bool(points[0].get("meets_goal")),
                "meets_goal_at_minimum": bool(points[1].get("meets_goal"))}}


def _fmt(value, digits=2):
    return "—" if value is None else f"{value:,.{digits}f}"


def markdown_report(report):
    analysis, entries = report["analysis"], report["candidate"]["propulsion"]
    verdict = analysis["verdict"]
    lines = ["# 동축 추진계 후보 점검", "",
             f"후보: `{report['candidate']['candidate']}`  ·  프로펠러: `{analysis['propeller']}`", "",
             "제조사 공표 정지추력표에 의한 서류 판정이다. 실측이 아니며 비행 가능 판정도 아니다.", "",
             f"후보 파일 SHA-256: `{report['candidate_sha256']}`", "",
             "## 판정", "",
             f"- 프로펠러 RPM 상한 준수: {'예' if verdict['within_propeller_limit'] else '아니오'}",
             f"- 공칭 전압에서 추력 목표 달성: {'예' if verdict['meets_goal_at_nominal'] else '아니오'}",
             f"- 최저 운용 전압에서 추력 목표 달성: {'예' if verdict['meets_goal_at_minimum'] else '아니오'}",
             "", "## 운용점", "",
             "| 조건 | 팩 전압 V | 부하 RPM | 상한 대비 | 단독 추력/모터 N | 동축 총추력 N | 필요 추력 N | T/W | 호버 추력비 | 목표 T/W에서의 질량 상한 kg |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for point in analysis["operating_points"]:
        if not point.get("thrust_available"):
            lines.append(f"| {point['label']} | {_fmt(point['pack_voltage_v'])} | {_fmt(point['loaded_rpm'], 0)} | "
                         f"{point['propeller_limit_usage'] * 100:.0f}% | 범위 밖 | — | — | — | — | — |")
            continue
        lines.append(
            f"| {point['label']} | {_fmt(point['pack_voltage_v'])} | {_fmt(point['loaded_rpm'], 0)} | "
            f"{point['propeller_limit_usage'] * 100:.0f}% | {_fmt(point['thrust_per_motor_isolated_n'])} | "
            f"{_fmt(point['thrust_coaxial_n'])} | {_fmt(point['required_thrust_n'])} | "
            f"{_fmt(point['thrust_to_weight'])} | {point['hover_thrust_fraction'] * 100:.0f}% | "
            f"{_fmt(point['mass_ceiling_at_goal_kg'])} |")

    lines.extend(["", "## 같은 속도에서의 프로펠러 계열 비교", "",
                  "최저 운용 전압의 부하 RPM 기준이다. 기각 이유를 기록으로 남기기 위한 표다.", "",
                  "| 프로펠러 | 계열 | RPM 상한 | 상한 대비 | 추력/모터 N |", "|---|---|---:|---:|---:|"])
    for row in analysis["propeller_comparison"]:
        mark = "" if row["within_limit"] else " ⚠"
        lines.append(f"| {row['propeller']} | {row['series']} | {_fmt(row['rpm_limit'], 0)} | "
                     f"{row['limit_usage'] * 100:.0f}%{mark} | {_fmt(row['thrust_per_motor_n'])} |")

    lines.extend(["", "## 동축 손실 가정에 대한 민감도", "",
                  f"현재 가정은 {entries['coaxial_thrust_loss']['value'] * 100:.0f}%이며 실측값이 아니다. "
                  "결론이 가정에 얼마나 의존하는지 확인한다.", "",
                  "| 가정 손실 | 동축 총추력 N | T/W | 목표 T/W에서의 질량 상한 kg |", "|---:|---:|---:|---:|"])
    for row in analysis["coaxial_loss_sensitivity"]:
        lines.append(f"| {row['assumed_loss'] * 100:.0f}% | {_fmt(row['thrust_coaxial_n'])} | "
                     f"{_fmt(row['thrust_to_weight'])} | {_fmt(row['mass_ceiling_at_goal_kg'])} |")

    assumed = [name for name in REQUIRED if entries[name]["status"] == "assumed"]
    lines.extend(["", "## 실측으로 교체해야 할 가정", ""]
                 + [f"- `{name}`: {entries[name]['source']}" for name in assumed]
                 + ["", "전류·체공시간은 계산하지 않는다. 측정된 모터 효율 곡선이 필요하며 이 저장소에는 없다.", ""])
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    parser.add_argument("--output", type=Path, help="Optional report directory; same-named reports are replaced")
    parser.add_argument("--strict", action="store_true",
                        help="Exit 2 unless the goal is met at the minimum operating voltage")
    args = parser.parse_args(argv)
    try:
        record = candidate_record(args.candidate)
        report = {**record, "analysis": analyse(record)}
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))

    analysis = report["analysis"]
    verdict = analysis["verdict"]
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        (args.output / "summary.md").write_text(markdown_report(report), encoding="utf-8")

    print(f"Candidate: {report['candidate']['candidate']}  propeller: {analysis['propeller']}")
    for point in analysis["operating_points"]:
        head = f"  {point['label']}: {point['loaded_rpm']:,.0f} RPM " \
               f"({point['propeller_limit_usage'] * 100:.0f}% of limit)"
        if not point.get("thrust_available"):
            print(f"{head} — {point['reason']}")
            continue
        print(f"{head}, coaxial {point['thrust_coaxial_n']:.1f} N vs required "
              f"{point['required_thrust_n']:.1f} N, T/W {point['thrust_to_weight']:.2f}, "
              f"mass ceiling {point['mass_ceiling_at_goal_kg']:.2f} kg")
    print(f"  Within propeller RPM limit: {verdict['within_propeller_limit']}")
    print(f"  Meets goal at nominal / minimum voltage: "
          f"{verdict['meets_goal_at_nominal']} / {verdict['meets_goal_at_minimum']}")
    if args.output is not None:
        print(f"Report: {args.output.resolve() / 'summary.md'}")
    print("Manufacturer static data only. No measurement, no current estimate, no flight judgement.")
    if not verdict["within_propeller_limit"]:
        return 2
    return 2 if args.strict and not verdict["meets_goal_at_minimum"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
