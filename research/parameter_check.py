"""Audit a parameter profile without filling unknown values or connecting hardware."""
import argparse
import json
from pathlib import Path

from parameter_profiles import DEFAULT_PROFILE, REQUIREMENTS, audit_profile, profile_record


def markdown_report(report):
    profile, audit = report["profile"], report["audit"]
    lines = ["# TVC 파라미터 점검", "", f"프로파일: `{profile['profile']}`", "",
             "오프라인 입력 점검이며 실기 보정·비행 가능 판정이 아니다.", "",
             f"원본 파일 SHA-256: `{report['profile_sha256']}`", "",
             "| 항목 | 값 | 단위 | 상태 |", "|---|---:|---|---|"]
    for name, entry in profile["parameters"].items():
        value = "미정" if entry["value"] is None else str(entry["value"])
        lines.append(f"| {name} | {value} | {entry['unit']} | {entry['status']} |")
    lines.extend(["", "## 계산별 입력 점검", ""])
    for name, check in audit["input_checks"].items():
        details = ", ".join(check["missing_parameters"] + check["restrictions"])
        lines.append(f"- `{name}`: " + ("입력값 있음 — 실측 여부와 타당성은 별도 검토" if check["inputs_available"] else f"입력 부족/제한: {details}"))
    lines.extend(["", "## 주의", ""] + [f"- {warning}" for warning in audit["warnings"]])
    lines.extend(["", "출처·측정 계획과 실행 당시 전체 값은 `summary.json`의 `profile`에 보존된다.", ""])
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--require", choices=tuple(REQUIREMENTS), help="Exit 2 when this calculation lacks inputs")
    parser.add_argument("--output", type=Path, help="Optional report directory; same-named reports are replaced")
    args = parser.parse_args(argv)
    try:
        record = profile_record(args.profile)
        report = {**record, "audit": audit_profile(record["profile"])}
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    audit = report["audit"]
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        (args.output / "summary.md").write_text(markdown_report(report), encoding="utf-8")
    print(f"Profile: {record['profile']['profile']}")
    print(f"Unknown: {len(audit['unknown_parameters'])}/{audit['parameter_count']}")
    for name, check in audit["input_checks"].items():
        label = "INPUTS AVAILABLE (offline only)" if check["inputs_available"] else "BLOCKED"
        print(f"  {name}: {label}")
        for detail in check["missing_parameters"] + check["restrictions"]:
            print(f"    - {detail}")
    if args.output is not None:
        print(f"Report: {args.output.resolve() / 'summary.md'}")
    print("No fallback values, firmware changes or hardware commands.")
    return 2 if args.require and not audit["input_checks"][args.require]["inputs_available"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
