"""SI parameter profiles and provenance checks for OFFLINE research only.

Drafts may contain unknown values. A consumer must request the inputs it needs;
missing values are never filled from the upstream profile. No check certifies flight.
This module deliberately uses only the Python standard library.
"""
import hashlib
import json
import math
from pathlib import Path

DEFAULT_PROFILE = Path(__file__).resolve().parent / "parameters/upstream_reference.json"
UPSTREAM_PROFILE_ID = "upstream_reference_not_flight_calibrated"
FRAMES = {
    "world": "NED", "body": "FRD", "rotation": "R_ned_from_frd",
    "tvc_order": "Rx(alpha) @ Ry(beta) @ [0, 0, -1]",
}
# Canonical units, not conversion hints. Convert measurements before entering them.
UNITS = {
    "mass": "kg", "gravity": "m/s^2", "tvc_arm": "m",
    "inertia_xx": "kg*m^2", "inertia_yy": "kg*m^2", "inertia_zz": "kg*m^2",
    "yaw_accel_per_legacy_mass_equivalent": "rad/s^2/kg_equiv",
    "yaw_torque_per_differential_thrust": "m", "max_total_thrust": "N",
    "tvc_angle_limit": "rad", "servo_rate_limit": "rad/s",
    "servo_design_time_constant": "s", "lqi_sample_period": "s",
    "actuator_study_sample_period": "s",
}
STATUSES = {"upstream_reference", "convention", "assumed", "cad", "measured", "unknown"}
SIGNED_PARAMETERS = {"yaw_accel_per_legacy_mass_equivalent", "yaw_torque_per_differential_thrust"}
REQUIREMENTS = {
    "translation": ("mass", "gravity"),
    "roll_study": ("mass", "gravity", "tvc_arm", "inertia_xx", "tvc_angle_limit",
                   "servo_rate_limit", "servo_design_time_constant", "actuator_study_sample_period"),
    "diagonal_inertia": ("inertia_xx", "inertia_yy", "inertia_zz"),
    "thrust_margin": ("mass", "gravity", "max_total_thrust"),
    "reference_lqi": ("mass", "gravity", "tvc_arm", "inertia_xx", "inertia_yy",
                      "yaw_accel_per_legacy_mass_equivalent", "lqi_sample_period"),
}


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite and numeric")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite and numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite and numeric")
    return result


def _entry(profile, name):
    if name not in UNITS:
        raise ValueError(f"Unrecognized parameter: {name}")
    entry = profile.get("parameters", {}).get(name)
    if not isinstance(entry, dict):
        raise ValueError(f"Missing parameter record: {name}")
    if not {"value", "unit", "status", "source"}.issubset(entry):
        raise ValueError(f"{name}: value, unit, status and source are required")
    if entry["unit"] != UNITS[name]:
        raise ValueError(f"{name}: expected {UNITS[name]}, got {entry['unit']}")
    if not isinstance(entry["status"], str) or entry["status"] not in STATUSES:
        raise ValueError(f"{name}: unsupported status")
    if not isinstance(entry["source"], str) or not entry["source"].strip():
        raise ValueError(f"{name}: a nonempty source or measurement plan is required")
    if (entry["value"] is None) != (entry["status"] == "unknown"):
        raise ValueError(f"{name}: null and status=unknown must be used together")
    if entry["value"] is not None:
        value = _number(entry["value"], name)
        if name not in SIGNED_PARAMETERS and value <= 0:
            raise ValueError(f"{name} must be positive")
        if name == "tvc_angle_limit" and value >= math.pi / 2:
            raise ValueError("tvc_angle_limit must be below pi/2 for this upright hover model")
    return entry


def parameter(profile, name, unit):
    entry = _entry(profile, name)
    if unit != entry["unit"]:
        raise ValueError(f"{name}: expected {unit}, got {entry['unit']}")
    if entry["value"] is None:
        raise ValueError(f"{name} is unknown; supply an identified value explicitly")
    return float(entry["value"])


def validate_profile(profile):
    if not isinstance(profile, dict):
        raise ValueError("Profile must be a JSON object")
    if type(profile.get("schema_version")) is not int or profile["schema_version"] != 1:
        raise ValueError("Unsupported parameter schema")
    if not isinstance(profile.get("profile"), str) or not profile["profile"].strip():
        raise ValueError("A nonempty profile identifier is required")
    if profile.get("frames") != FRAMES:
        raise ValueError("Unsupported frame convention")
    entries = profile.get("parameters")
    if not isinstance(entries, dict):
        raise ValueError("parameters must be a JSON object")
    if set(entries) != set(UNITS):
        missing, extra = sorted(set(UNITS) - set(entries)), sorted(set(entries) - set(UNITS))
        raise ValueError(f"Parameter names do not match schema; missing={missing}, extra={extra}")
    for name in UNITS:
        _entry(profile, name)
    inertia = [entries[f"inertia_{axis}{axis}"]["value"] for axis in "xyz"]
    if all(value is not None for value in inertia):
        # Necessary physical condition for principal moments, not full CAD validation.
        largest = max(inertia)
        if 2 * largest > sum(inertia) + largest * 1e-9:
            raise ValueError("Principal inertia values violate the triangle inequality")
    if "upstream_lqi_cost" in profile:
        cost = profile["upstream_lqi_cost"]
        if not isinstance(cost, dict) or not isinstance(cost.get("source"), str) or not cost["source"].strip():
            raise ValueError("upstream_lqi_cost requires a source")
        for key, size, positive in (("q_diagonal", 12, False), ("r_legacy_diagonal", 4, True)):
            values = cost.get(key)
            if not isinstance(values, list) or len(values) != size:
                raise ValueError(f"{key} must contain {size} values")
            numbers = [_number(value, key) for value in values]
            if any(value < 0 or (positive and value == 0) for value in numbers):
                raise ValueError(f"{key}: invalid cost weights")
    return profile


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"Non-finite JSON value: {value}")


def profile_record(path=DEFAULT_PROFILE):
    """Read once: snapshot and digest always describe the exact same input bytes."""
    source = Path(path).resolve()
    raw = source.read_bytes()
    profile = validate_profile(json.loads(raw.decode("utf-8-sig"),
                                         object_pairs_hook=_unique_object,
                                         parse_constant=_reject_constant))
    return {"profile": profile, "profile_path": str(source),
            "profile_sha256": hashlib.sha256(raw).hexdigest(), "offline_only": True}


def load_profile(path=DEFAULT_PROFILE):
    return profile_record(path)["profile"]


def input_check(profile, capability):
    if capability not in REQUIREMENTS:
        raise ValueError(f"Unknown capability: {capability}")
    validate_profile(profile)
    missing = [name for name in REQUIREMENTS[capability]
               if profile["parameters"][name]["value"] is None]
    restrictions = []
    if capability == "reference_lqi":
        if profile["profile"] != UPSTREAM_PROFILE_ID:
            restrictions.append("reference_lqi only accepts the upstream-reference profile")
        if "upstream_lqi_cost" not in profile:
            restrictions.append("upstream_lqi_cost is missing")
    return {"inputs_available": not missing and not restrictions,
            "missing_parameters": missing, "restrictions": restrictions}


def require_inputs(profile, capability):
    check = input_check(profile, capability)
    if not check["inputs_available"]:
        details = ", ".join(check["missing_parameters"] + check["restrictions"])
        raise ValueError(f"{capability} blocked by unknown inputs or restrictions: {details}. No upstream fallback is applied.")


def audit_profile(profile):
    validate_profile(profile)
    entries = profile["parameters"]
    checks = {name: input_check(profile, name) for name in REQUIREMENTS}
    warnings = ["Offline input checks only; not aircraft calibration, controller stability or flight authorization."]
    if any(item["status"] in {"upstream_reference", "assumed"} for item in entries.values()):
        warnings.append("Contains upstream reference or assumed values, not measurements of this aircraft.")
    hover = None
    ratio = None
    if checks["translation"]["inputs_available"]:
        hover = parameter(profile, "mass", "kg") * parameter(profile, "gravity", "m/s^2")
    if checks["thrust_margin"]["inputs_available"]:
        ratio = parameter(profile, "max_total_thrust", "N") / hover
        if ratio <= 1:
            warnings.append("Maximum total thrust does not exceed weight; no positive static hover thrust margin.")
    return {"parameter_count": len(entries),
            "status_counts": {status: sum(item["status"] == status for item in entries.values())
                              for status in sorted(STATUSES)},
            "unknown_parameters": [name for name, item in entries.items() if item["value"] is None],
            "input_checks": checks, "nominal_hover_total_thrust_n": hover,
            "max_thrust_to_weight_ratio": ratio, "warnings": warnings}
