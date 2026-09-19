import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from parameter_check import main as check_main
from parameter_profiles import (
    DEFAULT_PROFILE, audit_profile, input_check, load_profile, parameter,
    profile_record, require_inputs, validate_profile,
)

DRAFT_PATH = DEFAULT_PROFILE.with_name("my_airframe.json")


class ParameterProfileTests(unittest.TestCase):
    def setUp(self):
        self.reference = load_profile()
        self.draft = load_profile(DRAFT_PATH)

    def test_draft_does_not_inherit_reference_values(self):
        audit = audit_profile(self.draft)
        self.assertEqual(audit["status_counts"]["unknown"], 13)
        self.assertEqual(audit["status_counts"]["convention"], 1)
        self.assertIsNone(audit["nominal_hover_total_thrust_n"])
        with self.assertRaisesRegex(ValueError, "mass is unknown"):
            parameter(self.draft, "mass", "kg")

    def test_reference_availability_does_not_claim_flight_readiness(self):
        audit = audit_profile(self.reference)
        self.assertTrue(audit["input_checks"]["roll_study"]["inputs_available"])
        self.assertFalse(audit["input_checks"]["diagonal_inertia"]["inputs_available"])
        self.assertEqual(audit["status_counts"]["measured"], 0)
        self.assertAlmostEqual(audit["nominal_hover_total_thrust_n"], 4.2183)
        self.assertIsNone(audit["max_thrust_to_weight_ratio"])
        self.assertTrue(any("not aircraft calibration" in warning for warning in audit["warnings"]))

    def test_require_lists_missing_inputs_without_mutating_draft(self):
        original = copy.deepcopy(self.draft)
        with self.assertRaisesRegex(ValueError, "No upstream fallback"):
            require_inputs(self.draft, "roll_study")
        self.assertEqual(self.draft, original)
        self.assertIn("inertia_xx", input_check(self.draft, "roll_study")["missing_parameters"])

    def test_reject_zero_or_negative_positive_parameters(self):
        for name in ("mass", "tvc_arm", "inertia_zz", "max_total_thrust", "tvc_angle_limit",
                     "servo_rate_limit", "servo_design_time_constant", "actuator_study_sample_period"):
            for value in (0, -1):
                with self.subTest(name=name, value=value):
                    profile = copy.deepcopy(self.reference)
                    profile["parameters"][name].update(value=value, status="assumed")
                    with self.assertRaisesRegex(ValueError, "positive"):
                        validate_profile(profile)

    def test_reject_nonfinite_bool_and_string_values(self):
        for value in (float("nan"), float("inf"), -float("inf"), True, "0.430"):
            with self.subTest(value=value):
                profile = copy.deepcopy(self.reference)
                profile["parameters"]["mass"]["value"] = value
                with self.assertRaisesRegex(ValueError, "finite and numeric"):
                    validate_profile(profile)

    def test_reject_wrong_units(self):
        for name, unit in (("mass", "g"), ("tvc_angle_limit", "deg"), ("max_total_thrust", "kgf")):
            with self.subTest(name=name):
                profile = copy.deepcopy(self.reference)
                profile["parameters"][name]["unit"] = unit
                with self.assertRaisesRegex(ValueError, "expected"):
                    validate_profile(profile)

    def test_unknown_requires_null_and_known_requires_value(self):
        for value, status in ((None, "measured"), (1, "unknown")):
            profile = copy.deepcopy(self.reference)
            profile["parameters"]["mass"].update(value=value, status=status)
            with self.assertRaisesRegex(ValueError, "null and status=unknown"):
                validate_profile(profile)

    def test_source_and_status_required(self):
        for key, value in (("source", " "), ("status", "guessed"), ("status", {})):
            profile = copy.deepcopy(self.reference)
            profile["parameters"]["mass"][key] = value
            with self.assertRaises(ValueError):
                validate_profile(profile)

    def test_measured_and_cad_are_provenance_labels_not_certification(self):
        for status in ("measured", "cad"):
            profile = copy.deepcopy(self.draft)
            profile["parameters"]["mass"].update(value=.5, status=status,
                                                 source="Synthetic test fixture, not aircraft data")
            audit = audit_profile(profile)
            self.assertEqual(audit["status_counts"][status], 1)
            self.assertFalse(audit["input_checks"]["roll_study"]["inputs_available"])

    def test_reject_misspelled_or_missing_parameter_names(self):
        for name in ("mass", "max_total_thrust"):
            profile = copy.deepcopy(self.reference)
            profile["parameters"][name + "_typo"] = profile["parameters"].pop(name)
            with self.assertRaisesRegex(ValueError, "Parameter names"):
                validate_profile(profile)

    def test_reject_wrong_schema_and_frames(self):
        for value in (True, 1.0, 2):
            profile = copy.deepcopy(self.reference)
            profile["schema_version"] = value
            with self.assertRaisesRegex(ValueError, "schema"):
                validate_profile(profile)
        profile = copy.deepcopy(self.reference)
        profile["frames"]["body"] = "FLU"
        with self.assertRaisesRegex(ValueError, "frame"):
            validate_profile(profile)

    def test_principal_inertia_triangle(self):
        for values, valid in (((.01, .02, .025), True), ((.01, .02, .04), False)):
            profile = copy.deepcopy(self.reference)
            for axis, value in zip("xyz", values):
                profile["parameters"][f"inertia_{axis}{axis}"].update(value=value, status="assumed")
            if valid:
                require_inputs(profile, "diagonal_inertia")
            else:
                with self.assertRaisesRegex(ValueError, "triangle inequality"):
                    validate_profile(profile)

    def test_angle_limit_catches_degrees_entered_as_radians(self):
        self.reference["parameters"]["tvc_angle_limit"]["value"] = 15
        with self.assertRaisesRegex(ValueError, "pi/2"):
            validate_profile(self.reference)

    def test_lqi_cost_validation(self):
        for key, values in (("q_diagonal", [1] * 11), ("q_diagonal", [-1] * 12),
                            ("r_legacy_diagonal", [0] * 4), ("r_legacy_diagonal", [True] * 4)):
            profile = copy.deepcopy(self.reference)
            profile["upstream_lqi_cost"][key] = values
            with self.assertRaises(ValueError):
                validate_profile(profile)

    def test_custom_profile_cannot_silently_reuse_reference_lqi(self):
        self.reference["profile"] = "different_airframe"
        check = input_check(self.reference, "reference_lqi")
        self.assertFalse(check["inputs_available"])
        self.assertTrue(check["restrictions"])

    def test_insufficient_thrust_warns_without_inventing_values(self):
        self.reference["parameters"]["max_total_thrust"].update(value=3., status="assumed")
        audit = audit_profile(self.reference)
        self.assertLess(audit["max_thrust_to_weight_ratio"], 1.)
        self.assertTrue(any("no positive static hover" in warning for warning in audit["warnings"]))

    def test_snapshot_digest_and_utf8_bom(self):
        raw = b"\xef\xbb\xbf" + json.dumps(self.reference).encode("utf-8")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "profile.json"
            path.write_bytes(raw)
            record = profile_record(path)
        self.assertEqual(record["profile"], self.reference)
        self.assertEqual(record["profile_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertTrue(record["offline_only"])

    def test_duplicate_keys_and_nonfinite_json_rejected(self):
        for text in ('{"schema_version": 1, "schema_version": 1}', '{"value": NaN}', '{"value": Infinity}'):
            with self.subTest(text=text), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "invalid.json"
                path.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_cli_audit_accepts_draft_but_requirement_blocks(self):
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(check_main(["--profile", str(DRAFT_PATH), "--output", folder]), 0)
            report = json.loads((Path(folder) / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(report["profile"], self.draft)
            self.assertTrue((Path(folder) / "summary.md").is_file())
            self.assertEqual(check_main(["--profile", str(DRAFT_PATH), "--require", "roll_study"]), 2)


if __name__ == "__main__":
    unittest.main()
