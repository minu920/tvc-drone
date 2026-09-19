"""Offline checks for the propulsion screening tool. No hardware and no measured data."""
import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

import propulsion_check as pc


def _candidate(**overrides):
    record = pc.candidate_record()
    entries = record["candidate"]["propulsion"]
    for name, value in overrides.items():
        entries[name]["value"] = value
    return record


class PublishedDataTest(unittest.TestCase):
    def test_table_points_reproduce_exactly(self):
        for designation, data in pc.PROPELLER_DATA.items():
            for rpm, thrust_lbf, power_hp in data["rows"]:
                self.assertAlmostEqual(pc.thrust_newton(designation, rpm),
                                       thrust_lbf * pc.LBF_TO_N, places=9,
                                       msg=f"{designation} at {rpm} RPM")
                self.assertAlmostEqual(pc.shaft_power_watt(designation, rpm),
                                       power_hp * pc.HP_TO_W, places=9)

    def test_thrust_increases_with_speed(self):
        for designation, data in pc.PROPELLER_DATA.items():
            speeds = [row[0] for row in data["rows"]]
            values = [pc.thrust_newton(designation, rpm) for rpm in speeds]
            self.assertEqual(values, sorted(values), msg=designation)

    def test_extrapolation_is_refused(self):
        for rpm in (4999, 9001):
            with self.assertRaises(ValueError):
                pc.thrust_newton("12x4.5MR", rpm)

    def test_rpm_limits_follow_the_published_divisor(self):
        self.assertAlmostEqual(pc.rpm_limit("12x3.8SF"), 65000.0 / 12.0)
        self.assertAlmostEqual(pc.rpm_limit("12x4.5MR"), 105000.0 / 12.0)
        self.assertLess(pc.rpm_limit("12x3.8SF"), pc.rpm_limit("12x4.5MR"))


class CandidateValidationTest(unittest.TestCase):
    def test_default_candidate_loads_and_has_no_unknowns(self):
        record = pc.candidate_record()
        self.assertEqual(len(record["candidate_sha256"]), 64)
        for name in pc.REQUIRED:
            self.assertIsNotNone(record["candidate"]["propulsion"][name]["value"], name)

    def _write_and_expect_error(self, candidate, fragment):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "candidate.json"
            path.write_text(json.dumps(candidate), encoding="utf-8")
            with self.assertRaises(ValueError) as caught:
                pc.candidate_record(path)
        self.assertIn(fragment, str(caught.exception))

    def test_unknown_value_is_refused_rather_than_defaulted(self):
        candidate = json.loads(pc.DEFAULT_CANDIDATE.read_text(encoding="utf-8"))
        candidate["propulsion"]["target_mass"] = {"value": None, "unit": "kg",
                                                  "status": "unknown", "source": "Not weighed."}
        self._write_and_expect_error(candidate, "does not invent inputs")

    def test_null_value_must_match_unknown_status(self):
        candidate = json.loads(pc.DEFAULT_CANDIDATE.read_text(encoding="utf-8"))
        candidate["propulsion"]["target_mass"]["value"] = None
        self._write_and_expect_error(candidate, "null exactly when")

    def test_propeller_without_published_data_is_refused(self):
        candidate = json.loads(pc.DEFAULT_CANDIDATE.read_text(encoding="utf-8"))
        candidate["propulsion"]["propeller"]["value"] = "12x6E"
        self._write_and_expect_error(candidate, "No published data embedded")


class AnalysisTest(unittest.TestCase):
    def test_slow_flyer_would_exceed_its_limit_at_this_speed(self):
        """The reason the candidate moved from 12x3.8SF to 12x4.5MR; keep it as a regression."""
        analysis = pc.analyse(pc.candidate_record())
        by_name = {row["propeller"]: row for row in analysis["propeller_comparison"]}
        self.assertFalse(by_name["12x3.8SF"]["within_limit"])
        self.assertGreater(by_name["12x3.8SF"]["limit_usage"], 1.0)
        self.assertTrue(by_name["12x4.5MR"]["within_limit"])

    def test_minimum_voltage_is_never_the_optimistic_case(self):
        points = pc.analyse(pc.candidate_record())["operating_points"]
        nominal, minimum = points[0], points[1]
        self.assertLess(minimum["loaded_rpm"], nominal["loaded_rpm"])
        self.assertLess(minimum["thrust_coaxial_n"], nominal["thrust_coaxial_n"])
        self.assertLess(minimum["mass_ceiling_at_goal_kg"], nominal["mass_ceiling_at_goal_kg"])

    def test_required_thrust_includes_the_tilt_allowance(self):
        record = pc.candidate_record()
        entries = record["candidate"]["propulsion"]
        point = pc.analyse(record)["operating_points"][0]
        expected = (entries["thrust_to_weight_goal"]["value"] * point["target_weight_n"]
                    / math.cos(entries["tvc_tilt_limit"]["value"]))
        self.assertAlmostEqual(point["required_thrust_n"], expected, places=9)
        self.assertGreater(point["required_thrust_n"],
                           entries["thrust_to_weight_goal"]["value"] * point["target_weight_n"])

    def test_coaxial_loss_reduces_thrust_below_the_isolated_pair(self):
        point = pc.analyse(pc.candidate_record())["operating_points"][0]
        self.assertLess(point["thrust_coaxial_n"], point["thrust_pair_isolated_n"])
        self.assertAlmostEqual(point["thrust_pair_isolated_n"],
                               2 * point["thrust_per_motor_isolated_n"], places=9)

    def test_loss_sensitivity_is_monotonic(self):
        rows = pc.analyse(pc.candidate_record())["coaxial_loss_sensitivity"]
        self.assertGreater(len(rows), 1)
        ceilings = [row["mass_ceiling_at_goal_kg"] for row in rows]
        self.assertEqual(ceilings, sorted(ceilings, reverse=True))

    def test_speed_outside_the_published_range_reports_instead_of_guessing(self):
        record = _candidate(motor_kv=200)
        point = pc.analyse(record)["operating_points"][0]
        self.assertFalse(point["thrust_available"])
        self.assertIn("does not extrapolate", point["reason"])
        self.assertNotIn("thrust_coaxial_n", point)

    def test_report_renders_for_the_default_candidate(self):
        record = pc.candidate_record()
        text = pc.markdown_report({**record, "analysis": pc.analyse(record)})
        self.assertIn("12x4.5MR", text)
        self.assertIn(record["candidate_sha256"], text)


class ExitCodeTest(unittest.TestCase):
    def test_strict_mode_fails_when_the_goal_is_missed_at_minimum_voltage(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(pc.main([]), 0)
            self.assertEqual(pc.main(["--strict"]), 2)


if __name__ == "__main__":
    unittest.main()
