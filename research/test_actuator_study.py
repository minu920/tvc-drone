import contextlib
import copy
import io
from pathlib import Path
import tempfile
import unittest

import numpy as np

from actuator_study import ANGLE_LIMIT, DT, RATE_LIMIT, DEFAULT_CONFIG, StudyConfig, design, simulate, main
from parameter_profiles import DEFAULT_PROFILE, load_profile


class ActuatorStudyTests(unittest.TestCase):
    def test_design_poles(self):
        for tau in (None, 0.04):
            gain, poles = design(tau)
            self.assertTrue(np.isfinite(gain).all())
            self.assertLess(np.max(np.abs(poles)), 1.)

    def test_limits_and_finite(self):
        for kind in ("ideal_lqi", "lag_lqi"):
            states, commands, _, _ = simulate(kind, .04, duration=1.)
            self.assertTrue(np.isfinite(states).all())
            self.assertLessEqual(np.abs(commands).max(), ANGLE_LIMIT + 1e-12)
            self.assertLessEqual(np.abs(states[:, 2]).max(), ANGLE_LIMIT + 1e-12)
            self.assertLessEqual(np.abs(np.diff(states[:, 2])).max(), RATE_LIMIT * DT + 1e-12)

    def test_delayed_command_queue(self):
        states, commands, applied, _ = simulate("ideal_lqi", .04, 3, duration=.2)
        np.testing.assert_array_equal(applied[:3], 0.)
        np.testing.assert_array_equal(applied[3:], commands[:-3])
        self.assertAlmostEqual(states[3, 1], 0.)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            simulate("ideal_lqi", 0)
        with self.assertRaises(ValueError):
            design(-1)

    def test_explicit_reference_config_preserves_results(self):
        for kind in ("ideal_lqi", "lag_lqi"):
            old = simulate(kind, .04, duration=.2)
            explicit = simulate(kind, .04, duration=.2, config=StudyConfig.from_profile(load_profile()))
            for expected, actual in zip(old, explicit):
                np.testing.assert_array_equal(expected, actual)

    def test_custom_config_is_used_without_mutating_defaults(self):
        profile = copy.deepcopy(load_profile())
        profile["profile"] = "synthetic_test_only"
        changes = {"mass": .6, "actuator_study_sample_period": .01,
                   "tvc_angle_limit": .04, "servo_rate_limit": .5,
                   "servo_design_time_constant": .08}
        for name, value in changes.items():
            profile["parameters"][name].update(value=value, status="assumed", source="Synthetic test only")
        config = StudyConfig.from_profile(profile)
        self.assertAlmostEqual(config.effectiveness / DEFAULT_CONFIG.effectiveness, .6 / .430)
        for kind in ("ideal_lqi", "lag_lqi"):
            states, commands, _, _ = simulate(kind, .04, duration=.2, config=config)
            self.assertEqual(len(states), 21)
            self.assertLessEqual(np.abs(commands).max(), config.angle_limit + 1e-12)
            self.assertLessEqual(np.abs(np.diff(states[:, 2])).max(), config.rate_limit * config.dt + 1e-12)
        self.assertEqual(DEFAULT_CONFIG.dt, .02)

    def test_draft_cli_stops_before_creating_experiment_output(self):
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stderr(io.StringIO()):
            output = Path(folder) / "must-not-exist"
            with self.assertRaises(SystemExit) as failure:
                main(["--profile", str(DEFAULT_PROFILE.with_name("my_airframe.json")), "--output", str(output)])
            self.assertEqual(failure.exception.code, 2)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
