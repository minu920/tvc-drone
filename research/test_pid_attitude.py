"""Offline checks for the PID attitude controller and TVC allocation."""
import math
import unittest

import numpy as np

import parameter_profiles as pp
import pid_attitude as pa
import tvc_model as tm

UPSTREAM = pp.DEFAULT_PROFILE.parent / "upstream_reference.json"
MINE = pp.DEFAULT_PROFILE.parent / "my_airframe.json"


def _profile(path):
    return pp.profile_record(path)["profile"]


class AllocationTest(unittest.TestCase):
    """The allocation must be the exact inverse of the wrench the simulation integrates."""

    def test_round_trip_through_the_forward_model(self):
        thrust, arm, limit = 4.2, 0.083, math.radians(20.0)
        # Keep the demand inside the travel: L*sin(20 deg) is about 0.119 N.m per axis,
        # and less than that when both axes are commanded at once.
        for mx in (-0.05, -0.02, 0.0, 0.02, 0.05):
            for my in (-0.05, 0.0, 0.03):
                a, b, sat = pa.allocate_tvc((mx, my, 0.0), thrust, arm, 0.0, limit)
                self.assertFalse(sat, f"unexpected saturation at {mx}, {my}")
                _, moment = tm.body_wrench(thrust, a, b, arm)
                self.assertAlmostEqual(moment[0], mx, places=9)
                self.assertAlmostEqual(moment[1], my, places=9)

    def test_round_trip_with_reaction_torque(self):
        thrust, arm, limit, tau = 4.2, 0.083, math.radians(20.0), 0.03
        a, b, sat = pa.allocate_tvc((0.05, -0.03, 0.0), thrust, arm, tau, limit)
        self.assertFalse(sat)
        _, moment = tm.body_wrench(thrust, a, b, arm, reaction_torque_nm=tau)
        self.assertAlmostEqual(moment[0], 0.05, places=9)
        self.assertAlmostEqual(moment[1], -0.03, places=9)

    def test_saturation_is_reported_not_hidden(self):
        thrust, arm, limit = 4.2, 0.083, math.radians(5.0)
        a, b, sat = pa.allocate_tvc((5.0, 0.0, 0.0), thrust, arm, 0.0, limit)
        self.assertTrue(sat)
        self.assertLessEqual(abs(a), limit + 1e-12)
        self.assertLessEqual(abs(b), limit + 1e-12)

    def test_refuses_degenerate_inputs(self):
        limit = math.radians(10.0)
        for args in (((0.1, 0.0, 0.0), 0.0, 0.083), ((0.1, 0.0, 0.0), 4.2, 0.0),
                     ((float("nan"), 0.0, 0.0), 4.2, 0.083)):
            with self.assertRaises(ValueError):
                pa.allocate_tvc(args[0], args[1], args[2], 0.0, limit)

    def test_sign_convention_matches_the_model(self):
        """Positive roll moment needs a negative alpha, per M_x = -T a sin(alpha)."""
        a, _, _ = pa.allocate_tvc((0.1, 0.0, 0.0), 4.2, 0.083, 0.0, math.radians(20.0))
        self.assertLess(a, 0.0)


class YawTest(unittest.TestCase):
    def test_unmeasured_coefficient_raises_rather_than_defaulting(self):
        with self.assertRaises(ValueError) as caught:
            pa.yaw_torque(1.0, _profile(MINE))
        self.assertIn("unmeasured", str(caught.exception))

    def test_upstream_profile_also_lacks_it(self):
        """The upstream fit is a legacy mass-equivalent, not this coefficient."""
        with self.assertRaises(ValueError):
            pa.yaw_torque(1.0, _profile(UPSTREAM))


class PidTest(unittest.TestCase):
    def test_integral_state_is_clamped_not_just_the_output(self):
        pid = pa.Pid(0.0, 1.0, 0.0, output_limit=10.0, integral_limit=0.5)
        for _ in range(100):
            pid.update(1.0, 0.0, 0.01)
        self.assertLessEqual(abs(pid.integral), 0.5 + 1e-12)

    def test_setpoint_step_does_not_spike_the_derivative(self):
        pid = pa.Pid(0.0, 0.0, 1.0, output_limit=100.0, integral_limit=0.0)
        pid.update(0.0, 0.0, 0.01)
        out, _ = pid.update(10.0, 0.0, 0.01)      # setpoint jumps, measurement does not
        self.assertAlmostEqual(out, 0.0, places=12)

    def test_hold_integral_freezes_accumulation(self):
        pid = pa.Pid(0.0, 1.0, 0.0, output_limit=10.0, integral_limit=10.0)
        pid.update(1.0, 0.0, 0.1)
        held = pid.integral
        pid.update(1.0, 0.0, 0.1, hold_integral=True)
        self.assertAlmostEqual(pid.integral, held, places=12)

    def test_output_limit_flag(self):
        pid = pa.Pid(100.0, 0.0, 0.0, output_limit=1.0, integral_limit=0.0)
        out, limited = pid.update(1.0, 0.0, 0.01)
        self.assertTrue(limited)
        self.assertAlmostEqual(out, 1.0, places=12)

    def test_rejects_bad_configuration(self):
        for bad in ((-1.0, 0.0, 0.0, 1.0, 1.0), (1.0, 0.0, 0.0, 0.0, 1.0),
                    (1.0, 0.0, 0.0, 1.0, -1.0)):
            with self.assertRaises(ValueError):
                pa.Pid(*bad)
        with self.assertRaises(ValueError):
            pa.Pid(1.0, 0.0, 0.0, 1.0, 1.0, d_filter_alpha=1.0)


def _run(profile, tau=0.06, roll0=math.radians(5.0), duration=6.0, **kw):
    controller = pa.AttitudeController(6.0, (0.30, 0.60, 0.010),
                                       math.radians(120.0), 1.5, math.radians(8.0))
    return pa.simulate(profile, controller, tau, math.radians(160.0), 0.005, duration,
                       initial_attitude_rad=(roll0, 0.0), **kw)


class SimulationTest(unittest.TestCase):
    def test_converges_from_an_initial_offset(self):
        log = _run(_profile(UPSTREAM))
        self.assertLess(abs(math.degrees(log["roll"][-1])), 0.2)
        self.assertTrue(np.isfinite(log["roll"]).all())

    def test_states_stay_finite_and_within_the_travel_stop(self):
        log = _run(_profile(UPSTREAM))
        limit = math.radians(8.0) + 1e-9
        self.assertLessEqual(np.max(np.abs(log["act_alpha"])), limit)
        self.assertLessEqual(np.max(np.abs(log["act_beta"])), limit)

    def test_slower_servo_tracks_worse(self):
        fast = pa.metrics(_run(_profile(UPSTREAM), tau=0.02))
        slow = pa.metrics(_run(_profile(UPSTREAM), tau=0.12))
        self.assertGreater(slow["peak_tracking_error_deg"],
                           fast["peak_tracking_error_deg"])

    def test_disturbance_is_rejected_back_towards_level(self):
        log = _run(_profile(UPSTREAM), roll0=0.0, duration=8.0,
                   disturbance_nm=(0.05, 0.0), disturbance_at_s=1.0)
        tail = np.rad2deg(log["roll"][-200:])
        self.assertLess(float(np.max(np.abs(tail))), 3.0)

    def test_unmeasured_airframe_is_refused(self):
        with self.assertRaises(ValueError):
            _run(_profile(MINE))


if __name__ == "__main__":
    unittest.main()
