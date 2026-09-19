import copy
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from tvc_model import (
    NED_TO_ENU, FRD_TO_FLU, angular_acceleration, body_wrench, diagonal_inertia,
    hover_lqi, load_profile, parameter, rotation_ned_frd_to_enu_flu,
    split_thrust, thrust_direction, translational_acceleration,
)


class ModelContractTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile()
        self.mass = parameter(self.profile, "mass", "kg")
        self.g = parameter(self.profile, "gravity", "m/s^2")
        self.arm = parameter(self.profile, "tvc_arm", "m")

    def test_unknown_is_not_zero(self):
        with self.assertRaisesRegex(ValueError, "unknown"):
            diagonal_inertia(self.profile)
        with self.assertRaisesRegex(ValueError, "unknown"):
            parameter(self.profile, "max_total_thrust", "N")

    def test_reject_wrong_units_and_nan(self):
        with self.assertRaises(ValueError):
            parameter(self.profile, "mass", "N")
        other = copy.deepcopy(self.profile)
        other["parameters"]["mass"]["value"] = float("nan")
        with self.assertRaises(ValueError):
            parameter(other, "mass", "kg")

    def test_hover_balance(self):
        force, moment = body_wrench(self.mass * self.g, 0., 0., self.arm)
        np.testing.assert_allclose(translational_acceleration(self.profile, np.eye(3), force), 0., atol=1e-12)
        np.testing.assert_allclose(moment, 0., atol=1e-12)

    def test_freefall_at_arbitrary_attitude(self):
        rotation = Rotation.from_euler("xyz", [.2, -.4, .7]).as_matrix()
        force, moment = body_wrench(0., .1, -.2, self.arm)
        np.testing.assert_allclose(translational_acceleration(self.profile, rotation, force), [0., 0., self.g])
        # Synthetic inertia is only a mathematical fixture, NOT aircraft data.
        np.testing.assert_allclose(angular_acceleration(np.diag([.01, .02, .025]), np.zeros(3), moment), 0.)

    def test_tvc_unit_direction_and_signs(self):
        for alpha, beta in ((0., 0.), (.1, -.2), (-.3, .4)):
            self.assertAlmostEqual(np.linalg.norm(thrust_direction(alpha, beta)), 1.)
        fx, tx = body_wrench(self.mass * self.g, .01, 0., self.arm)
        fy, ty = body_wrench(self.mass * self.g, 0., .01, self.arm)
        self.assertGreater(fx[1], 0.)
        self.assertLess(tx[0], 0.)
        self.assertLess(fy[0], 0.)
        self.assertLess(ty[1], 0.)

    def test_reaction_torque_follows_tilted_axis(self):
        _, moment = body_wrench(0., .1, .2, self.arm, .03)
        np.testing.assert_allclose(moment, -.03 * thrust_direction(.1, .2))

    def test_thrust_conservation_and_impossible_request(self):
        motors = split_thrust(4., .6)
        self.assertAlmostEqual(motors.sum(), 4.)
        self.assertAlmostEqual(motors[0] - motors[1], .6)
        np.testing.assert_allclose(split_thrust(0., 0.), 0.)
        with self.assertRaises(ValueError):
            split_thrust(1., 1.1)
        with self.assertRaises(ValueError):
            body_wrench(-1., 0., 0., self.arm)

    def test_frame_transforms_preserve_vectors(self):
        rotation = Rotation.from_euler("xyz", [.3, -.1, .7]).as_matrix()
        vector = np.array([1., 2., 3.])
        converted = rotation_ned_frd_to_enu_flu(rotation)
        np.testing.assert_allclose(converted @ (FRD_TO_FLU @ vector), NED_TO_ENU @ rotation @ vector)
        np.testing.assert_allclose(NED_TO_ENU @ np.array([1., 2., 3.]), [2., 1., -3.])
        np.testing.assert_allclose(converted.T @ converted, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(converted), 1.)

    def test_invalid_rotation_and_inertia(self):
        with self.assertRaises(ValueError):
            rotation_ned_frd_to_enu_flu(np.diag([1., 1., -1.]))
        with self.assertRaises(ValueError):
            angular_acceleration(np.diag([1., 0., 1.]), np.zeros(3), np.zeros(3))

    def test_nonzero_angular_momentum_term(self):
        inertia = np.diag([.01, .02, .025])
        omega = np.array([1., 2., 3.])
        result = angular_acceleration(inertia, omega, np.zeros(3))
        np.testing.assert_allclose(inertia @ result + np.cross(omega, inertia @ omega), 0., atol=1e-12)

    def test_linearized_roll_pitch_matches_wrench(self):
        model = hover_lqi(self.profile)
        eps = 1e-6
        for column, name in ((0, "inertia_xx"), (1, "inertia_yy")):
            angles = [0., 0.]
            angles[column] = eps
            _, torque = body_wrench(self.mass * self.g, *angles, self.arm)
            gain = torque[column] / eps / parameter(self.profile, name, "kg*m^2")
            self.assertAlmostEqual(gain, model["B_c"][4 + column, column], places=7)
        self.assertLess(model["B_c"][7, 3], 0.)  # More upward thrust => less down acceleration.
        self.assertLess(np.max(np.abs(model["poles"])), 1.)


if __name__ == "__main__":
    unittest.main()
