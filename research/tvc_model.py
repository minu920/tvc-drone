"""Offline model contract; no PWM, firmware, serial or flight-ready calibration.

World NED, body FRD, right-handed. R maps body vectors into world vectors.
Actual tilt alpha/beta: Rx(alpha) @ Ry(beta), not servo shaft/PWM coordinates.
"""
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from parameter_profiles import DEFAULT_PROFILE, load_profile, parameter, require_inputs

NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def validate_rotation(rotation):
    r = np.asarray(rotation, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-9, rtol=0)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-9, rtol=0)):
        raise ValueError("Expected a proper 3x3 rotation matrix")
    return r


def rotation_ned_frd_to_enu_flu(rotation):
    return NED_TO_ENU @ validate_rotation(rotation) @ FRD_TO_FLU.T


def thrust_direction(alpha, beta):
    if not np.isfinite([alpha, beta]).all():
        raise ValueError("TVC angles must be finite radians")
    return np.array([-np.sin(beta), np.sin(alpha) * np.cos(beta),
                     -np.cos(alpha) * np.cos(beta)])


def body_wrench(total_thrust_n, alpha, beta, arm_m, reaction_torque_nm=0.):
    """Virtual wrench, not motor allocation. Positive reaction torque is +body z at neutral.

    Idealized shared coaxial thrust axis and effective pivot below CoM.
    Net reaction torque follows the tilted axis opposite the thrust direction.
    """
    if not np.isfinite([total_thrust_n, arm_m, reaction_torque_nm]).all() or total_thrust_n < 0 or arm_m < 0:
        raise ValueError("Thrust/arm must be nonnegative, all inputs finite")
    direction = thrust_direction(alpha, beta)
    force = total_thrust_n * direction
    moment = np.cross([0., 0., arm_m], force) - reaction_torque_nm * direction
    return force, moment


def translational_acceleration(profile, rotation, force_body_n):
    force = np.asarray(force_body_n, dtype=float)
    if force.shape != (3,) or not np.isfinite(force).all():
        raise ValueError("Expected a finite 3D body force")
    return (validate_rotation(rotation) @ force / parameter(profile, "mass", "kg")
            + np.array([0., 0., parameter(profile, "gravity", "m/s^2")]))


def angular_acceleration(inertia_kg_m2, omega_rad_s, moment_nm):
    inertia = np.asarray(inertia_kg_m2, dtype=float)
    omega, moment = np.asarray(omega_rad_s, dtype=float), np.asarray(moment_nm, dtype=float)
    if (inertia.shape != (3, 3) or not np.isfinite(inertia).all()
            or not np.allclose(inertia, inertia.T, atol=1e-12, rtol=0)
            or np.min(np.linalg.eigvalsh(inertia)) <= 0):
        raise ValueError("Inertia must be finite, symmetric and positive definite")
    if omega.shape != (3,) or moment.shape != (3,) or not np.isfinite([omega, moment]).all():
        raise ValueError("Expected finite 3D angular velocity and moment")
    return np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega))


def diagonal_inertia(profile):
    """Requires all three inertias; principal-axis approximation is explicit."""
    require_inputs(profile, "diagonal_inertia")
    values = [parameter(profile, f"inertia_{axis}{axis}", "kg*m^2") for axis in "xyz"]
    if min(values) <= 0:
        raise ValueError("All principal inertias must be positive")
    return np.diag(values)


def split_thrust(total_n, differential_n):
    """Ideal algebra only: T=T1+T2, D=T1-T2. Not the upstream PWM fit."""
    if not np.isfinite([total_n, differential_n]).all() or total_n < 0 or abs(differential_n) > total_n:
        raise ValueError("Non-reversing rotors require T >= 0 and abs(D) <= T")
    return np.array([(total_n + differential_n) / 2, (total_n - differential_n) / 2])


def legacy_coordinate_maps(profile):
    """Algebraic near-hover aliases only; NOT a sensor mount/frame calibration.

    x_si = S x_legacy: negate altitude, vertical rate and altitude integral.
    u_si = U u_legacy: alpha=u0, beta=-u1, D_N=g*u2, delta_T_N=u3.
    """
    s = np.diag([1., 1., 1., -1., 1., 1., 1., -1., 1., 1., 1., -1.])
    u = np.diag([1., -1., parameter(profile, "gravity", "m/s^2"), 1.])
    return s, u


def hover_lqi(profile):
    """12-state local small-angle model in SI, preserving upstream mathematical cost.

    x=[phi,theta,psi,z_down,p,q,r,v_down,int_phi,int_theta,int_psi,int_z_down].
    u=[alpha_rad,beta_rad,D_N,delta_T_N]. Yaw effectiveness is provisional,
    converted from upstream kg-equivalent semantics, not identified hardware.
    Lateral translation, nonlinear rotations and actuator dynamics are excluded.
    """
    require_inputs(profile, "reference_lqi")
    mass, g, arm = (parameter(profile, *item) for item in
                    (("mass", "kg"), ("gravity", "m/s^2"), ("tvc_arm", "m")))
    a, b = np.zeros((12, 12)), np.zeros((12, 4))
    a[:4, 4:8] = np.eye(4)
    a[8:12, :4] = np.eye(4)
    b[4, 0] = -mass * g * arm / parameter(profile, "inertia_xx", "kg*m^2")
    b[5, 1] = -mass * g * arm / parameter(profile, "inertia_yy", "kg*m^2")
    b[6, 2] = parameter(profile, "yaw_accel_per_legacy_mass_equivalent", "rad/s^2/kg_equiv") / g
    b[7, 3] = -1 / mass
    s, u = legacy_coordinate_maps(profile)
    inv_u = np.linalg.inv(u)
    q = s.T @ np.diag(profile["upstream_lqi_cost"]["q_diagonal"]) @ s
    r = inv_u.T @ np.diag(profile["upstream_lqi_cost"]["r_legacy_diagonal"]) @ inv_u
    dt = parameter(profile, "lqi_sample_period", "s")
    ad, bd, _, _, _ = cont2discrete((a, b, np.eye(12), np.zeros((12, 4))), dt)
    p = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    return {"A_c": a, "B_c": b, "A_d": ad, "B_d": bd, "Q": q, "R": r,
            "K": k, "poles": np.linalg.eigvals(ad - bd @ k)}
