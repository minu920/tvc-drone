"""Cascaded PID attitude control and TVC allocation for the coaxial airframe, offline.

Replaces the upstream inner-loop LQI with a cascade that can be tuned on the bench without
a mass or inertia estimate. The upstream outer position and velocity loops are already PID,
so this makes the whole stack one kind of controller.

What is reused and what is not:

  reused        the upstream pid_t semantics, which are sound: gains, output clamp, a clamp
                on the integral state rather than only on the output, derivative filtering,
                optional decimation, and an external anti-windup hold
  not reused    lqr_compute, the 12-state LQI formulation and its gain matrix

Allocation is derived from the forward model in tvc_model rather than assumed. With the
pivot at [0, 0, arm] in FRD and the thrust direction from Rx(alpha) Ry(beta) [0, 0, -1]:

    M_x = -T a sin(alpha) cos(beta) + tau sin(beta)
    M_y = -T a sin(beta)            - tau sin(alpha) cos(beta)
    M_z =                             tau cos(alpha) cos(beta)

which inverts in closed form for alpha and beta, so the allocation is the exact inverse of
the model the simulation integrates, not a small-angle stand-in. A round-trip test holds it
to that.

Yaw is not closed here. Yaw torque comes from differential thrust through a coefficient that
has to be measured on the assembled coaxial stack, and it is null in the airframe profile.
Requesting yaw without it raises instead of substituting a number.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

import parameter_profiles as pp
import tvc_model as tm


class Pid:
    """Mirrors the upstream pid_t so gains carry across to the firmware unchanged."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit, d_filter_alpha=0.0):
        if min(kp, ki, kd) < 0 or output_limit <= 0 or integral_limit < 0:
            raise ValueError("Gains must be non-negative and the output limit positive.")
        if not 0.0 <= d_filter_alpha < 1.0:
            raise ValueError("d_filter_alpha is a first-order blend in [0, 1).")
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit, self.integral_limit = output_limit, integral_limit
        self.d_filter_alpha = d_filter_alpha
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_measurement = None
        self.d_filtered = 0.0

    def update(self, setpoint, measurement, dt, hold_integral=False):
        if dt <= 0:
            raise ValueError("dt must be positive.")
        error = setpoint - measurement
        if not hold_integral:
            # Clamp the integral state, not just the output, so recovery is immediate.
            self.integral = float(np.clip(self.integral + error * dt,
                                          -self.integral_limit, self.integral_limit))
        # Derivative on measurement, so a setpoint step does not spike the output.
        if self.prev_measurement is None:
            raw = 0.0
        else:
            raw = -(measurement - self.prev_measurement) / dt
        self.prev_measurement = measurement
        self.d_filtered = (self.d_filter_alpha * self.d_filtered
                           + (1.0 - self.d_filter_alpha) * raw)
        out = self.kp * error + self.ki * self.integral + self.kd * self.d_filtered
        clipped = float(np.clip(out, -self.output_limit, self.output_limit))
        return clipped, abs(out) > self.output_limit + 1e-12


def allocate_tvc(moment_demand, total_thrust_n, arm_m, reaction_torque_nm, angle_limit_rad):
    """Gimbal angles that produce a roll and pitch moment demand. Exact model inverse.

    Returns (alpha, beta, saturated). The demand is met only within the gimbal travel, so a
    saturated result is reported rather than silently clipped and forgotten: the attitude
    loop needs it to hold its integrators.
    """
    mx, my, mz = (float(v) for v in moment_demand)
    if not np.isfinite([mx, my, mz]).all():
        raise ValueError("Moment demand must be finite.")
    if total_thrust_n <= 0 or arm_m <= 0:
        raise ValueError("Allocation needs positive thrust and a positive arm.")
    lever = total_thrust_n * arm_m
    tau = float(reaction_torque_nm)

    # With u = sin(alpha) cos(beta) and v = sin(beta) the forward model is linear:
    #     M_x = -L u + tau v
    #     M_y = -tau u - L v
    # so both angles come from one 2x2 inverse. Solving M_y for beta alone, as if the
    # reaction term were absent, leaves an error proportional to tau.
    det = lever * lever + tau * tau
    u = (-lever * mx - tau * my) / det
    v = (tau * mx - lever * my) / det

    over_travel = not (-1.0 <= v <= 1.0)
    beta = math.asin(float(np.clip(v, -1.0, 1.0)))
    c_beta = math.cos(beta)
    if abs(c_beta) < 1e-9:
        raise ValueError("Degenerate allocation: beta at 90 degrees.")
    s_alpha = u / c_beta
    over_travel = over_travel or not (-1.0 <= s_alpha <= 1.0)
    alpha = math.asin(float(np.clip(s_alpha, -1.0, 1.0)))

    saturated = (over_travel or abs(alpha) > angle_limit_rad + 1e-12
                 or abs(beta) > angle_limit_rad + 1e-12)
    return (float(np.clip(alpha, -angle_limit_rad, angle_limit_rad)),
            float(np.clip(beta, -angle_limit_rad, angle_limit_rad)), saturated)


def yaw_torque(differential_thrust_n, profile):
    """Yaw torque from differential thrust. Raises while the coefficient is unmeasured."""
    entry = profile["parameters"].get("yaw_torque_per_differential_thrust", {})
    if entry.get("value") is None:
        raise ValueError(
            "yaw_torque_per_differential_thrust is unmeasured. It cannot come from "
            "single-motor data: in trim the two rotors carry equal torque but unequal "
            "thrust, so this has to be measured on the assembled coaxial stack.")
    return float(entry["value"]) * float(differential_thrust_n)


class AttitudeController:
    """Angle loop sets a rate target; rate loop asks for a moment; allocation tilts."""

    def __init__(self, angle_gain, rate_gains, rate_limit_rad_s, moment_limit_nm,
                 angle_limit_rad, d_filter_alpha=0.6):
        self.angle = [Pid(angle_gain, 0.0, 0.0, rate_limit_rad_s, 0.0) for _ in range(2)]
        self.rate = [Pid(*rate_gains, moment_limit_nm, moment_limit_nm * 0.5,
                         d_filter_alpha) for _ in range(2)]
        self.angle_limit = angle_limit_rad
        self.last = {"rate_setpoint": [0.0, 0.0], "moment": [0.0, 0.0],
                     "saturated": False}

    def reset(self):
        for p in self.angle + self.rate:
            p.reset()

    def update(self, attitude_setpoint, attitude, rates, dt, total_thrust_n, arm_m,
               reaction_torque_nm=0.0):
        rate_sp, moment = [0.0, 0.0], [0.0, 0.0]
        for i in range(2):
            rate_sp[i], _ = self.angle[i].update(attitude_setpoint[i], attitude[i], dt)
            moment[i], _ = self.rate[i].update(rate_sp[i], rates[i], dt,
                                               hold_integral=self.last["saturated"])
        alpha, beta, saturated = allocate_tvc((moment[0], moment[1], 0.0), total_thrust_n,
                                              arm_m, reaction_torque_nm, self.angle_limit)
        self.last = {"rate_setpoint": rate_sp, "moment": moment, "saturated": saturated}
        return alpha, beta, saturated


def simulate(profile, controller, servo_tau_s, servo_rate_limit_rad_s, dt, duration_s,
             initial_attitude_rad=(0.05, 0.0), disturbance_nm=(0.0, 0.0),
             disturbance_at_s=None):
    """Two-axis closed loop on the tvc_model dynamics with a first-order servo.

    Attitude is integrated as small-angle roll and pitch about the hover point, which is the
    same linearisation the upstream LQI baseline uses, so the two are comparable. It is not a
    full nonlinear simulation and does not claim to be.
    """
    # Only Ixx and Iyy are needed. With yaw rate held at zero the gyroscopic term
    # omega x I omega reduces to [0, 0, p q (Iyy - Ixx)], so its roll and pitch components
    # vanish and Izz never enters. diagonal_inertia() is deliberately refused while Izz is
    # unmeasured, and that guard is respected here rather than worked around.
    i_xx = pp.parameter(profile, "inertia_xx", "kg*m^2")
    i_yy = pp.parameter(profile, "inertia_yy", "kg*m^2")
    if min(i_xx, i_yy) <= 0:
        raise ValueError("Roll and pitch inertias must be positive.")
    mass = pp.parameter(profile, "mass", "kg")
    gravity = pp.parameter(profile, "gravity", "m/s^2")
    arm = pp.parameter(profile, "tvc_arm", "m")
    thrust = mass * gravity                      # hover trim
    limit = controller.angle_limit

    attitude = np.array([initial_attitude_rad[0], initial_attitude_rad[1], 0.0])
    rates = np.zeros(3)
    actual = np.zeros(2)                         # servo output angles
    steps = int(round(duration_s / dt))
    log = {k: np.zeros(steps) for k in
           ("t", "roll", "pitch", "cmd_alpha", "cmd_beta", "act_alpha", "act_beta",
            "saturated", "moment_x", "moment_y")}

    for n in range(steps):
        t = n * dt
        cmd_a, cmd_b, sat = controller.update((0.0, 0.0), attitude[:2], rates[:2], dt,
                                              thrust, arm)
        # First-order servo with a rate limit, then the travel stop.
        for i, cmd in enumerate((cmd_a, cmd_b)):
            step = (cmd - actual[i]) * (dt / servo_tau_s)
            step = float(np.clip(step, -servo_rate_limit_rad_s * dt,
                                 servo_rate_limit_rad_s * dt))
            actual[i] = float(np.clip(actual[i] + step, -limit, limit))

        _, moment = tm.body_wrench(thrust, actual[0], actual[1], arm)
        if disturbance_at_s is not None and t >= disturbance_at_s:
            moment = moment + np.array([disturbance_nm[0], disturbance_nm[1], 0.0])
        rates = rates + np.array([moment[0] / i_xx, moment[1] / i_yy, 0.0]) * dt
        attitude = attitude + rates * dt

        for key, value in (("t", t), ("roll", attitude[0]), ("pitch", attitude[1]),
                           ("cmd_alpha", cmd_a), ("cmd_beta", cmd_b),
                           ("act_alpha", actual[0]), ("act_beta", actual[1]),
                           ("saturated", float(sat)), ("moment_x", moment[0]),
                           ("moment_y", moment[1])):
            log[key][n] = value
    return log


def metrics(log):
    roll_deg = np.rad2deg(log["roll"])
    return {"roll_rms_deg": float(np.sqrt(np.mean(roll_deg ** 2))),
            "roll_peak_deg": float(np.max(np.abs(roll_deg))),
            "final_roll_deg": float(roll_deg[-1]),
            "command_saturated_fraction": float(np.mean(log["saturated"])),
            "peak_command_deg": float(np.max(np.abs(np.rad2deg(log["cmd_alpha"])))),
            "peak_tracking_error_deg": float(
                np.max(np.abs(np.rad2deg(log["cmd_alpha"] - log["act_alpha"]))))}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", type=Path, default=pp.DEFAULT_PROFILE)
    p.add_argument("--angle-gain", type=float, default=6.0, help="rad/s per rad")
    p.add_argument("--rate-kp", type=float, default=0.30)
    p.add_argument("--rate-ki", type=float, default=0.60)
    p.add_argument("--rate-kd", type=float, default=0.010)
    p.add_argument("--rate-limit-deg-s", type=float, default=120.0)
    p.add_argument("--moment-limit-nm", type=float, default=1.5)
    p.add_argument("--angle-limit-deg", type=float, default=8.0)
    p.add_argument("--servo-tau-ms", type=float, default=60.0)
    p.add_argument("--servo-rate-deg-s", type=float, default=160.0)
    p.add_argument("--dt-ms", type=float, default=5.0)
    p.add_argument("--duration-s", type=float, default=6.0)
    p.add_argument("--initial-roll-deg", type=float, default=5.0)
    p.add_argument("--disturbance-nm", type=float, default=0.0)
    p.add_argument("--disturbance-at-s", type=float, default=None)
    p.add_argument("--output", type=Path)
    args = p.parse_args(argv)

    record = pp.profile_record(args.profile)
    profile = record["profile"]
    controller = AttitudeController(args.angle_gain,
                                    (args.rate_kp, args.rate_ki, args.rate_kd),
                                    math.radians(args.rate_limit_deg_s),
                                    args.moment_limit_nm,
                                    math.radians(args.angle_limit_deg))
    log = simulate(profile, controller, args.servo_tau_ms / 1000.0,
                   math.radians(args.servo_rate_deg_s), args.dt_ms / 1000.0,
                   args.duration_s,
                   initial_attitude_rad=(math.radians(args.initial_roll_deg), 0.0),
                   disturbance_nm=(args.disturbance_nm, 0.0),
                   disturbance_at_s=args.disturbance_at_s)
    result = metrics(log)

    report = {"profile": profile["profile"], "profile_sha256": record["profile_sha256"],
              "gains": {"angle_p_rad_s_per_rad": args.angle_gain,
                        "rate_kp": args.rate_kp, "rate_ki": args.rate_ki,
                        "rate_kd": args.rate_kd,
                        "rate_limit_deg_s": args.rate_limit_deg_s,
                        "moment_limit_nm": args.moment_limit_nm,
                        "gimbal_command_limit_deg": args.angle_limit_deg},
              "actuator": {"tau_ms": args.servo_tau_ms,
                           "rate_limit_deg_s": args.servo_rate_deg_s,
                           "status": "assumed, not identified"},
              "metrics": result,
              "limits": ["Small-angle roll and pitch about hover, matching the upstream "
                         "LQI baseline's linearisation. Not a nonlinear simulator.",
                         "Servo time constant and rate limit are assumptions.",
                         "Yaw is not simulated; its coefficient is unmeasured.",
                         "These gains are a starting point for bench tuning, not a result."],
              "offline_only": True}
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8")
        np.savez(args.output / "trace.npz", **log)

    print(f"Cascaded PID on {profile['profile']}")
    print(f"  angle P {args.angle_gain:.2f} -> rate PID "
          f"{args.rate_kp:.3f}/{args.rate_ki:.3f}/{args.rate_kd:.4f}, "
          f"gimbal limit {args.angle_limit_deg:.0f} deg")
    print(f"  servo tau {args.servo_tau_ms:.0f} ms, rate {args.servo_rate_deg_s:.0f} deg/s "
          f"(both assumed)")
    print(f"  roll RMS {result['roll_rms_deg']:.3f} deg, peak "
          f"{result['roll_peak_deg']:.3f} deg, settled at "
          f"{result['final_roll_deg']:+.3f} deg")
    print(f"  peak gimbal command {result['peak_command_deg']:.2f} deg, "
          f"saturated {result['command_saturated_fraction'] * 100:.1f}% of the run")
    print(f"  peak command-to-actual gap {result['peak_tracking_error_deg']:.2f} deg")
    if args.output is not None:
        print(f"  wrote summary.json and trace.npz to {args.output.resolve()}")
    print("Offline model. Gains are a bench starting point, not a tuned result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
