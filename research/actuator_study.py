"""Illustrative one-axis study, NOT a flight model or deployable controller.

Compare ideal-actuator LQI and lag-augmented LQI on the SAME plant and cost.
Both use the selected profile's sample period (50 Hz for the reference profile).
Servo angle is assumed known perfectly to the augmented
controller; a real system needs angle feedback or a separately validated observer.
The plant time-constant sweep and disturbances are hypotheses, not measurements.
Physical input provenance is recorded in the selected parameter profile.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".build/matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from parameter_profiles import DEFAULT_PROFILE, load_profile, parameter, profile_record, require_inputs

DURATION = 8.0


@dataclass(frozen=True)
class StudyConfig:
    dt: float
    effectiveness: float
    angle_limit: float
    rate_limit: float
    design_tau: float

    @classmethod
    def from_profile(cls, profile):
        require_inputs(profile, "roll_study")
        effectiveness = -(parameter(profile, "mass", "kg") * parameter(profile, "gravity", "m/s^2")
                          * parameter(profile, "tvc_arm", "m") / parameter(profile, "inertia_xx", "kg*m^2"))
        return cls(parameter(profile, "actuator_study_sample_period", "s"), effectiveness,
                   parameter(profile, "tvc_angle_limit", "rad"), parameter(profile, "servo_rate_limit", "rad/s"),
                   parameter(profile, "servo_design_time_constant", "s"))


DEFAULT_CONFIG = StudyConfig.from_profile(load_profile())
# Compatibility aliases for existing reference-only callers; computations use config.
DT, EFFECTIVENESS = DEFAULT_CONFIG.dt, DEFAULT_CONFIG.effectiveness
ANGLE_LIMIT, RATE_LIMIT = DEFAULT_CONFIG.angle_limit, DEFAULT_CONFIG.rate_limit
DESIGN_TAU = DEFAULT_CONFIG.design_tau


def design(tau=None, *, config=DEFAULT_CONFIG):
    if tau is None:
        # theta, angular rate, integral(theta); input = actual tilt (ideal).
        a = np.array([[0., 1., 0.], [0., 0., 0.], [1., 0., 0.]])
        b = np.array([[0.], [config.effectiveness], [0.]])
        q = np.diag([130., 4., 10.])
    else:
        if tau <= 0:
            raise ValueError("tau must be positive")
        # theta, angular rate, actual tilt, integral(theta); input = command.
        a = np.array([[0., 1., 0., 0.], [0., 0., config.effectiveness, 0.],
                      [0., 0., -1 / tau, 0.], [1., 0., 0., 0.]])
        b = np.array([[0.], [0.], [1 / tau], [0.]])
        q = np.diag([130., 4., 0., 10.])
    r = np.array([[100.]])
    ad, bd, _, _, _ = cont2discrete((a, b, np.eye(len(a)), np.zeros_like(b)), config.dt)
    p = solve_discrete_are(ad, bd, q, r)
    gain = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    return gain.ravel(), np.linalg.eigvals(ad - bd @ gain)


def simulate(kind, tau, delay_steps=0, duration=DURATION, *, config=DEFAULT_CONFIG):
    if kind not in ("ideal_lqi", "lag_lqi") or tau <= 0 or delay_steps < 0:
        raise ValueError("Invalid controller, time constant or delay")
    gain, _ = design(None if kind == "ideal_lqi" else config.design_tau, config=config)
    n = round(duration / config.dt)
    states = np.zeros((n + 1, 4))
    states[0, 0] = np.deg2rad(10.)
    commands = np.zeros(n)
    applied = np.zeros(n)
    queue = deque([0.] * delay_steps)
    saturated = np.zeros(n, dtype=bool)
    for k in range(n):
        state = states[k].copy()
        observed = state[[0, 1, 3]] if kind == "ideal_lqi" else state
        raw = -float(gain @ observed)
        command = float(np.clip(raw, -config.angle_limit, config.angle_limit))
        saturated[k] = abs(raw) > config.angle_limit
        queue.append(command)
        held = queue.popleft()
        commands[k], applied[k] = command, held
        # A hypothetical angular-acceleration disturbance between 2 and 3 sec.
        disturbance = 0.5 if 2. <= k * config.dt < 3. else 0.

        def derivative(s):
            servo_rate = np.clip((held - s[2]) / tau, -config.rate_limit, config.rate_limit)
            # Same simple clamped-integral anti-windup in both controllers.
            integral_rate = 0. if saturated[k] else s[0]
            return np.array([s[1], config.effectiveness * s[2] + disturbance,
                             servo_rate, integral_rate])

        # Integrate with 20 RK4 substeps per sample (1 ms for the reference profile).
        h = config.dt / 20
        for _ in range(20):
            a = derivative(state)
            b = derivative(state + h * a / 2)
            c = derivative(state + h * b / 2)
            d = derivative(state + h * c)
            state += h * (a + 2 * b + 2 * c + d) / 6
            state[3] = np.clip(state[3], -1., 1.)
        states[k + 1] = state
    return states, commands, applied, saturated


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/actuator-study")
    args = parser.parse_args(argv)
    try:
        record = profile_record(args.profile)
        config = StudyConfig.from_profile(record["profile"])
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    cases = []
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), layout="constrained")
    for col, tau in enumerate((0.040, 0.120)):
        for kind, label in (("ideal_lqi", "Ideal-actuator LQI"),
                            ("lag_lqi", f"Lag-aware LQI ({1000 * config.design_tau:g} ms design)")):
            states, command, _, saturated = simulate(kind, tau, config=config)
            t = np.arange(len(states)) * config.dt
            axes[0, col].plot(t, np.rad2deg(states[:, 0]), label=label)
            axes[1, col].plot(t, np.rad2deg(states[:, 2]), label=label)
            cases.append({"controller": kind, "plant_tau_ms": 1000 * tau,
                          "command_delay_ms": 0,
                          "roll_rms_deg": float(np.sqrt(np.mean(np.rad2deg(states[:, 0]) ** 2))),
                          "roll_peak_abs_deg": float(np.max(np.abs(np.rad2deg(states[:, 0])))),
                          "command_saturated_fraction": float(saturated.mean())})
            np.savez(args.output / f"{kind}-{round(tau * 1000)}ms.npz", time=t, states=states, command=command)
        axes[0, col].set_title(f"Hypothetical servo time constant: {1000 * tau:.0f} ms")
        axes[0, col].axvspan(2, 3, alpha=.08, color="black")
        axes[1, col].axhline(np.rad2deg(config.angle_limit), color="grey", ls=":")
        axes[1, col].axhline(-np.rad2deg(config.angle_limit), color="grey", ls=":")
        axes[1, col].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Roll (deg)")
    axes[1, 0].set_ylabel("Actual gimbal angle (deg)")
    for ax in axes.flat:
        ax.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Illustrative 1-axis simulation — NOT measured flight performance\n"
                 "Perfect state / servo-angle feedback; shaded = assumed disturbance", fontsize=11)
    fig.savefig(args.output / "comparison.png", dpi=150)
    plt.close(fig)
    sweep = []
    for tau in (0.020, 0.040, 0.080, 0.120):
        for delay in (0, 1, 3):
            for kind in ("ideal_lqi", "lag_lqi"):
                states, _, _, saturated = simulate(kind, tau, delay, config=config)
                sweep.append({"controller": kind, "plant_tau_ms": tau * 1000,
                              "command_delay_ms": delay * config.dt * 1000,
                              "roll_rms_deg": float(np.sqrt(np.mean(np.rad2deg(states[:, 0]) ** 2))),
                              "command_saturated_fraction": float(saturated.mean())})
    report = {"assumptions": __doc__, "sample_period_s": config.dt,
              "parameter_profile": record["profile"],
              "profile_path": record["profile_path"], "profile_sha256": record["profile_sha256"],
              "offline_only": True,
              "design_time_constant_s": config.design_tau, "rate_limit_deg_s": float(np.rad2deg(config.rate_limit)),
              "cases": cases, "sweep": sweep}
    (args.output / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(cases, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
