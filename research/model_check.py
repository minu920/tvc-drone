"""Audit SI/frame conversion against upstream algebra, not physical calibration."""
import argparse
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import runpy
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".build/matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from tvc_model import hover_lqi, legacy_coordinate_maps, parameter
from parameter_profiles import profile_record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/model-contract")
    args = parser.parse_args()
    record = profile_record()  # Intentionally fixed to upstream-reference equivalence.
    profile = record["profile"]
    model = hover_lqi(profile)
    s, u = legacy_coordinate_maps(profile)
    inverse_u = np.linalg.inv(u)
    source = ROOT / "apps/python/lqr_compute.py"
    with contextlib.redirect_stdout(io.StringIO()), patch.object(plt, "show"):
        upstream = runpy.run_path(str(source), run_name="__main__")
    plt.close("all")
    expected_a = s @ upstream["A_d"] @ s
    expected_b = s @ upstream["B_d"] @ inverse_u
    expected_k = u @ upstream["K_full"] @ s
    differences = {
        "A_d_max_abs_difference": float(np.max(np.abs(model["A_d"] - expected_a))),
        "B_d_max_abs_difference": float(np.max(np.abs(model["B_d"] - expected_b))),
        "K_max_abs_difference": float(np.max(np.abs(model["K"] - expected_k))),
    }
    checks = {
        "discrete_A_coordinate_equivalence": bool(np.allclose(model["A_d"], expected_a, atol=1e-12, rtol=0)),
        "discrete_B_coordinate_equivalence": bool(np.allclose(model["B_d"], expected_b, atol=1e-12, rtol=0)),
        "LQI_gain_equivalence_with_transformed_R": bool(np.allclose(model["K"], expected_k, atol=1e-7, rtol=0)),
        "linear_poles_inside_unit_circle": bool(np.max(np.abs(model["poles"])) < 1.),
    }
    report = {
        **record,
        "upstream_script_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "checks": checks, "differences": differences,
        "nominal_hover_total_thrust_n": parameter(profile, "mass", "kg") * parameter(profile, "gravity", "m/s^2"),
        "unknown_parameters": [name for name, data in profile["parameters"].items() if data["value"] is None],
        "warning": "Algebraic reference agreement, not sensor-frame calibration or full nonlinear flight validation. "
                   "No firmware/PWM gains generated. Izz, reaction torque coefficient, maximum thrust are unknown.",
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    np.savez(args.output / "si-hover-model.npz", **model, state_map=s, input_map=u)
    print(json.dumps({"checks": checks, "differences": differences,
                      "unknown_parameters": report["unknown_parameters"]}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
