"""Run the unmodified upstream LQI script offline; never open a serial port.

This checks reproducibility, NOT flight safety or nonlinear stability.
Generated files are written only to the requested results directory.
"""
import argparse
import contextlib
import hashlib
import importlib.metadata
import io
import json
import os
from pathlib import Path
import platform
import re
import runpy
import subprocess
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".build/matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/baseline")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = ROOT / "apps/python/lqr_compute.py"
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured), patch.object(plt, "show"):
        model = runpy.run_path(str(source), run_name="__main__")
    (args.output / "upstream-console.txt").write_text(captured.getvalue(), encoding="utf-8")
    # Preserve the upstream figure unchanged, including its original labels.
    plt.gcf().savefig(args.output / "upstream-lqi.png", dpi=150)
    plt.close("all")
    firmware = (ROOT / "drone-firmware/Core/Src/main.c").read_text(encoding="utf-8")
    block = re.search(r"const\s+float\s+K\[[^;]+?=\s*\{(.*?)\};", firmware, re.S)
    if block is None:
        raise RuntimeError("Could not locate firmware K; update the parser explicitly.")
    values = re.findall(r"[-+]?\d+\.\d+(?:[eE][-+]?\d+)?(?=f)", block[1])
    firmware_k = np.array([float(v) for v in values]).reshape(4, 12)
    checks = {
        "controllability_rank_12": int(model["rank"]) == 12,
        "unsaturated_linear_poles_inside_unit_circle": bool(np.max(np.abs(model["eig_d"])) < 1),
        "finite_simulation": bool(all(np.isfinite(model[k]).all() for k in ("xs", "igs", "us"))),
        "physical_state_shape": model["xs"].shape == (801, 8),
        "inputs_within_upstream_limits": bool(
            np.all(model["us"] >= model["u_min"] - 1e-12)
            and np.all(model["us"] <= model["u_max"] + 1e-12)
        ),
    }
    difference = float(np.max(np.abs(firmware_k - model["K_full"])))
    summary = {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "dependencies": {p: importlib.metadata.version(p) for p in ("numpy", "scipy", "matplotlib")},
        "upstream_lqi_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "checks": checks,
        "max_discrete_pole_magnitude": float(np.max(np.abs(model["eig_d"]))),
        "firmware_K_max_absolute_difference": difference,
        "firmware_K_matches_to_1e_6": difference < 1e-6,
        "warning": "Offline upstream reproduction only. Not a firmware-equivalent or flight validation. "
                   "The upstream yaw input is labelled kg in R but Nm in its plot; units need auditing.",
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    np.savez(args.output / "upstream-data.npz", **{k: model[k] for k in ("xs", "igs", "us", "K_full", "A_d", "B_d")})
    freeze = subprocess.check_output([sys.executable, "-m", "pip", "freeze"], text=True)
    (args.output / "environment-freeze.txt").write_text(freeze, encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
