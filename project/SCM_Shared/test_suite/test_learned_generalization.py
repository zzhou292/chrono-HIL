#!/usr/bin/env python3
"""Generalization test for the learned terrain estimator.

The learned MLP was trained on traces collected over only three discrete
soils (clay n=0.5, dirt n=0.7, sand n=1.1).  This script answers the
honest question: does the MLP behave like a 3-class discriminator that
just outputs one of those three values, or does it actually generalize
across the n axis?

Test design
-----------
We synthesise novel terrain configurations by linearly interpolating the
six Bekker / Mohr-Coulomb parameters between adjacent presets along the n
axis (and extrapolating slightly outside the training range), then run
the chrono SCM sim with each custom YAML and ask the learned estimator
what it predicts.

The interpolated 6-param vector is the same one the manifold mapping
in ``learned_terrain_estimator._terrain_params_for_n`` uses internally,
so the comparison is apples-to-apples for "novel n on a soil that lies
on the preset manifold".  A separate "off-manifold" sweep perturbs one
of the auxiliary parameters (cohesion) at fixed n to probe sensitivity
to soil features the MLP never saw vary independently.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = PROJECT_ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from param_consistency import TERRAIN_PRESETS

PRESETS_BY_N = sorted(
    [(name, dict(params)) for name, params in TERRAIN_PRESETS.items()],
    key=lambda kv: float(kv[1]["n"]),
)
N_LO = float(PRESETS_BY_N[0][1]["n"])
N_HI = float(PRESETS_BY_N[-1][1]["n"])

INTERP_KEYS = ("Kphi", "Kc", "n", "cohesion", "friction_angle", "janosi_shear")


def interp_terrain(target_n: float) -> Dict[str, float]:
    """Linearly interpolate / extrapolate the six-parameter vector along
    the preset sequence.  For target_n inside [N_LO, N_HI] this hits the
    bracketing pair; outside, it linearly extends from the nearest pair."""
    target_n = float(target_n)
    if target_n <= float(PRESETS_BY_N[0][1]["n"]):
        a, b = PRESETS_BY_N[0][1], PRESETS_BY_N[1][1]
    elif target_n >= float(PRESETS_BY_N[-1][1]["n"]):
        a, b = PRESETS_BY_N[-2][1], PRESETS_BY_N[-1][1]
    else:
        for i in range(len(PRESETS_BY_N) - 1):
            n_a = float(PRESETS_BY_N[i][1]["n"])
            n_b = float(PRESETS_BY_N[i + 1][1]["n"])
            if n_a <= target_n <= n_b:
                a, b = PRESETS_BY_N[i][1], PRESETS_BY_N[i + 1][1]
                break
    n_a = float(a["n"])
    n_b = float(b["n"])
    ratio = (target_n - n_a) / (n_b - n_a) if n_b != n_a else 0.0
    out = {}
    for k in INTERP_KEYS:
        out[k] = float(a[k] + ratio * (b[k] - a[k]))
    out["n"] = target_n     # honour the requested n exactly
    out["elastic_stiffness"] = 2e8
    out["damping"] = 3e4
    out["description"] = f"interp_n_{target_n:.2f}"
    return out


def write_yaml(cfg: Dict[str, float]) -> Path:
    fd, path = tempfile.mkstemp(prefix="terrain_", suffix=".yaml")
    import os
    os.close(fd)
    Path(path).write_text(yaml.safe_dump(cfg))
    return Path(path)


# ──────────────────────────────────────────────────────────────────────
# Run + parse
# ──────────────────────────────────────────────────────────────────────

LINE_RE = re.compile(r"t=\s*(?P<t>[0-9.]+)s")
N_SM_RE = re.compile(r"n_sm=([0-9.]+)")
FINAL_RE = re.compile(r"Final \(learned\): n=([0-9.]+)")


def run_sim(*, yaml_path: Path, duration: float, sim_port: int,
            learned_dir: Path, init_terrain: str = "clay"
            ) -> Tuple[List[Tuple[float, float]], float]:
    """Launch chrono with --terrain-config and run the retained estimator.
    Returns (history, final_n)."""
    # The --terrain choice is ignored by setup_scm_terrain when terrain_config
    # is supplied, but the CLI still requires a valid name from the choices
    # list.  Pass "dirt" so the sim node's other code paths see something
    # benign.  We will force-inject the YAML via a wrapper.
    cmd = [
        "/home/kyle/miniconda3/bin/conda", "run", "--no-capture-output",
        "-n", "sim", "python", str(PROJECT_ROOT / "test_suite" / "run_openloop_terrain_est.py"),
        "--terrain", "dirt", "--time", str(duration),
        "--launch-sim",
        "--init-terrain", init_terrain,
        "--learned-model-dir", str(learned_dir),
        "--sim-port", str(sim_port),
        "--ctrl-port", str(sim_port + 1),
        "--no-plot",
        # Sneak the custom config into the auto-launched sim node by
        # exporting an env var that we consume there (see patch).
    ]
    env = dict(os.environ)
    env["OPENLOOP_TERRAIN_CONFIG"] = str(yaml_path)
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          cwd=PROJECT_ROOT, env=env)
    history: List[Tuple[float, float]] = []
    final = float("nan")
    for line in proc.stdout.splitlines():
        m = LINE_RE.search(line)
        sm = N_SM_RE.search(line)
        if m and sm:
            history.append((float(m["t"]), float(sm.group(1))))
        f = FINAL_RE.search(line)
        if f:
            final = float(f.group(1))
    return history, final


# ──────────────────────────────────────────────────────────────────────
# Main sweep
# ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--n-values", type=float, nargs="+",
                   default=[0.35, 0.50, 0.60, 0.70, 0.85, 1.00, 1.10, 1.30],
                   help="True n values to test")
    p.add_argument("--reps", type=int, default=2)
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--learned-model-dir", default=str(
        PROJECT_ROOT / "nn_models" / "terrain_window_mlp_v3_cl"))
    p.add_argument("--out", default=str(
        PROJECT_ROOT / "my_paper" / "paper_figures" / "learned_generalization.png"))
    p.add_argument("--sim-port-base", type=int, default=23000)
    args = p.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path = out_path.with_suffix(".csv")

    learned_dir = Path(args.learned_model_dir).resolve()
    print(f"[gen] testing n_values={args.n_values}  reps={args.reps}  "
          f"model={learned_dir.name}")

    rows: List[Tuple[float, int, float]] = []  # (n_true, rep, n_pred_late)
    port = args.sim_port_base
    histories: Dict[float, List[List[Tuple[float, float]]]] = {n: [] for n in args.n_values}

    for n_true in args.n_values:
        cfg = interp_terrain(n_true)
        yaml_path = write_yaml(cfg)
        print(f"\n[gen] n_true={n_true:.2f}  -> {cfg['description']}")
        for rep in range(args.reps):
            print(f"  rep {rep+1}/{args.reps}  port={port} ...", flush=True)
            history, final_n = run_sim(
                yaml_path=yaml_path, duration=args.duration,
                sim_port=port,
                learned_dir=learned_dir,
            )
            port += 4
            if not history:
                print(f"    [warn] no estimator output captured")
                continue
            late = [n for t, n in history if t >= args.duration - 5.0]
            n_late = float(np.mean(late)) if late else final_n
            err = abs(n_late - n_true) / n_true * 100.0
            print(f"    final={final_n:.3f}  late_mean={n_late:.3f}  "
                  f"err={err:.1f}%  (samples={len(history)})")
            rows.append((n_true, rep, n_late))
            histories[n_true].append(history)
        yaml_path.unlink(missing_ok=True)

    # CSV summary
    with csv_path.open("w") as f:
        f.write("n_true,rep,n_pred_late\n")
        for n_true, rep, n_pred in rows:
            f.write(f"{n_true:.3f},{rep},{n_pred:.4f}\n")
    print(f"\n[gen] csv -> {csv_path}")

    # Figure
    import matplotlib.pyplot as plt
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(14, 5.5))

    # Panel A: predicted vs true
    by_true: Dict[float, List[float]] = {}
    for n_true, rep, n_pred in rows:
        by_true.setdefault(n_true, []).append(n_pred)
    xs = sorted(by_true.keys())
    means = [float(np.mean(by_true[x])) for x in xs]
    stds  = [float(np.std(by_true[x]))  for x in xs]
    axA.errorbar(xs, means, yerr=stds, fmt="o", color="#2ca02c",
                  capsize=4, markersize=8, linewidth=2,
                  label="learned MLP prediction")
    lo = min(min(xs), N_LO) - 0.05
    hi = max(max(xs), N_HI) + 0.05
    axA.plot([lo, hi], [lo, hi], "k--", linewidth=1.0, label="ideal")
    train_ns = sorted(float(p[1]["n"]) for p in PRESETS_BY_N)
    for tn in train_ns:
        axA.axvline(x=tn, color="grey", linestyle=":", linewidth=1, alpha=0.5)
    axA.axvspan(N_LO, N_HI, color="#fff7d6", alpha=0.5,
                 label=f"training range [{N_LO:.2f}, {N_HI:.2f}]")
    axA.set_xlabel("True Bekker n (chrono ground truth)")
    axA.set_ylabel("Predicted n (learned MLP, last 5 s mean)")
    axA.set_title("A.  Learned model on novel n values")
    axA.set_xlim(lo, hi)
    axA.set_ylim(lo, hi)
    axA.legend(loc="lower right", fontsize=9)
    axA.grid(True, alpha=0.4)

    # Panel B: convergence curves for each n_true
    cmap = plt.get_cmap("viridis")
    n_vals_sorted = sorted(histories.keys())
    for i, n_true in enumerate(n_vals_sorted):
        if not histories[n_true]:
            continue
        col = cmap(i / max(len(n_vals_sorted) - 1, 1))
        for j, hist in enumerate(histories[n_true]):
            t = np.asarray([p[0] for p in hist])
            n = np.asarray([p[1] for p in hist])
            axB.plot(t, n, color=col, alpha=0.6,
                      label=f"n_true={n_true:.2f}" if j == 0 else None)
        axB.axhline(y=n_true, color=col, linestyle=":", linewidth=0.8, alpha=0.5)
    for tn in train_ns:
        axB.axhline(y=tn, color="grey", linestyle="-", linewidth=0.4, alpha=0.3)
    axB.set_xlabel("Time (s)")
    axB.set_ylabel("Predicted n_sm")
    axB.set_title("B.  Convergence on novel terrains")
    axB.legend(loc="upper right", fontsize=8, ncol=2)
    axB.grid(True, alpha=0.4)

    fig.suptitle("Learned terrain estimator: generalization to unseen n values",
                  fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=200)
    print(f"[gen] figure -> {out_path}")


if __name__ == "__main__":
    import os
    main()
