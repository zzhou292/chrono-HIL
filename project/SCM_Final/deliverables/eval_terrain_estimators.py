#!/usr/bin/env python3
"""eval_terrain_estimators.py
==============================

Compare three terrain-estimation pipelines on the same Chrono SCM
ground-truth logs:

1. **Bekker-UKF**: Dallas-style state-augmented UKF with a 4-wheel
   double-track bicycle and analytical Bekker tire forces.
2. **NN-UKF**: same UKF but with the paper118-spec neural-network tire
   surrogate (uniform LHS over the SCM input box; widened Fz).
3. **Learned window MLP**: the proprioceptive sliding-window MLP at
   ``nn_models/terrain_window_mlp/`` (Buzhardt-style direct
   regression on IMU + wheel-encoder + steering window features —
   no force model in the loop).

Each estimator is fed the SAME Chrono SCM log
(``data/dallas_scm/{clay, sandy_loam, sand}.npz``) and the run produces:

* ``my_paper/paper_figures/terrain_estimator_comparison.png`` — 3-panel
  figure (Clay, Sandy loam, Sand) overlaying the three estimators
  against the true ``n``.
* ``my_paper/paper_figures/terrain_estimator_comparison.csv`` — final
  converged ``n`` and %-error per (terrain, estimator) cell.

Usage::

    conda activate sim
    python deliverables/eval_terrain_estimators.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deliverables"))
sys.path.insert(0, str(ROOT / "simulation"))

from ukf_paper_validation import (  # noqa: E402
    DALLAS_SCENARIOS, DallasScenario, SoilParams, run_dallas_from_log,
    Vehicle, _H_CG, WHEEL,
)
from learned_terrain_estimator import LearnedTerrainEstimator  # noqa: E402


SOIL_SAND = SoilParams(kc=900, kphi=1523400, n=1.10,
                        c=1000, phi=math.radians(30),
                        kx=0.025, ky=0.025)

SCENARIOS: List[Tuple[DallasScenario, Path]] = [
    (DALLAS_SCENARIOS[0], ROOT / "data/dallas_scm/clay.npz"),
    (DALLAS_SCENARIOS[1], ROOT / "data/dallas_scm/sandy_loam.npz"),
    (DallasScenario(label="Sand (true n=1.10, init=0.70)",
                     soil=SOIL_SAND, n_true=1.10, n_init=0.70),
     ROOT / "data/dallas_scm/sand.npz"),
]


def _run_learned_estimator(log_path: Path, n_init: float
                           ) -> Tuple[np.ndarray, np.ndarray, float]:
    """Replay the sliding-window MLP estimator against a Chrono SCM log."""
    model_dir = ROOT / "nn_models" / "terrain_window_mlp"
    est = LearnedTerrainEstimator(
        model_dir=str(model_dir),
        initial_terrain={"n": n_init},
        update_interval=1,
        verbose=False,
    )

    data = np.load(str(log_path))
    lead = float(data["lead_in"][0])
    mask = data["t"] >= lead
    t = data["t"][mask] - lead
    u = data["u"][mask]; v = data["v"][mask]; om = data["omega"][mask]
    ax_imu = data["ax"][mask]; ay_imu = data["ay"][mask]
    az_imu = data["az"][mask]
    roll_rate = data["roll_rate"][mask]; pitch_rate = data["pitch_rate"][mask]
    w_fl = data["w_fl"][mask]; w_fr = data["w_fr"][mask]
    w_rl = data["w_rl"][mask]; w_rr = data["w_rr"][mask]
    delta = data["delta_meas"][mask]; throttle = data["throttle_cmd"][mask]

    veh = Vehicle()
    L = veh.Lf + veh.Lr
    Fz_f_static = veh.m * veh.g * veh.Lr / L
    Fz_r_static = veh.m * veh.g * veh.Lf / L

    n_log: List[float] = [n_init]
    t_log: List[float] = [float(t[0])]
    for k in range(1, len(t)):
        u_safe = max(abs(float(u[k])), 0.5)
        alpha_f = (float(delta[k])
                   - math.atan2(float(v[k]) + veh.Lf * float(om[k]), u_safe))
        alpha_r = -math.atan2(float(v[k]) - veh.Lr * float(om[k]), u_safe)
        # Slip ratio approximated from wheel-encoder average.
        w_avg = 0.25 * float(w_fl[k] + w_fr[k] + w_rl[k] + w_rr[k])
        kappa = w_avg * WHEEL.r / u_safe - 1.0
        sr = 0.0   # steering rate not used by the estimator's MLP head
        omega_dot = (float(om[k]) - float(om[k - 1])) / max(
            float(t[k] - t[k - 1]), 1e-3)
        est.observe(
            kappa=kappa,
            alpha_f=alpha_f, alpha_r=alpha_r,
            u=float(u[k]),
            Fz_f=Fz_f_static, Fz_r=Fz_r_static,
            sr=sr,
            ay_imu=float(ay_imu[k]),
            omega_dot=omega_dot,
            omega=float(om[k]),
            v_lateral=float(v[k]),
            ax_imu=float(ax_imu[k]),
            az_imu=float(az_imu[k]),
            roll_rate=float(roll_rate[k]),
            pitch_rate=float(pitch_rate[k]),
            throttle_cmd=float(throttle[k]),
            sim_time=float(t[k]),
            wheel_omegas=(float(w_fl[k]), float(w_fr[k]),
                          float(w_rl[k]), float(w_rr[k])),
        )
        n_log.append(est.get_bekker_n())
        t_log.append(float(t[k]))

    t_arr = np.asarray(t_log); n_arr = np.asarray(n_log)
    T = t_arr[-1]
    converged = float(np.mean(n_arr[t_arr >= T * 0.75]))
    return t_arr, n_arr, converged


def main() -> int:
    out_dir = ROOT / "my_paper" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_path = out_dir / "terrain_estimator_comparison.png"
    csv_path = out_dir / "terrain_estimator_comparison.csv"

    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.6), sharey=False)
    rows: List[Dict[str, float]] = []

    for ax, (sc, log_path) in zip(axes, SCENARIOS):
        print(f"\n=== {sc.label}  log={log_path.name} ===")

        # 1. Bekker-UKF
        t_b, n_b, pct_b = run_dallas_from_log(log_path, sc, backend="bekker")
        conv_b = float(np.mean(n_b[t_b >= t_b[-1] * 0.75]))
        rows.append({"terrain": sc.label, "estimator": "Bekker-UKF",
                     "converged_n": conv_b, "n_true": sc.n_true,
                     "pct_err": pct_b})

        # 2. NN-UKF (whole-vehicle Fy surrogate, no calibration)
        t_n, n_n, pct_n = run_dallas_from_log(log_path, sc,
                                                backend="vehicle_fy")
        conv_n = float(np.mean(n_n[t_n >= t_n[-1] * 0.75]))
        rows.append({"terrain": sc.label, "estimator": "NN-UKF",
                     "converged_n": conv_n, "n_true": sc.n_true,
                     "pct_err": pct_n})

        # 3. Learned window MLP (Buzhardt-style direct regression)
        t_l, n_l, conv_l = _run_learned_estimator(log_path, sc.n_init)
        pct_l = 100.0 * abs(conv_l - sc.n_true) / sc.n_true
        rows.append({"terrain": sc.label, "estimator": "Learned MLP",
                     "converged_n": conv_l, "n_true": sc.n_true,
                     "pct_err": pct_l})

        print(f"  Bekker-UKF   converged n = {conv_b:.4f}  err = {pct_b:5.2f}%")
        print(f"  NN-UKF       converged n = {conv_n:.4f}  err = {pct_n:5.2f}%")
        print(f"  Learned MLP  converged n = {conv_l:.4f}  err = {pct_l:5.2f}%")

        ax.plot(t_b, n_b, color="#1f77b4", lw=1.3,
                label=f"Bekker-UKF ({pct_b:.1f}%)")
        ax.plot(t_n, n_n, color="#2ca02c", lw=1.3,
                label=f"NN-UKF ({pct_n:.1f}%)")
        ax.plot(t_l, n_l, color="#d62728", lw=1.3,
                label=f"Learned MLP ({pct_l:.1f}%)")
        ax.axhline(sc.n_true, color="k", lw=1.0, ls="-", label="True n")
        ax.axhline(sc.n_true * 1.10, color="k", lw=0.8, ls="--",
                   label="±10 % band")
        ax.axhline(sc.n_true * 0.90, color="k", lw=0.8, ls="--")
        ax.set_xlim(0.0, t_b[-1])
        n_min = min(sc.n_true, sc.n_init, 0.3) - 0.05
        n_max = max(sc.n_true, sc.n_init, 0.9) + 0.10
        ax.set_ylim(n_min, n_max)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("n  (sinkage exponent)")
        ax.set_title(sc.label, fontsize=10.5)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8.5, framealpha=0.92)

    fig.suptitle(
        "Terrain estimator comparison on Chrono SCM ground truth"
        "  (state-augmented UKF, Bekker vs NN tyre, vs proprioceptive window MLP)",
        fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180, bbox_inches="tight")
    print(f"\nWrote {fig_path}")

    df = pd.DataFrame(rows)
    df.to_csv(csv_path, index=False)
    print(f"Wrote {csv_path}")
    print("\nSummary:")
    print(df.to_string(index=False,
                       float_format=lambda x: f"{x:.4f}"
                       if isinstance(x, float) else str(x)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
