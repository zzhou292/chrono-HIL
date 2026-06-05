#!/usr/bin/env python3
"""4-line force-prediction time series per scenario:

  * Chrono ground truth
  * Vehicle NN (rate, deployed)
  * Rig NN (rate, 64-32)
  * Rig NN (static, 32-16)   ← restored from archive

Both rig variants evaluated offline on the same recorded trajectory
(the diag CSVs from the vehicle_rate_lhs runs).
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "simulation"))

from nn_tire_model import load_nn_tire_model   # noqa: E402
from param_consistency import (                # noqa: E402
    TERRAIN_PRESETS, terrain_preset_to_internal,
)

OUT_DIR = REPO / "my_paper" / "paper_figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)

ARCHIVE_NN = (REPO / "archive" /
              "2026-05-23_model_checkpoint_and_root_artifact_cleanup" /
              "nn_models_new_retired")
RIG_STATIC_DIR = ARCHIVE_NN / "rig_static_32_16"
RIG_RATE_DIR   = REPO / "nn_models" / "rig_rate_64_32"
VEH_STATIC_DIR = ARCHIVE_NN / "vehicle_static_32_16_lhs"
VEH_RATE_DIR   = REPO / "nn_models" / "vehicle_rate_64_32_lhs"

SCENARIOS = [
    ("sand", "sinusoidal",  5, 0, 900),
    ("sand", "sinusoidal",  7, 0, 900),
    ("sand", "right_left",  7, 0, 900),
    ("sand", "lane_change", 5, 0, 900),
    ("sand", "sinusoidal",  7, 4, 900),
    ("clay", "sinusoidal",  5, 0, 900),
    ("clay", "sinusoidal",  7, 0, 900),
    ("clay", "right_left",  7, 0, 900),
    ("clay", "sinusoidal",  7, 4, 900),
    ("dirt", "sinusoidal",  5, 0, 900),
    ("dirt", "sinusoidal",  7, 0, 900),
    ("dirt", "right_left",  7, 0, 900),
]


def _find_diag(variant, terrain, path, v, b, seed):
    pat = f"{variant}_{terrain}_{path}_v{v}_b{b}_s{seed}"
    # Prefer the freshly reproduced rig-vs-vehicle sweep; fall back to archived
    # runs for any variant not in the fresh set (e.g. static-model rows).
    roots = (sorted((REPO / "benchmarking" / "results").glob(
                 "rig_vs_vehicle_tire_sweep_*/raw"), reverse=True)
             + list(REPO.glob("archive/**/rig_vs_vehicle_tire_sweep_*/raw")))
    for root in roots:
        for run_dir in root.iterdir():
            if not run_dir.is_dir():
                continue
            if pat in run_dir.name:
                csvs = sorted(run_dir.rglob("diag_*.csv"))
                if csvs:
                    return csvs[0]
    return None


def _eval_nn(model, df: pd.DataFrame):
    alpha = df["alpha_f"].to_numpy()
    Fz = df["Fz_f_mean"].to_numpy()
    u = (df["u_meas"].to_numpy() if "u_meas" in df.columns
         else df["u_true"].to_numpy())
    kappa = (df["kappa_diag"].to_numpy() if "kappa_diag" in df.columns
             else np.zeros_like(alpha))
    pred = np.empty_like(alpha)
    for i in range(len(alpha)):
        try:
            _Fx, Fy_wheel = model.predict_numeric(
                float(alpha[i]), float(Fz[i]),
                float(max(u[i], 0.5)), float(kappa[i]),
            )
            pred[i] = -2.0 * Fy_wheel
        except Exception:
            pred[i] = float("nan")
    return pred


def _rmse(p, a):
    m = np.isfinite(p) & np.isfinite(a)
    return float(np.sqrt(np.mean((p[m] - a[m]) ** 2))) if m.any() else float("nan")


def render_one(terr, path, v, b, seed,
                rig_rate, rig_static, veh_rate, veh_static,
                t_start=2.0, t_end=12.0):
    diag = _find_diag("vehicle_rate_lhs", terr, path, v, b, seed)
    if diag is None:
        return None
    df = pd.read_csv(diag)
    df = df[(df["sim_time"] >= t_start) & (df["sim_time"] <= t_end)]
    if df.empty:
        return None
    t = df["sim_time"].to_numpy()
    actual = df["actual_Fy_front"].to_numpy()
    veh_rate_pred = df["pred_Fy_front"].to_numpy()        # logged (rate, deployed)
    veh_stat_pred = _eval_nn(veh_static, df)              # off-line eval
    rig_rate_pred = _eval_nn(rig_rate, df)
    rig_stat_pred = _eval_nn(rig_static, df)

    fig, ax = plt.subplots(figsize=(12.0, 5.4))
    ax.plot(t, actual,         color="#222",    lw=1.5,
            label="Chrono ground truth", zorder=5)
    ax.plot(t, veh_rate_pred,  color="#1f78b4", lw=1.3, alpha=0.95,
            label=f"Vehicle rate    (RMSE = {_rmse(veh_rate_pred, actual):.0f} N)",
            zorder=4)
    ax.plot(t, veh_stat_pred,  color="#0a3d62", lw=1.3, alpha=0.95, ls=":",
            label=f"Vehicle static  (RMSE = {_rmse(veh_stat_pred, actual):.0f} N)",
            zorder=4)
    ax.plot(t, rig_rate_pred,  color="#d95f02", lw=1.3, alpha=0.95,
            label=f"Rig     rate    (RMSE = {_rmse(rig_rate_pred, actual):.0f} N)",
            zorder=4)
    ax.plot(t, rig_stat_pred,  color="#7e3c8a", lw=1.3, alpha=0.95, ls="--",
            label=f"Rig     static  (RMSE = {_rmse(rig_stat_pred, actual):.0f} N)",
            zorder=4)

    ax.set_xlabel("Sim time (s)", fontsize=11)
    ax.set_ylabel("Front-axle lateral tire force  $F_y$  (N)", fontsize=11)
    ax.set_title(f"{terr} / {path} / v_ref = {v} m/s / bumpiness = {b}  "
                 f"(seed {seed})  —  rig × {{static, rate}} × vehicle",
                 fontsize=11)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
    fig.tight_layout()
    out = OUT_DIR / f"fig1_force_4way_{terr}_{path}_v{v}_b{b}_s{seed}.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return (out,
            _rmse(veh_rate_pred, actual), _rmse(veh_stat_pred, actual),
            _rmse(rig_rate_pred, actual), _rmse(rig_stat_pred, actual))


def main():
    rig_rate, rig_static, veh_rate, veh_static = {}, {}, {}, {}
    for terr in ("clay", "dirt", "sand"):
        tp = terrain_preset_to_internal(TERRAIN_PRESETS[terr])
        rig_rate[terr]   = load_nn_tire_model(RIG_RATE_DIR,   terrain_params=tp)
        rig_static[terr] = load_nn_tire_model(RIG_STATIC_DIR, terrain_params=tp)
        veh_rate[terr]   = load_nn_tire_model(VEH_RATE_DIR,   terrain_params=tp)
        veh_static[terr] = load_nn_tire_model(VEH_STATIC_DIR, terrain_params=tp)

    print(f"{'scenario':<48s} {'veh.r':>7s} {'veh.s':>7s} {'rig.r':>7s} "
          f"{'rig.s':>7s}")
    print("=" * 92)
    for (terr, path, v, b, seed) in SCENARIOS:
        result = render_one(terr, path, v, b, seed,
                             rig_rate[terr], rig_static[terr],
                             veh_rate[terr], veh_static[terr])
        if result is None:
            continue
        out, vr, vs, rr, rs = result
        scen = f"{terr}/{path}/v{v}/b{b}/s{seed}"
        winners = sorted([("veh.rate", vr), ("veh.static", vs),
                          ("rig.rate", rr), ("rig.static", rs)],
                         key=lambda x: x[1])
        print(f"{scen:<48s} {vr:>6.0f}N {vs:>6.0f}N {rr:>6.0f}N {rs:>6.0f}N "
              f"  → best: {winners[0][0]}")


if __name__ == "__main__":
    main()
