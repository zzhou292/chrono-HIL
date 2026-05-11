"""Benchmark tire models (Pacejka, TMeasy, MLP, ResNet, Rate-MLP, Rate-ResNet)
under closed-loop ACADOS NMPC across canonical terrains and reference paths.

Each (model, terrain, path) cell is repeated ``N_REPEATS`` times and the
crosstrack RMSE (after ``T_WARMUP`` s) is aggregated as mean / std.

Outputs
-------
* ``paper_figures/bench_tire_models.csv``   per-run RMSE table
* ``paper_figures/bench_tire_models_summary.csv`` aggregated mean ± std cells
* ``paper_figures/bench_tire_models.png``   compact heat-map matrix figure

Run with::

    python utilities/bench_tire_models.py --workers 4
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

# ─────────────────────────────────────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent          # SCM_Teleop/
SIM_DIR = ROOT / "simulation"
NN_DIR = ROOT / "nn_models"
FIG_DIR = ROOT / "my_paper" / "paper_figures"
PLOTS_DIR = ROOT / "plots" / "bench_tire_models"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)
FIG_DIR.mkdir(parents=True, exist_ok=True)

BASE_PORT = 6300

# ─────────────────────────────────────────────────────────────────────────────
# Sweep configuration
# ─────────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ModelSpec:
    label: str             # short display label (row title in figure)
    cli_args: tuple        # extra args appended to launch_decoupled.py


MODELS: List[ModelSpec] = [
    ModelSpec("Pacejka",     ("--model", "pacejka")),
    ModelSpec("TMeasy",      ("--model", "tmeasy")),
    ModelSpec("MLP",         ("--model", "nn", "--nn-model", "paper_v2_mlp_16_4",
                              "--no-symbolic-rates")),
    ModelSpec("ResNet",      ("--model", "nn", "--nn-model", "paper_v2_resnet_h16_b2",
                              "--no-symbolic-rates")),
    ModelSpec("Rate-MLP",    ("--model", "nn", "--nn-model", "paper_v2_mlp_rate_16_4")),
    ModelSpec("Rate-ResNet", ("--model", "nn", "--nn-model", "paper_v2_resnet_rate_h16_b2")),
]

TERRAINS = ["clay", "dirt", "sand"]
PATHS    = ["lane_change", "sinusoidal"]
SPEED    = 5.0
SIM_TIME = 18.0     # seconds of sim time per run
T_WARMUP = 2.0      # ignore first 2 s when computing RMSE
N_REPEATS_DEFAULT = 5
RUN_TIMEOUT = 240   # wall-clock seconds per run


# ─────────────────────────────────────────────────────────────────────────────
# Job spec
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Job:
    idx: int
    model: ModelSpec
    terrain: str
    path: str
    repeat: int
    sim_port: int
    ctrl_port: int

    @property
    def tag(self) -> str:
        m = self.model.label.lower().replace("-", "_")
        return f"{m}_{self.terrain}_{self.path}_r{self.repeat}"


# ─────────────────────────────────────────────────────────────────────────────
# Single-run launcher
# ─────────────────────────────────────────────────────────────────────────────
def _run_one(job: Job) -> Dict:
    plot_dir = PLOTS_DIR / job.tag
    plot_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(SIM_DIR / "launch_decoupled.py"),
        "--terrain",     job.terrain,
        "--path",        job.path,
        "--speed",       str(SPEED),
        "--time",        str(SIM_TIME),
        "--no-vis",
        "--no-plot",
        "--no-imu",                      # use analytical accel (avoid Chrono-Sensor licence/Vulkan deps in batch)
        "--sim-port",    str(job.sim_port),
        "--ctrl-port",   str(job.ctrl_port),
        "--plot-dir",    str(plot_dir),
    ]
    if job.path == "sinusoidal":
        cmd += ["--sine-amplitude", "2.0", "--sine-wavelength", "30.0"]
    cmd += list(job.model.cli_args)

    t0 = time.time()
    err: Optional[str] = None
    try:
        proc = subprocess.run(cmd, timeout=RUN_TIMEOUT, cwd=str(SIM_DIR),
                              capture_output=True, text=True)
        wall = time.time() - t0
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or "")[-400:]
    except subprocess.TimeoutExpired:
        wall = time.time() - t0
        err = "timeout"
        proc = None

    diag_csvs = sorted(plot_dir.glob("**/diag_*.csv"))
    rms = np.nan
    rms_speed = np.nan
    speed_bias = np.nan
    throttle_sat_frac = np.nan
    n_solves = 0
    mean_speed = np.nan

    if diag_csvs:
        try:
            df = pd.read_csv(diag_csvs[-1])
            n_solves = len(df)
            ss = df[df["sim_time"] >= T_WARMUP]
            if len(ss) and "crosstrack_err" in ss.columns:
                cte = ss["crosstrack_err"].abs().to_numpy(float)
                cte = cte[np.isfinite(cte)]
                if len(cte):
                    rms = float(np.sqrt(np.mean(cte ** 2)))
            if "u_meas" in df.columns:
                mean_speed = float(ss["u_meas"].mean()) if len(ss) else np.nan
            # Longitudinal tracking: speed_err is (u_meas - v_ref) recorded
            # by the controller every stage; fall back to recomputing from
            # u_meas/v_ref_0 if missing.
            if len(ss):
                if "speed_err" in ss.columns:
                    se = ss["speed_err"].to_numpy(float)
                elif "u_meas" in ss.columns and "v_ref_0" in ss.columns:
                    se = (ss["u_meas"] - ss["v_ref_0"]).to_numpy(float)
                else:
                    se = np.array([])
                se = se[np.isfinite(se)]
                if len(se):
                    rms_speed = float(np.sqrt(np.mean(se ** 2)))
                    speed_bias = float(np.mean(se))
                if "throttle" in ss.columns:
                    th = ss["throttle"].to_numpy(float)
                    throttle_sat_frac = float(np.mean(th >= 0.99))
        except Exception as e:
            err = (err or "") + f"|csv:{e}"

    return {
        "idx":              job.idx,
        "model":            job.model.label,
        "terrain":          job.terrain,
        "path":             job.path,
        "repeat":           job.repeat,
        "rms_cte_m":        rms,
        "rms_speed_mps":    rms_speed,
        "speed_bias_mps":   speed_bias,
        "throttle_sat_frac": throttle_sat_frac,
        "n_solves":         n_solves,
        "mean_speed":       mean_speed,
        "wall_s":           round(wall, 1),
        "error":            err,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Sweep driver
# ─────────────────────────────────────────────────────────────────────────────
def build_jobs(n_repeats: int) -> List[Job]:
    jobs: List[Job] = []
    idx = 0
    for model in MODELS:
        for terrain in TERRAINS:
            for path in PATHS:
                for r in range(n_repeats):
                    port = BASE_PORT + idx * 2
                    jobs.append(Job(
                        idx=idx,
                        model=model,
                        terrain=terrain,
                        path=path,
                        repeat=r,
                        sim_port=port,
                        ctrl_port=port + 1,
                    ))
                    idx += 1
    return jobs


def run_sweep(jobs: List[Job], workers: int) -> pd.DataFrame:
    print(f"[bench] Submitting {len(jobs)} jobs with {workers} workers")
    t0 = time.time()
    rows: List[Dict] = []
    with ThreadPoolExecutor(max_workers=workers) as ex:
        futures = {ex.submit(_run_one, j): j for j in jobs}
        for fut in as_completed(futures):
            j = futures[fut]
            try:
                row = fut.result()
            except Exception as e:
                row = {
                    "idx": j.idx, "model": j.model.label, "terrain": j.terrain,
                    "path": j.path, "repeat": j.repeat,
                    "rms_cte_m": float("nan"),
                    "rms_speed_mps": float("nan"),
                    "speed_bias_mps": float("nan"),
                    "throttle_sat_frac": float("nan"),
                    "n_solves": 0, "mean_speed": float("nan"),
                    "wall_s": -1, "error": f"runner:{e}",
                }
            elapsed = time.time() - t0
            done = len(rows) + 1
            print(f"[bench] {done:3d}/{len(jobs)}  "
                  f"{row['model']:<11s}  {row['terrain']:<5s}  "
                  f"{row['path']:<18s}  r{row['repeat']}  "
                  f"rms={row['rms_cte_m']:.3f}m  "
                  f"({row['wall_s']:.0f}s)  total={elapsed/60:.1f}min  "
                  f"{('ERR='+str(row['error'])[:60]) if row['error'] else ''}",
                  flush=True)
            rows.append(row)
    return pd.DataFrame(rows).sort_values("idx").reset_index(drop=True)


# ─────────────────────────────────────────────────────────────────────────────
# Aggregation + plotting
# ─────────────────────────────────────────────────────────────────────────────
def aggregate(df: pd.DataFrame) -> pd.DataFrame:
    # Backward compatibility: silently drop the legacy 'long_coupling'
    # column from older CSVs (we no longer sweep over coupling modes).
    if "long_coupling" in df.columns:
        df = df.drop(columns=["long_coupling"])
    keys = ["model", "terrain", "path"]
    g = df.groupby(keys)["rms_cte_m"].agg(["mean", "std", "count"]).reset_index()
    out = g.rename(columns={"mean": "rms_cte_mean_m",
                            "std":  "rms_cte_std_m",
                            "count": "n_runs"})
    # Longitudinal tracking metrics (speed RMSE / bias / throttle saturation).
    for col, alias in [("rms_speed_mps", "rms_speed_mean_mps"),
                       ("rms_speed_mps", "rms_speed_std_mps"),
                       ("speed_bias_mps", "speed_bias_mean_mps"),
                       ("throttle_sat_frac", "throttle_sat_mean_frac"),
                       ("mean_speed", "speed_mean_mps")]:
        if col not in df.columns:
            continue
        if alias.endswith("_std_mps"):
            agg = df.groupby(keys)[col].std().reset_index()
        else:
            agg = df.groupby(keys)[col].mean().reset_index()
        agg = agg.rename(columns={col: alias})
        out = out.merge(agg, on=keys, how="left")
    return out


def plot_matrix(summary: pd.DataFrame, out_png: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm

    if not len(summary):
        print("[bench] no rows; skipping figure")
        return

    model_order = [m.label for m in MODELS]
    cols = [(t, p) for t in TERRAINS for p in PATHS]
    short_path = {"lane_change": "LC", "sinusoidal": "Sine",
                  "double_lane_change": "DLC", "right_left": "RL"}

    M = np.full((len(model_order), len(cols)), np.nan)
    S = np.full_like(M, np.nan)
    for i, m in enumerate(model_order):
        for j, (t, p) in enumerate(cols):
            sel = summary[(summary["model"] == m) &
                          (summary["terrain"] == t) &
                          (summary["path"] == p)]
            if len(sel):
                M[i, j] = float(sel["rms_cte_mean_m"].iloc[0])
                S[i, j] = float(sel["rms_cte_std_m"].iloc[0])

    finite = M[np.isfinite(M)]
    if not finite.size:
        print("[bench] WARNING: no finite RMSE values; skipping figure")
        return
    vmin = max(1e-3, float(np.nanmin(finite)))
    vmax = max(vmin * 1.5, float(np.nanmax(finite)))

    fig_h = 0.45 * len(model_order) + 1.4
    fig_w = 0.95 * len(cols) + 2.2
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    cmap = plt.get_cmap("RdYlGn_r")
    masked = np.ma.masked_invalid(M)
    im = ax.imshow(masked, aspect="auto", cmap=cmap,
                   norm=LogNorm(vmin=vmin, vmax=vmax))

    ax.set_xticks(range(len(cols)))
    ax.set_xticklabels([f"{t}\n{short_path[p]}" for (t, p) in cols], fontsize=8)
    ax.set_yticks(range(len(model_order)))
    ax.set_yticklabels(model_order, fontsize=9)

    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            if np.isfinite(M[i, j]):
                v = M[i, j]
                s = S[i, j] if np.isfinite(S[i, j]) else 0.0
                txt = f"{v:.2f}\n±{s:.2f}"
                ax.text(j, i, txt, ha="center", va="center",
                        fontsize=7, color="black")
            else:
                ax.text(j, i, "—", ha="center", va="center",
                        fontsize=9, color="black")

    cbar = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
    cbar.set_label("Crosstrack RMSE (m)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    ax.set_title(f"Closed-loop NMPC tracking: tire model × terrain × path "
                 f"(mean ± std over {int(summary['n_runs'].max())} runs)",
                 fontsize=9)
    ax.tick_params(top=False, bottom=False, left=False, right=False)
    for spine in ax.spines.values():
        spine.set_visible(False)

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    print(f"[bench] wrote {out_png}")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--repeats", type=int, default=N_REPEATS_DEFAULT)
    p.add_argument("--models", nargs="*", default=None,
                   help="Subset of model labels to run (default: all)")
    p.add_argument("--replot-only", action="store_true",
                   help="Skip simulation; re-aggregate and re-plot from existing CSV")
    args = p.parse_args()

    if args.models:
        global MODELS
        keep = set(args.models)
        MODELS = [m for m in MODELS if m.label in keep]
        if not MODELS:
            print(f"[bench] no models matched {args.models}", file=sys.stderr)
            return 2

    raw_csv = FIG_DIR / "bench_tire_models.csv"
    summary_csv = FIG_DIR / "bench_tire_models_summary.csv"

    if args.replot_only:
        if not raw_csv.exists():
            print(f"[bench] no existing {raw_csv} to replot", file=sys.stderr)
            return 1
        df = pd.read_csv(raw_csv)
    else:
        jobs = build_jobs(args.repeats)
        df = run_sweep(jobs, args.workers)
        df.to_csv(raw_csv, index=False)
        print(f"[bench] wrote raw CSV {raw_csv}  ({len(df)} rows)")

    summary = aggregate(df)
    summary.to_csv(summary_csv, index=False)
    print(f"[bench] wrote summary CSV {summary_csv}")

    plot_matrix(summary, FIG_DIR / "bench_tire_models.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
