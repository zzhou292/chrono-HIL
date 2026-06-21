#!/usr/bin/env python3
"""Solve-time scaling of the safety filters vs. obstacle count (and MPPI K).

Standalone micro-benchmark -- no Chrono. Instantiates each filter and times
filter() against synthetic obstacle sets of growing size N, to find where the
per-step solve time crosses the safety-filter real-time budget (the shield
runs at ~10 Hz, so 100 ms/solve). To the filter a rock and a dynamic vehicle
are the same thing -- an (x, y, r) entry -- so N bounds "rocks + cars within
sensor range" jointly.

Outputs results.csv, a solve-time-vs-N figure, and a summary with each
filter's capacity (largest N whose p95 solve stays within budget).

  python safety_filter_scaling.py                 # dob_cbf + mppi, N sweep + K sweep
  python safety_filter_scaling.py --include-nmpc   # also the SLSQP NMPC shield
"""
from __future__ import annotations

import os
# Pin BLAS to 1 thread (matches the per-run sweep env; the MPPI rollout is many
# tiny matmuls that oversubscribe a multi-thread pool -- see CLAUDE.md).
for _v in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from param_consistency import (  # noqa: E402
    get_vehicle_params_for_demo, get_terrain_preset, terrain_preset_to_internal)
from nn_tire_model import load_nn_tire_model  # noqa: E402
from safety import make_safety_filter  # noqa: E402
from common import timestamped_result_dir, write_manifest, save_summary_markdown  # noqa: E402

BUDGET_MS = 100.0   # 10 Hz safety-filter budget
VEHICLE_STATE = dict(x=0.0, y=0.0, psi=0.0, u=4.0, v=0.0, omega=0.0, delta=0.0,
                     ax=0.0, ay=0.0)


def build_filter(flavor: str, nn, vp, tp, horizon: int, mppi_samples: int):
    kw = dict(vehicle_params=vp, nn_model=nn, terrain_params=tp)
    if flavor == "mppi":
        kw.update(horizon=horizon, n_samples=mppi_samples)
    elif flavor == "nmpc":
        kw.update(horizon=horizon, n_iter=6)
    else:
        kw.update(cbf_alpha=1.0, obstacle_buffer=0.25)
    f = make_safety_filter(flavor, **kw)
    if hasattr(f, "update_terrain"):
        try:
            f.update_terrain(tp)
        except Exception:
            pass
    return f


def make_obstacles(n: int, rng: np.random.Generator) -> list[tuple[float, float, float]]:
    """N obstacles packed AHEAD within sensor range (worst case: all count)."""
    obs = []
    for i in range(n):
        x = 6.0 + (i % 16) * 1.5 + rng.uniform(-0.3, 0.3)   # 6..~30 m ahead
        y = rng.uniform(-4.0, 4.0)
        obs.append((x, y, 1.5))
    return obs


def time_calls(f, obstacles, reps: int) -> tuple[float, float, float]:
    for _ in range(5):                       # warmup (first calls JIT/codegen)
        f.filter(0.0, 0.5, 0.0, VEHICLE_STATE, obstacles)
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        f.filter(0.0, 0.5, 0.0, VEHICLE_STATE, obstacles)
        ts.append((time.perf_counter() - t0) * 1000.0)
    a = np.asarray(ts)
    return float(a.mean()), float(np.percentile(a, 95)), float(a.max())


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--filters", nargs="+", default=["dob_cbf", "mppi"],
                   choices=["dob_cbf", "mppi", "nmpc"])
    p.add_argument("--include-nmpc", action="store_true", help="add the SLSQP NMPC shield")
    p.add_argument("--n-obstacles", nargs="+", type=int,
                   default=[0, 1, 2, 4, 8, 16, 32, 64, 128])
    p.add_argument("--mppi-samples", nargs="+", type=int,
                   default=[128, 256, 384, 512, 768, 1024])
    p.add_argument("--k-sweep-n", type=int, default=8, help="obstacle count for the MPPI-K sweep")
    p.add_argument("--horizon", type=int, default=12)
    p.add_argument("--default-samples", type=int, default=384)
    p.add_argument("--reps", type=int, default=40)
    p.add_argument("--terrain", default="clay")
    args = p.parse_args()
    filters = list(dict.fromkeys(args.filters + (["nmpc"] if args.include_nmpc else [])))

    out_dir = timestamped_result_dir("safety_filter_scaling")
    write_manifest(out_dir, args, "Safety-filter solve-time scaling vs obstacle count.")
    print(f"Output: {out_dir}")

    vp = get_vehicle_params_for_demo()
    tp = terrain_preset_to_internal(get_terrain_preset(args.terrain))
    nn = load_nn_tire_model(str(ROOT / "nn_models" / "vehicle_rate_64_32_lhs"), tp)
    rng = np.random.default_rng(0)

    rows = []
    # (1) Obstacle-count sweep, default settings.
    for flavor in filters:
        f = build_filter(flavor, nn, vp, tp, args.horizon, args.default_samples)
        for n in args.n_obstacles:
            obs = make_obstacles(n, rng)
            mean, p95, mx = time_calls(f, obs, args.reps)
            rows.append({"sweep": "obstacles", "filter": flavor, "n_obstacles": n,
                         "mppi_samples": args.default_samples if flavor == "mppi" else "",
                         "mean_ms": round(mean, 2), "p95_ms": round(p95, 2), "max_ms": round(mx, 2),
                         "over_budget": int(p95 > BUDGET_MS)})
            print(f"  [obstacles] {flavor:8s} N={n:4d}  mean={mean:6.2f}  p95={p95:6.2f}  max={mx:6.2f} ms")
    # (2) MPPI sample-count sweep at fixed N.
    if "mppi" in filters:
        for k in args.mppi_samples:
            f = build_filter("mppi", nn, vp, tp, args.horizon, k)
            obs = make_obstacles(args.k_sweep_n, rng)
            mean, p95, mx = time_calls(f, obs, args.reps)
            rows.append({"sweep": "mppi_samples", "filter": "mppi", "n_obstacles": args.k_sweep_n,
                         "mppi_samples": k, "mean_ms": round(mean, 2), "p95_ms": round(p95, 2),
                         "max_ms": round(mx, 2), "over_budget": int(p95 > BUDGET_MS)})
            print(f"  [mppi-K]    K={k:5d} (N={args.k_sweep_n})  mean={mean:6.2f}  p95={p95:6.2f} ms")

    df = pd.DataFrame(rows)
    df.to_csv(out_dir / "results.csv", index=False)
    plot_figures(df, out_dir, args)

    # Capacity summary: largest N within budget per filter (obstacle sweep).
    srows = []
    obs_df = df[df["sweep"] == "obstacles"]
    for flavor in filters:
        sub = obs_df[obs_df["filter"] == flavor].sort_values("n_obstacles")
        within = sub[sub["p95_ms"] <= BUDGET_MS]["n_obstacles"]
        cap = int(within.max()) if len(within) else -1
        first_over = sub[sub["p95_ms"] > BUDGET_MS]["n_obstacles"]
        srows.append({"filter": flavor,
                      "p95_at_N0_ms": float(sub[sub.n_obstacles == 0]["p95_ms"].iloc[0]) if (sub.n_obstacles == 0).any() else float("nan"),
                      "capacity_N_within_budget": cap,
                      "first_N_over_budget": int(first_over.min()) if len(first_over) else None,
                      "budget_ms": BUDGET_MS})
    summary = pd.DataFrame(srows)
    summary.to_csv(out_dir / "summary.csv", index=False)
    save_summary_markdown(out_dir, "Safety-Filter Solve-Time Scaling", summary, [
        f"Budget: {BUDGET_MS:.0f} ms/solve (the shield runs at ~10 Hz). N counts "
        "obstacles within sensor range; rocks and dynamic vehicles are identical "
        "(x,y,r) entries to the filter. capacity_N_within_budget = largest N whose "
        "p95 solve stays under budget; first_N_over_budget = where it crosses.",
    ])
    print(f"\nDone: {out_dir}")
    print(summary.to_string(index=False))


def plot_figures(df: pd.DataFrame, out_dir: Path, args) -> None:
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    obs_df = df[df["sweep"] == "obstacles"]
    k_df = df[df["sweep"] == "mppi_samples"]
    n_panels = 2 if not k_df.empty else 1
    fig, axes = plt.subplots(1, n_panels, figsize=(6.5 * n_panels, 4.2), squeeze=False)
    ax = axes[0][0]
    for flavor in obs_df["filter"].unique():
        s = obs_df[obs_df["filter"] == flavor].sort_values("n_obstacles")
        ax.plot(s["n_obstacles"], s["p95_ms"], marker="o", label=f"{flavor} (p95)")
    ax.axhline(BUDGET_MS, color="r", ls="--", lw=1.2, label=f"{BUDGET_MS:.0f} ms (10 Hz budget)")
    ax.set_xlabel("obstacles within range N"); ax.set_ylabel("solve time (ms)")
    ax.set_title("Solve time vs obstacle count"); ax.set_yscale("log")
    ax.grid(alpha=0.3, which="both"); ax.legend()
    if not k_df.empty:
        ax2 = axes[0][1]
        s = k_df.sort_values("mppi_samples")
        ax2.plot(s["mppi_samples"], s["p95_ms"], marker="s", color="#2d6cdf", label="MPPI (p95)")
        ax2.axhline(BUDGET_MS, color="r", ls="--", lw=1.2, label=f"{BUDGET_MS:.0f} ms budget")
        ax2.set_xlabel(f"MPPI samples K (N={args.k_sweep_n})"); ax2.set_ylabel("solve time (ms)")
        ax2.set_title("MPPI solve time vs sample count"); ax2.grid(alpha=0.3); ax2.legend()
    fig.tight_layout()
    fig.savefig(fig_dir / "safety_filter_scaling.png", dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    main()
