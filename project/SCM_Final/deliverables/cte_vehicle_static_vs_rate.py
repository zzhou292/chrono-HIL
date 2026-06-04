#!/usr/bin/env python3
"""Fresh CTE comparison: vehicle_static_32_16_lhs vs vehicle_rate_64_32_lhs
driving the standard MPC on a handful of scenarios.

The point: settle whether the static checkpoint's lower force-RMSE (seen
in `make_fig1_4way.py`) actually translates into better closed-loop
tracking, or whether the rate model's smoother predictions still win on
CTE.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "simulation"


@dataclass(frozen=True)
class Run:
    idx: int
    nn_model: str        # "vehicle_static_32_16_lhs" or "vehicle_rate_64_32_lhs"
    terrain: str
    path: str
    speed: float
    bumpiness: int
    seed: int
    sim_port: int
    ctrl_port: int
    out_dir: str
    sim_time: float = 12.0


SCENARIOS = [
    ("sand", "sinusoidal", 5, 0),
    ("sand", "sinusoidal", 7, 0),
    ("sand", "right_left", 7, 0),
    ("clay", "sinusoidal", 5, 0),
    ("clay", "right_left", 7, 0),
    ("dirt", "sinusoidal", 7, 0),
]


def _run_one(task: Run) -> dict:
    Path(task.out_dir).mkdir(parents=True, exist_ok=True)
    log = Path(task.out_dir) / "run.log"
    cmd = [
        sys.executable, "-u", str(SIM / "launch_decoupled.py"),
        "--terrain", task.terrain, "--path", task.path,
        "--speed", str(task.speed),
        "--time", str(task.sim_time + 5), "--lead-in", "5.0",
        "--bumpiness", str(task.bumpiness),
        "--sim-port", str(task.sim_port), "--ctrl-port", str(task.ctrl_port),
        "--plot-dir", task.out_dir, "--no-vis", "--no-plot",
        "--sim-diag-csv", str(Path(task.out_dir) / "sim_diag.csv"),
        "--model", "nn", "--nn-model", task.nn_model,
        "--rms-time-start", "2.0",
        # NO terrain estimator — we want to compare just the tire-NN effect
    ]
    env = os.environ.copy()
    env["ACADOS_UNIQUE_BUILD_DIR"] = "1"
    env.setdefault("ACADOS_SOURCE_DIR", str(Path.home() / "Documents/sbel/acados"))
    rc = -1
    t0 = time.time()
    try:
        with open(log, "w") as f:
            rc = subprocess.run(cmd, env=env, stdout=f,
                                 stderr=subprocess.STDOUT,
                                 timeout=180).returncode
    except subprocess.TimeoutExpired:
        pass
    wall = time.time() - t0

    # Pull CTE from the controller's diag CSV inside the run subdir
    diag_csvs = sorted(Path(task.out_dir).rglob("diag_*.csv"))
    if not diag_csvs:
        return dict(label=task.nn_model, terrain=task.terrain, path=task.path,
                    v=task.speed, b=task.bumpiness, seed=task.seed,
                    rc=rc, ok=False, rms_cte_m=float("nan"),
                    max_cte_m=float("nan"), wall_s=wall)
    df = pd.read_csv(diag_csvs[-1])
    df = df[df["sim_time"] >= 2.0]  # ignore startup transient
    if df.empty:
        return dict(label=task.nn_model, terrain=task.terrain, path=task.path,
                    v=task.speed, b=task.bumpiness, seed=task.seed,
                    rc=rc, ok=False, rms_cte_m=float("nan"),
                    max_cte_m=float("nan"), wall_s=wall)
    # Look up crosstrack column name (may be "crosstrack_err" or similar)
    cte_col = None
    for cand in ("crosstrack_err", "geom_path_err", "frenet_lat"):
        if cand in df.columns:
            cte_col = cand; break
    if cte_col is None:
        return dict(label=task.nn_model, terrain=task.terrain, path=task.path,
                    v=task.speed, b=task.bumpiness, seed=task.seed,
                    rc=rc, ok=False, rms_cte_m=float("nan"),
                    max_cte_m=float("nan"), wall_s=wall)
    cte = pd.to_numeric(df[cte_col], errors="coerce").dropna().to_numpy()
    return dict(label=task.nn_model, terrain=task.terrain, path=task.path,
                v=task.speed, b=task.bumpiness, seed=task.seed,
                rc=rc, ok=(rc == 0),
                rms_cte_m=float(np.sqrt(np.mean(cte ** 2))),
                max_cte_m=float(np.max(np.abs(cte))),
                wall_s=wall)


def main():
    out_root = REPO / "deliverables" / "runs_cte_static_vs_rate"
    out_root.mkdir(parents=True, exist_ok=True)

    tasks = []
    port = 55000
    seed = 900
    for nn in ("vehicle_rate_64_32_lhs", "vehicle_static_32_16_lhs"):
        for (terr, path, v, b) in SCENARIOS:
            label = f"{nn}_{terr}_{path}_v{int(v)}_b{b}_s{seed}"
            tasks.append(Run(
                idx=len(tasks), nn_model=nn,
                terrain=terr, path=path, speed=float(v),
                bumpiness=b, seed=seed,
                sim_port=port, ctrl_port=port + 1,
                out_dir=str(out_root / label),
            ))
            port += 4

    print(f"Running {len(tasks)} CTE comparison runs")
    # Prewarm acados serially with the first one to avoid codegen race
    print(f"[prewarm] {tasks[0].nn_model}/{tasks[0].terrain}/{tasks[0].path}")
    first = _run_one(tasks[0])
    results = [first]
    print(f"  ✓ first ok={first['ok']} cte={first['rms_cte_m']:.3f}m "
          f"wall={first['wall_s']:.1f}s")

    with ProcessPoolExecutor(max_workers=4) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            print(f"  {r['label']:<55s} cte={r['rms_cte_m']:.3f}m "
                  f"max={r['max_cte_m']:.3f}m wall={r['wall_s']:.1f}s",
                  flush=True)

    df = pd.DataFrame(results)
    df.to_csv(out_root / "results.csv", index=False)
    print(f"\nWrote {out_root / 'results.csv'}\n")

    print("=" * 92)
    print(f"{'scenario':<45s} {'rate CTE':>10s} {'static CTE':>12s}   winner")
    print("=" * 92)
    # Compare per scenario
    for (terr, path, v, b) in SCENARIOS:
        r_rate = df[(df.label == "vehicle_rate_64_32_lhs") & (df.terrain == terr)
                    & (df.path == path) & (df.v == v) & (df.b == b)]
        r_stat = df[(df.label == "vehicle_static_32_16_lhs") & (df.terrain == terr)
                    & (df.path == path) & (df.v == v) & (df.b == b)]
        if r_rate.empty or r_stat.empty:
            continue
        c_rate = float(r_rate.rms_cte_m.iloc[0])
        c_stat = float(r_stat.rms_cte_m.iloc[0])
        win = "STATIC" if c_stat < c_rate else "rate"
        print(f"{terr}/{path}/v{v}/b{b:<27s} {c_rate:>8.3f}m  {c_stat:>10.3f}m  → {win}")
    print()
    print(f"Rate mean   RMS CTE: {df[df.label=='vehicle_rate_64_32_lhs'].rms_cte_m.mean():.3f} m")
    print(f"Static mean RMS CTE: {df[df.label=='vehicle_static_32_16_lhs'].rms_cte_m.mean():.3f} m")


if __name__ == "__main__":
    main()
