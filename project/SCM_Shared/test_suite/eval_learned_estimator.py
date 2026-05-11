#!/usr/bin/env python3
"""Offline evaluation of the learned window estimator on saved traces.

Replays each CSV trace through the LearnedTerrainEstimator's ``observe()``
loop at the original sample cadence and reports the running n-estimate vs
the true n.  This is a fast (~ms per trace) sanity check that avoids
launching the chrono sim, and is the same code path the live runner uses.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "simulation"))

from learned_terrain_estimator import LearnedTerrainEstimator


def replay(trace: Path, est: LearnedTerrainEstimator) -> tuple[float, float, float]:
    with trace.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return float("nan"), float("nan"), 0.0
    n_true = float(rows[0]["n_true"])

    # Reset internal buffer/state by reconstructing the estimator each call —
    # caller passes a fresh instance per trace.
    last_n = None
    n_history: list[float] = []
    t_history: list[float] = []
    for r in rows:
        t = float(r["t"])
        u = float(r["u"]); v = float(r["v"]); om = float(r["omega"])
        ax = float(r["ax"]); ay = float(r["ay"])
        wfl = float(r["wheel_omega_fl"]); wfr = float(r["wheel_omega_fr"])
        wrl = float(r["wheel_omega_rl"]); wrr = float(r["wheel_omega_rr"])
        delta = float(r["steering_angle"])
        thr = float(r["throttle_cmd"])

        est.set_throttle(thr)
        # Bicycle-model alpha_f, used by the observe() interface to recover delta.
        import math
        u_safe = max(abs(u), 0.5)
        alpha_f = -math.atan2(v + 1.593 * om, u_safe) + delta

        est.observe(
            kappa=0.0, alpha_f=alpha_f, alpha_r=0.0,
            u=u, Fz_f=6500.0, Fz_r=6000.0, sr=0.0,
            ay_imu=ay, omega_dot=0.0,
            omega=om, v_lateral=v, sim_time=t,
            wheel_omegas=(wfl, wfr, wrl, wrr),
            ax_imu=ax, throttle_cmd=thr,
        )
        if est._buffer_ready:
            last_n = est.get_bekker_n()
            t_history.append(t)
            n_history.append(last_n)

    n_final = est.get_bekker_n() if last_n is not None else float("nan")
    n_late = (float(np.mean(n_history[-10:])) if len(n_history) >= 10
              else float(np.mean(n_history)) if n_history else float("nan"))
    return n_true, n_final, n_late


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--trace-dir", default=str(
        Path(__file__).parent.parent / "data" / "terrain_traces"))
    p.add_argument("--model-dir", default=str(
        Path(__file__).parent.parent / "nn_models" / "terrain_window_mlp_v3_cl"))
    args = p.parse_args()

    traces = sorted(Path(args.trace_dir).glob("*.csv"))
    print(f"[eval] {len(traces)} traces")

    by_terr: dict[str, list[tuple[float, float, float, str]]] = {}
    for t in traces:
        terrain = t.name.split("_")[0]
        est = LearnedTerrainEstimator(model_dir=args.model_dir,
                                       initial_terrain={"n": 0.5},
                                       update_interval=1, verbose=False)
        n_true, n_final, n_late = replay(t, est)
        by_terr.setdefault(terrain, []).append((n_true, n_final, n_late, t.name))

    print(f"\n{'terrain':<6s} {'n_true':>6s} {'n_final mean±std':>22s} "
          f"{'n_late mean±std':>22s} {'err_late':>10s}")
    for terrain, vals in sorted(by_terr.items()):
        n_true = vals[0][0]
        finals = np.asarray([v[1] for v in vals])
        lates  = np.asarray([v[2] for v in vals])
        err = float(np.mean(np.abs(lates - n_true)) / n_true * 100.0)
        print(f"{terrain:<6s} {n_true:>6.2f} "
              f"{finals.mean():>10.3f}±{finals.std():.3f} "
              f"        {lates.mean():>10.3f}±{lates.std():.3f}    "
              f"{err:>6.1f}%")


if __name__ == "__main__":
    main()
