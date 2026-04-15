#!/usr/bin/env python3
"""
Compare offline vs sequential online residual adaptation using SCM_Teleop diag logs.

This uses the same residual target as l4acados_residual_experiment.py:
discrete one-step residuals for [u, v, omega] on top of a nominal model.
"""

from __future__ import annotations

import argparse
import copy
import json
import random
import re
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parent.parent

from l4acados_residual_experiment import (
    ScaledResidualMLP,
    _build_nominal_mpc,
    _build_training_arrays,
    _make_f_expl_function,
    _make_stage_params,
    _resolve_terrain,
    _set_seed,
    _train_residual_model,
)


RUN_RE = re.compile(
    r"(?P<ts>\d{8}_\d{6}_\d{6})_"
    r"(?P<terrain>sand|clay|dirt)_"
    r"(?P<path>lane_change|double_lane_change|right_left|sinusoidal)_"
    r"acados_static_mlp$"
)


@dataclass
class RunInfo:
    ts: str
    terrain: str
    path: str
    diag_csv: Path


def _collect_static_runs(plot_root: Path) -> list[RunInfo]:
    out = []
    for d in sorted(plot_root.iterdir()):
        if not d.is_dir():
            continue
        m = RUN_RE.match(d.name)
        if not m:
            continue
        diags = sorted(d.glob("diag_*.csv"))
        if not diags:
            continue
        out.append(
            RunInfo(
                ts=m.group("ts"),
                terrain=m.group("terrain"),
                path=m.group("path"),
                diag_csv=diags[-1].resolve(),
            )
        )
    return sorted(out, key=lambda r: r.ts)


def _build_scaled_model_from_ckpt(ckpt: dict) -> ScaledResidualMLP:
    model = ScaledResidualMLP(
        input_dim=int(ckpt["input_dim"]),
        output_dim=int(ckpt["output_dim"]),
        hidden_sizes=tuple(ckpt["hidden_sizes"]),
        x_mean=np.asarray(ckpt["x_mean"], dtype=np.float32),
        x_std=np.asarray(ckpt["x_std"], dtype=np.float32),
        y_mean=np.asarray(ckpt["y_mean"], dtype=np.float32),
        y_std=np.asarray(ckpt["y_std"], dtype=np.float32),
    )
    model.core.load_state_dict(ckpt["state_dict"])
    model.eval()
    return model


def _eval_rmse(model: ScaledResidualMLP, X: np.ndarray, Y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Returns (nominal_rmse, corrected_rmse) for channels [du, dv, domega]."""
    nominal_rmse = np.sqrt(np.mean(Y**2, axis=0))
    with torch.no_grad():
        pred = model(torch.tensor(X, dtype=torch.float32)).cpu().numpy()
    corrected_err = Y - pred
    corrected_rmse = np.sqrt(np.mean(corrected_err**2, axis=0))
    return nominal_rmse, corrected_rmse


def _fine_tune_online(
    model: ScaledResidualMLP,
    X: np.ndarray,
    Y: np.ndarray,
    epochs: int,
    batch_size: int,
    lr: float,
) -> float:
    ds = TensorDataset(torch.tensor(X, dtype=torch.float32), torch.tensor(Y, dtype=torch.float32))
    dl = DataLoader(ds, batch_size=batch_size, shuffle=True)
    opt = torch.optim.Adam(model.core.parameters(), lr=lr, weight_decay=1e-6)
    loss_fn = torch.nn.MSELoss()

    t0 = time.time()
    model.train()
    for _ in range(epochs):
        for xb, yb in dl:
            opt.zero_grad()
            pred = model(xb)
            loss = loss_fn(pred, yb)
            loss.backward()
            opt.step()
    model.eval()
    return float(time.time() - t0)


def _aggregate_by_runs(
    model: ScaledResidualMLP,
    runs: list[RunInfo],
    run_data: dict[Path, tuple[np.ndarray, np.ndarray]],
) -> dict:
    nominal_all = []
    corrected_all = []
    for r in runs:
        X, Y = run_data[r.diag_csv]
        nominal_rmse, corrected_rmse = _eval_rmse(model, X, Y)
        nominal_all.append(nominal_rmse)
        corrected_all.append(corrected_rmse)
    nominal_m = np.mean(np.asarray(nominal_all), axis=0)
    corrected_m = np.mean(np.asarray(corrected_all), axis=0)
    imp = 100.0 * (nominal_m - corrected_m) / np.maximum(nominal_m, 1e-9)
    return {
        "nominal_rmse_du_dv_domega": nominal_m.tolist(),
        "corrected_rmse_du_dv_domega": corrected_m.tolist(),
        "improvement_pct_du_dv_domega": imp.tolist(),
    }


def _stratified_mixed_train(pool: list[RunInfo], n: int, seed: int) -> list[RunInfo]:
    rng = random.Random(seed)
    by_terrain = {"sand": [], "clay": [], "dirt": []}
    for r in pool:
        by_terrain[r.terrain].append(r)

    for key in by_terrain:
        rng.shuffle(by_terrain[key])

    non_empty = [k for k, v in by_terrain.items() if v]
    if not non_empty:
        return []

    picked = []
    i = 0
    while len(picked) < n:
        key = non_empty[i % len(non_empty)]
        if by_terrain[key]:
            picked.append(by_terrain[key].pop())
        else:
            key2 = None
            for cand in non_empty:
                if by_terrain[cand]:
                    key2 = cand
                    break
            if key2 is None:
                break
            picked.append(by_terrain[key2].pop())
        i += 1
    return picked


def main():
    p = argparse.ArgumentParser(description="Offline vs online residual adaptation study")
    p.add_argument("--plot-root", default=str(PROJECT_ROOT / "simulation" / "plots"))
    p.add_argument("--nominal-model", default="pacejka", choices=["pacejka", "tmeasy", "linear", "nn"])
    p.add_argument("--nominal-nn-model", default="paper_v1_mlp_16_4")
    p.add_argument("--holdout-runs", type=int, default=24)
    p.add_argument("--dirt-train-runs", type=int, default=30)
    p.add_argument("--offline-epochs", type=int, default=80)
    p.add_argument("--offline-batch-size", type=int, default=512)
    p.add_argument("--offline-lr", type=float, default=1e-3)
    p.add_argument("--online-epochs", type=int, default=4)
    p.add_argument("--online-batch-size", type=int, default=512)
    p.add_argument("--online-lr", type=float, default=2e-4)
    p.add_argument("--hidden", nargs="+", type=int, default=[64, 64])
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--out-json", default=str(PROJECT_ROOT / "simulation" / "logs" / "l4acados_online_offline_study.json"))
    args = p.parse_args()

    _set_seed(int(args.seed))

    plot_root = Path(args.plot_root).expanduser().resolve()
    runs = _collect_static_runs(plot_root)
    if len(runs) < (args.holdout_runs + 10):
        raise RuntimeError(f"Not enough runs: found {len(runs)}")

    holdout = runs[-args.holdout_runs :]
    pool = runs[: -args.holdout_runs]
    dirt_pool = [r for r in pool if r.terrain == "dirt"]
    if len(dirt_pool) < args.dirt_train_runs:
        raise RuntimeError(
            f"Requested {args.dirt_train_runs} dirt train runs but only {len(dirt_pool)} are available."
        )

    dirt_train = dirt_pool[: args.dirt_train_runs]
    mixed_train = _stratified_mixed_train(pool, len(dirt_train), seed=args.seed + 1)
    if len(mixed_train) < len(dirt_train):
        raise RuntimeError("Could not create mixed train split of requested size.")

    print(f"Total static runs: {len(runs)}")
    print(f"Pool runs: {len(pool)}")
    print(f"Holdout runs: {len(holdout)}")
    print(f"Dirt-train runs: {len(dirt_train)}")
    print(f"Mixed-train runs: {len(mixed_train)}")

    # Build nominal model once; stage terrain parameters are set per run.
    mpc, _ = _build_nominal_mpc(
        tire_model=args.nominal_model,
        nn_model=args.nominal_nn_model,
        terrain_name="clay",
        mpc_dt=0.1,
        mpc_n=30,
        no_lat_transfer=False,
    )
    f_fun = _make_f_expl_function(mpc)
    p_stage_by_terrain = {
        t: _make_stage_params(mpc, _resolve_terrain(t)) for t in ("sand", "clay", "dirt")
    }

    needed = {r.diag_csv: r for r in (dirt_train + mixed_train + holdout)}
    run_data = {}
    for i, r in enumerate(needed.values(), start=1):
        X, Y = _build_training_arrays(r.diag_csv, f_fun, p_stage_by_terrain[r.terrain])
        run_data[r.diag_csv] = (X, Y)
        if i == 1 or i % 10 == 0 or i == len(needed):
            print(f"  built dataset {i}/{len(needed)}: {r.diag_csv.name} -> {X.shape}")

    def stack_xy(run_list: list[RunInfo]) -> tuple[np.ndarray, np.ndarray]:
        Xs, Ys = [], []
        for rr in run_list:
            X, Y = run_data[rr.diag_csv]
            Xs.append(X)
            Ys.append(Y)
        return np.concatenate(Xs, axis=0), np.concatenate(Ys, axis=0)

    X_dirt, Y_dirt = stack_xy(dirt_train)
    X_mix, Y_mix = stack_xy(mixed_train)

    print("Training offline dirt-only residual model...")
    ckpt_dirt, dirt_metrics = _train_residual_model(
        X=X_dirt,
        Y=Y_dirt,
        epochs=int(args.offline_epochs),
        batch_size=int(args.offline_batch_size),
        lr=float(args.offline_lr),
        seed=int(args.seed),
        hidden_sizes=tuple(args.hidden),
    )
    model_dirt = _build_scaled_model_from_ckpt(ckpt_dirt)

    print("Training offline mixed residual model...")
    ckpt_mix, mix_metrics = _train_residual_model(
        X=X_mix,
        Y=Y_mix,
        epochs=int(args.offline_epochs),
        batch_size=int(args.offline_batch_size),
        lr=float(args.offline_lr),
        seed=int(args.seed + 1),
        hidden_sizes=tuple(args.hidden),
    )
    model_mix = _build_scaled_model_from_ckpt(ckpt_mix)

    holdout_offline_dirt = _aggregate_by_runs(model_dirt, holdout, run_data)
    holdout_offline_mix = _aggregate_by_runs(model_mix, holdout, run_data)

    # Sequential online adaptation over holdout stream (evaluate pre-update).
    model_online = copy.deepcopy(model_dirt)
    stream_rows = []
    update_times = []
    for idx, rr in enumerate(holdout, start=1):
        Xr, Yr = run_data[rr.diag_csv]
        n_rmse, d_rmse = _eval_rmse(model_dirt, Xr, Yr)
        _, o_rmse = _eval_rmse(model_online, Xr, Yr)
        _, m_rmse = _eval_rmse(model_mix, Xr, Yr)
        stream_rows.append(
            {
                "idx": idx,
                "ts": rr.ts,
                "terrain": rr.terrain,
                "path": rr.path,
                "n_samples": int(Xr.shape[0]),
                "nominal_rmse": n_rmse.tolist(),
                "offline_dirt_rmse": d_rmse.tolist(),
                "offline_mixed_rmse": m_rmse.tolist(),
                "online_preupdate_rmse": o_rmse.tolist(),
            }
        )
        dt_upd = _fine_tune_online(
            model=model_online,
            X=Xr,
            Y=Yr,
            epochs=int(args.online_epochs),
            batch_size=int(args.online_batch_size),
            lr=float(args.online_lr),
        )
        update_times.append(dt_upd)
        print(
            f"  stream {idx:02d}/{len(holdout)} {rr.terrain:5s} {rr.path:16s} "
            f"pre-online-du/dv/dw={np.round(o_rmse,4)}  update={dt_upd:.2f}s"
        )

    # Aggregate stream metrics.
    nominal_arr = np.asarray([r["nominal_rmse"] for r in stream_rows], dtype=float)
    dirt_arr = np.asarray([r["offline_dirt_rmse"] for r in stream_rows], dtype=float)
    mixed_arr = np.asarray([r["offline_mixed_rmse"] for r in stream_rows], dtype=float)
    online_arr = np.asarray([r["online_preupdate_rmse"] for r in stream_rows], dtype=float)

    def _imp(base: np.ndarray, corrected: np.ndarray) -> np.ndarray:
        return 100.0 * (base - corrected) / np.maximum(base, 1e-9)

    result = {
        "config": vars(args),
        "counts": {
            "total_static_runs": len(runs),
            "pool_runs": len(pool),
            "holdout_runs": len(holdout),
            "dirt_train_runs": len(dirt_train),
            "mixed_train_runs": len(mixed_train),
        },
        "holdout_composition": {
            "terrains": {t: sum(1 for r in holdout if r.terrain == t) for t in ("sand", "clay", "dirt")},
            "paths": {
                pth: sum(1 for r in holdout if r.path == pth)
                for pth in ("lane_change", "double_lane_change", "right_left", "sinusoidal")
            },
        },
        "offline_train_metrics": {
            "dirt": dirt_metrics,
            "mixed": mix_metrics,
        },
        "holdout_offline_aggregate": {
            "dirt": holdout_offline_dirt,
            "mixed": holdout_offline_mix,
        },
        "stream_aggregate_preupdate": {
            "nominal_rmse_du_dv_domega": nominal_arr.mean(axis=0).tolist(),
            "offline_dirt_rmse_du_dv_domega": dirt_arr.mean(axis=0).tolist(),
            "offline_mixed_rmse_du_dv_domega": mixed_arr.mean(axis=0).tolist(),
            "online_preupdate_rmse_du_dv_domega": online_arr.mean(axis=0).tolist(),
            "offline_dirt_improvement_pct": _imp(nominal_arr.mean(axis=0), dirt_arr.mean(axis=0)).tolist(),
            "offline_mixed_improvement_pct": _imp(nominal_arr.mean(axis=0), mixed_arr.mean(axis=0)).tolist(),
            "online_preupdate_improvement_pct": _imp(nominal_arr.mean(axis=0), online_arr.mean(axis=0)).tolist(),
            "online_vs_offline_dirt_delta_pct": _imp(dirt_arr.mean(axis=0), online_arr.mean(axis=0)).tolist(),
        },
        "online_update_timing_s": {
            "mean": float(np.mean(update_times)),
            "median": float(np.median(update_times)),
            "p90": float(np.percentile(update_times, 90.0)),
        },
        "stream_rows": stream_rows,
    }

    out_json = Path(args.out_json).expanduser().resolve()
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(result, indent=2))

    print("\n=== Summary ===")
    agg = result["stream_aggregate_preupdate"]
    print("Nominal RMSE [du,dv,dw]:", np.round(agg["nominal_rmse_du_dv_domega"], 5))
    print("Offline dirt RMSE       :", np.round(agg["offline_dirt_rmse_du_dv_domega"], 5))
    print("Offline mixed RMSE      :", np.round(agg["offline_mixed_rmse_du_dv_domega"], 5))
    print("Online pre-update RMSE  :", np.round(agg["online_preupdate_rmse_du_dv_domega"], 5))
    print("Offline dirt improve %  :", np.round(agg["offline_dirt_improvement_pct"], 2))
    print("Offline mixed improve % :", np.round(agg["offline_mixed_improvement_pct"], 2))
    print("Online improve %        :", np.round(agg["online_preupdate_improvement_pct"], 2))
    print("Online vs dirt delta %  :", np.round(agg["online_vs_offline_dirt_delta_pct"], 2))
    print("Online update mean/med/p90 (s):",
          f"{result['online_update_timing_s']['mean']:.3f}/"
          f"{result['online_update_timing_s']['median']:.3f}/"
          f"{result['online_update_timing_s']['p90']:.3f}")
    print(f"\nWrote {out_json}")


if __name__ == "__main__":
    main()

