#!/usr/bin/env python3
"""Evaluate trained tire-force surrogates on a CSV dataset.

The trainer's checkpoint metadata is used when available.  Older checkpoints
are also supported by inferring the compact static/rate feature schema from
`input_size` and `rate_augmented`.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from train_variant import (  # noqa: E402
    MLP,
    OP_COLS,
    OUT_COLS,
    RICH_RATE_SOURCE_COLS,
    RICH_SENSOR_COLS,
    ResNet,
    TERRAIN_COLS,
    compute_named_rates,
    compute_rates,
)


def _r2(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y_true - y_pred) ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2)) + 1e-12
    return 1.0 - ss_res / ss_tot


def _metrics(y_true: np.ndarray, y_pred: np.ndarray) -> dict[str, float]:
    err = y_pred - y_true
    rmse = np.sqrt(np.mean(err**2, axis=0))
    mae = np.mean(np.abs(err), axis=0)
    return {
        "r2_fx": _r2(y_true[:, 0], y_pred[:, 0]),
        "r2_fy": _r2(y_true[:, 1], y_pred[:, 1]),
        "rmse_fx": float(rmse[0]),
        "rmse_fy": float(rmse[1]),
        "mae_fx": float(mae[0]),
        "mae_fy": float(mae[1]),
    }


def _infer_feature_cols(ckpt: dict) -> list[str]:
    feature_cols = ckpt.get("feature_cols")
    if feature_cols:
        return list(feature_cols)

    input_size = int(ckpt.get("input_size", 0))
    if bool(ckpt.get("rate_augmented", False)) and input_size == 14:
        return OP_COLS + ["d_slip_ratio", "d_slip_angle", "d_velocity"] + TERRAIN_COLS
    if input_size == 11:
        return OP_COLS + TERRAIN_COLS
    raise ValueError(f"Cannot infer feature columns for checkpoint input_size={input_size}")


def _build_model(ckpt: dict) -> torch.nn.Module:
    arch = ckpt.get("architecture_type", "mlp")
    input_size = int(ckpt["input_size"])
    if arch == "mlp":
        model = MLP(input_size=input_size, hidden_sizes=list(ckpt.get("hidden_sizes", [16, 8])))
    elif arch == "resnet":
        model = ResNet(
            input_size=input_size,
            hidden_dim=int(ckpt.get("hidden_dim", 16)),
            n_blocks=int(ckpt.get("n_blocks", 2)),
        )
    else:
        raise ValueError(f"Unsupported architecture_type={arch!r}")
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()
    return model


def _prepare_dataframe(df: pd.DataFrame, feature_cols: list[str], record_dt: float) -> pd.DataFrame:
    needed_rates = [c[2:] for c in feature_cols if c.startswith("d_") and c not in df.columns]
    if needed_rates:
        if set(needed_rates) == {"slip_ratio", "slip_angle", "velocity"}:
            df = compute_rates(df, record_dt=record_dt)
        else:
            df = compute_named_rates(df, sorted(needed_rates), record_dt=record_dt)

    missing = [c for c in feature_cols + OUT_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"CSV is missing required columns: {missing}")

    return df


def _row_subset(df: pd.DataFrame, speed_min: float | None) -> pd.DataFrame:
    mask = np.ones(len(df), dtype=bool)
    if speed_min is not None:
        mask &= df["velocity"].to_numpy(dtype=float) >= float(speed_min)
    for col in OUT_COLS:
        mask &= np.isfinite(df[col].to_numpy(dtype=float))
    return df.loc[mask].reset_index(drop=True)


def _axle_name(df: pd.DataFrame) -> np.ndarray:
    if "axle_id" in df.columns:
        return np.where(df["axle_id"].to_numpy(dtype=int) == 0, "front", "rear")
    scenario_ids = df["scenario_id"].to_numpy(dtype=np.int64)
    return np.where(scenario_ids >= 1_000_000, "rear", "front")


def _save_scatter(out_dir: Path, name: str, y_true: np.ndarray, y_pred: np.ndarray, title: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(9, 4), constrained_layout=True)
    for ax, idx, label in zip(axes, [0, 1], ["Fx", "Fy"]):
        yt = y_true[:, idx]
        yp = y_pred[:, idx]
        if len(yt) > 6000:
            rng = np.random.default_rng(7)
            keep = rng.choice(len(yt), size=6000, replace=False)
            yt = yt[keep]
            yp = yp[keep]
        lim_lo = float(min(np.min(yt), np.min(yp)))
        lim_hi = float(max(np.max(yt), np.max(yp)))
        ax.scatter(yt, yp, s=5, alpha=0.22, linewidths=0)
        ax.plot([lim_lo, lim_hi], [lim_lo, lim_hi], "k--", lw=1.0)
        ax.set_xlabel(f"Actual {label} [N]")
        ax.set_ylabel(f"Predicted {label} [N]")
        ax.set_title(label)
        ax.grid(True, alpha=0.25)
    fig.suptitle(title)
    fig.savefig(out_dir / f"{name}_predicted_vs_actual.png", dpi=220)
    plt.close(fig)


def evaluate_model(
    data_csv: Path,
    model_dir: Path,
    out_dir: Path,
    record_dt: float,
    speed_min: float | None,
) -> tuple[pd.DataFrame, dict]:
    ckpt = torch.load(model_dir / "best_terrain_nn.pt", map_location="cpu", weights_only=False)
    with open(model_dir / "scalers.pkl", "rb") as f:
        scalers = pickle.load(f)
    feature_cols = _infer_feature_cols(ckpt)

    df = pd.read_csv(data_csv)
    df = _prepare_dataframe(df, feature_cols, record_dt=record_dt)
    df = _row_subset(df, speed_min=speed_min)

    X = df[feature_cols].to_numpy(dtype=np.float32)
    y_true = df[OUT_COLS].to_numpy(dtype=np.float32)
    finite = np.isfinite(X).all(axis=1) & np.isfinite(y_true).all(axis=1)
    df = df.loc[finite].reset_index(drop=True)
    X = X[finite]
    y_true = y_true[finite]

    Xs = scalers["X"].transform(X)
    model = _build_model(ckpt)
    with torch.no_grad():
        yp_s = model(torch.tensor(Xs, dtype=torch.float32)).detach().cpu().numpy()
    y_pred = scalers["y"].inverse_transform(yp_s)

    rows = []
    model_name = model_dir.name
    base = {
        "model": model_name,
        "subset": "all",
        "n_rows": int(len(df)),
        **_metrics(y_true.astype(float), y_pred.astype(float)),
    }
    rows.append(base)

    axle = _axle_name(df)
    for subset in ["front", "rear"]:
        m = axle == subset
        if np.any(m):
            rows.append(
                {
                    "model": model_name,
                    "subset": subset,
                    "n_rows": int(np.sum(m)),
                    **_metrics(y_true[m].astype(float), y_pred[m].astype(float)),
                }
            )

    for col in ["terrain", "bumpiness", "path"]:
        if col in df.columns:
            for value, idxs in df.groupby(col).groups.items():
                idx = np.fromiter(idxs, dtype=int)
                rows.append(
                    {
                        "model": model_name,
                        "subset": f"{col}={value}",
                        "n_rows": int(len(idx)),
                        **_metrics(y_true[idx].astype(float), y_pred[idx].astype(float)),
                    }
                )

    pred_df = df[["scenario_id", "timestep"]].copy()
    if "axle_id" in df.columns:
        pred_df["axle_id"] = df["axle_id"].to_numpy()
    pred_df["Fx_actual"] = y_true[:, 0]
    pred_df["Fy_actual"] = y_true[:, 1]
    pred_df["Fx_pred"] = y_pred[:, 0]
    pred_df["Fy_pred"] = y_pred[:, 1]
    pred_df.to_csv(out_dir / f"{model_name}_predictions.csv", index=False)
    _save_scatter(out_dir, model_name, y_true, y_pred, model_name)

    payload = {
        "model": model_name,
        "data_csv": str(data_csv),
        "feature_cols": feature_cols,
        "speed_min": speed_min,
        "rows": rows,
    }
    return pd.DataFrame(rows), payload


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True)
    p.add_argument("--model-dir", nargs="+", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--record-dt", type=float, default=0.012)
    p.add_argument("--speed-min", type=float, default=None)
    args = p.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data_csv = Path(args.data)

    all_rows = []
    payloads = []
    for model_dir in args.model_dir:
        metrics_df, payload = evaluate_model(
            data_csv=data_csv,
            model_dir=Path(model_dir),
            out_dir=out_dir,
            record_dt=float(args.record_dt),
            speed_min=args.speed_min,
        )
        all_rows.append(metrics_df)
        payloads.append(payload)

    summary = pd.concat(all_rows, ignore_index=True)
    summary.to_csv(out_dir / "metrics.csv", index=False)
    (out_dir / "summary.json").write_text(json.dumps(payloads, indent=2))
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
