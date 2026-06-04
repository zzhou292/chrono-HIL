#!/usr/bin/env python3
"""
Train an experiment variant for tire force prediction (Fx, Fy).

Variants covered:
- arch:    mlp | resnet
- mode:    static | temporal | rate | axle_rate | rich | rich_rate

This script writes outputs compatible with `simulation/nn_tire_model.py`:
- best_terrain_nn.pt   (checkpoint with model_state_dict + metadata)
- scalers.pkl          (sklearn StandardScaler objects for X and y)
- test_metrics.json    (paper-friendly metrics)
"""

from __future__ import annotations

import argparse
import json
import logging
import pickle
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from torch.utils.data import DataLoader, TensorDataset


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


def set_global_seed(seed: int) -> None:
    """Reproducible init + shuffling (CPU)."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


OpMode = Literal["static", "temporal", "rate", "axle_rate", "rich", "rich_rate"]
Arch = Literal["mlp", "resnet"]


OP_COLS = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate"]
TERRAIN_COLS = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
OUT_COLS = ["Fx", "Fy"]
RICH_SENSOR_COLS = [
    "axle_id",
    "slip_ratio",
    "slip_angle",
    "velocity",
    "vertical_load",
    "steering_rate",
    "steering_angle",
    "u_body",
    "v_body",
    "yaw_rate",
    "ax_imu",
    "ay_imu",
    "measured_kappa",
    "axle_kappa",
    "wheel_omega_axle",
    "dFz_lateral_kin",
    "dFz_lateral_imu",
    "throttle_cmd",
    "brake_cmd",
    "accel_cmd",
    "jerk_cmd",
]
RICH_RATE_SOURCE_COLS = [
    "slip_ratio",
    "slip_angle",
    "velocity",
    "v_body",
    "yaw_rate",
    "ax_imu",
    "ay_imu",
    "axle_kappa",
]
RICH_RATE_COLS = [f"d_{c}" for c in RICH_RATE_SOURCE_COLS]


def _apply_physical_filters(df: pd.DataFrame, mode: OpMode) -> pd.DataFrame:
    """Drop numerically valid but physically impossible outliers.

    A few corrupted rows can dominate StandardScaler statistics (especially in
    velocity/rate channels) and effectively zero out those features.
    """
    base_mask = (
        df["slip_ratio"].between(-1.2, 1.2)
        & df["slip_angle"].between(-0.7, 0.7)
        & df["velocity"].between(0.25, 20.0)
        & df["vertical_load"].between(1000.0, 10000.0)
        & df["steering_rate"].between(-2.0, 2.0)
        & df["Fx"].between(-5.0e4, 5.0e4)
        & df["Fy"].between(-5.0e4, 5.0e4)
    )

    if mode in ("rate", "rich_rate"):
        for c, lo, hi in (
            ("d_slip_ratio", -5.0, 5.0),
            ("d_slip_angle", -2.0, 2.0),
            ("d_velocity", -10.0, 10.0),
            ("d_v_body", -10.0, 10.0),
            ("d_yaw_rate", -5.0, 5.0),
            ("d_ax_imu", -80.0, 80.0),
            ("d_ay_imu", -80.0, 80.0),
            ("d_axle_kappa", -10.0, 10.0),
        ):
            if c in df.columns:
                base_mask &= df[c].between(lo, hi)

    dropped = int((~base_mask).sum())
    if dropped > 0:
        logger.warning("Dropping %d physically-invalid rows before training", dropped)
    return df.loc[base_mask].reset_index(drop=True)


def _check_rate_feature_independence(df: pd.DataFrame, allow_duplicate: bool) -> None:
    """Guard against duplicated rate channels caused by data logging bugs."""
    req = {"steering_rate", "d_slip_angle"}
    if not req.issubset(df.columns):
        return

    diff = (df["steering_rate"] - df["d_slip_angle"]).abs().to_numpy(dtype=np.float64)
    finite = np.isfinite(diff)
    if not finite.any():
        return

    dup_frac = float(np.mean(diff[finite] < 1e-8))
    if dup_frac < 0.995:
        return

    msg = (
        "Detected duplicated rate features: steering_rate and d_slip_angle are "
        f"identical in {dup_frac * 100:.2f}% of rows. "
        "This indicates a data-collection schema bug and creates train/inference "
        "feature mismatch for rate-augmented models."
    )
    if allow_duplicate:
        logger.warning("%s Proceeding because --allow-duplicate-rate-features was set.", msg)
    else:
        raise ValueError(
            msg + " Recollect/repair the dataset or pass "
            "--allow-duplicate-rate-features to bypass this guard."
        )


def build_temporal_windows(df: pd.DataFrame, K: int, dt_nn: float, record_dt: float) -> tuple[np.ndarray, np.ndarray]:
    stride = max(1, int(round(dt_nn / record_dt)))
    X_list: list[np.ndarray] = []
    y_list: list[np.ndarray] = []

    for _, g in df.groupby("scenario_id"):
        g = g.sort_values("timestep").reset_index(drop=True)
        idxs = list(range(0, len(g), stride))
        if len(idxs) < K:
            continue

        ops = g[OP_COLS].values.astype(np.float32)
        terr = g[TERRAIN_COLS].iloc[0].values.astype(np.float32)
        y = g[OUT_COLS].values.astype(np.float32)

        for w in range(K - 1, len(idxs)):
            win = []
            for j in range(K):
                win.append(ops[idxs[w - j]])  # most recent first
            win_ops = np.concatenate(win, axis=0)  # K*5
            X_list.append(np.concatenate([win_ops, terr], axis=0))  # K*5 + 6
            y_list.append(y[idxs[w]])

    if not X_list:
        raise ValueError("No temporal windows produced (check K/dt_nn/record_dt and dataset).")
    return np.vstack(X_list), np.vstack(y_list)


def compute_rates(df: pd.DataFrame, record_dt: float) -> pd.DataFrame:
    df = df.sort_values(["scenario_id", "timestep"]).reset_index(drop=True)
    for col, name in zip(["slip_ratio", "slip_angle", "velocity"], ["d_slip_ratio", "d_slip_angle", "d_velocity"]):
        df[name] = df.groupby("scenario_id")[col].diff() / record_dt
    df = df.dropna(subset=["d_slip_ratio", "d_slip_angle", "d_velocity"]).reset_index(drop=True)
    return df


def compute_named_rates(df: pd.DataFrame, cols: list[str], record_dt: float) -> pd.DataFrame:
    df = df.sort_values(["scenario_id", "timestep"]).reset_index(drop=True)
    rate_cols = []
    for col in cols:
        name = f"d_{col}"
        df[name] = df.groupby("scenario_id")[col].diff() / record_dt
        rate_cols.append(name)
    df = df.dropna(subset=rate_cols).reset_index(drop=True)
    return df


class MLP(nn.Module):
    def __init__(self, input_size: int, hidden_sizes: list[int], output_size: int = 2):
        super().__init__()
        layers: list[nn.Module] = []
        prev = input_size
        for h in hidden_sizes:
            layers.append(nn.Linear(prev, h))
            prev = h
        layers.append(nn.Linear(prev, output_size))
        self.layers = nn.ModuleList(layers)
        for layer in self.layers:
            if isinstance(layer, nn.Linear):
                nn.init.xavier_normal_(layer.weight)

    def forward(self, x):
        for layer in self.layers[:-1]:
            x = torch.tanh(layer(x))
        return self.layers[-1](x)


class ResidualBlock(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.fc1 = nn.Linear(dim, dim)
        self.fc2 = nn.Linear(dim, dim)
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.xavier_normal_(self.fc2.weight)

    def forward(self, x):
        h = torch.tanh(self.fc1(x))
        h = self.fc2(h)
        return torch.tanh(h + x)


class ResNet(nn.Module):
    def __init__(self, input_size: int, hidden_dim: int, n_blocks: int, output_size: int = 2):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.n_blocks = n_blocks
        self.input_proj = nn.Linear(input_size, hidden_dim)
        nn.init.xavier_normal_(self.input_proj.weight)
        self.blocks = nn.ModuleList([ResidualBlock(hidden_dim) for _ in range(n_blocks)])
        self.output_proj = nn.Linear(hidden_dim, output_size)
        nn.init.xavier_normal_(self.output_proj.weight)

    def forward(self, x):
        h = torch.tanh(self.input_proj(x))
        for b in self.blocks:
            h = b(h)
        return self.output_proj(h)


@dataclass
class Metrics:
    r2_fx: float
    r2_fy: float
    rmse_fx: float
    rmse_fy: float
    mae_fx: float
    mae_fy: float


def r2(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2) + 1e-12
    return float(1.0 - ss_res / ss_tot)


def train_one(
    X: np.ndarray,
    y: np.ndarray,
    model: nn.Module,
    output_dir: Path,
    epochs: int,
    batch_size: int,
    lr: float,
    patience: int,
    ckpt_meta: dict,
    data_loader_seed: int | None = None,
    groups: np.ndarray | None = None,
):
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info("Using device %s", device)
    model = model.to(device)

    if groups is None:
        X_train, X_tmp, y_train, y_tmp = train_test_split(X, y, test_size=0.2, random_state=42)
        X_val, X_test, y_val, y_test = train_test_split(X_tmp, y_tmp, test_size=0.5, random_state=42)
    else:
        unique_groups = np.unique(groups)
        if len(unique_groups) < 5:
            raise ValueError("--split-by-scenario needs at least 5 unique scenarios")
        train_groups, tmp_groups = train_test_split(unique_groups, test_size=0.2, random_state=42)
        val_groups, test_groups = train_test_split(tmp_groups, test_size=0.5, random_state=42)
        train_mask = np.isin(groups, train_groups)
        val_mask = np.isin(groups, val_groups)
        test_mask = np.isin(groups, test_groups)
        logger.info(
            "scenario split: train=%d val=%d test=%d groups",
            len(train_groups),
            len(val_groups),
            len(test_groups),
        )
        X_train, y_train = X[train_mask], y[train_mask]
        X_val, y_val = X[val_mask], y[val_mask]
        X_test, y_test = X[test_mask], y[test_mask]

    scaler_X = StandardScaler().fit(X_train)
    scaler_y = StandardScaler().fit(y_train)

    X_train_s = torch.tensor(scaler_X.transform(X_train), dtype=torch.float32, device=device)
    y_train_s = torch.tensor(scaler_y.transform(y_train), dtype=torch.float32, device=device)
    X_val_s = torch.tensor(scaler_X.transform(X_val), dtype=torch.float32, device=device)
    y_val_s = torch.tensor(scaler_y.transform(y_val), dtype=torch.float32, device=device)
    X_test_s = torch.tensor(scaler_X.transform(X_test), dtype=torch.float32, device=device)

    with open(output_dir / "scalers.pkl", "wb") as f:
        pickle.dump({"X": scaler_X, "y": scaler_y}, f)

    opt = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    crit = nn.MSELoss()
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(opt, factor=0.5, patience=15)

    gen = None
    if data_loader_seed is not None:
        gen = torch.Generator()
        gen.manual_seed(int(data_loader_seed))
    dl = DataLoader(
        TensorDataset(X_train_s, y_train_s),
        batch_size=batch_size,
        shuffle=True,
        generator=gen,
    )

    best_val = float("inf")
    best_state = None
    patience_ctr = 0

    for ep in range(epochs):
        model.train()
        total = 0.0
        for xb, yb in dl:
            opt.zero_grad()
            loss = crit(model(xb), yb)
            loss.backward()
            opt.step()
            total += loss.item() * len(xb)
        train_loss = total / len(X_train_s)

        model.eval()
        with torch.no_grad():
            val_loss = float(crit(model(X_val_s), y_val_s).item())
        scheduler.step(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            patience_ctr = 0
        else:
            patience_ctr += 1

        if ep == 0 or (ep + 1) % 10 == 0:
            logger.info(f"epoch {ep+1:4d}/{epochs} train={train_loss:.6f} val={val_loss:.6f} best={best_val:.6f}")

        if patience_ctr >= patience:
            logger.info(f"early stopping at epoch {ep+1}")
            break

    if best_state is None:
        raise RuntimeError("Training failed to produce a best checkpoint.")
    model.load_state_dict(best_state)

    # Test metrics (in original units)
    model.eval()
    with torch.no_grad():
        y_pred_s = model(X_test_s).detach().cpu().numpy()
    y_pred = scaler_y.inverse_transform(y_pred_s)

    y_test = y_test.astype(np.float64)
    y_pred = y_pred.astype(np.float64)

    rmse = np.sqrt(np.mean((y_test - y_pred) ** 2, axis=0))
    mae = np.mean(np.abs(y_test - y_pred), axis=0)
    m = Metrics(
        r2_fx=r2(y_test[:, 0], y_pred[:, 0]),
        r2_fy=r2(y_test[:, 1], y_pred[:, 1]),
        rmse_fx=float(rmse[0]),
        rmse_fy=float(rmse[1]),
        mae_fx=float(mae[0]),
        mae_fy=float(mae[1]),
    )

    # Save checkpoint in nn_tire_model-compatible format
    ckpt = {
        "model_state_dict": model.state_dict(),
        **ckpt_meta,
        "input_size": int(X.shape[1]),
        "output_size": 2,
        "val_loss": float(best_val),
    }
    torch.save(ckpt, output_dir / "best_terrain_nn.pt")

    # Save paper-friendly metrics
    arch = ckpt_meta.copy()
    arch["n_params"] = int(sum(p.numel() for p in model.parameters()))
    payload = {
        "architecture": arch,
        "test": asdict(m),
    }
    (output_dir / "test_metrics.json").write_text(json.dumps(payload, indent=2))
    logger.info(f"✓ saved: {output_dir}/best_terrain_nn.pt, scalers.pkl, test_metrics.json")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True, help="CSV path (static or timeseries depending on mode)")
    p.add_argument("--output-dir", required=True)
    p.add_argument("--arch", required=True, choices=["mlp", "resnet"])
    p.add_argument("--mode", required=True, choices=["static", "temporal", "rate", "axle_rate", "rich", "rich_rate"])
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-2)
    p.add_argument("--patience", type=int, default=50)
    p.add_argument(
        "--seed",
        type=int,
        default=None,
        help="If set, fixes Python/NumPy/PyTorch RNG and DataLoader shuffle for reproducible runs.",
    )

    # Temporal
    p.add_argument("--K", type=int, default=1, help="Temporal window K (temporal mode only)")
    p.add_argument("--dt-nn", type=float, default=0.1, help="Temporal spacing (s) between window entries")
    p.add_argument("--record-dt", type=float, default=0.005, help="Dataset recording interval (s)")
    p.add_argument(
        "--allow-duplicate-rate-features",
        action="store_true",
        help="Bypass guard that rejects rate CSVs with duplicated steering_rate and d_slip_angle.",
    )

    # MLP size
    p.add_argument("--hidden", type=int, nargs="+", default=None)
    p.add_argument(
        "--split-by-scenario",
        action="store_true",
        help="Use complete scenario IDs for train/val/test so adjacent timesteps do not leak across splits.",
    )

    # ResNet size
    p.add_argument("--hidden-dim", type=int, default=16)
    p.add_argument("--n-blocks", type=int, default=2)

    args = p.parse_args()

    data_path = Path(args.data).resolve()
    out_dir = Path(args.output_dir).resolve()

    df = pd.read_csv(data_path)

    arch: Arch = args.arch
    mode: OpMode = args.mode

    # Remove pathological rows before feature construction.
    df = _apply_physical_filters(df, mode)

    if mode == "static":
        # Expect row-wise samples with OP_COLS + TERRAIN_COLS + OUT_COLS
        cols = OP_COLS + TERRAIN_COLS + OUT_COLS
        missing = [c for c in cols if c not in df.columns]
        if missing:
            raise ValueError(f"Static CSV missing columns: {missing}")
        X = df[OP_COLS + TERRAIN_COLS].values.astype(np.float32)
        y = df[OUT_COLS].values.astype(np.float32)
        temporal_K = 1
        rate_aug = False
        feature_cols = OP_COLS + TERRAIN_COLS
        offline_only = False
        source_df = df

    elif mode == "temporal":
        if args.K <= 1:
            raise ValueError("Temporal mode requires --K >= 2")
        required = ["scenario_id", "timestep"] + OP_COLS + TERRAIN_COLS + OUT_COLS
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise ValueError(f"Temporal CSV missing columns: {missing}")
        X, y = build_temporal_windows(df, K=args.K, dt_nn=args.dt_nn, record_dt=args.record_dt)
        temporal_K = int(args.K)
        rate_aug = False
        feature_cols = [f"hist_{i}_{c}" for i in range(args.K) for c in OP_COLS] + TERRAIN_COLS
        offline_only = False
        source_df = None

    elif mode in ("rate", "axle_rate"):
        rate_cols = ["d_slip_ratio", "d_slip_angle", "d_velocity"]
        use_axle_id = mode == "axle_rate"
        if all(c in df.columns for c in rate_cols):
            # Rate columns pre-computed (e.g. from collect_rate_data)
            required = OP_COLS + rate_cols + TERRAIN_COLS + OUT_COLS
            if use_axle_id:
                required = ["axle_id"] + required
            missing = [c for c in required if c not in df.columns]
            if missing:
                raise ValueError(f"{mode} CSV missing columns: {missing}")
            _check_rate_feature_independence(
                df, allow_duplicate=bool(args.allow_duplicate_rate_features)
            )
            logger.info("Using pre-computed rate columns from CSV")
            feature_cols = (["axle_id"] if use_axle_id else []) + OP_COLS + rate_cols + TERRAIN_COLS
            X = df[feature_cols].values.astype(np.float32)
            y = df[OUT_COLS].values.astype(np.float32)
            source_df = df
        else:
            # Compute rates from time-series data
            required = ["scenario_id", "timestep"] + OP_COLS + TERRAIN_COLS + OUT_COLS
            if use_axle_id:
                required.append("axle_id")
            missing = [c for c in required if c not in df.columns]
            if missing:
                raise ValueError(f"{mode} CSV missing columns: {missing}")
            df_r = compute_rates(df, record_dt=args.record_dt)
            feature_cols = (["axle_id"] if use_axle_id else []) + OP_COLS + rate_cols + TERRAIN_COLS
            X = df_r[feature_cols].values.astype(np.float32)
            y = df_r[OUT_COLS].values.astype(np.float32)
            source_df = df_r
        temporal_K = 1
        rate_aug = True
        offline_only = False

    elif mode == "rich":
        cols = RICH_SENSOR_COLS + TERRAIN_COLS + OUT_COLS
        missing = [c for c in cols if c not in df.columns]
        if missing:
            raise ValueError(f"Rich CSV missing columns: {missing}")
        feature_cols = RICH_SENSOR_COLS + TERRAIN_COLS
        X = df[feature_cols].values.astype(np.float32)
        y = df[OUT_COLS].values.astype(np.float32)
        temporal_K = 1
        rate_aug = False
        offline_only = True
        source_df = df

    else:  # rich_rate
        if all(c in df.columns for c in RICH_RATE_COLS):
            df_r = df.copy()
        else:
            required = ["scenario_id", "timestep"] + RICH_SENSOR_COLS + TERRAIN_COLS + OUT_COLS
            missing = [c for c in required if c not in df.columns]
            if missing:
                raise ValueError(f"Rich-rate CSV missing columns: {missing}")
            df_r = compute_named_rates(df, RICH_RATE_SOURCE_COLS, record_dt=args.record_dt)
            df_r = _apply_physical_filters(df_r, mode)
        feature_cols = RICH_SENSOR_COLS + RICH_RATE_COLS + TERRAIN_COLS
        missing = [c for c in feature_cols + OUT_COLS if c not in df_r.columns]
        if missing:
            raise ValueError(f"Rich-rate CSV missing columns: {missing}")
        X = df_r[feature_cols].values.astype(np.float32)
        y = df_r[OUT_COLS].values.astype(np.float32)
        temporal_K = 1
        rate_aug = True
        offline_only = True
        source_df = df_r

    # Filter finite
    mask = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    X, y = X[mask], y[mask]
    groups = None
    if args.split_by_scenario:
        if source_df is None or "scenario_id" not in source_df.columns:
            raise ValueError("--split-by-scenario is not available for this mode/dataset")
        scenario_ids = source_df.loc[mask, "scenario_id"].to_numpy(dtype=np.int64)
        # Rear-axle rows are encoded as base_scenario_id + 1_000_000.  Fold
        # them back so front/rear rows from the same run stay in the same split.
        groups = np.mod(scenario_ids, 1_000_000)
    logger.info(f"dataset: X={X.shape}, y={y.shape}")

    if args.seed is not None:
        set_global_seed(int(args.seed))
        logger.info(f"global seed={args.seed}")

    # Build model + metadata
    if arch == "mlp":
        hidden = args.hidden or [16, 8]
        model = MLP(input_size=X.shape[1], hidden_sizes=list(hidden))
        meta = {
            "architecture_type": "mlp",
            "hidden_sizes": list(hidden),
            "temporal_K": temporal_K,
            "rate_augmented": bool(rate_aug),
            "mode": mode,
            "feature_cols": list(feature_cols),
            "offline_only": bool(offline_only),
            "split_by_scenario": bool(args.split_by_scenario),
            **({"torch_seed": int(args.seed)} if args.seed is not None else {}),
        }
    else:
        model = ResNet(input_size=X.shape[1], hidden_dim=args.hidden_dim, n_blocks=args.n_blocks)
        meta = {
            "architecture_type": "resnet",
            "hidden_dim": int(args.hidden_dim),
            "n_blocks": int(args.n_blocks),
            "temporal_K": temporal_K,
            "rate_augmented": bool(rate_aug),
            "mode": mode,
            "feature_cols": list(feature_cols),
            "offline_only": bool(offline_only),
            "split_by_scenario": bool(args.split_by_scenario),
            **({"torch_seed": int(args.seed)} if args.seed is not None else {}),
        }

    train_one(
        X=X,
        y=y,
        model=model,
        output_dir=out_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        patience=args.patience,
        ckpt_meta=meta,
        data_loader_seed=args.seed,
        groups=groups,
    )


if __name__ == "__main__":
    main()
