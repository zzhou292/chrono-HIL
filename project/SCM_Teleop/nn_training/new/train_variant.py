#!/usr/bin/env python3
"""
Train an experiment variant for tire force prediction (Fx, Fy).

Variants covered:
- arch:    mlp | resnet
- mode:    static | temporal | rate

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


OpMode = Literal["static", "temporal", "rate"]
Arch = Literal["mlp", "resnet"]


OP_COLS = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate"]
TERRAIN_COLS = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
OUT_COLS = ["Fx", "Fy"]


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
):
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info("Using device %s", device)
    model = model.to(device)

    X_train, X_tmp, y_train, y_tmp = train_test_split(X, y, test_size=0.2, random_state=42)
    X_val, X_test, y_val, y_test = train_test_split(X_tmp, y_tmp, test_size=0.5, random_state=42)

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
    p.add_argument("--mode", required=True, choices=["static", "temporal", "rate"])
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

    # MLP size
    p.add_argument("--hidden", type=int, nargs="+", default=None)

    # ResNet size
    p.add_argument("--hidden-dim", type=int, default=16)
    p.add_argument("--n-blocks", type=int, default=2)

    args = p.parse_args()

    data_path = Path(args.data).resolve()
    out_dir = Path(args.output_dir).resolve()

    df = pd.read_csv(data_path)

    arch: Arch = args.arch
    mode: OpMode = args.mode

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

    else:  # rate
        rate_cols = ["d_slip_ratio", "d_slip_angle", "d_velocity"]
        if all(c in df.columns for c in rate_cols):
            # Rate columns pre-computed (e.g. from collect_rate_data)
            required = OP_COLS + rate_cols + TERRAIN_COLS + OUT_COLS
            missing = [c for c in required if c not in df.columns]
            if missing:
                raise ValueError(f"Rate CSV missing columns: {missing}")
            logger.info("Using pre-computed rate columns from CSV")
            X = df[OP_COLS + rate_cols + TERRAIN_COLS].values.astype(np.float32)
            y = df[OUT_COLS].values.astype(np.float32)
        else:
            # Compute rates from time-series data
            required = ["scenario_id", "timestep"] + OP_COLS + TERRAIN_COLS + OUT_COLS
            missing = [c for c in required if c not in df.columns]
            if missing:
                raise ValueError(f"Rate CSV missing columns: {missing}")
            df_r = compute_rates(df, record_dt=args.record_dt)
            X = df_r[OP_COLS + rate_cols + TERRAIN_COLS].values.astype(np.float32)
            y = df_r[OUT_COLS].values.astype(np.float32)
        temporal_K = 1
        rate_aug = True

    # Filter finite
    mask = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    X, y = X[mask], y[mask]
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
    )


if __name__ == "__main__":
    main()

