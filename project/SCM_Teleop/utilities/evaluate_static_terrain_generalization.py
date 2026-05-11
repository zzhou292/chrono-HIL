#!/usr/bin/env python3
"""Evaluate static tire-model generalization under held-out terrain regions.

Random row splits overestimate true generalization for this dataset because
nearby terrain vectors appear in both train and test. This utility compares:

1. Standard random row split
2. Group split where entire terrain regions are held out

Terrain regions are defined by KMeans over the 6D terrain parameter subspace.
"""

from __future__ import annotations

import argparse
import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable
import sys

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.cluster import KMeans
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import GroupShuffleSplit, train_test_split
from sklearn.preprocessing import StandardScaler
from torch.utils.data import DataLoader, TensorDataset

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nn_training.train_variant import (
    MLP,
    ResNet,
    OP_COLS,
    OUT_COLS,
    TERRAIN_COLS,
    _apply_physical_filters,
    set_global_seed,
)


@dataclass
class SplitMetrics:
    name: str
    n_train: int
    n_val: int
    n_test: int
    r2_fx: float
    r2_fy: float
    rmse_fx: float
    rmse_fy: float
    mae_fx: float
    mae_fy: float


def _build_model(args: argparse.Namespace, input_size: int) -> nn.Module:
    if args.arch == "mlp":
        return MLP(input_size=input_size, hidden_sizes=list(args.hidden), output_size=2)
    return ResNet(
        input_size=input_size,
        hidden_dim=int(args.hidden_dim),
        n_blocks=int(args.n_blocks),
        output_size=2,
    )


def _as_loader(X: np.ndarray, y: np.ndarray, batch_size: int, shuffle: bool) -> DataLoader:
    ds = TensorDataset(
        torch.tensor(X, dtype=torch.float32),
        torch.tensor(y, dtype=torch.float32),
    )
    return DataLoader(ds, batch_size=batch_size, shuffle=shuffle)


def _fit_and_eval(
    X_train: np.ndarray,
    y_train: np.ndarray,
    X_val: np.ndarray,
    y_val: np.ndarray,
    X_test: np.ndarray,
    y_test: np.ndarray,
    args: argparse.Namespace,
    split_name: str,
) -> SplitMetrics:
    set_global_seed(int(args.seed))
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")

    scaler_X = StandardScaler().fit(X_train)
    scaler_y = StandardScaler().fit(y_train)

    X_train_s = scaler_X.transform(X_train)
    X_val_s = scaler_X.transform(X_val)
    X_test_s = scaler_X.transform(X_test)
    y_train_s = scaler_y.transform(y_train)
    y_val_s = scaler_y.transform(y_val)

    model = _build_model(args, input_size=X_train.shape[1]).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    crit = nn.MSELoss()
    train_loader = _as_loader(X_train_s, y_train_s, args.batch_size, shuffle=True)

    X_val_t = torch.tensor(X_val_s, dtype=torch.float32, device=device)
    y_val_t = torch.tensor(y_val_s, dtype=torch.float32, device=device)
    X_test_t = torch.tensor(X_test_s, dtype=torch.float32, device=device)

    best_val = float("inf")
    best_state = None
    patience_ctr = 0

    for _epoch in range(args.epochs):
        model.train()
        for xb, yb in train_loader:
            xb = xb.to(device)
            yb = yb.to(device)
            opt.zero_grad()
            loss = crit(model(xb), yb)
            loss.backward()
            opt.step()

        model.eval()
        with torch.no_grad():
            val_loss = float(crit(model(X_val_t), y_val_t).item())
        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            patience_ctr = 0
        else:
            patience_ctr += 1
        if patience_ctr >= args.patience:
            break

    if best_state is None:
        raise RuntimeError(f"no checkpoint captured for split {split_name}")
    model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        y_pred_s = model(X_test_t).detach().cpu().numpy()
    y_pred = scaler_y.inverse_transform(y_pred_s)

    return SplitMetrics(
        name=split_name,
        n_train=len(X_train),
        n_val=len(X_val),
        n_test=len(X_test),
        r2_fx=float(r2_score(y_test[:, 0], y_pred[:, 0])),
        r2_fy=float(r2_score(y_test[:, 1], y_pred[:, 1])),
        rmse_fx=float(np.sqrt(mean_squared_error(y_test[:, 0], y_pred[:, 0]))),
        rmse_fy=float(np.sqrt(mean_squared_error(y_test[:, 1], y_pred[:, 1]))),
        mae_fx=float(mean_absolute_error(y_test[:, 0], y_pred[:, 0])),
        mae_fy=float(mean_absolute_error(y_test[:, 1], y_pred[:, 1])),
    )


def _terrain_groups(df: pd.DataFrame, n_clusters: int, seed: int) -> np.ndarray:
    terrain = df[TERRAIN_COLS].to_numpy(dtype=np.float64)
    terrain_s = StandardScaler().fit_transform(terrain)
    km = KMeans(n_clusters=n_clusters, n_init=10, random_state=seed)
    return km.fit_predict(terrain_s)


def _group_split_indices(
    groups: np.ndarray,
    test_size: float,
    val_size: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    idx = np.arange(len(groups))
    gss_test = GroupShuffleSplit(n_splits=1, test_size=test_size, random_state=seed)
    train_val_idx, test_idx = next(gss_test.split(idx, groups=groups))

    train_val_groups = groups[train_val_idx]
    inner_idx = np.arange(len(train_val_idx))
    val_frac_inside = val_size / max(1e-8, 1.0 - test_size)
    gss_val = GroupShuffleSplit(n_splits=1, test_size=val_frac_inside, random_state=seed + 1)
    train_inner, val_inner = next(gss_val.split(inner_idx, groups=train_val_groups))

    train_idx = train_val_idx[train_inner]
    val_idx = train_val_idx[val_inner]
    return train_idx, val_idx, test_idx


def _print_metrics(metrics: Iterable[SplitMetrics]) -> None:
    for m in metrics:
        print(
            f"{m.name:18s} "
            f"n=({m.n_train},{m.n_val},{m.n_test}) "
            f"R2 Fx/Fy=({m.r2_fx:.4f},{m.r2_fy:.4f}) "
            f"RMSE Fx/Fy=({m.rmse_fx:.1f},{m.rmse_fy:.1f}) "
            f"MAE Fx/Fy=({m.mae_fx:.1f},{m.mae_fy:.1f})"
        )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", default="data/normal_mlp_resnet/scm_static_100k_v4.csv")
    p.add_argument("--arch", choices=["mlp", "resnet"], default="mlp")
    p.add_argument("--hidden", nargs="+", type=int, default=[16, 4])
    p.add_argument("--hidden-dim", type=int, default=16)
    p.add_argument("--n-blocks", type=int, default=2)
    p.add_argument("--epochs", type=int, default=60)
    p.add_argument("--patience", type=int, default=12)
    p.add_argument("--batch-size", type=int, default=512)
    p.add_argument("--lr", type=float, default=1e-2)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--terrain-clusters", type=int, default=64)
    p.add_argument("--group-repeats", type=int, default=3)
    p.add_argument("--test-size", type=float, default=0.1)
    p.add_argument("--val-size", type=float, default=0.1)
    p.add_argument("--max-rows", type=int, default=0, help="optional cap for faster debug")
    p.add_argument("--output-json", type=str, default="")
    p.add_argument("--cpu", action="store_true")
    args = p.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    df = pd.read_csv(Path(args.data))
    df = _apply_physical_filters(df, "static")
    if args.max_rows > 0 and len(df) > args.max_rows:
        df = df.sample(n=args.max_rows, random_state=args.seed).reset_index(drop=True)

    X = df[OP_COLS + TERRAIN_COLS].to_numpy(dtype=np.float32)
    y = df[OUT_COLS].to_numpy(dtype=np.float32)
    groups = _terrain_groups(df, args.terrain_clusters, args.seed)

    # Random row split baseline
    X_train, X_tmp, y_train, y_tmp = train_test_split(
        X, y, test_size=args.test_size + args.val_size, random_state=args.seed
    )
    rel_val = args.val_size / (args.test_size + args.val_size)
    X_val, X_test, y_val, y_test = train_test_split(
        X_tmp, y_tmp, test_size=1.0 - rel_val, random_state=args.seed
    )
    metrics = [
        _fit_and_eval(X_train, y_train, X_val, y_val, X_test, y_test, args, "random_row_split")
    ]

    # Held-out terrain region splits
    for rep in range(args.group_repeats):
        train_idx, val_idx, test_idx = _group_split_indices(
            groups,
            test_size=args.test_size,
            val_size=args.val_size,
            seed=args.seed + 100 * rep,
        )
        metrics.append(
            _fit_and_eval(
                X[train_idx],
                y[train_idx],
                X[val_idx],
                y[val_idx],
                X[test_idx],
                y[test_idx],
                args,
                f"terrain_group_{rep+1}",
            )
        )

    _print_metrics(metrics)

    if args.output_json:
        payload = {
            "config": vars(args),
            "metrics": [asdict(m) for m in metrics],
        }
        Path(args.output_json).write_text(json.dumps(payload, indent=2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
