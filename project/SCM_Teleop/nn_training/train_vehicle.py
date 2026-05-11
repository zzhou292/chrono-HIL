#!/usr/bin/env python3
"""Train a vehicle-data NN with wider physical filters than the rig-tuned defaults."""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Make train_variant importable
sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_variant import MLP, OP_COLS, TERRAIN_COLS, OUT_COLS, train_one, set_global_seed


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--hidden", type=int, nargs="+", default=[16, 4])
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-2)
    p.add_argument("--patience", type=int, default=50)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    df = pd.read_csv(args.data)

    # Vehicle-appropriate physical filters (wider than rig defaults)
    mask = (
        df["slip_ratio"].between(-3.0, 3.0)
        & df["slip_angle"].between(-1.5, 1.5)
        & df["velocity"].between(0.25, 20.0)
        & df["vertical_load"].between(500.0, 15000.0)
        & df["steering_rate"].between(-5.0, 5.0)
        & df["Fx"].between(-5e4, 5e4)
        & df["Fy"].between(-5e4, 5e4)
    )
    df = df[mask].reset_index(drop=True)
    print(f"After vehicle filters: {len(df)} rows")

    X = df[OP_COLS + TERRAIN_COLS].values.astype(np.float32)
    y = df[OUT_COLS].values.astype(np.float32)

    # Remove non-finite
    fin = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    X, y = X[fin], y[fin]
    print(f"Dataset: X={X.shape}, y={y.shape}")

    set_global_seed(args.seed)
    model = MLP(input_size=X.shape[1], hidden_sizes=list(args.hidden))
    meta = {
        "architecture_type": "mlp",
        "hidden_sizes": list(args.hidden),
        "temporal_K": 1,
        "rate_augmented": False,
        "torch_seed": args.seed,
    }

    train_one(
        X=X, y=y, model=model,
        output_dir=Path(args.output_dir),
        epochs=args.epochs, batch_size=args.batch_size,
        lr=args.lr, patience=args.patience,
        ckpt_meta=meta, data_loader_seed=args.seed,
    )


if __name__ == "__main__":
    main()
