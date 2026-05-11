#!/usr/bin/env python3
"""Fine-tune a rig-trained NN model on vehicle data.

Loads pre-trained weights, fits new scalers on vehicle data,
then fine-tunes the model with a low learning rate to adapt the
force scale while preserving learned terrain sensitivity.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_variant import MLP, OP_COLS, TERRAIN_COLS, OUT_COLS, train_one, set_global_seed


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True, help="Vehicle CSV data")
    p.add_argument("--pretrained", required=True, help="Path to rig model dir (nn_models/paper_v2_mlp_32_16)")
    p.add_argument("--output-dir", required=True, help="Output dir for fine-tuned model")
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-3, help="Fine-tune learning rate (lower than init)")
    p.add_argument("--patience", type=int, default=40)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    df = pd.read_csv(args.data)

    # Vehicle-appropriate physical filters
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

    fin = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    X, y = X[fin], y[fin]
    print(f"Dataset: X={X.shape}, y={y.shape}")

    # Load pre-trained checkpoint
    pretrained_dir = Path(args.pretrained)
    ckpt = torch.load(pretrained_dir / "best_terrain_nn.pt", map_location="cpu", weights_only=False)
    hidden_sizes = ckpt["hidden_sizes"]
    input_size = ckpt["input_size"]
    print(f"Pre-trained model: hidden={hidden_sizes}, input={input_size}, params={sum(p.numel() for p in MLP(input_size, hidden_sizes).parameters())}")

    set_global_seed(args.seed)
    model = MLP(input_size=input_size, hidden_sizes=hidden_sizes)
    model.load_state_dict(ckpt["model_state_dict"])
    print("Loaded pre-trained weights")

    meta = {
        "architecture_type": "mlp",
        "hidden_sizes": hidden_sizes,
        "temporal_K": 1,
        "rate_augmented": False,
        "torch_seed": args.seed,
        "finetuned_from": str(pretrained_dir),
    }

    # train_one will fit new scalers on vehicle data and fine-tune
    train_one(
        X=X, y=y, model=model,
        output_dir=Path(args.output_dir),
        epochs=args.epochs, batch_size=args.batch_size,
        lr=args.lr, patience=args.patience,
        ckpt_meta=meta, data_loader_seed=args.seed,
    )


if __name__ == "__main__":
    main()
