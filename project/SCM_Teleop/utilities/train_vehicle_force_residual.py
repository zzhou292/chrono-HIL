#!/usr/bin/env python3
"""Train a vehicle-domain force-residual model from open-loop trace CSVs."""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "simulation"))

from force_residual_adapter import train_force_residual  # noqa: E402
from param_consistency import TERRAIN_PRESETS  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--trace-glob",
        default="logs/vehicle_force_traces/*.csv",
        help="Glob for detailed vehicle-trace CSVs",
    )
    p.add_argument(
        "--output",
        default="logs/force_residual/vehicle_force_residual_model.pt",
        help="Output checkpoint path",
    )
    p.add_argument("--hidden", default="64,64", help="Hidden layer sizes (comma-separated)")
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=512)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weight-decay", type=float, default=0.0)
    p.add_argument("--smooth-window", type=int, default=1)
    p.add_argument("--loss", choices=["mse", "huber"], default="mse")
    p.add_argument("--output-l1", type=float, default=0.0)
    p.add_argument("--scenario-split", action="store_true")
    p.add_argument("--min-time", type=float, default=2.0)
    args = p.parse_args()

    csv_paths = sorted(glob.glob(str((ROOT / args.trace_glob).resolve())))
    if not csv_paths:
        print(f"No trace CSVs matched: {args.trace_glob}", file=sys.stderr)
        return 1

    hidden = tuple(int(x) for x in args.hidden.split(",") if x)
    print(f"Training from {len(csv_paths)} trace CSV(s)")
    train_force_residual(
        csv_paths,
        str((ROOT / args.output).resolve()),
        TERRAIN_PRESETS,
        hidden_sizes=hidden,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        weight_decay=args.weight_decay,
        min_time=args.min_time,
        smooth_window=args.smooth_window,
        loss_type=args.loss,
        output_l1_penalty=args.output_l1,
        scenario_split=args.scenario_split,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
