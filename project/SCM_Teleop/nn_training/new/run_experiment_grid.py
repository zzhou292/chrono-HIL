#!/usr/bin/env python3
"""
Run a grid of training jobs (MLP/ResNet × static/temporal/rate × sizes) and
write outputs under `project/SCM_Teleop/nn_models/<tag>/...`.

This is intentionally simple/reproducible: it shells out to `train_variant.py`
so each model is a self-contained run directory.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--timeseries", required=True, help="Source time-series CSV")
    p.add_argument("--static", required=True, help="Prepared static CSV (from prepare_universal_dataset.py)")
    p.add_argument("--tag", default="exp_v1", help="Prefix tag for model directories")
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--patience", type=int, default=50)
    p.add_argument("--record-dt", type=float, default=0.005)
    p.add_argument(
        "--dt-nn",
        type=float,
        default=0.1,
        help="Temporal window spacing (s); passed to train_variant temporal mode",
    )
    args = p.parse_args()

    script_dir = Path(__file__).resolve().parent
    project_dir = script_dir.parent.parent  # SCM_Teleop/
    models_dir = project_dir / "nn_models"
    train_script = script_dir / "train_variant.py"

    grids = []

    # Static MLP
    for hidden in ([12, 2], [16, 4], [16, 8], [24, 12], [32, 16]):
        grids.append(dict(arch="mlp", mode="static", data=args.static, hidden=hidden, name=f"{args.tag}_mlp_{'_'.join(map(str,hidden))}"))

    # Static ResNet
    for hdim, nblk in ((8, 2), (16, 2), (16, 4), (32, 2)):
        grids.append(dict(arch="resnet", mode="static", data=args.static, hidden_dim=hdim, n_blocks=nblk, name=f"{args.tag}_resnet_h{hdim}_b{nblk}"))

    # Temporal (K)
    for K in (3, 5, 10):
        for hidden in ([16, 8], [24, 12]):
            grids.append(dict(arch="mlp", mode="temporal", data=args.timeseries, K=K, hidden=hidden,
                              name=f"{args.tag}_mlp_temporal_K{K}_{'_'.join(map(str,hidden))}"))
        for hdim, nblk in ((16, 2), (32, 2)):
            grids.append(dict(arch="resnet", mode="temporal", data=args.timeseries, K=K, hidden_dim=hdim, n_blocks=nblk,
                              name=f"{args.tag}_resnet_temporal_K{K}_h{hdim}_b{nblk}"))

    # Rate (static K=1, but includes finite-diff rates)
    for hidden in ([16, 8], [24, 12]):
        grids.append(dict(arch="mlp", mode="rate", data=args.timeseries, hidden=hidden,
                          name=f"{args.tag}_mlp_rate_{'_'.join(map(str,hidden))}"))
    for hdim, nblk in ((16, 2), (32, 2)):
        grids.append(dict(arch="resnet", mode="rate", data=args.timeseries, hidden_dim=hdim, n_blocks=nblk,
                          name=f"{args.tag}_resnet_rate_h{hdim}_b{nblk}"))

    ok = 0
    for i, cfg in enumerate(grids, 1):
        out_dir = models_dir / cfg["name"]
        metrics = out_dir / "test_metrics.json"
        if metrics.exists():
            print(f"[{i:02d}/{len(grids)}] SKIP {cfg['name']} (exists)")
            ok += 1
            continue

        cmd = [
            sys.executable,
            str(train_script),
            "--data",
            str(Path(cfg["data"]).resolve()),
            "--output-dir",
            str(out_dir),
            "--arch",
            cfg["arch"],
            "--mode",
            cfg["mode"],
            "--epochs",
            str(args.epochs),
            "--batch-size",
            str(args.batch_size),
            "--patience",
            str(args.patience),
            "--record-dt",
            str(args.record_dt),
        ]

        if cfg["mode"] == "temporal":
            cmd += ["--K", str(cfg["K"]), "--dt-nn", str(args.dt_nn)]

        if cfg["arch"] == "mlp":
            cmd += ["--hidden"] + [str(h) for h in cfg["hidden"]]
        else:
            cmd += ["--hidden-dim", str(cfg["hidden_dim"]), "--n-blocks", str(cfg["n_blocks"])]

        print(f"[{i:02d}/{len(grids)}] TRAIN {cfg['name']}")
        res = subprocess.run(cmd)
        if res.returncode != 0:
            raise SystemExit(res.returncode)
        ok += 1

    print(f"✓ trained/verified {ok}/{len(grids)} models")


if __name__ == "__main__":
    main()

