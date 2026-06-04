#!/usr/bin/env python3
"""Train a 5G-Traffic-Generator N-HiTS checkpoint and export a latency config.

This script wraps the public 0913ktg/5G-Traffic-Generator repo so the paper
workflow can reproduce the learned traffic trace used by the simulator.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo-dir", default="/tmp/5G-Traffic-Generator")
    p.add_argument("--dataset", default="youtube")
    p.add_argument("--datatype", choices=["ul", "dl"], default="ul")
    p.add_argument("--experiment-id", default="scm_youtube_ul")
    p.add_argument("--hyperopt-max-evals", type=int, default=1)
    p.add_argument("--inference-size", type=int, default=200)
    p.add_argument("--traffic-scale", type=float, default=45.0)
    p.add_argument("--capacity-bps", type=float, default=9e6)
    p.add_argument("--output-root", default=str(ROOT / "data" / "5g_generated"))
    p.add_argument("--skip-train", action="store_true",
                   help="Only run inference/export from an existing checkpoint.")
    return p.parse_args()


def patch_repo(repo_dir: Path) -> None:
    nhits = repo_dir / "N-HiTS-5G"
    tsdataset = nhits / "src" / "data" / "tsdataset.py"
    text = tsdataset.read_text()
    text = text.replace("X.drop(['unique_id', 'ds'], 1, inplace=True)",
                        "X.drop(['unique_id', 'ds'], axis=1, inplace=True)")
    tsdataset.write_text(text)

    inference = nhits / "inference.py"
    text = inference.read_text()
    text = text.replace(
        "dir = f'hp_result/{dataset}/{dataset}_{horizon}/hyperopt_{data_type}.p'",
        "dir = f'hp_result/{dataset}/{dataset}_{horizon}/hyperopt_{experiment_id}.p'",
    )
    text = text.replace(
        "ckpt = torch.load(f'best_ckpt/{dataset}/{dataset}_{horizon}/{data_type}.ckpt')",
        "ckpt = torch.load(f'best_ckpt/{dataset}/{dataset}_{horizon}/{experiment_id}.ckpt')",
    )
    text = text.replace(
        "for i in tqdm(range(int(args.size))):\n        batch = next(iter(train_loader))",
        "loader_iter = iter(train_loader)\n"
        "    for i in tqdm(range(int(args.size))):\n"
        "        try:\n"
        "            batch = next(loader_iter)\n"
        "        except StopIteration:\n"
        "            loader_iter = iter(train_loader)\n"
        "            batch = next(loader_iter)",
    )
    inference.write_text(text)


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, cwd=str(cwd), check=True)


def export_outputs(args: argparse.Namespace, repo_dir: Path) -> Path:
    nhits = repo_dir / "N-HiTS-5G"
    out_dir = Path(args.output_root) / args.experiment_id
    out_dir.mkdir(parents=True, exist_ok=True)

    horizon = 10
    ckpt = nhits / "best_ckpt" / args.dataset / f"{args.dataset}_{horizon}" / f"{args.experiment_id}.ckpt"
    hp = nhits / "hp_result" / args.dataset / f"{args.dataset}_{horizon}" / f"hyperopt_{args.experiment_id}.p"
    npy = nhits / "inference" / args.datatype / f"{args.dataset}_{horizon}_{args.experiment_id}.npy"
    for src in (ckpt, hp, npy):
        if not src.exists():
            raise FileNotFoundError(src)
        shutil.copy2(src, out_dir / src.name)

    arr = np.asarray(np.load(npy), dtype=float)
    arr = np.maximum(arr, 0.0)
    if args.datatype == "ul":
        df = pd.DataFrame({"UL_bitrate": arr, "DL_bitrate": arr * 0.35})
    else:
        df = pd.DataFrame({"DL_bitrate": arr, "UL_bitrate": arr * 0.35})
    traffic_csv = out_dir / "generated_traffic.csv"
    df.to_csv(traffic_csv, index=False)

    summary = {
        "dataset": args.dataset,
        "datatype": args.datatype,
        "experiment_id": args.experiment_id,
        "horizon": horizon,
        "n_samples": int(arr.size),
        "mean_bps": float(np.mean(arr)),
        "std_bps": float(np.std(arr)),
        "min_bps": float(np.min(arr)),
        "p50_bps": float(np.percentile(arr, 50)),
        "p95_bps": float(np.percentile(arr, 95)),
        "max_bps": float(np.max(arr)),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    rel_path = Path("../../data/5g_generated") / args.experiment_id / "generated_traffic.csv"
    profile = {
        "description": f"N-HiTS-5G generated {args.dataset} {args.datatype.upper()} traffic mapped to scheduled latency",
        "sample_period_s": 0.05,
        "duration_s": 60.0,
        "loop": True,
        "seed": 41,
        "channels": {
            "control": {
                "source": {
                    "kind": "5g_repo_csv",
                    "path": str(rel_path),
                    "direction": "UL_bitrate" if args.datatype == "ul" else "DL_bitrate",
                    "start_index": 0,
                },
                "traffic_scale": args.traffic_scale,
                "base_delay_ms": 18.0,
                "jitter_ms": 3.0,
                "min_delay_ms": 4.0,
                "max_delay_ms": 450.0,
                "capacity_bps": args.capacity_bps,
                "queue_gain_ms": 20.0,
                "segments": [
                    {"start_s": 0.0, "end_s": 8.0, "regime": "good"},
                    {"start_s": 8.0, "end_s": 16.0, "regime": "poor", "extra_ms": 60.0},
                    {"start_s": 16.0, "end_s": 24.0, "regime": "good"},
                    {"start_s": 24.0, "end_s": 30.0, "regime": "outage", "extra_ms": 120.0},
                    {"start_s": 30.0, "end_s": 60.0, "regime": "nominal"},
                ],
            },
            "manual": {"copy_from": "control"},
            "camera": {"copy_from": "control", "scale": 1.45, "offset_ms": 8.0},
        },
    }
    profile_dir = ROOT / "config" / "latency_profiles"
    profile_dir.mkdir(parents=True, exist_ok=True)
    profile_path = profile_dir / f"5g_nhits_{args.dataset}_{args.datatype}_{args.experiment_id}.json"
    profile_path.write_text(json.dumps(profile, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"Exported: {out_dir}")
    print(f"Latency profile: {profile_path}")
    return out_dir


def main() -> None:
    args = parse_args()
    repo_dir = Path(args.repo_dir).expanduser().resolve()
    nhits = repo_dir / "N-HiTS-5G"
    if not nhits.exists():
        raise FileNotFoundError(f"N-HiTS-5G repo directory not found: {nhits}")

    patch_repo(repo_dir)
    if not args.skip_train:
        run([
            sys.executable, "-u", "model_train.py",
            "--dataset", args.dataset,
            "--datatype", args.datatype,
            "--hyperopt_max_evals", str(args.hyperopt_max_evals),
            "--experiment_id", args.experiment_id,
        ], nhits)
    run([
        sys.executable, "-u", "inference.py",
        "--dataset", args.dataset,
        "--datatype", args.datatype,
        "--experiment_id", args.experiment_id,
        "--size", str(args.inference_size),
    ], nhits)
    export_outputs(args, repo_dir)


if __name__ == "__main__":
    main()
