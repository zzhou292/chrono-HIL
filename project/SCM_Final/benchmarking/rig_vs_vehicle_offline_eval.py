#!/usr/bin/env python3
"""Offline force-prediction eval for the active rig + vehicle checkpoints.

Evaluates each surrogate on TWO held-out CSVs so the comparison is fair on
both axes:

    1. archive/2026-05-23_final_cleanup/old_data/closed_loop_v3_rich_20260514_combined/training_data_rich_tire_frame.csv
       (legacy "canonical-preset + jitter" closed-loop data; in-distribution for
        the old vehicle models, OOD for everyone else)

    2. data/whole_vehicle/lhs/training_data_rich_tire_frame.csv
       (new LHS-terrain closed-loop data; in-distribution for the new
        vehicle_*_lhs models, OOD for the original vehicle models and for
        the rig)

Historical multi-checkpoint evals can be replayed by restoring the archived
checkpoints from archive/2026-05-23_model_checkpoint_and_root_artifact_cleanup/.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]

CHECKPOINTS = [
    ("rig_rate",     ROOT / "nn_models" / "rig_rate_64_32"),
    ("vehicle_rate", ROOT / "nn_models" / "vehicle_rate_64_32_lhs"),
]

EVAL_CSVS = {
    "canonical_jitter": ROOT / "data" / "closed_loop_v3_rich_20260514_combined"
                              / "training_data_rich_tire_frame.csv",
    "lhs_terrain":      ROOT / "data" / "closed_loop_v4_lhs"
                              / "training_data_rich_tire_frame.csv",
}


def run_eval(checkpoints: list[Path], data_csv: Path, out_dir: Path) -> Path:
    """Wrap nn_training/evaluate_tire_model.py on a list of checkpoints."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(ROOT / "nn_training" / "evaluate_tire_model.py"),
        "--data", str(data_csv),
        "--model-dir", *[str(c) for c in checkpoints],
        "--output-dir", str(out_dir),
    ]
    print(">>", " ".join(cmd))
    subprocess.run(cmd, check=True)
    return out_dir / "metrics.csv"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default=str(ROOT / "benchmarking" / "results"
                                            / "rig_vs_vehicle_offline_eval_v2"))
    args = p.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Filter to checkpoints actually on disk so the script works mid-pipeline.
    avail = [(name, d) for name, d in CHECKPOINTS
             if (d / "best_terrain_nn.pt").exists()]
    print(f"Available checkpoints: {len(avail)} / {len(CHECKPOINTS)}")
    for name, d in avail:
        print(f"  {name:>22}  {d}")

    name2dir = dict(avail)
    if not name2dir:
        print("No checkpoints to evaluate"); return

    all_rows = []
    for eval_name, csv_path in EVAL_CSVS.items():
        if not csv_path.exists():
            print(f"!! eval set missing: {csv_path}  -- skip")
            continue
        sub_out = out / eval_name
        try:
            metrics_csv = run_eval([d for _, d in avail], csv_path, sub_out)
        except subprocess.CalledProcessError as e:
            print(f"!! evaluate_tire_model failed on {eval_name}: {e}")
            continue
        m = pd.read_csv(metrics_csv)
        m["eval_set"] = eval_name
        # Map model-dir name back to friendly label.
        dir_to_label = {d.name: name for name, d in avail}
        m["label"] = m["model"].map(lambda x: dir_to_label.get(x, x))
        all_rows.append(m)
    if not all_rows: return

    combined = pd.concat(all_rows, ignore_index=True)
    combined.to_csv(out / "all_metrics.csv", index=False)
    print(f"wrote {out / 'all_metrics.csv'}")

    # Pretty per-axle summary, by (label, eval_set).
    print()
    for subset in ("all", "front", "rear"):
        s = combined[combined["subset"] == subset]
        if s.empty: continue
        pv = s.pivot_table(index="label", columns="eval_set",
                            values=["r2_fx", "r2_fy", "rmse_fx", "rmse_fy"])
        # Order labels by our preference if present.
        order = [n for n, _ in CHECKPOINTS]
        pv = pv.reindex([o for o in order if o in pv.index])
        print(f"\n=== subset = {subset} ===")
        print(pv.round(3).to_string())

    print()
    print(f"all done -> {out}")


if __name__ == "__main__":
    main()
