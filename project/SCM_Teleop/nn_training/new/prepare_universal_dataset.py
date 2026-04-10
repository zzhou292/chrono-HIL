#!/usr/bin/env python3
"""
Prepare universal datasets for static / temporal / rate model training.

Design goals:
- Use ONE underlying time-series CSV as the source of truth (scenario_id, timestep, ...).
- Produce:
  1) A static (row-wise) CSV by subsampling within each scenario.
  2) A manifest JSON describing provenance (so it’s paper/audit friendly).

This script does *not* compute temporal windows or rates; those are handled
inside each training script to avoid duplicated CSV explosions.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd


REQUIRED_TIME_COLS = ["scenario_id", "timestep"]
REQUIRED_FEATURE_COLS = [
    "slip_ratio",
    "slip_angle",
    "velocity",
    "vertical_load",
    "steering_rate",
    "bekker_Kphi",
    "bekker_Kc",
    "bekker_n",
    "mohr_cohesion",
    "mohr_friction",
    "janosi_shear",
    "Fx",
    "Fy",
]


def prepare_static_from_timeseries(
    timeseries_csv: Path, static_csv: Path, subsample: int
) -> dict:
    static_csv.parent.mkdir(parents=True, exist_ok=True)

    # Stream process to avoid loading multi-GB files.
    first = True
    total_rows = 0
    reader = pd.read_csv(timeseries_csv, chunksize=500_000)
    for chunk in reader:
        missing = [c for c in (REQUIRED_TIME_COLS + REQUIRED_FEATURE_COLS) if c not in chunk.columns]
        if missing:
            raise ValueError(f"Missing columns in {timeseries_csv}: {missing}")

        # Subsample within each scenario_id
        chunk["_row"] = chunk.groupby("scenario_id").cumcount()
        sub = chunk[chunk["_row"] % subsample == 0].drop(columns=["_row"])

        # Keep consistent static schema (same as v6 scripts)
        keep = [
            "slip_ratio",
            "slip_angle",
            "velocity",
            "vertical_load",
            "steering_rate",
            "bekker_Kphi",
            "bekker_Kc",
            "bekker_n",
            "mohr_cohesion",
            "mohr_friction",
            "janosi_shear",
            "Fx",
            "Fy",
        ]
        sub = sub[keep]

        sub.to_csv(static_csv, mode="w" if first else "a", header=first, index=False)
        first = False
        total_rows += len(sub)

    return {
        "static_csv": str(static_csv),
        "static_subsample": subsample,
        "static_rows": int(total_rows),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--timeseries",
        required=True,
        help="Path to source time-series CSV (scenario_id,timestep,...)",
    )
    p.add_argument(
        "--out-dir",
        required=True,
        help="Output directory (will contain static.csv and manifest.json)",
    )
    p.add_argument(
        "--static-subsample",
        type=int,
        default=10,
        help="Keep every Nth row per scenario for static training (default: 10)",
    )
    args = p.parse_args()

    timeseries_csv = Path(args.timeseries).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    static_csv = out_dir / f"{timeseries_csv.stem}_static_subsample{args.static_subsample}.csv"
    manifest_path = out_dir / "manifest.json"

    info = {
        "source_timeseries_csv": str(timeseries_csv),
        "outputs": {},
    }
    info["outputs"]["static"] = prepare_static_from_timeseries(
        timeseries_csv, static_csv, args.static_subsample
    )

    manifest_path.write_text(json.dumps(info, indent=2))
    print(f"✓ Wrote static dataset: {static_csv}")
    print(f"✓ Wrote manifest:       {manifest_path}")


if __name__ == "__main__":
    main()

