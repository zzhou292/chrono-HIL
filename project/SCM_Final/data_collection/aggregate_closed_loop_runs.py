#!/usr/bin/env python3
"""Aggregate successful per-run closed-loop tire CSVs from one or more shards."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd


def _read_csvs(shards: list[Path], name: str) -> list[pd.DataFrame]:
    frames: list[pd.DataFrame] = []
    for shard in shards:
        for csv_path in sorted((shard / "per_run").glob(f"scn_*/{name}")):
            try:
                df = pd.read_csv(csv_path)
            except Exception as exc:
                print(f"skip {csv_path}: {exc}")
                continue
            if len(df) == 0:
                continue
            frames.append(df)
    return frames


def _read_manifest_rows(shards: list[Path]) -> list[dict]:
    rows: list[dict] = []
    for shard in shards:
        for run_dir in sorted((shard / "per_run").glob("scn_*")):
            cfg_path = run_dir / "terrain.yaml"
            scenario_id = int(run_dir.name.split("_")[-1])
            row = {"scenario_id": scenario_id, "shard": str(shard)}
            if cfg_path.exists():
                try:
                    row["terrain_config"] = json.dumps(json.loads(cfg_path.read_text()), sort_keys=True)
                except Exception:
                    row["terrain_config"] = cfg_path.read_text()
            rows.append(row)
    return rows


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--shard", type=Path, action="append", required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()

    out = args.output
    out.mkdir(parents=True, exist_ok=True)

    compact = _read_csvs(args.shard, "tire.csv")
    rich = _read_csvs(args.shard, "tire_rich.csv")
    if not compact:
        raise SystemExit("No compact tire.csv files found")

    compact_df = pd.concat(compact, ignore_index=True)
    compact_df.to_csv(out / "training_data.csv", index=False)
    compact_tire = compact_df.copy()
    compact_tire["Fy"] = -compact_tire["Fy"]
    compact_tire.to_csv(out / "training_data_tire_frame.csv", index=False)

    if rich:
        rich_df = pd.concat(rich, ignore_index=True)
        rich_df.to_csv(out / "training_data_rich.csv", index=False)
        rich_tire = rich_df.copy()
        rich_tire["Fy"] = -rich_tire["Fy"]
        rich_tire.to_csv(out / "training_data_rich_tire_frame.csv", index=False)
    else:
        rich_df = pd.DataFrame()

    manifest = pd.DataFrame(_read_manifest_rows(args.shard))
    if not manifest.empty:
        manifest.to_csv(out / "manifest.csv", index=False)

    print(f"compact rows: {len(compact_df):,}")
    print(f"rich rows:    {len(rich_df):,}")
    print(f"output:       {out}")


if __name__ == "__main__":
    main()
