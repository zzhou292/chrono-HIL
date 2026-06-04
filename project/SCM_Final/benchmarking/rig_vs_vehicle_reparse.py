#!/usr/bin/env python3
"""Re-parse the per-run logs of a completed sweep with the current common.py.

When the diag CSV / collision log / shield log parsers were strengthened
to tolerate ragged or zero-byte files, every previously-failed run with
intact on-disk logs becomes recoverable. This script re-walks a sweep
result directory and rebuilds ``results.csv`` from the existing run
dirs --- no new Chrono runs needed.

Use case: the multi-filter sweep produced 71/128 worker exceptions even
though every Chrono process completed normally; the failures were all
in ``parse_*_csv`` on the parent side. With the fix in place we just
need to re-parse, not re-run.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    RunResult, parse_collision_csv, parse_diag_csv,
    parse_log_summary, parse_shield_csv, parse_sim_diag_csv,
    find_diag_csv, write_results_csv,
)


def reparse_one(run_dir: Path, row: pd.Series,
                experiment_name: str) -> RunResult:
    """Rebuild a RunResult from an existing run_dir."""
    log_path = run_dir / "run.log"
    text = log_path.read_text() if log_path.exists() else ""

    # parse_log_summary doesn't need the dir.
    controller_mode = str(row.get("controller_mode", "standard"))
    nn_model = str(row.get("nn_model", ""))
    variant = str(row.get("variant", ""))
    mpc_model = str(row.get("mpc_model", "nn"))

    # diag_csv is referenced in the row but a stale path may not exist on
    # disk -- so re-locate it from the run_dir (find_diag_csv globs).
    diag = find_diag_csv(run_dir, controller_mode, 0.0)
    collision = run_dir / "collision_log.csv"
    sim_diag = run_dir / "sim_diag.csv"
    shield = None
    for name in ("mppi_shield_log.csv", "nmpc_shield_log.csv", "cbf_filter_log.csv"):
        p = run_dir / name
        if p.exists():
            shield = p
            break

    sim_completed = "Simulation complete" in text
    # If the run produced a diag CSV and the log says simulation complete,
    # call it OK; otherwise leave as the original status so genuine timeouts
    # stay marked.
    status = "ok" if (diag is not None and sim_completed) else str(row.get("status", "unknown"))

    speed = float(row.get("speed_mps", 0.0))

    result = RunResult(
        experiment=experiment_name,
        variant=variant,
        controller_mode=controller_mode,
        mpc_model=mpc_model,
        nn_model=nn_model,
        terrain=str(row.get("terrain", "")),
        path=str(row.get("path", "")),
        speed_mps=speed,
        bumpiness=int(row.get("bumpiness", 0)) if pd.notna(row.get("bumpiness")) else 0,
        seed=int(row.get("seed", 0)) if pd.notna(row.get("seed")) else 0,
        run_dir=str(run_dir),
        rc=0 if status == "ok" else int(row.get("rc", -1)),
        wall_s=float(row.get("wall_s", 0.0)) if pd.notna(row.get("wall_s")) else 0.0,
        status=status,
        diag_csv=str(diag) if diag else "",
        collision_csv=str(collision) if collision.exists() else "",
        shield_csv=str(shield) if shield else "",
    )
    for k, v in parse_log_summary(text).items():
        setattr(result, k, v)
    if diag is not None:
        try:
            for k, v in parse_diag_csv(diag, controller_mode, speed).items():
                if hasattr(result, k):
                    setattr(result, k, v)
                else:
                    result.extra[k] = v
        except Exception as exc:
            result.extra["reparse_diag_exc"] = repr(exc)
    if collision.exists():
        try:
            for k, v in parse_collision_csv(collision).items():
                setattr(result, k, v)
        except Exception as exc:
            result.extra["reparse_coll_exc"] = repr(exc)
    if sim_diag.exists():
        try:
            for k, v in parse_sim_diag_csv(sim_diag).items():
                # Only fill if not already set by collision_log.
                current = getattr(result, k, math.nan)
                if k in ("collisions", "near_misses"):
                    if not (collision.exists() and (collision.stat().st_size > 0)):
                        setattr(result, k, v)
                elif not isinstance(current, (int,)) and (
                        not math.isfinite(float(current))):
                    setattr(result, k, v)
        except Exception as exc:
            result.extra["reparse_simdiag_exc"] = repr(exc)
    if shield is not None:
        try:
            for k, v in parse_shield_csv(shield).items():
                setattr(result, k, v)
        except Exception as exc:
            result.extra["reparse_shield_exc"] = repr(exc)
    return result


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("sweep_dir", help="Sweep results directory containing "
                                      "results.csv and raw/.")
    args = p.parse_args()

    sweep_dir = Path(args.sweep_dir)
    rcsv = sweep_dir / "results.csv"
    if not rcsv.exists():
        print(f"no results.csv in {sweep_dir}"); return

    df = pd.read_csv(rcsv)
    print(f"original: {len(df)} rows, ok={(df['status']=='ok').sum()}")

    experiment_name = df["experiment"].iloc[0] if "experiment" in df.columns else "rig_vs_vehicle_filter_sweep"
    new_rows = []
    for _, r in df.iterrows():
        rd = Path(str(r.get("run_dir", "")))
        if rd.exists():
            new_rows.append(reparse_one(rd, r, experiment_name))
        else:
            # Keep the original row as-is.
            d = {k: r[k] for k in df.columns if k in r.index}
            new_rows.append(None)
            print(f"  WARNING: run_dir gone, keeping original row: {r.get('variant')} {r.get('terrain')}/{r.get('path')}")

    # Backup old and write new
    bkp = sweep_dir / f"results.csv.bak-{int(time.time())}"
    df.to_csv(bkp, index=False)
    print(f"backed up to {bkp}")

    # Compose final list with reparsed RunResults
    out_rows: list[dict] = []
    for new_res, (_, orig) in zip(new_rows, df.iterrows()):
        if new_res is not None:
            d = asdict(new_res)
            extra = d.pop("extra", {}) or {}
            d.update({f"extra_{k}": v for k, v in extra.items()})
        else:
            d = orig.to_dict()
        out_rows.append(d)
    pd.DataFrame(out_rows).to_csv(rcsv, index=False)

    ok_after = sum(1 for r in new_rows if r is not None and r.status == "ok")
    print(f"reparsed: {len(new_rows)} rows, ok={ok_after}")


if __name__ == "__main__":
    main()
