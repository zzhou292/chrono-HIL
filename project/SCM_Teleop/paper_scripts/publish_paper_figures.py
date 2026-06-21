#!/usr/bin/env python3
"""Copy canonical figures from the latest suite outputs into my_paper/paper_figures/.

For each sweep type, find the most recent results folder produced by
``run_paper_suite.py`` (or the sweep scripts directly) and copy a small,
curated set of figures + summary CSVs into ``my_paper/paper_figures/`` using
the filenames that ``my_paper/abstract.tex`` already references.  Any extra
figures that are useful for the paper but not yet in the abstract are copied
with descriptive names so the LaTeX can pick them up without renaming.

Run from the project root inside the ``sim`` conda env.  Pass ``--suite-dir``
to publish a specific suite folder; otherwise the script scans
``paper_scripts/results/`` for the newest matching directories.
"""

from __future__ import annotations

import argparse
import importlib
import json
import re
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import pandas as pd

# Result folders are named ``<prefix>_YYYYMMDD_HHMMSS``. We anchor on the
# timestamp suffix so ``safety_filter_sweep`` does not accidentally match
# ``safety_filter_sweep_planner_aware_<ts>``.
TIMESTAMP_RE = re.compile(r"_\d{8}_\d{6}$")

# Run-key columns used to dedupe across multiple result folders when filling
# gaps from a re-run. Keeping the latest occurrence preferentially picks
# values from the most recent (re-run) folder. ``distribution`` and
# ``case_label`` are included so the terrain_estimator sweep (no ``variant``
# column) does not collapse distinct OOD cases that share the same terrain.
RUN_KEY_COLS = ["variant", "distribution", "case_label",
                "terrain", "path", "speed_mps", "bumpiness", "seed"]

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = Path(__file__).resolve().parent / "results"
PAPER_FIG_DIR = PROJECT_ROOT / "my_paper" / "paper_figures"


@dataclass(frozen=True)
class PublishSpec:
    """One sweep type's mapping from suite-output filenames to paper filenames.

    ``plotter_module`` names a paper_scripts/ module exposing ``plot_figures``;
    when set, the publish step merges *all* matching folders' ``results.csv``
    rows (deduped on the run key) and re-runs that plotter against the merged
    CSV before copying figures. This lets gap-fill reruns top up an earlier
    partial sweep without the publish step regressing to the smaller dataset.
    """
    prefix: str                     # results-folder name prefix
    figures: dict[str, str]         # src filename -> destination filename
    extra_files: dict[str, str]     # extra src filename (e.g. summary csv) -> dst filename
    plotter_module: Optional[str] = None


SPECS: list[PublishSpec] = [
    PublishSpec(
        prefix="mpc_tire_model_sweep",
        figures={
            "tire_model_summary.png": "bench_tire_models.png",
            "tire_model_rms_cte_heatmap.png": "bench_tire_models_heatmap.png",
            "tire_model_metric_distributions.png": "bench_tire_models_distributions.png",
            "force_predicted_vs_actual_by_model.png": "tire_force_predicted_vs_actual.png",
            "force_prediction_error_summary.png": "tire_force_prediction_error.png",
        },
        extra_files={
            "results.csv": "bench_tire_models_results.csv",
            "summary_by_model.csv": "bench_tire_models_summary.csv",
        },
        plotter_module="mpc_tire_model_sweep",
    ),
    PublishSpec(
        prefix="mpcc_vs_mpc_speed_tracking",
        figures={
            "mpcc_variant_summary.png": "mpcc_vs_mpc_summary.png",
            "mpcc_speed_tracking_pareto.png": "mpcc_vs_mpc_pareto.png",
            "mpcc_speed_ratio_heatmap.png": "mpcc_vs_mpc_speed_heatmap.png",
        },
        extra_files={
            "results.csv": "mpcc_vs_mpc_results.csv",
            "summary_by_variant.csv": "mpcc_vs_mpc_summary.csv",
        },
        plotter_module="mpcc_vs_mpc_speed_tracking",
    ),
    PublishSpec(
        prefix="safety_filter_sweep_planner_aware",
        figures={
            "safety_filter_summary.png": "safety_filter_planner_aware_summary.png",
            "safety_collision_heatmap.png":
                "safety_filter_planner_aware_collision_heatmap.png",
            "safety_filter_metric_distributions.png":
                "safety_filter_planner_aware_distributions.png",
        },
        extra_files={
            "results.csv": "safety_filter_planner_aware_results.csv",
            "summary_by_filter.csv": "safety_filter_planner_aware_summary.csv",
        },
        plotter_module="safety_filter_sweep",
    ),
    PublishSpec(
        prefix="safety_filter_sweep",
        figures={
            "safety_filter_summary.png": "safety_filter_summary.png",
            "safety_collision_heatmap.png": "safety_filter_collision_heatmap.png",
            "safety_filter_metric_distributions.png": "safety_filter_distributions.png",
        },
        extra_files={
            "results.csv": "safety_filter_results.csv",
            "summary_by_filter.csv": "safety_filter_summary.csv",
        },
        plotter_module="safety_filter_sweep",
    ),
    PublishSpec(
        prefix="dob_cbf_nn_ablation",
        figures={
            "dob_cbf_nn_ablation_summary.png": "dob_cbf_nn_ablation_summary.png",
            "dob_cbf_nn_ablation_heatmap.png": "dob_cbf_nn_ablation_heatmap.png",
        },
        extra_files={"results.csv": "dob_cbf_nn_ablation_results.csv"},
        plotter_module="dob_cbf_nn_ablation",
    ),
    # autonomous_obstacle_tire_model_sweep encodes the safety flavor in its
    # output prefix (e.g. ``_mppi_mpc_blind``, ``_barrier_only``).  List the
    # variants we expect from the orchestrator; whichever ran most recently
    # for that exact prefix is what gets published.
    PublishSpec(
        prefix="autonomous_obstacle_tire_model_sweep_mppi_mpc_blind",
        figures={
            "autonomous_obstacle_tire_model_summary.png":
                "autonomous_obstacle_tire_summary.png",
            "autonomous_obstacle_collision_heatmap.png":
                "autonomous_obstacle_collision_heatmap.png",
            "autonomous_obstacle_clearance_heatmap.png":
                "autonomous_obstacle_clearance_heatmap.png",
            "autonomous_obstacle_metric_distributions.png":
                "autonomous_obstacle_distributions.png",
        },
        extra_files={"results.csv": "autonomous_obstacle_results.csv"},
        plotter_module="autonomous_obstacle_tire_model_sweep",
    ),
    PublishSpec(
        prefix="autonomous_obstacle_tire_model_sweep_barrier_only",
        figures={
            "autonomous_obstacle_tire_model_summary.png":
                "autonomous_obstacle_barrier_only_summary.png",
            "autonomous_obstacle_collision_heatmap.png":
                "autonomous_obstacle_barrier_only_collision_heatmap.png",
            "autonomous_obstacle_clearance_heatmap.png":
                "autonomous_obstacle_barrier_only_clearance_heatmap.png",
        },
        extra_files={"results.csv": "autonomous_obstacle_barrier_only_results.csv"},
        plotter_module="autonomous_obstacle_tire_model_sweep",
    ),
    PublishSpec(
        prefix="terrain_estimator_benchmark",
        figures={
            "terrain_estimator_summary.png": "closed_loop_estimator_learned.png",
            "terrain_estimator_true_vs_estimated.png":
                "closed_loop_estimator_true_vs_estimated.png",
            "terrain_estimator_error_heatmap.png":
                "closed_loop_estimator_error_heatmap.png",
        },
        extra_files={
            "results.csv": "closed_loop_estimator_results.csv",
            "summary_by_distribution.csv": "closed_loop_estimator_summary.csv",
        },
        plotter_module="terrain_estimator_benchmark",
    ),
    PublishSpec(
        prefix="latency_profile_figure",
        figures={
            "latency_profile_timeseries.png": "latency_profile_timeseries.png",
            "latency_profile_histogram.png": "latency_profile_histogram.png",
        },
        extra_files={
            "latency_profile_samples.csv": "latency_profile_samples.csv",
            "summary.csv": "latency_profile_summary.csv",
        },
    ),
    PublishSpec(
        prefix="latency_compensation_sweep",
        figures={
            "latency_compensation_summary_mpc_on.png":
                "latency_compensation_summary.png",
            "latency_collision_heatmap.png":
                "latency_compensation_collision_heatmap.png",
            "latency_metric_distributions.png":
                "latency_compensation_distributions.png",
        },
        extra_files={"results.csv": "latency_compensation_results.csv"},
        plotter_module="latency_compensation_sweep",
    ),
    PublishSpec(
        prefix="throttle_dob_ablation",
        figures={
            "throttle_dob_summary.png": "throttle_dob_ablation_summary.png",
            "throttle_dob_speed_ratio_heatmap.png":
                "throttle_dob_ablation_speed_heatmap.png",
            "throttle_dob_metric_distributions.png":
                "throttle_dob_ablation_distributions.png",
        },
        extra_files={
            "results.csv": "throttle_dob_ablation_results.csv",
            "summary_by_variant.csv": "throttle_dob_ablation_summary.csv",
        },
        plotter_module="throttle_dob_ablation",
    ),
    PublishSpec(
        prefix="mppi_seed_ablation",
        figures={
            "mppi_seed_ablation_summary.png": "mppi_seed_ablation_summary.png",
            "mppi_seed_ablation_collision_heatmap.png":
                "mppi_seed_ablation_collision_heatmap.png",
            "mppi_seed_ablation_metric_distributions.png":
                "mppi_seed_ablation_distributions.png",
        },
        extra_files={
            "results.csv": "mppi_seed_ablation_results.csv",
            "summary_by_variant.csv": "mppi_seed_ablation_summary.csv",
        },
        plotter_module="mppi_seed_ablation",
    ),
    PublishSpec(
        prefix="tire_model_with_estimator_ablation",
        figures={
            "tire_estimator_summary.png":
                "tire_model_with_estimator_summary.png",
            "tire_estimator_rms_cte_heatmap.png":
                "tire_model_with_estimator_rms_cte_heatmap.png",
            "tire_estimator_metric_distributions.png":
                "tire_model_with_estimator_distributions.png",
        },
        extra_files={
            "results.csv": "tire_model_with_estimator_results.csv",
            "summary_by_variant.csv": "tire_model_with_estimator_summary.csv",
        },
        plotter_module="tire_model_with_estimator_ablation",
    ),
    PublishSpec(
        prefix="sigma_gate_ablation",
        figures={
            "sigma_gate_ablation_summary.png": "sigma_gate_ablation_summary.png",
            "sigma_gate_ablation_collision_heatmap.png":
                "sigma_gate_ablation_collision_heatmap.png",
            "sigma_gate_ablation_metric_distributions.png":
                "sigma_gate_ablation_distributions.png",
        },
        extra_files={
            "results.csv": "sigma_gate_ablation_results.csv",
            "summary_by_variant.csv": "sigma_gate_ablation_summary.csv",
        },
        plotter_module="sigma_gate_ablation",
    ),
    # Human-in-the-loop safety-filter rounds. Figures are pre-rendered by
    # the HIL script itself; no plotter_module re-plot because the merge
    # key and grouping differ from the autonomous sweeps.
    PublishSpec(
        prefix="human_delay_compensation_rounds",
        figures={
            "human_delay_compensation_summary.png":
                "human_delay_compensation_summary.png",
            "human_delay_collision_heatmap.png":
                "human_delay_collision_heatmap.png",
        },
        extra_files={
            "results.csv": "human_delay_compensation_results.csv",
            "summary_by_filter_delay.csv": "human_delay_compensation_summary.csv",
        },
    ),
]


def matching_dirs(prefix: str, suite_dir: Path | None) -> list[Path]:
    """All results folders named exactly ``<prefix>_<timestamp>``, oldest first.

    Suite-dir filtering is intentionally not applied here: gap-fill reruns
    typically happen *outside* the original suite dir, so the merge across
    folders needs to include them.
    """
    def matches(name: str) -> bool:
        m = TIMESTAMP_RE.search(name)
        return m is not None and name[: m.start()] == prefix

    return sorted(
        (p for p in RESULTS_ROOT.iterdir() if p.is_dir() and matches(p.name)),
        key=lambda p: p.stat().st_mtime,
    )


def _load_plotter(module_name: str) -> Callable[[Path, Path], None] | None:
    """Import paper_scripts/<module_name>.plot_figures."""
    script_dir = Path(__file__).resolve().parent
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    try:
        mod = importlib.import_module(module_name)
    except Exception as e:
        print(f"  [warn] failed to import {module_name}: {e}")
        return None
    fn = getattr(mod, "plot_figures", None)
    if fn is None:
        print(f"  [warn] {module_name} has no plot_figures()")
    return fn


def _row_count(p: Path) -> int:
    try:
        return len(pd.read_csv(p / "results.csv"))
    except Exception:
        return 0


def _select_merge_window(dirs: list[Path]) -> list[Path]:
    """Pick the largest-row folder as the anchor; keep it and anything newer.

    Older folders (smoke runs, abandoned partial sweeps) sit below the anchor
    and are dropped, so the merge cannot accidentally over-aggregate. Gap-fill
    reruns naturally land after the anchor and so are included.
    """
    if not dirs:
        return []
    counts = [(d, _row_count(d)) for d in dirs]
    anchor = max(counts, key=lambda kv: kv[1])[0]
    anchor_mtime = anchor.stat().st_mtime
    return [d for d in dirs if d.stat().st_mtime >= anchor_mtime - 1.0]


def _merge_results(dirs: list[Path]) -> pd.DataFrame:
    """Concat results.csv from each dir, dedupe on the run key (keep latest)."""
    frames = []
    for d in dirs:
        p = d / "results.csv"
        if p.exists():
            try:
                frames.append(pd.read_csv(p))
            except Exception:
                continue
    if not frames:
        return pd.DataFrame()
    merged = pd.concat(frames, ignore_index=True)
    key_cols = [c for c in RUN_KEY_COLS if c in merged.columns]
    if key_cols:
        merged = merged.drop_duplicates(subset=key_cols, keep="last")
    return merged.reset_index(drop=True)


def publish_spec(spec: PublishSpec, suite_dir: Path | None, manifest_rows: list[dict]) -> bool:
    all_dirs = matching_dirs(spec.prefix, suite_dir)
    if not all_dirs:
        manifest_rows.append({"prefix": spec.prefix, "status": "no_match",
                              "source": None, "copied": [], "merged_rows": 0,
                              "merged_from": []})
        return False

    # Window the merge to the largest folder and anything newer; this is the
    # current generation. Older smoke / abandoned-sweep folders sit below the
    # anchor and are ignored so we don't over-aggregate.
    dirs = _select_merge_window(all_dirs)

    # Build the merged dataset across every matching folder. The most recent
    # folder hosts the regenerated figures so its raw/ tree backs the plots.
    merged = _merge_results(dirs)
    primary = dirs[-1]
    fig_src = primary / "figures"
    fig_src.mkdir(parents=True, exist_ok=True)

    if spec.plotter_module and not merged.empty:
        plotter = _load_plotter(spec.plotter_module)
        if plotter is not None:
            merged_csv = primary / "results_merged.csv"
            merged.to_csv(merged_csv, index=False)
            try:
                plotter(merged_csv, primary)
            except Exception as e:
                print(f"  [warn] {spec.plotter_module}.plot_figures failed: {e}")

    copied: list[str] = []
    for src_name, dst_name in spec.figures.items():
        src = fig_src / src_name
        if src.exists():
            shutil.copy2(src, PAPER_FIG_DIR / dst_name)
            copied.append(dst_name)
    for src_name, dst_name in spec.extra_files.items():
        # ``results.csv`` is replaced by the merged version so the published
        # CSV always matches the published figures.
        if src_name == "results.csv" and not merged.empty:
            merged.to_csv(PAPER_FIG_DIR / dst_name, index=False)
            copied.append(dst_name)
            continue
        src = primary / src_name
        if src.exists():
            shutil.copy2(src, PAPER_FIG_DIR / dst_name)
            copied.append(dst_name)
    manifest_rows.append({
        "prefix": spec.prefix,
        "status": "ok" if copied else "no_files",
        "source": str(primary),
        "copied": copied,
        "merged_rows": int(len(merged)),
        "merged_from": [str(d) for d in dirs],
    })
    return bool(copied)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--suite-dir", type=Path, default=None,
                   help="If set, only publish results produced at/after this suite folder's mtime.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    PAPER_FIG_DIR.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    any_copied = False
    for spec in SPECS:
        any_copied |= publish_spec(spec, args.suite_dir, rows)
    manifest_dst = PAPER_FIG_DIR / "publish_manifest.json"
    manifest_dst.write_text(json.dumps(rows, indent=2))
    print("Publish manifest:")
    for row in rows:
        if row["status"] == "ok":
            n_dirs = len(row.get("merged_from", []))
            n_rows = row.get("merged_rows", 0)
            extra = f" (merged {n_rows} rows from {n_dirs} folders)" if n_dirs > 1 else ""
            print(f"  [ok]   {row['prefix']}: {len(row['copied'])} files from {row['source']}{extra}")
        else:
            print(f"  [{row['status']}] {row['prefix']}: source={row['source']}")
    print(f"Wrote {manifest_dst}")
    if not any_copied:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
