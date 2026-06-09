#!/usr/bin/env python3
"""Regenerate EVERY figure in ``my_paper/paper.tex`` from the latest
benchmark result data --- one command, all outputs land in
``my_paper/paper_figures/`` with the exact filenames ``paper.tex`` uses.

    conda activate sim
    export ACADOS_SOURCE_DIR=$HOME/Documents/sbel/acados
    python benchmarking/make_paper_figures.py

This does NOT re-run the Chrono sweeps; it (re)plots from the newest
``benchmarking/results/<sweep>_*`` folders (and, for the UKF estimator
figures, re-runs the lightweight estimator pass over already-collected
SCM logs --- no Chrono). Run the sweeps first if the data is stale:

    python benchmarking/run.py --tier paper            # closed-loop sweeps
    python benchmarking/brake_test.py                  # brake stops
    python benchmarking/collision_warning_test.py      # FCW lead times
    python benchmarking/open_loop_terrain_estimator_benchmark.py
    python benchmarking/rig_vs_vehicle_tire_sweep.py
    # §VI UKF estimator benches (collect once, then this script re-plots):
    python benchmarking/bench_terrain_estimators_lhs.py --open-loop-throttle 0.75 --out-name lhs100_fair
    python benchmarking/bench_terrain_estimators_lhs.py --open-loop-throttle -1 --target-speed 5 --log-suffix _cl --out-name lhs100_cl

Every step is isolated; a step whose input data is missing is reported
SKIP/FAIL and does not abort the rest.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # benchmarking/
ROOT = HERE.parent                               # SCM_Final/
PY = sys.executable
FIGDIR = ROOT / "my_paper" / "paper_figures"
RESULTS = HERE / "results"


def _run(label: str, argv: list[str]) -> tuple[str, str]:
    try:
        r = subprocess.run(argv, cwd=str(ROOT), capture_output=True, text=True,
                           timeout=1800)
        if r.returncode == 0:
            return "ok", ""
        tail = (r.stderr or r.stdout).strip().splitlines()
        return "FAIL", (tail[-1] if tail else f"rc={r.returncode}")
    except Exception as e:  # noqa: BLE001
        return "FAIL", str(e)


def _copy_latest(label: str, src_glob: str, dst_name: str) -> tuple[str, str]:
    hits = sorted(RESULTS.glob(src_glob), key=lambda p: p.stat().st_mtime)
    if not hits:
        return "SKIP", f"no source matching results/{src_glob}"
    import shutil
    shutil.copy2(hits[-1], FIGDIR / dst_name)
    return "ok", f"<- {hits[-1].relative_to(RESULTS)}"


# (label, argv) figure generators, each writing straight into paper_figures/
STEPS = [
    ("sweep heatmaps (publish)", [PY, "-u", str(HERE / "publish_paper_figures.py")]),
    ("cte_master_heatmap",       [PY, "-u", str(HERE / "make_fig_cte_master_heatmap.py")]),
    ("tire_model_with_estimator_rms_cte_heatmap",
                                 [PY, "-u", str(HERE / "make_fig_tire_estimator_box.py")]),
    ("sys_arch",                 [PY, "-u", str(HERE / "make_fig_sys_arch.py")]),
    ("estimator scatters (Fig 6/7)",
                                 [PY, "-u", str(HERE / "make_estimator_diag_figs.py")]),
    ("lhs100_fair + estimator_overall (Figs 8/9, LIVE closed-loop)",
                                 [PY, "-u", str(HERE / "plot_estimator_lhs_live.py")]),
    ("terrain_estimator_comparison (canonical, LIVE 4-backend)",
                                 [PY, "-u", str(HERE / "plot_estimator_comparison_live.py")]),
    ("closed_loop_estimator_backends (cl reality check, re-plot)",
                                 [PY, "-u", str(HERE / "plot_cl_estimator_backends.py")]),
    ("cw_brake_validation (Fig 19)",
                                 [PY, "-u", str(HERE / "make_fig_cw_brake_validation.py")]),
    ("cw_timeline + cw_lead_vs_terrain (Fig 20/21)",
                                 [PY, "-u", str(HERE / "make_fig_collision_warning.py")]),
    ("fig1_force_4way (Fig 5)",  [PY, "-u", str(HERE / "make_fig1_4way.py")]),
]

# figures a sweep writes into its own result folder; copy to the paper name
COPIES = [
    ("rig_vs_vehicle_paired_bars_v2 (Fig 4)",
     "rig_vs_vehicle_tire_sweep_*/rig_vs_vehicle_paired_bars.png",
     "rig_vs_vehicle_paired_bars_v2.png"),
]


def main() -> int:
    FIGDIR.mkdir(parents=True, exist_ok=True)
    print(f"Regenerating paper figures into {FIGDIR}\n")
    rows = []
    for label, argv in STEPS:
        status, note = _run(label, argv)
        rows.append((status, label, note))
        print(f"  [{status:4}] {label}" + (f"  -- {note}" if note else ""))
    for label, src, dst in COPIES:
        status, note = _copy_latest(label, src, dst)
        rows.append((status, label, note))
        print(f"  [{status:4}] {label}  {note}")

    nbad = sum(1 for s, _, _ in rows if s != "ok")
    print(f"\n{len(rows) - nbad}/{len(rows)} figure groups OK"
          + (f"; {nbad} need attention (see SKIP/FAIL above)" if nbad else ""))
    return 1 if nbad else 0


if __name__ == "__main__":
    raise SystemExit(main())
