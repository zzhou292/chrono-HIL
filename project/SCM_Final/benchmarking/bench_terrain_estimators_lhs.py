#!/usr/bin/env python3
"""bench_terrain_estimators_lhs.py
====================================

Run an N-sample uniform LHS sweep over the Bekker--Mohr terrain box
(paper118 Table I) and compare three terrain-estimation pipelines on
the same Chrono SCM ground-truth logs:

1. **Bekker-UKF** — Dallas-style state-augmented UKF, 4-wheel
   double-track bicycle, analytical Bekker--Wong tire force.
2. **NN-UKF** — same UKF with the paper118-spec neural-network tire
   surrogate (uniform LHS rig sweep, widened α / Fz).
3. **Learned MLP** — the deployed sliding-window proprioceptive
   regressor (``nn_models/terrain_window_mlp/``).

Outputs:

* ``my_paper/paper_figures/terrain_estimator_bench_<N>.png`` —
  scatter (true n vs. estimated n) per estimator with ±10 %% and
  ±20 %% bands, plus an error-CDF panel.
* ``my_paper/paper_figures/terrain_estimator_bench_<N>.csv`` —
  per-(terrain, estimator) row with terrain parameters, converged n,
  and percentage error.
* ``data/dallas_scm/lhs<N>/`` — per-scenario YAMLs +
  NPZ logs, kept so the benchmark is replayable without rerunning
  Chrono.

Usage::

    conda activate sim
    python deliverables/bench_terrain_estimators_lhs.py --n 100 --workers 6
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deliverables"))
sys.path.insert(0, str(ROOT / "simulation"))


# Bekker-Mohr box (paper118 Table I + project SCM range). The LHS
# sampler in ``param_consistency.generate_lhs_terrain_yaml_dicts``
# already produces samples in this same box; we use that helper
# verbatim so the sweep is consistent with the rest of the project.
import param_consistency  # noqa
from param_consistency import generate_lhs_terrain_yaml_dicts  # noqa


def _lhs_with_n_range(n_samples: int, seed: int,
                       n_min: float, n_max: float):
    """Wrap ``generate_lhs_terrain_yaml_dicts`` so the ``bekker_n``
    axis is restricted to ``[n_min, n_max]`` (matches the
    ``terrain_window_mlp`` training range and the SCM physical
    regime). Patches ``TRAINING_RANGES_V6`` only for the duration
    of this call so we don't disturb the rest of the project."""
    orig = param_consistency.TRAINING_RANGES_V6["bekker_n"]
    if (float(n_min), float(n_max)) == orig:
        return generate_lhs_terrain_yaml_dicts(n_samples, seed=seed)
    param_consistency.TRAINING_RANGES_V6 = {
        **param_consistency.TRAINING_RANGES_V6,
        "bekker_n": (float(n_min), float(n_max)),
    }
    try:
        return generate_lhs_terrain_yaml_dicts(n_samples, seed=seed)
    finally:
        param_consistency.TRAINING_RANGES_V6 = {
            **param_consistency.TRAINING_RANGES_V6,
            "bekker_n": orig,
        }


@dataclass(frozen=True)
class ScenarioCfg:
    idx: int
    n_true: float
    yaml_path: str
    log_path: str
    steer_amp: float = 0.60
    steer_period: float = 3.0
    open_loop_throttle: float = 0.75   # -1 means PI cruise to target_speed
    target_speed: float = 5.0


def _scm_worker(cfg: ScenarioCfg) -> Dict[str, float]:
    """Run one Chrono SCM scenario as a SUBPROCESS so PyChrono's
    global state is reset between workers."""
    cmd = [
        sys.executable, "-u",
        str(ROOT / "data_collection" / "run_dallas_scm.py"),
        "--terrain-config", cfg.yaml_path,
        "--steer-amp-rad", f"{cfg.steer_amp:.3f}",
        "--steer-period", f"{cfg.steer_period:.3f}",
        "--time", "50", "--lead-in", "3",
        "--output", cfg.log_path,
    ]
    if cfg.open_loop_throttle >= 0:
        cmd += ["--open-loop-throttle", f"{cfg.open_loop_throttle:.3f}"]
    else:
        cmd += ["--target-speed", f"{cfg.target_speed:.3f}"]
    out = subprocess.run(cmd, cwd=str(ROOT),
                          capture_output=True, text=True, timeout=300)
    ok = out.returncode == 0 and Path(cfg.log_path).exists()
    return {"idx": cfg.idx, "ok": int(ok),
            "log_path": cfg.log_path,
            "stderr": out.stderr[-800:] if not ok else ""}


def _est_worker(cfg: ScenarioCfg) -> List[Dict[str, float]]:
    """Pool worker — pickleable, no shared state between calls."""
    # Set up the imports inside the worker so each child process gets
    # its own module instances (avoids torch / CasADi global-state
    # contention across parallel evaluators).
    import os, sys
    sys.path.insert(0, str(ROOT / "deliverables"))
    sys.path.insert(0, str(ROOT / "simulation"))
    return _run_estimators(cfg)


def _run_estimators(cfg: ScenarioCfg) -> List[Dict[str, float]]:
    """Run all three estimators on one log, return one row per estimator."""
    from ukf_paper_validation import (DallasScenario, SoilParams,  # noqa
                                       run_dallas_from_log)
    with open(cfg.yaml_path) as f:
        params = json.load(f)
    soil = SoilParams(
        kc=float(params["Kc"]), kphi=float(params["Kphi"]),
        n=float(params["n"]),
        c=float(params["cohesion"]),
        phi=math.radians(float(params["friction_angle"])),
        kx=float(params["janosi_shear"]),
        ky=float(params["janosi_shear"]),
    )
    sc = DallasScenario(
        label=f"lhs#{cfg.idx} (n={cfg.n_true:.3f})",
        soil=soil, n_true=float(cfg.n_true),
        n_init=0.70,  # neutral dirt prior used by the deployment runtime
    )

    rows: List[Dict[str, float]] = []

    # Bekker-UKF
    try:
        _, n_b, pct_b = run_dallas_from_log(Path(cfg.log_path), sc,
                                              backend="bekker")
        conv_b = float(np.mean(n_b[len(n_b) // 4 * 3:]))
    except Exception as exc:
        pct_b = float("nan"); conv_b = float("nan")
        print(f"  bekker failed on idx={cfg.idx}: {exc}", file=sys.stderr)

    # NN-UKF (whole-vehicle Fy surrogate)
    try:
        _, n_n, pct_n = run_dallas_from_log(Path(cfg.log_path), sc,
                                              backend="vehicle_fy")
        conv_n = float(np.mean(n_n[len(n_n) // 4 * 3:]))
    except Exception as exc:
        pct_n = float("nan"); conv_n = float("nan")
        print(f"  vehicle_fy failed on idx={cfg.idx}: {exc}",
              file=sys.stderr)

    # Learned window MLP — import only when needed (heavy import chain)
    from eval_terrain_estimators import _run_learned_estimator  # noqa
    try:
        _, n_l, conv_l = _run_learned_estimator(Path(cfg.log_path),
                                                  sc.n_init)
        pct_l = 100.0 * abs(conv_l - sc.n_true) / sc.n_true
    except Exception as exc:
        pct_l = float("nan"); conv_l = float("nan")
        print(f"  learned failed on idx={cfg.idx}: {exc}", file=sys.stderr)

    base = {"idx": cfg.idx, "n_true": sc.n_true,
            "Kphi": params["Kphi"], "Kc": params["Kc"],
            "cohesion": params["cohesion"],
            "friction_angle": params["friction_angle"],
            "janosi_shear": params["janosi_shear"]}
    rows.append({**base, "estimator": "Bekker-UKF",
                 "converged_n": conv_b, "pct_err": pct_b})
    rows.append({**base, "estimator": "NN-UKF",
                 "converged_n": conv_n, "pct_err": pct_n})
    rows.append({**base, "estimator": "Learned MLP",
                 "converged_n": conv_l, "pct_err": pct_l})
    return rows


def _plot(df: pd.DataFrame, n_label: int, fig_path: Path) -> None:
    estimators = ["Bekker-UKF", "NN-UKF", "Learned MLP"]
    colors = {"Bekker-UKF": "#1f77b4", "NN-UKF": "#2ca02c",
              "Learned MLP": "#d62728"}
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.0))
    ax_scatter, ax_cdf = axes

    # Left: scatter true n vs estimated n
    diag_lo, diag_hi = 0.25, 1.40
    ax_scatter.plot([diag_lo, diag_hi], [diag_lo, diag_hi],
                     color="k", lw=1.0, ls="-", label="y=x")
    for band, alpha_b in [(0.10, 0.13), (0.20, 0.07)]:
        x = np.linspace(diag_lo, diag_hi, 100)
        ax_scatter.fill_between(x, x * (1 - band), x * (1 + band),
                                  color="k", alpha=alpha_b,
                                  label=f"±{int(band*100)} %")
    for name in estimators:
        sub = df[df["estimator"] == name].dropna(subset=["converged_n"])
        ax_scatter.scatter(sub["n_true"], sub["converged_n"],
                           s=28, alpha=0.65, color=colors[name],
                           edgecolors="white", linewidths=0.5,
                           label=name)
    ax_scatter.set_xlabel("True $n$")
    ax_scatter.set_ylabel("Estimated $n$ (tail-window mean)")
    ax_scatter.set_xlim(diag_lo, diag_hi)
    ax_scatter.set_ylim(diag_lo, diag_hi)
    ax_scatter.set_title(f"Convergence across {n_label} LHS terrains")
    ax_scatter.grid(alpha=0.3)
    ax_scatter.legend(loc="upper left", fontsize=9, framealpha=0.92)

    # Right: CDF of %-error
    for name in estimators:
        sub = df[df["estimator"] == name].dropna(subset=["pct_err"])
        errs = np.sort(sub["pct_err"].to_numpy())
        if errs.size == 0:
            continue
        y = np.arange(1, errs.size + 1) / errs.size
        med = float(np.median(errs))
        mean = float(np.mean(errs))
        p90 = float(np.percentile(errs, 90))
        ax_cdf.plot(errs, y, lw=1.6, color=colors[name],
                    label=f"{name}  med={med:.1f}%  mean={mean:.1f}%  p90={p90:.1f}%")
    ax_cdf.set_xlabel("|Δn| / n_true   (%-error)")
    ax_cdf.set_ylabel("Empirical CDF")
    ax_cdf.set_xlim(0.0, min(100.0,
                              float(df["pct_err"].dropna().max()) * 1.05))
    ax_cdf.set_ylim(0.0, 1.0)
    ax_cdf.set_title("Error CDF")
    ax_cdf.grid(alpha=0.3)
    ax_cdf.legend(loc="lower right", fontsize=8.5, framealpha=0.92)
    for thresh in (5.0, 10.0, 20.0):
        ax_cdf.axvline(thresh, color="gray", lw=0.5, ls="--")

    fig.suptitle(
        f"Terrain estimator benchmark — {n_label} uniform-LHS Bekker–Mohr "
        f"terrains on Chrono SCM",
        fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180, bbox_inches="tight")
    print(f"Wrote {fig_path}")


def main() -> int:
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--n", type=int, default=100,
                   help="LHS sample count.")
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel SCM workers (per CLAUDE.md §Parallelism).")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--skip-collection", action="store_true",
                   help="Reuse logs already on disk (only re-run estimators).")
    p.add_argument("--out-name", default=None,
                   help="Override the output figure / CSV stem.")
    p.add_argument("--n-min", type=float, default=0.40,
                   help="Lower bound on the Bekker n axis. Defaults "
                        "to 0.40 (matches the MLP's training range "
                        "and the SCM physical regime).")
    p.add_argument("--n-max", type=float, default=1.30,
                   help="Upper bound on the Bekker n axis. Defaults "
                        "to 1.30 (matches the MLP's training range).")
    p.add_argument("--steer-amp-rad", type=float, default=0.60,
                   help="Steering amplitude per scenario (rad).")
    p.add_argument("--steer-period", type=float, default=3.0,
                   help="Steering sinusoid period per scenario (s).")
    p.add_argument("--open-loop-throttle", type=float, default=0.75,
                   help="Constant throttle for Buzhardt-style scripted "
                        "open-loop excitation. Set to -1 to use PI "
                        "cruise at --target-speed instead.")
    p.add_argument("--target-speed", type=float, default=5.0,
                   help="PI cruise setpoint when --open-loop-throttle=-1.")
    p.add_argument("--log-suffix", default="",
                   help="Optional suffix appended to the SCM log directory "
                        "(``data/dallas_scm/lhs<N><suffix>``). Useful to "
                        "keep CL and OL benchmarks side-by-side.")
    p.add_argument("--replot-only", action="store_true",
                   help="Skip all SCM collection AND the estimator pass; just "
                        "re-draw the figure from the existing <out-name>.csv. "
                        "Used by make_paper_figures.py to regenerate the figure "
                        "fast without re-running the benchmark.")
    args = p.parse_args()

    name = args.out_name or f"terrain_estimator_bench_{args.n}"
    out_dir = ROOT / "my_paper" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_path = out_dir / f"{name}.png"
    csv_path = out_dir / f"{name}.csv"

    if args.replot_only:
        if not csv_path.exists():
            raise FileNotFoundError(
                f"{csv_path} not found -- run the benchmark once first")
        _plot(pd.read_csv(csv_path), args.n, fig_path)
        return 0
    log_dir = (ROOT / "data" / "dallas_scm"
               / f"lhs{args.n}{args.log_suffix}")
    log_dir.mkdir(parents=True, exist_ok=True)

    # 1. Generate LHS terrain samples
    print(f"Generating {args.n} LHS terrain samples (seed={args.seed}, "
          f"n in [{args.n_min}, {args.n_max}])...")
    lhs = _lhs_with_n_range(args.n, args.seed, args.n_min, args.n_max)
    cfgs: List[ScenarioCfg] = []
    for i, terrain in enumerate(lhs):
        yaml_path = log_dir / f"scn_{i:03d}.yaml"
        log_path = log_dir / f"scn_{i:03d}.npz"
        # Stringify to plain JSON (load_terrain_config tolerates both).
        if not yaml_path.exists():
            with open(yaml_path, "w") as f:
                json.dump({k: float(v) for k, v in terrain.items()
                            if isinstance(v, (int, float))}, f, indent=2)
        cfgs.append(ScenarioCfg(
            idx=i, n_true=float(terrain["n"]),
            yaml_path=str(yaml_path), log_path=str(log_path),
            steer_amp=float(args.steer_amp_rad),
            steer_period=float(args.steer_period),
            open_loop_throttle=float(args.open_loop_throttle),
            target_speed=float(args.target_speed),
        ))

    # 2. Run Chrono SCM (parallel)
    if not args.skip_collection:
        print(f"Running {len(cfgs)} Chrono SCM scenarios on {args.workers} "
              f"parallel workers...")
        completed = 0
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = {ex.submit(_scm_worker, c): c for c in cfgs}
            for fut in as_completed(futs):
                r = fut.result()
                completed += 1
                tag = "OK " if r["ok"] else "FAIL"
                print(f"  [{completed:3d}/{len(cfgs)}] {tag} idx={r['idx']}"
                      + (f"  {r['stderr'][:200]}" if not r["ok"] else ""),
                      flush=True)

    # 3. Run estimators on each log (parallel — pickleable workers).
    # Each estimator pass is independent: UKF / learned MLP load their
    # weights from disk and emit summary numbers. Pool gives us the same
    # speedup the SCM phase had.
    cfgs_to_run = [c for c in cfgs if Path(c.log_path).exists()]
    print(f"\nRunning estimators on {len(cfgs_to_run)} logs "
          f"on {args.workers} parallel workers...")
    all_rows: List[Dict[str, float]] = []
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_est_worker, c): c for c in cfgs_to_run}
        for fut in as_completed(futs):
            c = futs[fut]
            done += 1
            try:
                rows = fut.result()
            except Exception as exc:
                print(f"  [{done:3d}/{len(cfgs_to_run)}] FAIL idx={c.idx}: "
                      f"{exc}", flush=True)
                continue
            all_rows.extend(rows)
            print(f"  [{done:3d}/{len(cfgs_to_run)}] idx={c.idx}  "
                  f"n_true={c.n_true:.3f}  "
                  f"bekker={rows[0]['pct_err']:6.2f}%  "
                  f"nn={rows[1]['pct_err']:6.2f}%  "
                  f"mlp={rows[2]['pct_err']:6.2f}%", flush=True)

    df = pd.DataFrame(all_rows)
    df.to_csv(csv_path, index=False)
    print(f"\nWrote {csv_path}")

    # 4. Summary
    print("\n--- Per-estimator error summary ---")
    grp = df.groupby("estimator")["pct_err"].agg(
        ["mean", "median",
         lambda s: s.quantile(0.90),
         lambda s: s.quantile(0.95),
         "count"])
    grp.columns = ["mean", "median", "p90", "p95", "n"]
    print(grp.to_string(float_format=lambda x: f"{x:.2f}"))
    pct10 = df.groupby("estimator").apply(
        lambda g: (g["pct_err"] <= 10).mean() * 100
    ).rename("pct_within_10pct")
    pct20 = df.groupby("estimator").apply(
        lambda g: (g["pct_err"] <= 20).mean() * 100
    ).rename("pct_within_20pct")
    print("\n", pct10, "\n", pct20)

    # 5. Plot
    _plot(df, args.n, fig_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
