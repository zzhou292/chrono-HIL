#!/usr/bin/env python3
"""Closed-loop validation on random unseen SCM soils.

Workflow
--------
1. Reads ``data/terrain_yamls_random/manifest.csv`` (created by
   ``generate_random_terrains.py``).
2. For each ``terrainN.yaml``, launches the full closed-loop pipeline
   via ``launch_decoupled.py`` with ``--terrain-config <yaml>``,
   ``--terrain-estimator`` (learned MLP backend), and ``--te-verbose``
   so the live ``n_raw / n_smoothed`` predictions are written to the
   controller log.
3. After the run, finds the controller's diagnostic CSV in
   ``plots/<run-stamp>/diag_*.csv`` and pulls
   ``crosstrack_err``, ``heading_err_deg``, ``speed_err``,
   ``n_terrain_est`` (the *applied* terrain n from MPC) and
   ``terrain_update_applied`` columns.
4. Computes:
     * Estimator convergence: mean ± std of ``n_smoothed`` over the
       second half of the run, vs. ``true_n`` from the manifest.
     * Tracking quality: RMS crosstrack (m), RMS heading error (deg),
       RMS speed error (m/s), all over the second half of the run.
5. Writes a per-terrain summary CSV plus a multi-panel figure.
"""

from __future__ import annotations

import argparse
import csv
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


CONDA_BIN = "/home/kyle/miniconda3/bin/conda"

LRN_RE = re.compile(
    r"\[LRN\].*?u=(?P<u>[-+\d.]+)\s+ay=(?P<ay>[-+\d.]+)\s+"
    r"omega=(?P<omega>[-+\d.]+)\s+slip_mean=(?P<slip>[-+\d.]+)\s+"
    r"->\s+n_raw=(?P<nraw>[-+\d.]+)\s+n_sm=(?P<nsm>[-+\d.]+)"
)


def _run_one(*, label: str, yaml_path: Path, preset_proxy: str,
             duration: float, sim_port: int, ctrl_port: int,
             plot_dir: Path, log_path: Path, sine_amp: float,
             sine_wl: float, speed: float, project_root: Path) -> bool:
    cmd = [
        CONDA_BIN, "run", "--no-capture-output", "-n", "sim", "python",
        str(project_root / "simulation" / "launch_decoupled.py"),
        "--terrain", preset_proxy,
        "--terrain-config", str(yaml_path),
        "--path", "sinusoidal",
        "--no-vis", "--no-plot",
        "--time", str(duration),
        "--speed", str(speed),
        "--sine-amplitude", str(sine_amp),
        "--sine-wavelength", str(sine_wl),
        "--terrain-estimator",
        "--te-verbose",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--plot-dir", str(plot_dir),
    ]
    print("  $", " ".join(shlex.quote(c) for c in cmd))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    with log_path.open("wb") as f:
        proc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT,
                              cwd=str(project_root),
                              timeout=duration + 90.0)
    return proc.returncode == 0


def _find_diag_csv(plot_dir: Path, since_ts: float) -> Optional[Path]:
    """Return the most-recent diag_*.csv produced after ``since_ts``."""
    candidates: List[Tuple[float, Path]] = []
    for sub in plot_dir.glob("*"):
        if not sub.is_dir():
            continue
        for csvp in sub.glob("diag_*.csv"):
            try:
                mt = csvp.stat().st_mtime
            except FileNotFoundError:
                continue
            if mt >= since_ts:
                candidates.append((mt, csvp))
    if not candidates:
        return None
    candidates.sort(key=lambda kv: kv[0], reverse=True)
    return candidates[0][1]


def _parse_diag_csv(path: Path) -> Dict[str, np.ndarray]:
    """Pull just the columns we need from the controller diagnostic CSV."""
    if not path.exists():
        return {}
    cols_needed = ["sim_time", "crosstrack_err", "heading_err_deg",
                   "speed_err", "n_terrain_est", "terrain_update_applied"]
    rows: Dict[str, List[float]] = {c: [] for c in cols_needed}
    with path.open() as f:
        rdr = csv.DictReader(f)
        for r in rdr:
            for c in cols_needed:
                v = r.get(c, "")
                try:
                    rows[c].append(float(v))
                except (ValueError, TypeError):
                    rows[c].append(float("nan"))
    return {c: np.array(rows[c]) for c in cols_needed}


def _parse_lrn_log(path: Path) -> List[Dict[str, float]]:
    if not path.exists():
        return []
    out: List[Dict[str, float]] = []
    txt = path.read_text(errors="replace")
    for m in LRN_RE.finditer(txt):
        out.append({k: float(v) for k, v in m.groupdict().items()})
    return out


def _half_stats(arr: np.ndarray) -> Tuple[float, float, float, float]:
    """Return (mean, std, rms, count) over the back half of ``arr``,
    ignoring NaNs."""
    if arr.size == 0:
        return float("nan"), float("nan"), float("nan"), 0
    sub = arr[arr.size // 2:]
    sub = sub[np.isfinite(sub)]
    if sub.size == 0:
        return float("nan"), float("nan"), float("nan"), 0
    mean = float(sub.mean())
    std = float(sub.std())
    rms = float(np.sqrt(np.mean(np.square(sub))))
    return mean, std, rms, int(sub.size)


def _load_manifest(path: Path) -> List[Dict]:
    rows: List[Dict] = []
    with path.open() as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--manifest", default=str(
        Path(__file__).parent.parent / "data" / "terrain_yamls_random"
        / "manifest.csv"))
    p.add_argument("--out-dir", default=str(
        Path(__file__).parent.parent / "my_paper" / "paper_figures"))
    p.add_argument("--logs-dir", default=str(
        Path(__file__).parent.parent / "logs" / "cl_random"))
    p.add_argument("--plot-dir", default=str(
        Path(__file__).parent.parent / "plots" / "cl_random"))
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--sim-port-base", type=int, default=37000)
    p.add_argument("--terrains", nargs="*", default=None,
                   help="Optional subset of labels to run (e.g. terrain1 "
                        "terrain3)")
    p.add_argument("--replot-only", action="store_true",
                   help="Skip simulation and re-parse existing logs/diag CSVs")
    args = p.parse_args()

    project_root = Path(__file__).parent.parent
    manifest_rows = _load_manifest(Path(args.manifest))
    if args.terrains:
        wanted = set(args.terrains)
        manifest_rows = [r for r in manifest_rows if r["label"] in wanted]
    if not manifest_rows:
        print("[validate] no terrains to run, exiting"); sys.exit(1)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = Path(args.logs_dir)
    logs_dir.mkdir(parents=True, exist_ok=True)
    plot_root = Path(args.plot_dir)
    plot_root.mkdir(parents=True, exist_ok=True)

    summary: List[Dict] = []
    parsed_per_terrain: Dict[str, Dict] = {}

    port = args.sim_port_base
    for row in manifest_rows:
        label = row["label"]
        true_n = float(row["true_n"])
        proxy = row["preset_proxy"]
        yaml_path = Path(row["yaml"])

        log_path = logs_dir / f"{label}_learned.log"
        plot_dir = plot_root / label

        print(f"\n[validate] {label}  true_n={true_n:.3f}  proxy={proxy}  "
              f"yaml={yaml_path.name}")

        if args.replot_only:
            ok = log_path.exists()
            t_before = 0.0
        else:
            t_before = time.time()
            ok = _run_one(label=label, yaml_path=yaml_path,
                          preset_proxy=proxy,
                          duration=args.duration,
                          sim_port=port, ctrl_port=port + 1,
                          plot_dir=plot_dir, log_path=log_path,
                          sine_amp=args.sine_amplitude,
                          sine_wl=args.sine_wavelength,
                          speed=args.speed,
                          project_root=project_root)
            port += 4

        diag_csv = _find_diag_csv(plot_dir, since_ts=t_before)
        diag = _parse_diag_csv(diag_csv) if diag_csv else {}
        lrn_rows = _parse_lrn_log(log_path)

        # --- Estimator convergence (live n_smoothed from controller log) ---
        n_sm = np.array([r["nsm"] for r in lrn_rows]) if lrn_rows else np.array([])
        n_mean, n_std, _, n_cnt = _half_stats(n_sm)
        n_bias = (n_mean - true_n) if np.isfinite(n_mean) else float("nan")

        # --- Estimator commitment (n actually used by MPC) ---
        n_te = diag.get("n_terrain_est", np.array([]))
        n_te_mean, n_te_std, _, _ = _half_stats(n_te)

        # --- Tracking metrics from diagnostic CSV ---
        ct = diag.get("crosstrack_err", np.array([]))
        he = diag.get("heading_err_deg", np.array([]))
        se = diag.get("speed_err", np.array([]))
        _, _, ct_rms, _ = _half_stats(ct)
        _, _, he_rms, _ = _half_stats(he)
        _, _, se_rms, _ = _half_stats(se)

        applied = diag.get("terrain_update_applied", np.array([]))
        applied_pct = (
            float(np.nanmean(applied) * 100.0) if applied.size else float("nan")
        )

        summ = {
            "label": label, "true_n": true_n, "proxy": proxy,
            "coh_scale": float(row["coh_scale"]),
            "phi_delta_deg": float(row["phi_delta_deg"]),
            "lrn_lines": len(lrn_rows),
            "n_sm_mean": n_mean, "n_sm_std": n_std,
            "n_bias": n_bias, "n_abs_err": abs(n_bias),
            "n_applied_mean": n_te_mean, "n_applied_std": n_te_std,
            "applied_pct": applied_pct,
            "rms_crosstrack_m": ct_rms,
            "rms_heading_err_deg": he_rms,
            "rms_speed_err_mps": se_rms,
            "diag_csv": str(diag_csv) if diag_csv else "",
            "ok": int(ok),
        }
        print(f"   est:    n_sm  = {n_mean:.3f} ± {n_std:.3f}  "
              f"(bias={n_bias:+.3f}, |err|={abs(n_bias):.3f}, "
              f"{n_cnt} samples)")
        print(f"   applied n_te  = {n_te_mean:.3f} ± {n_te_std:.3f}  "
              f"(MPC committed {applied_pct:.0f}% of steps)")
        print(f"   tracking: ct_rms={ct_rms:.3f} m  "
              f"hdg_rms={he_rms:.2f}°  spd_rms={se_rms:.3f} m/s")
        summary.append(summ)
        parsed_per_terrain[label] = {
            "lrn": lrn_rows, "diag": diag, "true_n": true_n,
        }

    # ── write summary CSV ─────────────────────────────────────────
    csv_path = out_dir / "random_terrain_closed_loop_learned.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print(f"\n[validate] summary CSV -> {csv_path}")

    # ── pretty plot ───────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt
        n_terr = len(summary)
        fig, axes = plt.subplots(n_terr, 2, figsize=(12, 2.4 * n_terr),
                                 sharex="col")
        if n_terr == 1:
            axes = np.array([axes])
        for i, s in enumerate(summary):
            label = s["label"]
            true_n = s["true_n"]
            data = parsed_per_terrain[label]
            lrn = data["lrn"]
            diag = data["diag"]

            # Left col: n estimate over time
            ax = axes[i, 0]
            if lrn:
                # Use the controller log's implicit ordering as the time axis
                t = np.linspace(0.0, args.duration, len(lrn))
                n_raw = np.array([r["nraw"] for r in lrn])
                n_sm = np.array([r["nsm"] for r in lrn])
                ax.plot(t, n_raw, color="tab:gray", alpha=0.4, label="n_raw")
                ax.plot(t, n_sm, color="tab:blue", lw=1.6, label="n_smooth")
            ax.axhline(true_n, color="tab:red", ls="--",
                       label=f"true n={true_n:.3f}")
            ax.set_ylim(0.3, 1.4)
            ax.set_ylabel(f"{label}\nn estimate")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right", fontsize=7)
            if i == 0:
                ax.set_title("Terrain estimator (live)")

            # Right col: tracking errors over time
            ax2 = axes[i, 1]
            t_diag = diag.get("sim_time", np.array([]))
            if t_diag.size:
                t_diag = t_diag - t_diag[0]
            ct = diag.get("crosstrack_err", np.array([]))
            he = diag.get("heading_err_deg", np.array([]))
            if t_diag.size:
                ax2.plot(t_diag, ct, color="tab:purple",
                         lw=1.0, label="crosstrack [m]")
                ax2_b = ax2.twinx()
                ax2_b.plot(t_diag, he, color="tab:orange",
                           lw=1.0, alpha=0.7, label="heading [°]")
                ax2.set_ylim(-0.5, 0.5)
                ax2_b.set_ylim(-15, 15)
                ax2.set_ylabel("crosstrack [m]", color="tab:purple")
                ax2_b.set_ylabel("heading [°]", color="tab:orange")
                ax2.grid(alpha=0.3)
            if i == 0:
                ax2.set_title("MPC tracking")

        axes[-1, 0].set_xlabel("time [s]")
        axes[-1, 1].set_xlabel("time [s]")
        fig.suptitle(
            f"Random unseen soils (learned sliding-window MLP, "
            f"v={args.speed} m/s, sine amp={args.sine_amplitude} m)"
        )
        fig.tight_layout()
        png = out_dir / "random_terrain_closed_loop_learned.png"
        fig.savefig(png, dpi=140)
        print(f"[validate] figure -> {png}")
    except Exception as e:
        print(f"[validate] plot skipped: {e}")


if __name__ == "__main__":
    main()
