#!/usr/bin/env python3
"""Run the closed-loop MPC + learned terrain estimator on each preset
soil and quantify how well the estimator locks onto the true ``n``.

Strategy
--------
Reuses ``launch_decoupled.py`` so the experiment matches the real
deployment path exactly (Chrono SCM + acados MPC + learned estimator
all wired through the same ZMQ topics).  We pass ``--te-verbose`` so
the controller prints lines like::

    [LRN] u=4.50 ay=+0.32 omega=-0.11 slip_mean=0.013 -> n_raw=0.55 n_sm=0.51

These lines are parsed offline to compute mean/std of the smoothed
prediction over the back half of the run (steady-state).  A summary
table is written to ``my_paper/paper_figures/closed_loop_estimator_summary.csv``
along with a 3-panel time-series plot.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}

LRN_RE = re.compile(
    r"\[LRN\].*?u=(?P<u>[-+\d.]+)\s+ay=(?P<ay>[-+\d.]+)\s+"
    r"omega=(?P<omega>[-+\d.]+)\s+slip_mean=(?P<slip>[-+\d.]+)\s+"
    r"->\s+n_raw=(?P<nraw>[-+\d.]+)\s+n_sm=(?P<nsm>[-+\d.]+)"
)


CONDA_BIN = "/home/kyle/miniconda3/bin/conda"


def _run_one(*, terrain: str, duration: float, sim_port: int, ctrl_port: int,
             sine_amp: float, sine_wl: float,
             speed: float, log_path: Path, project_root: Path) -> bool:
    cmd = [
        CONDA_BIN, "run", "--no-capture-output", "-n", "sim", "python",
        str(project_root / "simulation" / "launch_decoupled.py"),
        "--terrain", terrain,
        "--path", "sinusoidal",
        "--no-vis", "--no-plot", "--no-csv",
        "--time", str(duration),
        "--speed", str(speed),
        "--sine-amplitude", str(sine_amp),
        "--sine-wavelength", str(sine_wl),
        "--terrain-estimator",
        "--te-verbose",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
    ]
    print("  $", " ".join(shlex.quote(c) for c in cmd))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("wb") as f:
        proc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT,
                              cwd=str(project_root),
                              timeout=duration + 60.0)
    return proc.returncode == 0


def _parse_log(log_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    txt = log_path.read_text(errors="replace")
    for m in LRN_RE.finditer(txt):
        rows.append({k: float(v) for k, v in m.groupdict().items()})
    return rows


def _summarise(rows: List[Dict[str, float]], true_n: float, half: bool = True
               ) -> Dict[str, float]:
    if not rows:
        return {"n_mean": float("nan"), "n_std": float("nan"),
                "n_bias": float("nan"), "n_abs_err": float("nan"),
                "n_count": 0}
    n_arr = np.array([r["nsm"] for r in rows])
    sub = n_arr[len(n_arr)//2:] if half else n_arr
    return {
        "n_mean": float(sub.mean()),
        "n_std": float(sub.std()),
        "n_bias": float(sub.mean() - true_n),
        "n_abs_err": float(abs(sub.mean() - true_n)),
        "n_count": len(sub),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--terrains", nargs="+", default=["clay", "dirt", "sand"])
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--out-dir", default=str(
        Path(__file__).parent.parent / "my_paper" / "paper_figures"))
    p.add_argument("--logs-dir", default=str(
        Path(__file__).parent.parent / "logs" / "cl_validate"))
    p.add_argument("--sim-port-base", type=int, default=34000)
    args = p.parse_args()

    project_root = Path(__file__).parent.parent
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = Path(args.logs_dir)
    logs_dir.mkdir(parents=True, exist_ok=True)

    summary: List[Dict] = []
    parsed_per_terrain: Dict[str, List[Dict[str, float]]] = {}

    port = args.sim_port_base
    for terr in args.terrains:
        true_n = TRUE_N[terr]
        log_path = logs_dir / f"cl_{terr}_learned.log"
        print(f"\n[validate] terrain={terr}  true_n={true_n:.2f}  "
              f"speed={args.speed}  log={log_path}")
        ok = _run_one(terrain=terr, duration=args.duration,
                      sim_port=port, ctrl_port=port + 1,
                      sine_amp=args.sine_amplitude,
                      sine_wl=args.sine_wavelength,
                      speed=args.speed,
                      log_path=log_path,
                      project_root=project_root)
        port += 4
        rows = _parse_log(log_path)
        summ = _summarise(rows, true_n=true_n, half=True)
        summ["terrain"] = terr
        summ["true_n"] = true_n
        summ["lines"] = len(rows)
        summ["ok"] = ok
        print(f"   parsed {len(rows)} predictions  "
              f"n_sm[2nd half]= {summ['n_mean']:.3f} ± {summ['n_std']:.3f}  "
              f"bias={summ['n_bias']:+.3f}  |err|={summ['n_abs_err']:.3f}")
        summary.append(summ)
        parsed_per_terrain[terr] = rows

    csv_path = out_dir / "closed_loop_estimator_summary_learned.csv"
    with csv_path.open("w") as f:
        f.write("terrain,true_n,lines,n_mean,n_std,n_bias,n_abs_err,ok\n")
        for s in summary:
            f.write(f"{s['terrain']},{s['true_n']:.3f},{s['lines']},"
                    f"{s['n_mean']:.4f},{s['n_std']:.4f},"
                    f"{s['n_bias']:+.4f},{s['n_abs_err']:.4f},{int(s['ok'])}\n")
    print(f"\n[validate] summary written to {csv_path}")

    # Time-series plot if matplotlib present
    try:
        import matplotlib.pyplot as plt
        n_terr = len(args.terrains)
        fig, axes = plt.subplots(n_terr, 1, figsize=(9, 2.4 * n_terr),
                                 sharex=True)
        if n_terr == 1:
            axes = [axes]
        for ax, terr in zip(axes, args.terrains):
            rows = parsed_per_terrain[terr]
            if not rows:
                continue
            true_n = TRUE_N[terr]
            t = np.arange(len(rows)) / max(1, len(rows)) * args.duration
            n_sm = np.array([r["nsm"] for r in rows])
            n_raw = np.array([r["nraw"] for r in rows])
            ax.plot(t, n_raw, color="tab:gray", alpha=0.4, label="n_raw")
            ax.plot(t, n_sm, color="tab:blue", lw=1.6, label="n_smooth")
            ax.axhline(true_n, color="tab:red", ls="--",
                       label=f"true n={true_n:.2f}")
            ax.set_ylim(0.3, 1.4)
            ax.set_ylabel(f"{terr}\nn estimate")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
        axes[-1].set_xlabel("Time index (relative)")
        fig.suptitle("Closed-loop estimator (learned sliding-window MLP)")
        fig.tight_layout()
        png = out_dir / "closed_loop_estimator_learned.png"
        fig.savefig(png, dpi=140)
        print(f"[validate] plot written to {png}")
    except Exception as e:
        print(f"[validate] plot skipped: {e}")


if __name__ == "__main__":
    main()
