#!/usr/bin/env python3
"""Figure: WHY the force-only UKF fails firm sand, and why the proprioceptive
channel fixes it (paper Sec. VI observability analysis).

Left  — n-signal vs measurement noise: |dF_y(n:0.7->1.1)| at the small-slip
        closed-loop operating point vs the large-slip scripted-steer point,
        against the lateral-accel measurement-noise floor (m*R_ay). The
        firm-soil n-channel is below the noise floor at tracking slip -> n is
        unobservable through force.
Right  — closed-loop n^(t) on canonical sand (true n=1.10): the force-only
        NN-UKF drifts the WRONG way and stalls; the proprioceptive-augmented
        UKF tracks to ~1.1. Trajectories read from the live diag CSVs.
"""
from __future__ import annotations
import glob, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "ukf_observability.png"
sys.path.insert(0, str(ROOT / "deliverables")); sys.path.insert(0, str(ROOT / "simulation"))


def _signal_panel(ax):
    from ukf_paper_validation import _vehicle_fy_total, manifold_soil_from_n, Vehicle
    veh = Vehicle()
    ns = np.linspace(0.55, 1.10, 16)
    pts = {"closed-loop tracking\n(small slip)": dict(u=5.0, v=0.08, omega=0.10, delta=0.05),
           "scripted ID maneuver\n(0.6 rad steer)": dict(u=5.0, v=0.9, omega=0.45, delta=0.55)}
    noise = veh.m * 0.3   # m * R_ay (lateral-accel meas-noise std)
    for (name, op), col in zip(pts.items(), ["#d62728", "#2ca02c"]):
        fy = np.array([_vehicle_fy_total(np.array([0, 0, 0, op["u"], op["v"], op["omega"], n]),
                                         op["delta"], manifold_soil_from_n(n))[0] for n in ns])
        ax.plot(ns, fy - fy[np.argmin(abs(ns - 0.7))], color=col, lw=1.8, label=name)
    ax.axhspan(-noise, noise, color="0.7", alpha=0.45, lw=0)
    ax.text(0.57, noise * 0.55, "± lateral-accel\nmeasurement noise", fontsize=8, color="0.3")
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("sinkage exponent $n$"); ax.set_ylabel(r"$F_y(n) - F_y(0.7)$  [N]")
    ax.set_title("Lateral-force n-signal vs. measurement noise", fontsize=10.5)
    ax.legend(fontsize=8.5, loc="upper left"); ax.grid(alpha=0.3)


def _traj_panel(ax):
    def tail_traj(glob_pat):
        ds = glob.glob(glob_pat)
        if not ds:
            return None
        d = pd.read_csv(sorted(ds)[0])
        t = pd.to_numeric(d["sim_time"], errors="coerce")
        n = pd.to_numeric(d["n_terrain_est"], errors="coerce")
        m = np.isfinite(t) & np.isfinite(n)
        return t[m].to_numpy(), n[m].to_numpy()
    series = [
        ("force-only NN-UKF", "/tmp/ttrans/clestall/NN-UKF(live)_sand_v5_s720/*/diag_*.csv", "#d62728"),
        ("MLP (vibration)", "/tmp/ttrans/clestall/MLP(deployed)_sand_v5_s720/*/diag_*.csv", "#4c78a8"),
        ("force+proprio UKF (deployed)", "/tmp/aug_smoke_sand/*/diag_*.csv", "#59a14f"),
    ]
    for name, pat, col in series:
        tr = tail_traj(pat)
        if tr is None:
            continue
        ax.plot(tr[0], tr[1], color=col, lw=1.6, label=name)
    ax.axhline(1.10, color="k", ls="-", lw=1.0, label="true $n$ (sand)")
    ax.axhline(0.70, color="0.5", ls=":", lw=1.0, label="init prior")
    ax.set_xlabel("t [s]"); ax.set_ylabel(r"estimated $n$")
    ax.set_ylim(0.4, 1.25)
    ax.set_title("Closed-loop $n$ on firm sand", fontsize=10.5)
    ax.legend(fontsize=8.0, loc="center right"); ax.grid(alpha=0.3)


def main():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11.6, 4.3))
    _signal_panel(a1)
    _traj_panel(a2)
    fig.suptitle("Firm-soil n is unobservable through lateral force at tracking slip "
                 "— the proprioceptive channel restores it", fontsize=11.5, y=1.02)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=180, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
