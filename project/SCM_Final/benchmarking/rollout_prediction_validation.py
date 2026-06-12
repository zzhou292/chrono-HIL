#!/usr/bin/env python3
"""Open-loop rollout validation of the NMPC's predicted trajectory vs the plant.

The controller logs its full predicted horizon trajectory each solve
(``mpc_predictions.npz``: times[T], Z[T,6,N+1] = predicted [x,y,psi,u,v,omega]
at stage dt, set LOG_MPC_PREDICTIONS=1). This pairs each prediction with the
*actual* trajectory from the run's diag CSV and reports how far the NMPC's
prediction drifts from reality as a function of horizon time -- broken out by
component and terrain. The longitudinal-speed (u) drift is the deployment-level
measure of Fx-prediction quality; the lateral position / heading drift measures
Fy (including the rear axle).

Usage:
  python benchmarking/rollout_prediction_validation.py <results_dir_glob>
"""
from __future__ import annotations
import sys, glob, os, re
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "my_paper" / "paper_figures"
TERR_GROUP = {"clay": "soft (clay)", "dirt": "mid (dirt)", "sand": "firm (sand)"}


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _pairs(results_glob):
    out = []
    for npz in glob.glob(os.path.join(results_glob, "**", "mpc_predictions.npz"), recursive=True):
        rd = os.path.dirname(npz)
        diags = glob.glob(os.path.join(rd, "diag_*.csv"))
        if not diags:
            continue
        m = re.search(r"diag_(clay|dirt|sand)_", os.path.basename(diags[0]))
        if m:
            out.append((m.group(1), npz, diags[0]))
    return out


def _errors_for_run(npz, diag, max_h=40):
    p = np.load(npz)
    times, Z, dt = p["times"], p["Z"], float(p["dt"])  # Z: (T,6,N+1)
    d = pd.read_csv(diag)
    t = pd.to_numeric(d["sim_time"], errors="coerce").to_numpy()
    cols = {k: pd.to_numeric(d.get(c), errors="coerce").to_numpy()
            for k, c in [("x", "x_fa_true"), ("y", "y_fa_true"),
                         ("psi", "psi_true"), ("u", "u_true")]}
    ok = np.isfinite(t) & np.isfinite(cols["x"]) & np.isfinite(cols["u"])
    t, = (t[ok],)
    for k in cols:
        cols[k] = cols[k][ok]
    if len(t) < 5:
        return None
    interp = {k: (lambda tt, v=cols[k]: np.interp(tt, t, v, left=np.nan, right=np.nan)) for k in cols}
    H = min(max_h, Z.shape[2] - 1)
    pos = np.full((len(times), H + 1), np.nan)
    spd = np.full_like(pos, np.nan)
    hdg = np.full_like(pos, np.nan)
    for i, t0 in enumerate(times):
        th = t0 + np.arange(H + 1) * dt
        ax_, ay_ = interp["x"](th), interp["y"](th)
        au, apsi = interp["u"](th), interp["psi"](th)
        px, py, ppsi, pu = Z[i, 0], Z[i, 1], Z[i, 2], Z[i, 3]
        pos[i] = np.hypot(px[:H + 1] - ax_, py[:H + 1] - ay_)
        spd[i] = np.abs(pu[:H + 1] - au)
        hdg[i] = np.abs(_wrap(ppsi[:H + 1] - apsi))
    return dt, np.nanmean(pos, axis=0), np.nanmean(spd, axis=0), np.nanmean(hdg, axis=0)


def main():
    results_glob = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "benchmarking" / "results" / "mpc_tire_model_sweep_*")
    pairs = _pairs(results_glob)
    if not pairs:
        print(f"no (mpc_predictions.npz, diag) pairs under {results_glob}"); return
    print(f"found {len(pairs)} runs with predictions")
    agg = {}  # terrain -> list of (dt, pos, spd, hdg)
    for terr, npz, diag in pairs:
        r = _errors_for_run(npz, diag)
        if r:
            agg.setdefault(terr, []).append(r)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0))
    titles = ["position drift (m)", "longitudinal-speed drift (m/s)  [Fx]", "heading drift (rad)  [Fy]"]
    print("\nopen-loop prediction drift at horizon = 1 / 2 / 4 s, by terrain:")
    for terr in ["clay", "dirt", "sand"]:
        if terr not in agg:
            continue
        runs = agg[terr]
        dt = runs[0][0]
        pos = np.nanmean([r[1] for r in runs], axis=0)
        spd = np.nanmean([r[2] for r in runs], axis=0)
        hdg = np.nanmean([r[3] for r in runs], axis=0)
        h = np.arange(len(pos)) * dt
        for ax, series in zip(axes, [pos, spd, hdg]):
            ax.plot(h, series, lw=2, label=TERR_GROUP[terr])
        def at(s, tt):
            return s[min(int(round(tt / dt)), len(s) - 1)]
        print(f"  {terr:5s}: pos {at(pos,1):.2f}/{at(pos,2):.2f}/{at(pos,4):.2f} m | "
              f"u {at(spd,1):.2f}/{at(spd,2):.2f}/{at(spd,4):.2f} m/s | "
              f"psi {at(hdg,1):.3f}/{at(hdg,2):.3f}/{at(hdg,4):.3f} rad")
    for ax, ttl in zip(axes, titles):
        ax.set_xlabel("prediction horizon (s)"); ax.set_title(ttl, fontsize=10.5)
        ax.grid(alpha=0.3); ax.legend(fontsize=8.5)
    fig.suptitle("Open-loop NMPC prediction drift vs the Chrono plant (lower = model predicts reality better)",
                 fontsize=11)
    fig.tight_layout()
    FIG.mkdir(parents=True, exist_ok=True)
    out = FIG / "rollout_prediction_validation.png"
    fig.savefig(out, dpi=170, bbox_inches="tight")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
