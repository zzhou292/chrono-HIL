#!/usr/bin/env python3
"""Calibrate the feedforward motion-resistance term c_drag(n) for the NMPC.

The NMPC longitudinal channel is purely kinematic, u_dot = ax + du_dot_resid,
and du_dot_resid is currently 0 -- so on deformable soil the prediction
over-speeds (Sec. III-E rollout validation). The missing term is sinkage drag.
We recover it directly from the logged rollouts: for each solve the controller
logged its predicted speed u_pred(tau); paired against the actual u(t+tau), the
SIGNED drift Delta u(tau) = u_pred - u_act grows at a rate equal to the plant's
unmodelled deceleration. Its slope over the horizon IS c_drag (m/s^2), fit per
terrain and mapped to the sinkage exponent n so the controller can index it
from the live estimate n_hat.

Usage:
    python benchmarking/calibrate_motion_resistance.py \
        --glob 'benchmarking/results/mpc_tire_model_sweep_*'
"""
from __future__ import annotations
import argparse, glob, os, re, sys
from pathlib import Path
import numpy as np, pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "simulation"))
from param_consistency import get_terrain_preset  # noqa: E402

TERRAINS = ("clay", "dirt", "sand")


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


def _signed_speed_drift(npz, diag, max_h=40):
    """Mean signed u_pred - u_act vs horizon stage for one run."""
    p = np.load(npz)
    times, Z, dt = p["times"], p["Z"], float(p["dt"])
    d = pd.read_csv(diag)
    t = pd.to_numeric(d["sim_time"], errors="coerce").to_numpy()
    u = pd.to_numeric(d.get("u_true"), errors="coerce").to_numpy()
    ok = np.isfinite(t) & np.isfinite(u)
    t, u = t[ok], u[ok]
    if len(t) < 5:
        return None, dt
    H = min(max_h, Z.shape[2] - 1)
    drift = np.full((len(times), H + 1), np.nan)
    for i, t0 in enumerate(times):
        th = t0 + np.arange(H + 1) * dt
        au = np.interp(th, t, u, left=np.nan, right=np.nan)
        drift[i] = Z[i, 3][:H + 1] - au          # signed: + = over-prediction
    return np.nanmean(drift, axis=0), dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="benchmarking/results/mpc_tire_model_sweep_*")
    ap.add_argument("--fit-lo", type=float, default=0.3, help="fit window start (s)")
    ap.add_argument("--fit-hi", type=float, default=3.0, help="fit window end (s)")
    args = ap.parse_args()

    pairs = _pairs(args.glob)
    if not pairs:
        print(f"no (mpc_predictions.npz, diag) pairs under {args.glob}")
        return
    by_terr = {tr: [] for tr in TERRAINS}
    dt = 0.1
    for terr, npz, diag in pairs:
        dr, dt = _signed_speed_drift(npz, diag)
        if dr is not None:
            by_terr[terr].append(dr)

    print(f"{len(pairs)} runs; horizon dt={dt:.2f}s")
    print("\nterrain   n      drift@1s  drift@2s  drift@4s   c_drag(slope, m/s^2)")
    rows = []
    for terr in TERRAINS:
        runs = by_terr[terr]
        if not runs:
            continue
        L = min(len(r) for r in runs)
        mean = np.nanmean(np.vstack([r[:L] for r in runs]), axis=0)
        h = np.arange(L) * dt
        n_val = float(get_terrain_preset(terr)["n"])
        lo, hi = int(args.fit_lo / dt), min(int(args.fit_hi / dt), L)
        # slope of signed drift over the fit window = unmodelled decel = c_drag
        A = np.vstack([h[lo:hi], np.ones(hi - lo)]).T
        slope, intercept = np.linalg.lstsq(A, mean[lo:hi], rcond=None)[0]
        def at(ts):
            return float(mean[min(int(round(ts / dt)), L - 1)])
        print(f"{terr:6s} {n_val:5.2f}  {at(1):8.2f}  {at(2):8.2f}  {at(4):8.2f}   {slope:+.3f}")
        rows.append({"terrain": terr, "n": round(n_val, 3),
                     "c_drag": round(float(max(slope, 0.0)), 3),
                     "drift_1s": round(at(1), 3), "drift_2s": round(at(2), 3),
                     "drift_4s": round(at(4), 3)})

    out = Path(__file__).resolve().parent.parent / "my_paper" / "paper_figures" / "motion_resistance_calib.csv"
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f"\nwrote {out}")
    print("\nc_drag(n) lookup (for controller, ascending n):")
    for r in sorted(rows, key=lambda x: x["n"]):
        print(f"  n={r['n']:.2f} -> c_drag={r['c_drag']:.3f} m/s^2")


if __name__ == "__main__":
    main()
