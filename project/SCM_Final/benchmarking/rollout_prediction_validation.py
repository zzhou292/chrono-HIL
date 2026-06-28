#!/usr/bin/env python3
"""Open-loop rollout validation of the NMPC's predicted trajectory vs the plant.

The controller logs its full predicted horizon trajectory each solve
(``mpc_predictions.npz``: times[T], Z[T,6,N+1] = predicted [x,y,psi,u,v,omega]
at stage dt, set LOG_MPC_PREDICTIONS=1). For one representative run this draws
each predicted horizon as a faded "fan" branching off the *actual* Chrono plant
trace (Dallas-2021-style predicted-vs-actual overlay), so the reader literally
sees the prediction evolve against reality: the heading horizons hug the plant
while the longitudinal-speed horizons run away from it on firm soil -- the
Fx motion-resistance bias the throttle DOB absorbs.

It also prints the aggregate drift at horizon = 1/2/4 s by terrain (mean over
all matched runs), which backs the quantitative claims in the text.

Usage:
  python benchmarking/rollout_prediction_validation.py [results_dir_glob] \
      [--terrain sand] [--profile v7] [--every 1.2]
"""
from __future__ import annotations
import argparse, glob, os, re
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


def _actual(diag):
    """Ground-truth plant trace, finite-masked and sorted by time."""
    d = pd.read_csv(diag)
    t = pd.to_numeric(d["sim_time"], errors="coerce").to_numpy()
    cols = {k: pd.to_numeric(d.get(c), errors="coerce").to_numpy()
            for k, c in [("x", "x_fa_true"), ("y", "y_fa_true"),
                         ("psi", "psi_true"), ("u", "u_true")]}
    ok = np.isfinite(t) & np.isfinite(cols["x"]) & np.isfinite(cols["u"])
    t = t[ok]
    cols = {k: v[ok] for k, v in cols.items()}
    return t, cols


def _errors_for_run(npz, diag, max_h=40):
    p = np.load(npz)
    times, Z, dt = p["times"], p["Z"], float(p["dt"])  # Z: (T,6,N+1)
    t, cols = _actual(diag)
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


def _print_aggregate(pairs):
    agg = {}
    for terr, npz, diag in pairs:
        r = _errors_for_run(npz, diag)
        if r:
            agg.setdefault(terr, []).append(r)
    print("\nopen-loop prediction drift at horizon = 1 / 2 / 4 s, by terrain "
          "(mean over matched runs):")
    for terr in ["clay", "dirt", "sand"]:
        if terr not in agg:
            continue
        runs = agg[terr]
        dt = runs[0][0]
        pos = np.nanmean([r[1] for r in runs], axis=0)
        spd = np.nanmean([r[2] for r in runs], axis=0)
        hdg = np.nanmean([r[3] for r in runs], axis=0)
        at = lambda s, tt: s[min(int(round(tt / dt)), len(s) - 1)]
        print(f"  {terr:5s} (n={len(runs):2d}): pos {at(pos,1):.2f}/{at(pos,2):.2f}/{at(pos,4):.2f} m | "
              f"u {at(spd,1):.2f}/{at(spd,2):.2f}/{at(spd,4):.2f} m/s | "
              f"psi {at(hdg,1):.3f}/{at(hdg,2):.3f}/{at(hdg,4):.3f} rad")


def _pick_run(pairs, terrain, profile):
    """Prefer the requested (terrain, speed-profile) run; fall back gracefully."""
    cand = [p for p in pairs if p[0] == terrain] or pairs
    pref = [p for p in cand if profile and profile in os.path.basename(os.path.dirname(p[1]))]
    return (pref or cand)[0]


def plot_overlay(npz, diag, terrain, every_s=1.2, max_h=40, out=None):
    p = np.load(npz)
    times, Z, dt = p["times"], p["Z"], float(p["dt"])
    t, cols = _actual(diag)
    H = min(max_h, Z.shape[2] - 1)
    horizon_s = H * dt

    # Launch a predicted-horizon fan every ~every_s of sim time.
    solve_dt = float(np.median(np.diff(times))) if len(times) > 1 else dt
    stride = max(1, int(round(every_s / max(solve_dt, 1e-3))))
    launches = list(range(0, len(times), stride))
    cmap = plt.get_cmap("viridis")

    fig, (axxy, axu, axp) = plt.subplots(1, 3, figsize=(15.5, 3.7))

    # (a) position: X-Y trajectory overlay (Dallas-style path comparison)
    axxy.plot(cols["x"], cols["y"], color="black", lw=2.2, zorder=5,
              label="actual path")
    for j, i in enumerate(launches):
        c = cmap(j / max(1, len(launches) - 1))
        axxy.plot(Z[i, 0, :H + 1], Z[i, 1, :H + 1], color=c, lw=1.0, alpha=0.7,
                  zorder=3, label="predicted horizons ($4\\,$s)" if j == 0 else None)
        # actual front-axle position at the same wall-clock as the horizon end,
        # so the gap to the predicted endpoint is the along-track position drift.
        te = times[i] + H * dt
        axxy.plot(Z[i, 0, H], Z[i, 1, H], 'o', color=c, ms=3.5, zorder=4)
        axxy.plot(np.interp(te, t, cols["x"]), np.interp(te, t, cols["y"]),
                  'x', color=c, ms=5, mew=1.4, zorder=4)
    axxy.set_xlabel("$x$ (m)"); axxy.set_ylabel("$y$ (m)")
    axxy.grid(alpha=0.3)
    axxy.legend(fontsize=8.0, loc="best")

    # (b) longitudinal speed u, (c) heading psi -- vs time
    for ax, comp, actual_key, ylab in [
            (axu, 3, "u", "longitudinal speed $u$ (m/s)"),
            (axp, 2, "psi", "heading $\\psi$ (rad)")]:
        # actual plant trace (bold black)
        av = cols[actual_key]
        if actual_key == "psi":
            av = np.unwrap(av)
        ax.plot(t, av, color="black", lw=2.2, zorder=5,
                label="actual (Chrono plant)")
        # predicted-horizon fans (color-graded by launch time)
        for j, i in enumerate(launches):
            th = times[i] + np.arange(H + 1) * dt
            pv = Z[i, comp, :H + 1].copy()
            if actual_key == "psi":
                pv = np.unwrap(pv)
            c = cmap(j / max(1, len(launches) - 1))
            ax.plot(th, pv, color=c, lw=1.0, alpha=0.7, zorder=3,
                    label="predicted horizons ($4\\,$s)" if j == 0 else None)
            # mark the launch point on the actual trace
            ax.plot(times[i], np.interp(times[i], t, av), 'o', color=c,
                    ms=3.5, zorder=4)
        ax.set_xlabel("time (s)"); ax.set_ylabel(ylab)
        ax.grid(alpha=0.3); ax.set_xlim(t.min(), t.max())
    axu.legend(fontsize=8.5, loc="best")
    fig.suptitle(
        f"Open-loop NMPC prediction vs. the Chrono plant "
        f"({TERR_GROUP.get(terrain, terrain)}, sinusoidal): each "
        f"{horizon_s:.0f} s predicted horizon branches off the actual trace",
        fontsize=11)
    fig.tight_layout()
    FIG.mkdir(parents=True, exist_ok=True)
    out = out or (FIG / "rollout_prediction_validation.png")
    fig.savefig(out, dpi=190, bbox_inches="tight")
    print(f"wrote {out}  (run: {os.path.basename(os.path.dirname(npz))}, "
          f"{len(launches)} horizons every {every_s:g}s)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glob", nargs="?",
                    default=str(ROOT / "benchmarking" / "results" / "mpc_tire_model_sweep_*"))
    ap.add_argument("--terrain", default="sand", help="terrain to feature in the overlay")
    ap.add_argument("--profile", default="v7", help="speed-profile token to prefer (e.g. v7)")
    ap.add_argument("--every", type=float, default=1.2, help="seconds between predicted fans")
    args = ap.parse_args()

    pairs = _pairs(args.glob)
    if not pairs:
        print(f"no (mpc_predictions.npz, diag) pairs under {args.glob}"); return
    print(f"found {len(pairs)} runs with predictions")
    terr, npz, diag = _pick_run(pairs, args.terrain, args.profile)
    plot_overlay(npz, diag, terr, every_s=args.every)
    _print_aggregate(pairs)


if __name__ == "__main__":
    main()
