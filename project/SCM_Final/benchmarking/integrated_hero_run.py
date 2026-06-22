#!/usr/bin/env python3
"""Integrated end-to-end 'hero' run: the whole stack in one mission.

The paper's thesis is that the *integration* is the contribution, yet every
layer is otherwise ablated in isolation. This script runs all of them together
on a single mission and plots the result as one timeline, so the integration is
shown concretely rather than asserted:

  terrain-aware NMPC + differentiable tire surrogate
  + online terrain estimator (Fused-UKF)         -> adapts n across a soil change
  + g--g terrain-aware speed planner + throttle DOB -> speed tracks the grip limit
  + DOB-CBF safety filter + forward collision warning -> handles the hazards
  all under the learned 5G command/camera latency.

The mission drives a sinusoidal reference across a clay->sand soil transition
through a rock field. Reproduce:

  python benchmarking/integrated_hero_run.py
"""
from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(HERE))
from common import timestamped_result_dir  # noqa: E402
from spatial_terrain import SpatialTransitionSpec, local_n_at  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--path", default="sinusoidal")
    p.add_argument("--time", type=float, default=45.0)
    p.add_argument("--speed", type=float, default=6.0)
    p.add_argument("--start", default="clay")
    p.add_argument("--end", default="sand")
    p.add_argument("--transition-x", type=float, default=45.0)
    p.add_argument("--transition-width", type=float, default=8.0)
    p.add_argument("--rocks", type=int, default=8)
    p.add_argument("--latency-profile-json",
                   default="config/latency_profiles/5g_hil_usable.json")
    p.add_argument("--base-port", type=int, default=12600)
    p.add_argument("--no-run", action="store_true",
                   help="Skip the sim; just re-plot from --reuse-dir.")
    p.add_argument("--reuse-dir", default="")
    return p.parse_args()


def run_mission(args, out_dir: Path) -> None:
    sim_diag = out_dir / "sim_diag.csv"
    cw_csv = out_dir / "collision_warning.csv"
    cmd = [
        sys.executable, str(ROOT / "simulation" / "launch_decoupled.py"),
        "--vis-mode", "none", "--path", args.path, "--time", str(args.time),
        "--speed", str(args.speed),
        # soil change along +x
        "--terrain-transition", "--terrain-start", args.start,
        "--terrain-end", args.end, "--transition-x", str(args.transition_x),
        "--transition-width", str(args.transition_width),
        # online estimator (deployed Fused-UKF)
        "--terrain-estimator", "--terrain-estimator-backend", "nn_ukf_aug",
        # safety filter + advisory warning
        "--safety-filter", "--safety-flavor", "dob_cbf",
        "--collision-warning", "--collision-warning-csv", str(cw_csv),
        # hazards spread along the whole traverse, near the sinusoidal envelope
        "--rocks", str(args.rocks),
        "--rock-zone-x", "15", "135", "--rock-zone-y", "-3.5", "3.5",
        "--rock-min-spacing", "7.0", "--rock-size", "0.6", "1.4",
        # 5G latency on both channels
        "--latency-profile-json", str(ROOT / args.latency_profile_json),
        "--sim-port", str(args.base_port), "--ctrl-port", str(args.base_port + 1),
        "--sim-diag-csv", str(sim_diag),
    ]
    env = dict(os.environ)
    env.setdefault("ACADOS_SOURCE_DIR", str(Path.home() / "Documents/sbel/acados"))
    # send the CBF intervention log into this run's dir
    env["HIL_RUN_LOG_DIR"] = str(out_dir)
    print("Running integrated mission ...\n  " + " ".join(cmd))
    log = (out_dir / "run.log").open("w")
    t0 = max((os.path.getmtime(p) for p in glob.glob(str(ROOT / "plots" / "*"))), default=0)
    subprocess.run(cmd, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT, env=env, timeout=900)
    # locate the controller diag produced by this run (newest after t0)
    diags = [p for p in glob.glob(str(ROOT / "plots" / "*" / "diag_*.csv"))
             if os.path.getmtime(p) >= t0]
    if diags:
        newest = max(diags, key=os.path.getmtime)
        (out_dir / "controller_diag.csv").write_bytes(Path(newest).read_bytes())
        print(f"  controller diag: {newest}")
    else:
        print("  WARNING: no controller diag found")


def plot_hero(args, out_dir: Path) -> None:
    spec = SpatialTransitionSpec(args.start, args.end, args.transition_x, args.transition_width)

    def _read(p):
        try:
            return pd.read_csv(p)
        except Exception:
            return pd.DataFrame()
    sd = _read(out_dir / "sim_diag.csv")
    cd = _read(out_dir / "controller_diag.csv")
    if sd.empty:
        raise SystemExit(f"sim_diag.csv empty/missing in {out_dir} -- the mission did "
                         f"not run; check run.log")

    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.4), sharex=True)
    x0, x1 = 0, float(sd["x"].max())
    xs = np.linspace(x0, x1, 400)

    # (1) terrain estimate vs ground truth n(x)
    ax = axes[0]
    ax.plot(xs, [local_n_at(x, spec) for x in xs], "k--", lw=1.6, label="true $n(x)$")
    if "n_terrain_est" in cd and "x_fa_meas" in cd:
        m = cd["n_terrain_est"].notna() & (cd["n_terrain_est"] > 0)
        ax.plot(cd["x_fa_meas"][m], cd["n_terrain_est"][m], color="#2d6cdf", lw=1.3,
                label=r"estimated $\hat n$ (Fused-UKF)")
    ax.axvspan(args.transition_x - args.transition_width / 2,
               args.transition_x + args.transition_width / 2, color="0.85", zorder=0,
               label=f"{args.start}$\\to${args.end} blend")
    ax.set_ylabel("Bekker $n$"); ax.legend(fontsize=8, loc="best")
    ax.set_title("Integrated mission: online terrain estimate adapts across the soil change")

    # (2) speed: g--g planner reference vs achieved (DOB closes the gap)
    ax = axes[1]
    if "v_ref_0" in cd and "x_fa_meas" in cd:
        ax.plot(cd["x_fa_meas"], cd["v_ref_0"], color="#9b59b6", lw=1.3,
                label="g--g speed reference")
        ax.plot(cd["x_fa_meas"], cd["u_meas"], color="#28c76f", lw=1.3, label="achieved speed")
    else:
        ax.plot(sd["x"], sd["speed"], color="#28c76f", lw=1.3, label="achieved speed")
    ax.axvspan(args.transition_x - args.transition_width / 2,
               args.transition_x + args.transition_width / 2, color="0.85", zorder=0)
    ax.set_ylabel("speed (m/s)"); ax.legend(fontsize=8, loc="best")

    # (3) safety: clearance to the nearest hazard + true filter interventions
    ax = axes[2]
    CAP = 8.0  # clamp display: once the nearest hazard is far, the value is uninformative
    clr = sd["nearest_clearance_m"].clip(upper=CAP)
    ax.plot(sd["x"], clr, color="#b0392b", lw=1.2, label=f"clearance to nearest hazard (cap {CAP:g} m)")
    ax.axhline(0, color="0.5", ls=":", lw=1)
    # Genuine avoidance: rows where a CBF obstacle constraint was binding
    # (active_constraints > 0), not the every-tick actuator smoothing.
    cbf = _read(out_dir / "cbf_filter_log.csv")
    if not cbf.empty and "x" in cbf:
        ev = cbf[cbf.get("active_constraints", 0) > 0] if "active_constraints" in cbf else cbf
        ev = ev.drop_duplicates("x")
        if len(ev):
            yv = np.interp(ev["x"], sd["x"], clr)
            ax.scatter(ev["x"], yv, s=12, color="#e0a30c", zorder=3,
                       label=f"CBF avoidance active ({len(ev)})")
    coll = sd["collisions"].diff().fillna(0) > 0 if "collisions" in sd else pd.Series(dtype=bool)
    if coll.any():
        ax.scatter(sd["x"][coll], clr[coll], s=40, marker="x", color="k",
                   label="collision", zorder=4)
    ax.axvspan(args.transition_x - args.transition_width / 2,
               args.transition_x + args.transition_width / 2, color="0.85", zorder=0)
    ax.set_ylabel("clearance (m)"); ax.set_xlabel("distance travelled $x$ (m)")
    ax.legend(fontsize=8, loc="best")

    fig.tight_layout()
    out = out_dir / "integrated_hero_run.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def main() -> None:
    args = parse_args()
    if args.no_run and args.reuse_dir:
        out_dir = Path(args.reuse_dir)
    else:
        out_dir = timestamped_result_dir("integrated_hero_run")
        run_mission(args, out_dir)
    print(f"Output: {out_dir}")
    plot_hero(args, out_dir)


if __name__ == "__main__":
    main()
