#!/usr/bin/env python3
"""Paper experiment: generate raw data and figures for one 5G latency profile."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from latency_profile import LatencyProfile  # noqa: E402
from common import timestamped_result_dir, write_manifest  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--profile-json",
        default=str(ROOT / "config" / "latency_profiles" / "5g_good_bad_control.json"),
        help="Latency profile JSON to sample.",
    )
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--dt", type=float, default=0.05)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    profile = LatencyProfile.from_json(args.profile_json)
    out_dir = timestamped_result_dir("latency_profile_figure")
    write_manifest(out_dir, args, "Sampled 5G-like latency profile for paper figures.")
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    times = []
    rows = []
    n = max(1, int(args.duration / args.dt) + 1)
    for i in range(n):
        t = i * args.dt
        times.append(t)
        rows.append({
            "time_s": t,
            "control_delay_ms": 1000.0 * profile.delay(t, "control"),
            "manual_delay_ms": 1000.0 * profile.delay(t, "manual"),
            "camera_delay_ms": 1000.0 * profile.delay(t, "camera"),
        })
    df = pd.DataFrame(rows)
    df.to_csv(out_dir / "latency_profile_samples.csv", index=False)

    fig, ax = plt.subplots(figsize=(11, 4.2), constrained_layout=True)
    ax.plot(df["time_s"], df["control_delay_ms"], label="control", linewidth=1.8)
    ax.plot(df["time_s"], df["manual_delay_ms"], label="manual", linewidth=1.2, alpha=0.8)
    ax.plot(df["time_s"], df["camera_delay_ms"], label="camera", linewidth=1.2, alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("One-way latency (ms)")
    ax.set_title("Scheduled 5G-like latency profile")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(fig_dir / "latency_profile_timeseries.png", dpi=240)
    plt.close(fig)

    # The profile is bimodal: a low-latency "good-window" bulk plus discrete
    # outage spikes (control->450 ms, camera->660 ms) with an empty gap in
    # between, so a log-x histogram keeps both regimes legible instead of
    # crushing the bulk into the leftmost linear bins. control==manual here, so
    # we plot the two distinct channels named in the caption (control, camera).
    fig, ax = plt.subplots(figsize=(6.0, 4.2), constrained_layout=True)
    allv = pd.concat([df["control_delay_ms"], df["camera_delay_ms"]])
    edges = np.logspace(np.log10(max(1.0, allv.min())),
                        np.log10(allv.max() * 1.05), 28)
    for col, label, color in [
        ("control_delay_ms", "control", "#4c78a8"),
        ("camera_delay_ms", "camera", "#59a14f"),
    ]:
        ax.hist(df[col], bins=edges, alpha=0.55, label=label, color=color)
        ax.axvline(df[col].mean(), color=color, ls="--", lw=1.3)
    ax.set_xscale("log")
    ax.set_xlabel("one-way latency (ms, log scale)")
    ax.set_ylabel("samples")
    ax.set_title("5G uplink latency distribution (dashed = channel mean)")
    ax.grid(alpha=0.25, which="both")
    ax.legend()
    fig.savefig(fig_dir / "latency_profile_histogram.png", dpi=200)
    plt.close(fig)

    summary = df.drop(columns=["time_s"]).agg(["mean", "std", "min", "max"]).T
    summary.to_csv(out_dir / "summary.csv")
    print(f"Profile: {profile.describe()}")
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
