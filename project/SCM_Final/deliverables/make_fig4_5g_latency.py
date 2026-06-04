#!/usr/bin/env python3
"""Single-panel 5G latency figure: time-series + histogram side by side."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SAMPLES = ROOT / "benchmarking" / "results" / "latency_profile_figure_20260524_000355" / "latency_profile_samples.csv"
OUT = Path(__file__).parent / "figures" / "fig4_5g_latency.png"


COLORS = {
    "control_delay_ms": "#1f77b4",
    "camera_delay_ms": "#d62728",
}
LABELS = {
    "control_delay_ms": "Uplink (control command)",
    "camera_delay_ms": "Downlink (driver camera)",
}


def main():
    df = pd.read_csv(SAMPLES)
    OUT.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.6),
                             gridspec_kw=dict(width_ratios=[2.2, 1.0]))

    # Left: time-series. Show a representative 30 s window.
    ax = axes[0]
    t = df["time_s"].to_numpy()
    win_max_t = min(t.max(), 30.0)
    mask = t <= win_max_t
    for col, color in COLORS.items():
        ax.plot(t[mask], df[col].to_numpy()[mask],
                label=LABELS[col], color=color, lw=1.4)
    ax.set_xlabel("Sim time (s)")
    ax.set_ylabel("End-to-end latency (ms)")
    ax.set_title(f"Per-channel latency trace (5G good/bad/outage profile, first {win_max_t:.0f} s)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)

    # Right: histogram with mean / 95th-pct annotations.
    ax = axes[1]
    bins = np.linspace(0,
                       max(df["control_delay_ms"].max(),
                           df["camera_delay_ms"].max()) * 1.02,
                       40)
    for col, color in COLORS.items():
        vals = df[col].dropna().to_numpy()
        ax.hist(vals, bins=bins, alpha=0.55, color=color,
                label=LABELS[col].split()[0],
                edgecolor="white", linewidth=0.4)
        mean = float(np.mean(vals))
        p95 = float(np.percentile(vals, 95))
        ax.axvline(mean, color=color, lw=1.2, ls="--", alpha=0.9)
        ax.text(mean, ax.get_ylim()[1] * 0.92 if col == "control_delay_ms" else ax.get_ylim()[1] * 0.78,
                f"  μ={mean:.0f} ms\n  p95={p95:.0f} ms",
                color=color, fontsize=8, va="top")
    ax.set_xlabel("Latency (ms)")
    ax.set_ylabel("Samples")
    ax.set_title("Latency distribution")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    fig.suptitle("Generated 5G latency profile applied to teleop channels",
                 fontsize=12, y=1.0)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
