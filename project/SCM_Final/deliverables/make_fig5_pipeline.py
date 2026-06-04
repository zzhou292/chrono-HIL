#!/usr/bin/env python3
"""Render the 5G latency-generation pipeline architecture (figure 5)."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt


def box(ax, x, y, w, h, text, face="#e8f0fa", edge="#1f4e79", fontsize=9, ha="center"):
    rect = mpatches.FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.08,rounding_size=0.08",
        linewidth=1.4, edgecolor=edge, facecolor=face,
    )
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, wrap=True)


def arrow(ax, x1, y1, x2, y2, label=None, label_offset=(0, 0.07), color="#333"):
    ax.annotate(
        "", xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle="->", lw=1.6, color=color, mutation_scale=14),
    )
    if label:
        ax.text((x1 + x2) / 2 + label_offset[0],
                (y1 + y2) / 2 + label_offset[1],
                label, ha="center", va="center",
                fontsize=8, color="#222",
                bbox=dict(boxstyle="round,pad=0.18", facecolor="white",
                          edgecolor="#aaa", linewidth=0.7))


def main():
    out = Path(__file__).parent / "figures" / "fig5_5g_pipeline_architecture.png"
    out.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11.5, 4.6))
    ax.set_xlim(0, 11.5)
    ax.set_ylim(0, 4.6)
    ax.axis("off")

    # Row 1: training time
    ax.text(0.05, 4.3, "Offline (training)",
            fontsize=9, color="#666", fontstyle="italic")
    box(ax, 0.2, 3.0, 2.0, 0.9,
        "Public 5G uplink\ndataset\n(Choi et al., 2023)",
        face="#fff4e0", edge="#b06000")
    box(ax, 2.6, 3.0, 2.0, 0.9,
        "N-HiTS forecaster\n(train_5g_nhits.py)",
        face="#e6f4ea", edge="#1f6f37")
    arrow(ax, 2.2, 3.45, 2.6, 3.45, label="bitrate traces")

    box(ax, 5.0, 3.0, 2.2, 0.9,
        "Trained checkpoint\n+ synthetic\nbitrate trace (CSV)",
        face="#e6f4ea", edge="#1f6f37")
    arrow(ax, 4.6, 3.45, 5.0, 3.45)

    # Row 2: deploy time
    ax.text(0.05, 2.0, "Runtime (per simulation tick)",
            fontsize=9, color="#666", fontstyle="italic")

    # Profile JSON
    box(ax, 0.2, 0.55, 2.0, 1.05,
        "5G profile JSON\n(config/latency_profiles/)\nbitrate → queue-load\nparameters",
        face="#fff4e0", edge="#b06000", fontsize=8.5)
    # Vertical arrow from trace box down to profile json
    arrow(ax, 6.1, 3.0, 6.1, 2.05, label="bake into config")
    arrow(ax, 5.9, 1.95, 1.5, 1.6, label=None)

    # LatencyProfile sampler
    box(ax, 2.6, 0.55, 2.2, 1.05,
        "LatencyProfile sampler\n(simulation/latency_profile.py)\nb(t) → schedule",
        face="#e8f0fa", edge="#1f4e79", fontsize=8.5)
    arrow(ax, 2.2, 1.08, 2.6, 1.08, label="b(t) [bps]")

    # Queue-load model
    box(ax, 5.2, 0.55, 2.4, 1.05,
        "Queue-load model\nτ(t) = base + queue_gain ×\n(b(t)/capacity)\nclip to [min,max]",
        face="#e8f0fa", edge="#1f4e79", fontsize=8.5)
    arrow(ax, 4.8, 1.08, 5.2, 1.08, label="bps→ms")

    # Per-channel delays
    box(ax, 8.0, 1.40, 3.0, 0.85,
        "uplink (command) τ_cmd(t)",
        face="#f3e6f3", edge="#7a2c7a", fontsize=9)
    box(ax, 8.0, 0.30, 3.0, 0.85,
        "downlink (camera) τ_cam(t)\n= s · τ_cmd(t)",
        face="#f3e6f3", edge="#7a2c7a", fontsize=9)
    arrow(ax, 7.6, 1.30, 8.0, 1.78, label=None)
    arrow(ax, 7.6, 0.85, 8.0, 0.72, label=None)

    # Title
    fig.suptitle("5G latency-generation pipeline", fontsize=12, y=0.985)

    # Legend
    legend = [
        mpatches.Patch(facecolor="#fff4e0", edgecolor="#b06000", label="data / config"),
        mpatches.Patch(facecolor="#e6f4ea", edgecolor="#1f6f37", label="learned model"),
        mpatches.Patch(facecolor="#e8f0fa", edgecolor="#1f4e79", label="runtime simulator"),
        mpatches.Patch(facecolor="#f3e6f3", edgecolor="#7a2c7a", label="injected per-channel delay"),
    ]
    ax.legend(handles=legend, loc="upper right", fontsize=8, ncol=1,
              frameon=True, framealpha=0.9)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
