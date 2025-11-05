#!/usr/bin/env python3
"""
Visualize recorded waypoint files produced by proj_HIL_teleopcity.

Usage:
    python visualize_waypoints.py path.json [--resample 0.5] [--smooth 1.0]

This script plots the X/Y path and overlays heading arrows. Optional resampling
and smoothing mirrors the in-sim processing so you can inspect what the driver sees.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def load_samples(path: Path) -> List[Tuple[float, np.ndarray, np.ndarray]]:
    data = json.loads(path.read_text())
    samples = []
    for entry in data.get("samples", []):
        time = entry["time"]
        pos = np.array(entry["pos"], dtype=float)
        rot = np.array(entry["rot"], dtype=float)  # quaternion w, x, y, z
        samples.append((time, pos, rot))
    if not samples:
        raise RuntimeError(f"No samples found in {path}")
    return samples


def quaternion_to_yaw(quat: np.ndarray) -> float:
    # Chrono uses [e0, e1, e2, e3] = [w, x, y, z]
    w, x, y, z = quat
    # yaw (around Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def resample_points(points: np.ndarray, spacing: float) -> np.ndarray:
    if len(points) < 2:
        return points
    spacing = max(spacing, 1e-3)
    resampled = [points[0]]
    accum = 0.0
    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]
        delta = end - start
        seg_len = np.linalg.norm(delta)
        if seg_len < 1e-6:
            continue
        direction = delta / seg_len
        dist = spacing
        while dist < seg_len:
            resampled.append(start + direction * dist)
            dist += spacing
        resampled.append(end)
    return np.array(resampled)


def smooth_points(points: np.ndarray, spacing: float, window: float) -> np.ndarray:
    if len(points) <= 2 or window <= 0.0:
        return points
    window_half = max(int(round(max(window / spacing, 1.0))), 1)
    smoothed = np.empty_like(points)
    for i in range(len(points)):
        start = max(0, i - window_half)
        end = min(len(points) - 1, i + window_half)
        smoothed[i] = points[start : end + 1].mean(axis=0)
    smoothed[0] = points[0]
    smoothed[-1] = points[-1]
    return smoothed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("json_path", type=Path, help="Recorded waypoint JSON file")
    parser.add_argument("--resample", type=float, default=None, help="Resample spacing in meters")
    parser.add_argument("--smooth", type=float, default=0.0, help="Smoothing window in meters")
    parser.add_argument("--arrow-step", type=int, default=10, help="Plot every Nth heading arrow")
    args = parser.parse_args()

    samples = load_samples(args.json_path)
    times = [s[0] for s in samples]
    positions = np.array([s[1] for s in samples])
    headings = [quaternion_to_yaw(s[2]) for s in samples]

    plot_points = positions
    if args.resample:
        plot_points = resample_points(plot_points, args.resample)
    if args.smooth and args.resample:
        plot_points = smooth_points(plot_points, args.resample, args.smooth)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(positions[:, 0], positions[:, 1], "o-", alpha=0.3, label="Raw samples")
    ax.plot(plot_points[:, 0], plot_points[:, 1], "-", linewidth=2, label="Processed path")

    arrow_step = max(args.arrow_step, 1)
    arrow_positions = positions[::arrow_step]
    arrow_headings = headings[::arrow_step]
    u = np.cos(arrow_headings)
    v = np.sin(arrow_headings)
    ax.quiver(
        arrow_positions[:, 0],
        arrow_positions[:, 1],
        u,
        v,
        angles="xy",
        scale_units="xy",
        scale=1.5,
        color="tab:red",
        width=0.003,
        label="Heading",
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"Waypoint Path: {args.json_path.name}\nSamples: {len(samples)}, duration: {times[-1]-times[0]:.2f}s")
    ax.legend()
    ax.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
