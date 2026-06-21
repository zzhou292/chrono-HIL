#!/usr/bin/env python3
"""Time-series of the joint (n, phi) terrain estimator.

The joint estimator (``nn_models/terrain_window_joint_n_phi``) is a
sliding-window regressor: each 4 s window of vehicle-dynamics features
yields one (n_hat, phi_hat) pair. Sliding the window across a trace
therefore produces a time series of both estimates. This script picks a
handful of representative rich-excitation soils spanning the n/phi box,
slides the joint model across each, and plots n_hat(t) and phi_hat(t)
against the true soil parameters.

Output: my_paper/paper_figures/joint_estimator_timeseries.png
"""

from __future__ import annotations

import json
import pickle
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from train_terrain_window_mlp import load_trace, compute_window_features  # noqa: E402

MODEL_DIR = ROOT / "nn_models" / "terrain_window_joint_n_phi"
TRACE_DIR = ROOT / "data" / "terrain_traces_rich"
OUT = ROOT / "my_paper" / "paper_figures" / "joint_estimator_timeseries.png"

_NAME_RE = re.compile(r"_n(\d+)_phi(\d+)")


class _HeadMLP(nn.Module):
    """Mirror of HeadMLP in utilities/exp_joint_n_phi.py (2-output head).

    The submodule must be named ``net`` so the saved state_dict keys
    (``net.0.weight`` ...) load without remapping.
    """

    def __init__(self, n_in: int, hidden: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_in, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, 2),
        )

    def forward(self, x):
        return self.net(x)


def build_model(n_in: int, hidden: int) -> nn.Module:
    return _HeadMLP(n_in, hidden)


def soil_params_from_name(stem: str) -> tuple[float, float] | None:
    m = _NAME_RE.search(stem)
    if not m:
        return None
    # filename encodes n*1000 and phi*10, e.g. _n0552_phi192 -> n=0.552 phi=19.2
    return int(m.group(1)) / 1000.0, int(m.group(2)) / 10.0


def slide(model, scaler, csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (window_end_time, n_hat, phi_hat) for one trace."""
    t, dyn, thr, _, _ = load_trace(csv_path)
    win_s = float(scaler["win_seconds"])
    dt = float(np.median(np.diff(t)))
    win_n = max(int(round(win_s / dt)), 8)
    stride_n = max(int(round(0.3 / dt)), 1)
    warmup_n = max(int(round(1.5 / dt)), 0)
    x_mean, x_std = scaler["x_mean"], scaler["x_std"]
    y_mean, y_std = scaler["y_mean"], scaler["y_std"]

    times, feats = [], []
    end = warmup_n + win_n
    while end <= len(t):
        window = dyn[end - win_n:end]
        thr_w = thr[end - win_n:end]
        if np.all(np.isfinite(window)):
            feats.append(compute_window_features(window, thr_w))
            times.append(float(t[end - 1] - t[0]))
        end += stride_n
    if not feats:
        return np.array([]), np.array([]), np.array([])
    X = (np.asarray(feats) - x_mean) / x_std
    with torch.no_grad():
        Y = model(torch.tensor(X, dtype=torch.float32)).numpy()
    Y = Y * y_std + y_mean
    return np.asarray(times), Y[:, 0], Y[:, 1]


def main() -> None:
    cfg = json.loads((MODEL_DIR / "config.json").read_text())
    with open(MODEL_DIR / "scaler.pkl", "rb") as f:
        scaler = pickle.load(f)
    model = build_model(cfg["n_features"], cfg["hidden"])
    model.load_state_dict(torch.load(MODEL_DIR / "weights.pt", map_location="cpu"))
    model.eval()

    # Pick four soils nearest the corners of the (n, phi) sampling box so
    # the figure shows the estimator separating both parameters.
    traces = {}
    for csv in sorted(TRACE_DIR.glob("*_seed0.csv")):
        sp = soil_params_from_name(csv.stem)
        if sp is not None:
            traces[csv] = sp
    if not traces:
        sys.exit(f"no traces in {TRACE_DIR}")
    ns = [v[0] for v in traces.values()]
    ps = [v[1] for v in traces.values()]
    corners = [(min(ns), min(ps)), (min(ns), max(ps)),
               (max(ns), min(ps)), (max(ns), max(ps))]
    picks = []
    for cn, cp in corners:
        best = min(traces.items(),
                   key=lambda kv: ((kv[1][0] - cn) / 0.4) ** 2
                                  + ((kv[1][1] - cp) / 16.0) ** 2)
        if best not in picks:
            picks.append(best)

    colors = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd"]
    fig, (ax_n, ax_p) = plt.subplots(2, 1, figsize=(7.4, 6.0), sharex=True)
    for (csv, (n_true, phi_true)), c in zip(picks, colors):
        tt, nh, ph = slide(model, scaler, csv)
        if tt.size == 0:
            continue
        lbl = f"n={n_true:.2f}, $\\phi$={phi_true:.1f}$^\\circ$"
        ax_n.plot(tt, nh, color=c, lw=1.6, label=lbl)
        ax_n.axhline(n_true, color=c, ls="--", lw=0.9, alpha=0.6)
        ax_p.plot(tt, ph, color=c, lw=1.6, label=lbl)
        ax_p.axhline(phi_true, color=c, ls="--", lw=0.9, alpha=0.6)

    ax_n.set_ylabel(r"$\hat n$ (Bekker sinkage exponent)")
    ax_n.set_title("Joint terrain estimator: $\\hat n$ and $\\hat\\phi$ over time")
    ax_n.grid(alpha=0.3)
    ax_n.legend(fontsize=8, ncols=2)
    ax_p.set_ylabel(r"$\hat\phi$ (friction angle, deg)")
    ax_p.set_xlabel("trace time (s)")
    ax_p.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT, dpi=240)
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
