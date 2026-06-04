#!/usr/bin/env python3
"""LSTM variant of the sliding-window terrain estimator.

Same inputs as the MLP trainer (vehicle dynamics + IMU + wheel rates +
throttle), but instead of summarising each 4-second window to 26
hand-crafted statistics, we feed the **raw downsampled time series**
through a 2-layer LSTM and predict Bekker n from the final hidden state.

Hypothesis: the MLP cannot exploit *transient* events (an accel burst
2 s ago) to disambiguate steady-state ambiguity later in the window —
which is the suspected reason clay closed-loop NMPC bimodally flips
between predicting clay and dirt. An LSTM should retain that earlier
context.

The trainer reuses ``train_terrain_window_mlp.load_trace`` so we read
exactly the same broad-v7 CSVs.
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
import time as wall_time
from pathlib import Path
from typing import List, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# Reuse MLP trainer's data loaders
sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_terrain_window_mlp import (  # noqa: E402
    CSV_COLS_USED, WHEEL_RADIUS, load_trace,
)


# ────────────────────────────────────────────────────────────────────────
# Raw-sequence feature construction (no hand-crafted statistics)
# ────────────────────────────────────────────────────────────────────────
# Each timestep gets the same 13 raw channels the MLP windowing used,
# plus 2 derived: throttle (commanded) and rear-axle wheel slip ratio.
RAW_FEATURE_NAMES = list(CSV_COLS_USED) + ["throttle_cmd", "rear_slip"]
N_RAW_FEATURES = len(RAW_FEATURE_NAMES)


def _raw_seq(window: np.ndarray, thr_w: np.ndarray) -> np.ndarray:
    """Concatenate the raw window (T, 13) with throttle and per-step slip
    so the LSTM sees the same exogenous channels at every timestep."""
    u = window[:, 0]
    w_avg = 0.5 * (window[:, 7] + window[:, 8])   # rear axle
    u_safe = np.maximum(np.abs(u), 0.5)
    slip = (w_avg * WHEEL_RADIUS - u) / u_safe
    return np.column_stack([window, thr_w, slip])


def build_windows_seq(traces, *, win_seconds=4.0, stride_seconds=0.4,
                       warmup_seconds=1.5, target_dt=0.030,
                       return_counts: bool = False):
    """Return (X seqs, y, terrain_per_sample, src_per_sample[, win_per_trace]).

    Sequences are downsampled to ``target_dt`` so every sample has the
    same ``seq_len`` regardless of the raw trace sample rate. With
    win=4s, dt=0.030 → seq_len ≈ 133.
    """
    seq_len = int(round(win_seconds / target_dt))
    X = []
    y = []
    terr = []
    src = []
    win_per_trace: list[int] = []
    for t, dyn, thr, n_true, terrain in traces:
        added = 0
        if t.size < 50:
            win_per_trace.append(0)
            continue
        dt = float(np.median(np.diff(t)))
        if dt <= 0:
            win_per_trace.append(0)
            continue
        win_n   = max(int(round(win_seconds / dt)), 8)
        stride_n = max(int(round(stride_seconds / dt)), 1)
        warmup_n = max(int(round(warmup_seconds / dt)), 0)

        end = warmup_n + win_n
        while end <= len(t):
            window = dyn[end - win_n: end]
            thr_w  = thr[end - win_n: end]
            if not np.all(np.isfinite(window)):
                end += stride_n
                continue
            raw = _raw_seq(window, thr_w)
            # Downsample to fixed length seq_len
            idx = np.linspace(0, raw.shape[0] - 1, seq_len).astype(int)
            X.append(raw[idx])
            y.append(n_true)
            terr.append(terrain)
            src.append("")
            added += 1
            end += stride_n
        win_per_trace.append(added)

    if not X:
        raise RuntimeError("no training windows built")
    Xa = np.stack(X, axis=0).astype(np.float32)
    ya = np.asarray(y, dtype=np.float32)
    if return_counts:
        return Xa, ya, terr, src, win_per_trace
    return Xa, ya, terr, src


# ────────────────────────────────────────────────────────────────────────
# Model
# ────────────────────────────────────────────────────────────────────────
class TerrainWindowLSTM(nn.Module):
    def __init__(self, n_in: int, hidden: int = 64, n_layers: int = 2,
                 dropout: float = 0.1):
        super().__init__()
        self.lstm = nn.LSTM(
            input_size=n_in,
            hidden_size=hidden,
            num_layers=n_layers,
            batch_first=True,
            dropout=dropout if n_layers > 1 else 0.0,
        )
        self.head = nn.Sequential(
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )

    def forward(self, x):                 # x: (B, T, F)
        _, (h_n, _) = self.lstm(x)        # h_n: (n_layers, B, H)
        return self.head(h_n[-1])         # (B, 1)


# ────────────────────────────────────────────────────────────────────────
# Train / eval loop
# ────────────────────────────────────────────────────────────────────────
def _scale_fit(X: np.ndarray):
    """Compute per-feature mean/std across (samples, time) jointly."""
    flat = X.reshape(-1, X.shape[-1])
    mean = flat.mean(axis=0)
    std = flat.std(axis=0)
    std[std < 1e-6] = 1.0
    return mean.astype(np.float32), std.astype(np.float32)


def _scale_apply(X, mean, std):
    return (X - mean) / std


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--win-seconds", type=float, default=4.0)
    ap.add_argument("--stride-seconds", type=float, default=0.4)
    ap.add_argument("--warmup-seconds", type=float, default=1.5)
    ap.add_argument("--target-dt", type=float, default=0.030)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--layers", type=int, default=2)
    ap.add_argument("--dropout", type=float, default=0.10)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--normalize-y", action="store_true")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--max-traces", type=int, default=None)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    import re
    csvs = sorted(p for p in args.trace_dir.glob("*.csv") if p.name != "manifest.csv")
    if args.max_traces:
        csvs = csvs[:args.max_traces]
    print(f"[lstm-train] {len(csvs)} traces from {args.trace_dir}")

    # The 'terrain' CSV field is the preset-proxy (clay/dirt/sand) — only
    # 3 unique values. For LHS-aware splitting we need the actual LHS cell
    # identifier, which is in the FILENAME (e.g. "rich0020_n0401_phi333").
    def _cell_from_filename(p: Path) -> str:
        m = re.match(r"(rich\d+_n\d+_phi\d+)_", p.name)
        return m.group(1) if m else p.stem

    traces = []
    trace_cells = []
    for c in csvs:
        try:
            traces.append(load_trace(c))
            trace_cells.append(_cell_from_filename(c))
        except Exception as e:
            print(f"  skip {c.name}: {e}")
    print(f"[lstm-train] loaded {len(traces)} traces, "
          f"{len(set(trace_cells))} unique LHS cells")

    t0 = wall_time.time()
    X, y, _terr_all, _, win_per_trace = build_windows_seq(
        traces,
        win_seconds=args.win_seconds,
        stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds,
        target_dt=args.target_dt,
        return_counts=True,
    )
    # Propagate the LHS cell label to each window
    cell_per_window: List[str] = []
    for cell, n in zip(trace_cells, win_per_trace):
        cell_per_window.extend([cell] * n)
    cell_per_window = np.asarray(cell_per_window)
    print(f"[lstm-train] windows: {X.shape}  y: {y.shape}  "
          f"({(wall_time.time()-t0):.1f}s)")

    # LHS-cell-aware split — held-out cells never appear in training
    cells = sorted(set(cell_per_window.tolist()))
    rng = np.random.default_rng(args.seed)
    rng.shuffle(cells)
    n_val = max(1, int(len(cells) * args.val_frac))
    val_cells = set(cells[:n_val])
    train_mask = np.array([c not in val_cells for c in cell_per_window])
    val_mask = ~train_mask
    print(f"[lstm-train] train_cells={len(cells)-n_val} val_cells={n_val} "
          f"train_windows={int(train_mask.sum())} val_windows={int(val_mask.sum())}")

    # Standardize inputs on train, apply to val
    x_mean, x_std = _scale_fit(X[train_mask])
    X = _scale_apply(X, x_mean, x_std)

    # Normalize y
    if args.normalize_y:
        y_mean = float(y[train_mask].mean()); y_std = float(y[train_mask].std() + 1e-9)
    else:
        y_mean = 0.0; y_std = 1.0
    y_norm = (y - y_mean) / y_std

    # Tensors
    device = torch.device(args.device)
    Xt = torch.from_numpy(X)
    yt = torch.from_numpy(y_norm.astype(np.float32))
    train_idx = np.where(train_mask)[0]
    val_idx   = np.where(val_mask)[0]

    model = TerrainWindowLSTM(n_in=X.shape[-1], hidden=args.hidden,
                               n_layers=args.layers,
                               dropout=args.dropout).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr)

    print(f"[lstm-train] model params: "
          f"{sum(p.numel() for p in model.parameters()):,}")
    print(f"[lstm-train] device: {device}")

    best_val = float("inf"); best_state = None
    n_train = len(train_idx); bs = args.batch
    t_train = wall_time.time()
    for ep in range(1, args.epochs + 1):
        model.train()
        rng.shuffle(train_idx)
        losses = []
        for i in range(0, n_train, bs):
            idx = train_idx[i:i+bs]
            xb = Xt[idx].to(device)
            yb = yt[idx].to(device).unsqueeze(-1)
            pred = model(xb)
            loss = ((pred - yb) ** 2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
            losses.append(float(loss))
        with torch.no_grad():
            model.eval()
            v_losses = []
            for i in range(0, len(val_idx), bs):
                idx = val_idx[i:i+bs]
                xb = Xt[idx].to(device)
                yb = yt[idx].to(device).unsqueeze(-1)
                pred = model(xb)
                v_losses.append(float(((pred - yb) ** 2).mean()))
            tr = float(np.mean(losses)); va = float(np.mean(v_losses))
        if va < best_val:
            best_val = va
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        if ep == 1 or ep % 5 == 0 or ep == args.epochs:
            print(f"  ep={ep:3d}  train_mse={tr:.5f}  val_mse={va:.5f}  "
                  f"(best={best_val:.5f})  elapsed={(wall_time.time()-t_train)/60:.1f}min")

    # Save
    torch.save(best_state, args.out_dir / "weights.pt")
    with open(args.out_dir / "scaler.pkl", "wb") as f:
        pickle.dump(dict(x_mean=x_mean, x_std=x_std,
                          y_mean=np.array([y_mean]), y_std=np.array([y_std])), f)
    cfg = dict(
        win_seconds=args.win_seconds,
        stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds,
        target_dt=args.target_dt,
        hidden=args.hidden,
        layers=args.layers,
        dropout=args.dropout,
        n_in=int(X.shape[-1]),
        seq_len=int(X.shape[1]),
        normalize_y=bool(args.normalize_y),
        output_names=["n"],
        architecture="lstm",
        feature_names=RAW_FEATURE_NAMES,
    )
    with open(args.out_dir / "config.json", "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"[lstm-train] best val_mse={best_val:.5f}  rmse={np.sqrt(best_val)*y_std:.4f}")
    print(f"[lstm-train] saved to {args.out_dir}")


if __name__ == "__main__":
    main()
