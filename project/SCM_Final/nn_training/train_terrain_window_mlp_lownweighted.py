#!/usr/bin/env python3
"""Task 1 try 2: re-train v7 MLP with weighted MSE upweighting low-n samples.

The standard MSE loss makes the model regress toward the training-distribution
mean (≈0.85 since broad_v7 is approximately uniform on [0.4, 1.3]). At low n,
the slip / dynamic signatures fade, so the model defaults to "dirt-ish",
producing the systematic +0.15–0.20 bias we see at true_n ≤ 0.5.

This trainer keeps the same architecture and data but adds a per-sample
weight
    w(n) = 1 + alpha * max(0, threshold - n)
so samples with n < threshold contribute alpha-times more to the loss. The
hope is that the model trades a small amount of mid/high-n accuracy for
much-needed low-n improvement.
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
import time as wall_time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_terrain_window_mlp import (   # noqa: E402
    FEATURE_NAMES, N_FEATURES, TerrainWindowMLP, build_windows, load_trace,
    compute_window_features,
)


def _build_windows_with_cells(traces, trace_cells, *, win_seconds, stride_seconds,
                               warmup_seconds):
    """Single pass over traces — returns X, y, cell label per window."""
    X = []
    y = []
    cells = []
    for (t, dyn, thr, n_true, _terr), cell in zip(traces, trace_cells):
        if t.size < 50:
            continue
        dt = float(np.median(np.diff(t)))
        if dt <= 0:
            continue
        win_n = max(int(round(win_seconds / dt)), 8)
        stride_n = max(int(round(stride_seconds / dt)), 1)
        warmup_n = max(int(round(warmup_seconds / dt)), 0)
        end = warmup_n + win_n
        while end <= len(t):
            window = dyn[end - win_n: end]
            thr_w = thr[end - win_n: end]
            if not np.all(np.isfinite(window)):
                end += stride_n
                continue
            feat = compute_window_features(window, thr_w)
            X.append(feat)
            y.append(n_true)
            cells.append(cell)
            end += stride_n
    return (np.stack(X, axis=0),
            np.asarray(y, dtype=np.float64),
            np.asarray(cells))


def _sample_weights(y, alpha: float, threshold: float):
    """Per-sample weight = 1 + alpha * max(0, threshold - y).

    With alpha=4, threshold=0.55:
      n=0.40 → w=1.60
      n=0.45 → w=1.40
      n=0.55 → w=1.00
      n≥0.55 → w=1.00
    """
    weights = 1.0 + alpha * np.maximum(0.0, threshold - y)
    return weights.astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--win-seconds", type=float, default=4.0)
    ap.add_argument("--stride-seconds", type=float, default=0.4)
    ap.add_argument("--warmup-seconds", type=float, default=1.5)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--epochs", type=int, default=400)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--normalize-y", action="store_true")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--low-n-alpha", type=float, default=4.0,
                    help="Loss weight on low-n samples: w = 1 + alpha*max(0, "
                         "threshold - n). 0 = standard MSE.")
    ap.add_argument("--low-n-threshold", type=float, default=0.55)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed); np.random.seed(args.seed)

    import re
    csvs = sorted(p for p in args.trace_dir.glob("*.csv") if p.name != "manifest.csv")
    print(f"[train-lown] {len(csvs)} traces, alpha={args.low_n_alpha}, "
          f"threshold={args.low_n_threshold}")

    def cell_id(p: Path) -> str:
        m = re.match(r"(rich\d+_n\d+_phi\d+)_", p.name)
        return m.group(1) if m else p.stem

    traces = []
    trace_cells = []
    for c in csvs:
        try:
            traces.append(load_trace(c))
            trace_cells.append(cell_id(c))
        except Exception:
            pass

    t0 = wall_time.time()
    X, y, cell_per_window = _build_windows_with_cells(
        traces, trace_cells,
        win_seconds=args.win_seconds,
        stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds,
    )
    print(f"[train-lown] X={X.shape}  y={y.shape}  build={wall_time.time()-t0:.1f}s")

    # Cell-aware split
    cells = sorted(set(cell_per_window.tolist()))
    rng = np.random.default_rng(args.seed)
    rng.shuffle(cells)
    n_val = max(1, int(len(cells) * args.val_frac))
    val_cells = set(cells[:n_val])
    tr_mask = np.array([c not in val_cells for c in cell_per_window])
    va_mask = ~tr_mask
    print(f"[train-lown] train_cells={len(cells)-n_val} val_cells={n_val} "
          f"tr_windows={tr_mask.sum()} va_windows={va_mask.sum()}")

    # Sample weights (only on train)
    w_all = _sample_weights(y, args.low_n_alpha, args.low_n_threshold)
    print(f"[train-lown] weight range: [{w_all.min():.2f}, {w_all.max():.2f}]  "
          f"mean low-n w = {w_all[y <= args.low_n_threshold].mean():.2f}")

    # Standardize
    x_mean = X[tr_mask].mean(axis=0).astype(np.float64)
    x_std  = X[tr_mask].std(axis=0).astype(np.float64); x_std[x_std < 1e-6] = 1.0
    Xn = ((X - x_mean) / x_std).astype(np.float32)

    if args.normalize_y:
        y_mean = float(y[tr_mask].mean()); y_std = float(y[tr_mask].std() + 1e-9)
    else:
        y_mean = 0.0; y_std = 1.0
    yn = ((y - y_mean) / y_std).astype(np.float32)

    device = torch.device(args.device)
    model = TerrainWindowMLP(n_in=N_FEATURES, hidden=args.hidden, n_out=1).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr)

    Xt = torch.from_numpy(Xn)
    yt = torch.from_numpy(yn)
    wt = torch.from_numpy(w_all.astype(np.float32))
    tr_idx = np.where(tr_mask)[0]; va_idx = np.where(va_mask)[0]
    bs = args.batch

    best = float("inf"); best_state = None
    for ep in range(1, args.epochs + 1):
        model.train()
        rng.shuffle(tr_idx)
        losses = []
        for i in range(0, len(tr_idx), bs):
            idx = tr_idx[i:i+bs]
            xb = Xt[idx].to(device)
            yb = yt[idx].to(device)
            wb = wt[idx].to(device)
            pred = model(xb).squeeze(-1)
            sqerr = (pred - yb) ** 2
            loss = (sqerr * wb).sum() / wb.sum()
            opt.zero_grad(); loss.backward(); opt.step()
            losses.append(float(loss.detach()))
        model.eval()
        with torch.no_grad():
            v_losses = []
            for i in range(0, len(va_idx), bs):
                idx = va_idx[i:i+bs]
                xb = Xt[idx].to(device)
                yb = yt[idx].to(device)
                pred = model(xb).squeeze(-1)
                v_losses.append(float(((pred - yb) ** 2).mean()))
            va = float(np.mean(v_losses)); tr = float(np.mean(losses))
        if va < best:
            best = va
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        if ep == 1 or ep % 25 == 0 or ep == args.epochs:
            print(f"  ep={ep:3d}  train_loss={tr:.5f}  val_mse={va:.5f}  "
                  f"best={best:.5f}")

    torch.save(best_state, args.out_dir / "weights.pt")
    with open(args.out_dir / "scaler.pkl", "wb") as f:
        pickle.dump(dict(x_mean=x_mean, x_std=x_std,
                          y_mean=np.array([y_mean]), y_std=np.array([y_std])), f)
    cfg = dict(win_seconds=args.win_seconds, stride_seconds=args.stride_seconds,
                warmup_seconds=args.warmup_seconds, hidden=args.hidden,
                normalize_y=bool(args.normalize_y), output_names=["n"],
                architecture="mlp_lownweighted",
                low_n_alpha=args.low_n_alpha,
                low_n_threshold=args.low_n_threshold)
    (args.out_dir / "config.json").write_text(json.dumps(cfg, indent=2))
    print(f"[train-lown] best val_mse={best:.5f}  saved → {args.out_dir}")


if __name__ == "__main__":
    main()
