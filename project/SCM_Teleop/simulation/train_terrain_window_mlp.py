#!/usr/bin/env python3
"""Train a sliding-window MLP that regresses Bekker n from vehicle dynamics.

Reads CSV traces produced by ``collect_terrain_traces.py`` and trains a small
MLP whose input is a fixed-dimensional set of statistics computed over a
sliding window of vehicle dynamics features (no oracle tire forces).

The features mirror exactly what ``LearnedTerrainEstimator`` will compute at
inference time, so the same numpy code is shared between the two.
"""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


# ────────────────────────────────────────────────────────────────────────
# Feature extraction (shared with LearnedTerrainEstimator at inference time)
# ────────────────────────────────────────────────────────────────────────

WHEEL_RADIUS = 0.4625   # HMMWV wheel radius (m), see param_consistency.py

# Per-window feature names (order matters — mirrored in inference code).
FEATURE_NAMES = [
    "u_mean", "u_std", "u_max",
    "v_std",
    "omega_mean", "omega_std", "omega_p95",
    "ay_std", "ay_p95",
    "ax_mean", "ax_std",
    "abs_steer_mean", "abs_steer_p95",
    "wheel_slip_mean", "wheel_slip_std", "wheel_slip_p95",
    # dimensionless coupling channels — make features speed-invariant:
    "lat_grip",         # max(|ay|) / max(u^2 * |delta|, eps)
    "yaw_gain",         # max(|omega|) / (u_mean * max(|delta|) + eps)
    "long_drag",        # 1 - u_mean / (10 * throttle_mean + eps)
    "long_slip_mean",   # mean((wheel*R - u) / max(u, 1)) on driven axle
    "throttle_mean",
]


def compute_window_features(window: np.ndarray, throttle_window: np.ndarray) -> np.ndarray:
    """Extract feature vector from a window of vehicle dynamics samples.

    ``window`` columns (in order):
        u, v, omega, ax, ay, w_fl, w_fr, w_rl, w_rr, steering_angle
    ``throttle_window`` is a 1-D array of commanded throttles aligned with the
    same time index.
    """
    eps = 1e-3
    u  = window[:, 0]
    v  = window[:, 1]
    om = window[:, 2]
    ax = window[:, 3]
    ay = window[:, 4]
    w_fl = window[:, 5]
    w_fr = window[:, 6]
    w_rl = window[:, 7]
    w_rr = window[:, 8]
    delta = window[:, 9]

    abs_om = np.abs(om)
    abs_ay = np.abs(ay)
    abs_d  = np.abs(delta)

    u_mean = float(np.mean(u))
    u_std  = float(np.std(u))
    u_max  = float(np.max(u))

    v_std  = float(np.std(v))

    om_mean = float(np.mean(abs_om))
    om_std  = float(np.std(om))
    om_p95  = float(np.percentile(abs_om, 95))

    ay_std  = float(np.std(ay))
    ay_p95  = float(np.percentile(abs_ay, 95))

    ax_mean = float(np.mean(ax))
    ax_std  = float(np.std(ax))

    abs_steer_mean = float(np.mean(abs_d))
    abs_steer_p95  = float(np.percentile(abs_d, 95))

    # Wheel slip = (mean wheel speed * R - u) / max(u, eps).  Positive when
    # wheels spin faster than ground (driven axle slipping).
    w_avg = 0.5 * (w_rl + w_rr)         # rear (driven) axle
    u_safe = np.maximum(np.abs(u), 0.5)
    slip = (w_avg * WHEEL_RADIUS - u) / u_safe
    abs_slip = np.abs(slip)
    slip_mean = float(np.mean(abs_slip))
    slip_std  = float(np.std(slip))
    slip_p95  = float(np.percentile(abs_slip, 95))

    delta_max = float(np.max(abs_d))
    u_sq_d = u_max ** 2 * max(delta_max, eps)
    lat_grip = float(np.max(abs_ay)) / max(u_sq_d, eps)

    yaw_gain = float(np.max(abs_om)) / (u_mean * max(delta_max, eps) + eps)

    thr_mean = float(np.mean(throttle_window))
    long_drag = 1.0 - u_mean / max(10.0 * thr_mean + eps, 1.0)
    long_slip_mean = float(np.mean(slip))

    return np.array([
        u_mean, u_std, u_max,
        v_std,
        om_mean, om_std, om_p95,
        ay_std, ay_p95,
        ax_mean, ax_std,
        abs_steer_mean, abs_steer_p95,
        slip_mean, slip_std, slip_p95,
        lat_grip,
        yaw_gain,
        long_drag,
        long_slip_mean,
        thr_mean,
    ], dtype=np.float64)


N_FEATURES = len(FEATURE_NAMES)


# ────────────────────────────────────────────────────────────────────────
# Dataset construction
# ────────────────────────────────────────────────────────────────────────

CSV_COLS_USED = [
    "u", "v", "omega", "ax", "ay",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]


def load_trace(csv_path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, str]:
    """Returns (t, dyn_window_cols, throttle_col, n_true, terrain_name)."""
    import csv as _csv
    with csv_path.open() as f:
        reader = _csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty trace: {csv_path}")
    t = np.array([float(r["t"]) for r in rows], dtype=np.float64)
    n_true = float(rows[0]["n_true"])
    terrain = rows[0]["terrain"]
    dyn = np.stack([
        np.array([float(r[c]) for r in rows], dtype=np.float64)
        for c in CSV_COLS_USED
    ], axis=1)   # shape (T, 10)
    thr = np.array([float(r["throttle_cmd"]) for r in rows], dtype=np.float64)
    return t, dyn, thr, n_true, terrain


def build_windows(traces: List[Tuple[np.ndarray, np.ndarray, np.ndarray, float, str]],
                  *, win_seconds: float = 4.0,
                  stride_seconds: float = 0.4,
                  warmup_seconds: float = 1.5,
                  ) -> Tuple[np.ndarray, np.ndarray, List[str], List[str]]:
    """Build (X, y, terrain_per_sample, source_csv_per_sample)."""
    X: List[np.ndarray] = []
    y: List[float] = []
    terr: List[str] = []
    src: List[str] = []

    for t, dyn, thr, n_true, terrain in traces:
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
            thr_w  = thr[end - win_n: end]
            if not np.all(np.isfinite(window)):
                end += stride_n
                continue
            feat = compute_window_features(window, thr_w)
            X.append(feat)
            y.append(n_true)
            terr.append(terrain)
            src.append("")
            end += stride_n

    return (np.stack(X, axis=0),
            np.asarray(y, dtype=np.float64),
            terr, src)


# ────────────────────────────────────────────────────────────────────────
# Model
# ────────────────────────────────────────────────────────────────────────

class TerrainWindowMLP(nn.Module):
    def __init__(self, n_in: int, hidden: int = 64, n_out: int = 1):
        super().__init__()
        self.n_out = int(n_out)
        self.net = nn.Sequential(
            nn.Linear(n_in, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, self.n_out),
        )

    def forward(self, x):
        y = self.net(x)
        return y.squeeze(-1) if self.n_out == 1 else y


# ────────────────────────────────────────────────────────────────────────
# Training loop
# ────────────────────────────────────────────────────────────────────────

def train(args):
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    trace_dirs = [Path(d) for d in args.trace_dir]

    csvs: list[Path] = []
    per_dir_counts: list[int] = []
    for d in trace_dirs:
        cs = sorted(d.glob("*.csv"))
        per_dir_counts.append(len(cs))
        csvs.extend(cs)
    csvs = sorted(csvs)
    for d, c in zip(trace_dirs, per_dir_counts):
        print(f"[train]   {c:4d} traces from {d}")
    if args.holdout_substr:
        held = [p for p in csvs if args.holdout_substr in p.name]
        train_csvs = [p for p in csvs if args.holdout_substr not in p.name]
        print(f"[train] found {len(csvs)} traces — train={len(train_csvs)} "
              f"held-out={len(held)} (substr='{args.holdout_substr}')")
    else:
        train_csvs = csvs
        held = []
        print(f"[train] found {len(csvs)} traces total")
    traces = [load_trace(p) for p in train_csvs]
    held_traces = [load_trace(p) for p in held]

    X, y, terr_per, _ = build_windows(
        traces, win_seconds=args.win_seconds,
        stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds,
    )
    print(f"[train] {X.shape[0]} windows total, feature_dim={X.shape[1]}")

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(X.shape[0])
    X = X[perm]
    y = y[perm]
    terr_per = [terr_per[i] for i in perm]

    n_val = int(round(args.val_frac * X.shape[0]))
    X_val, y_val = X[:n_val], y[:n_val]
    terr_val = terr_per[:n_val]
    X_tr,  y_tr  = X[n_val:], y[n_val:]

    x_mean = X_tr.mean(axis=0)
    x_std  = X_tr.std(axis=0) + 1e-6

    Xs_tr  = (X_tr  - x_mean) / x_std
    Xs_val = (X_val - x_mean) / x_std

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[train] device={device}")
    model = TerrainWindowMLP(N_FEATURES, hidden=args.hidden).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)

    Xt_tr  = torch.tensor(Xs_tr,  dtype=torch.float32, device=device)
    yt_tr  = torch.tensor(y_tr,   dtype=torch.float32, device=device)
    Xt_val = torch.tensor(Xs_val, dtype=torch.float32, device=device)
    yt_val = torch.tensor(y_val,  dtype=torch.float32, device=device)

    best_val = float("inf")
    best_state = None
    bs = args.batch
    n_tr = Xt_tr.shape[0]
    for ep in range(1, args.epochs + 1):
        model.train()
        idx = torch.randperm(n_tr, device=device)
        losses = []
        for s in range(0, n_tr, bs):
            sel = idx[s: s + bs]
            yp = model(Xt_tr[sel])
            loss = ((yp - yt_tr[sel]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(loss.item())
        model.eval()
        with torch.no_grad():
            yp_val = model(Xt_val)
            v_loss = ((yp_val - yt_val) ** 2).mean().item()
        if v_loss < best_val:
            best_val = v_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        if ep % 10 == 0 or ep == 1:
            print(f"  ep={ep:3d}  train_mse={np.mean(losses):.5f}  val_mse={v_loss:.5f}")

    print(f"[train] best val_mse={best_val:.5f}  rmse={best_val**0.5:.4f}")

    if best_state is not None:
        model.load_state_dict(best_state)

    model.eval()
    with torch.no_grad():
        yp_val = model(Xt_val).cpu().numpy()
    by_terr: dict[str, list] = {}
    for ter, true, pred in zip(terr_val, y_val, yp_val):
        by_terr.setdefault(ter, []).append((true, pred))
    print("[train] per-terrain validation RMSE:")
    for ter, pairs in sorted(by_terr.items()):
        true_arr = np.asarray([p[0] for p in pairs])
        pred_arr = np.asarray([p[1] for p in pairs])
        rmse = float(np.sqrt(np.mean((pred_arr - true_arr) ** 2)))
        bias = float(np.mean(pred_arr - true_arr))
        print(f"  {ter:>5s}  n={len(pairs):4d}  rmse={rmse:.4f}  bias={bias:+.4f}  "
              f"pred_mean={pred_arr.mean():.3f}±{pred_arr.std():.3f}")

    bin_edges = np.array([0.40, 0.55, 0.65, 0.75, 0.90, 1.05, 1.25])
    print("[train] per-n-bin validation RMSE:")
    for i in range(len(bin_edges) - 1):
        lo, hi = float(bin_edges[i]), float(bin_edges[i + 1])
        mask = (y_val >= lo) & (y_val < hi)
        if mask.sum() == 0:
            continue
        true_b = y_val[mask]
        pred_b = yp_val[mask]
        rmse = float(np.sqrt(np.mean((pred_b - true_b) ** 2)))
        bias = float(np.mean(pred_b - true_b))
        print(f"  n∈[{lo:.2f},{hi:.2f})  k={int(mask.sum()):4d}  "
              f"rmse={rmse:.4f}  bias={bias:+.4f}")

    weights_path = out_dir / "weights.pt"
    torch.save(model.state_dict(), weights_path)
    with open(out_dir / "scaler.pkl", "wb") as f:
        pickle.dump({"x_mean": x_mean, "x_std": x_std,
                     "feature_names": FEATURE_NAMES,
                     "win_seconds": args.win_seconds,
                     "hidden": args.hidden}, f)
    with open(out_dir / "config.json", "w") as f:
        json.dump({
            "n_features": int(N_FEATURES),
            "output_names": ["n"],
            "feature_names": FEATURE_NAMES,
            "hidden": int(args.hidden),
            "win_seconds": float(args.win_seconds),
            "stride_seconds": float(args.stride_seconds),
            "warmup_seconds": float(args.warmup_seconds),
            "best_val_mse": float(best_val),
        }, f, indent=2)
    print(f"[train] saved to {out_dir}")

    if held_traces:
        Xh, yh, terr_h, _ = build_windows(
            held_traces, win_seconds=args.win_seconds,
            stride_seconds=args.stride_seconds,
            warmup_seconds=args.warmup_seconds,
        )
        Xhs = (Xh - x_mean) / x_std
        with torch.no_grad():
            yph = model(torch.tensor(Xhs, dtype=torch.float32, device=device)).cpu().numpy()
        print(f"\n[train] HONEST held-out evaluation ({Xh.shape[0]} windows from "
              f"{len(held_traces)} unseen traces):")
        by_terr_h: dict[str, list] = {}
        for ter, true, pred in zip(terr_h, yh, yph):
            by_terr_h.setdefault(ter, []).append((true, pred))
        for ter, pairs in sorted(by_terr_h.items()):
            true_arr = np.asarray([p[0] for p in pairs])
            pred_arr = np.asarray([p[1] for p in pairs])
            rmse = float(np.sqrt(np.mean((pred_arr - true_arr) ** 2)))
            bias = float(np.mean(pred_arr - true_arr))
            print(f"  {ter:>5s}  n={len(pairs):4d}  rmse={rmse:.4f}  "
                  f"bias={bias:+.4f}  pred_mean={pred_arr.mean():.3f}±"
                  f"{pred_arr.std():.3f}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--trace-dir", nargs="+", default=[str(
        Path(__file__).parent.parent / "data" / "terrain_traces")],
        help="One or more directories of CSV traces to load")
    p.add_argument("--out-dir", default=str(
        Path(__file__).parent.parent / "nn_models" / "terrain_window_mlp"))
    p.add_argument("--win-seconds", type=float, default=4.0)
    p.add_argument("--stride-seconds", type=float, default=0.3)
    p.add_argument("--warmup-seconds", type=float, default=1.5)
    p.add_argument("--epochs", type=int, default=120)
    p.add_argument("--batch", type=int, default=128)
    p.add_argument("--hidden", type=int, default=64)
    p.add_argument("--lr", type=float, default=2e-3)
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--holdout-substr", default=None,
                   help="If set, traces whose filename contains this string "
                        "are held out (no windows from them used in train/val).")
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
