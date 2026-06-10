#!/usr/bin/env python3
"""Heteroscedastic window-MLP terrain estimator.

Same 26-feature sliding-window inputs and architecture as the deployed
``terrain_window_mlp`` (see ``train_terrain_window_mlp.py``), but with a SECOND
output head that predicts the (log) variance of n. Trained with the Gaussian
negative-log-likelihood so the network learns *where* it is uncertain (e.g.
off-manifold soils, low-excitation windows). The online NN-UKF consumes this
per-sample sigma as the measurement noise R_n of its proprioceptive channel, so
the force-vs-vibration fusion is optimally weighted with no hand-set noise.

Output: weights.pt (2-output state_dict), scaler.pkl (+ heteroscedastic flag),
config.json (output_names = ["n", "log_var"]).
"""
import argparse, glob, json, pickle
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

import sys
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
from train_terrain_window_mlp import (  # noqa: E402
    load_trace, build_windows, feature_names_for, N_FEATURES,
)

ROOT = _HERE.parent
LOGV_MIN, LOGV_MAX = -6.0, 3.0   # clamp log-variance (normalised-y units)


class HetWindowMLP(nn.Module):
    """Shared trunk; heads for mean n and log-variance."""
    def __init__(self, n_in: int, hidden: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_in, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, 2),     # [mean, log_var]
        )

    def forward(self, x):
        out = self.net(x)
        mean = out[:, 0:1]
        logv = torch.clamp(out[:, 1:2], LOGV_MIN, LOGV_MAX)
        return mean, logv


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--trace-dir", nargs="+",
                   default=[str(ROOT / "data/terrain_estimator/traces_broad_v7")])
    p.add_argument("--out-dir", default=str(ROOT / "nn_models/terrain_window_mlp_het"))
    p.add_argument("--win-seconds", type=float, default=4.0)
    p.add_argument("--stride-seconds", type=float, default=0.4)
    p.add_argument("--warmup-seconds", type=float, default=1.5)
    p.add_argument("--feature-version", choices=["v1", "v2"], default="v1")
    p.add_argument("--hidden", type=int, default=64)
    p.add_argument("--epochs", type=int, default=160)
    p.add_argument("--batch", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val-frac", type=float, default=0.15)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-traces", type=int, default=0, help="0 = all")
    args = p.parse_args()

    csvs = []
    for d in args.trace_dir:
        csvs += sorted(glob.glob(str(Path(d) / "*.csv")))
    if args.max_traces:
        csvs = csvs[:args.max_traces]
    print(f"[het] {len(csvs)} traces from {args.trace_dir}")
    traces = []
    for c in csvs:
        try:
            traces.append(load_trace(Path(c)))
        except Exception:
            pass
    X, y, terr_per, _ = build_windows(
        traces, win_seconds=args.win_seconds, stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds, feature_version=args.feature_version)
    print(f"[het] {X.shape[0]} windows, feature_dim={X.shape[1]}")

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(X.shape[0]); X = X[perm]; y = y[perm]
    terr_per = [terr_per[i] for i in perm]
    nval = int(round(args.val_frac * X.shape[0]))
    Xv, yv, terr_v = X[:nval], y[:nval], terr_per[:nval]
    Xt, yt = X[nval:], y[nval:]
    x_mean = Xt.mean(0); x_std = Xt.std(0) + 1e-6
    y_mean = float(yt.mean()); y_std = float(yt.std() + 1e-6)
    Xts = (Xt - x_mean) / x_std; Xvs = (Xv - x_mean) / x_std
    ytn = (yt - y_mean) / y_std; yvn = (yv - y_mean) / y_std

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = HetWindowMLP(N_FEATURES, hidden=args.hidden).to(dev)
    opt = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    XT = torch.tensor(Xts, dtype=torch.float32, device=dev)
    yT = torch.tensor(ytn, dtype=torch.float32, device=dev).unsqueeze(1)
    XV = torch.tensor(Xvs, dtype=torch.float32, device=dev)
    yV = torch.tensor(yvn, dtype=torch.float32, device=dev).unsqueeze(1)

    def nll(mean, logv, target):
        return (0.5 * torch.exp(-logv) * (mean - target) ** 2 + 0.5 * logv).mean()

    best = float("inf"); best_state = None; n_tr = XT.shape[0]
    for ep in range(1, args.epochs + 1):
        model.train(); idx = torch.randperm(n_tr, device=dev)
        for s in range(0, n_tr, args.batch):
            sel = idx[s:s + args.batch]
            m, lv = model(XT[sel]); loss = nll(m, lv, yT[sel])
            opt.zero_grad(); loss.backward(); opt.step()
        model.eval()
        with torch.no_grad():
            mv, lvv = model(XV)
            val_mse = float(((mv - yV) ** 2).mean())   # track prediction accuracy
            val_nll = float(nll(mv, lvv, yV))
        if val_mse < best:
            best = val_mse
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        if ep % 20 == 0 or ep == 1:
            print(f"  ep={ep:3d} val_mse={val_mse:.5f} (rmse_n={ (val_mse**0.5)*y_std:.4f}) val_nll={val_nll:.4f}")
    model.load_state_dict(best_state)
    print(f"[het] best val rmse_n = {(best**0.5)*y_std:.4f}")

    # calibration: does predicted sigma track actual error? (per terrain)
    model.eval()
    with torch.no_grad():
        mv, lvv = model(XV)
    pred_n = mv.cpu().numpy().ravel() * y_std + y_mean
    sig_n = np.exp(0.5 * lvv.cpu().numpy().ravel()) * y_std
    err = np.abs(pred_n - yv)
    print("[het] per-terrain: rmse_n | mean predicted sigma  (should track)")
    for ter in sorted(set(terr_v)):
        mask = np.array([t == ter for t in terr_v])
        if mask.sum() == 0:
            continue
        print(f"  {ter:>5s} n={int(mask.sum()):4d}  rmse={np.sqrt((err[mask]**2).mean()):.4f}"
              f"  pred_sigma={sig_n[mask].mean():.4f}")
    print(f"[het] overall corr(|err|, pred_sigma) = {np.corrcoef(err, sig_n)[0,1]:.3f}")

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), out / "weights.pt")
    with open(out / "scaler.pkl", "wb") as f:
        pickle.dump({"x_mean": x_mean, "x_std": x_std,
                     "feature_names": feature_names_for(args.feature_version),
                     "feature_version": args.feature_version,
                     "win_seconds": args.win_seconds, "hidden": args.hidden,
                     "heteroscedastic": True,
                     "y_mean": np.array([y_mean]), "y_std": np.array([y_std])}, f)
    with open(out / "config.json", "w") as f:
        json.dump({"n_features": int(N_FEATURES), "output_names": ["n", "log_var"],
                   "feature_version": args.feature_version, "hidden": int(args.hidden),
                   "win_seconds": float(args.win_seconds),
                   "stride_seconds": float(args.stride_seconds),
                   "warmup_seconds": float(args.warmup_seconds),
                   "heteroscedastic": True, "logv_clamp": [LOGV_MIN, LOGV_MAX],
                   "best_val_rmse_n": float((best**0.5)*y_std)}, f, indent=2)
    print(f"[het] saved to {out}")


if __name__ == "__main__":
    main()
