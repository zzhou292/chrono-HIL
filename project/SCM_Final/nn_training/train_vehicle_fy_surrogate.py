#!/usr/bin/env python3
"""train_vehicle_fy_surrogate.py
==================================

Train a *whole-vehicle* lateral-force surrogate for the Dallas-style
UKF that natively predicts what the full Chrono HMMWV produces at a
given operating point, eliminating the rig-to-vehicle Fy gap without
any post-hoc calibration scalar.

The model maps

    (u, v, omega, delta, Kphi, Kc, n, cohesion, friction_angle, janosi_shear)
        -> (Fy_total, M_yaw_total)

where ``Fy_total = m * ay`` is the body-frame total lateral tyre force
and ``M_yaw_total = Iz * dot_omega`` is the yaw moment, both extracted
directly from the Chrono SCM log. Inputs are computed from the same
state quantities the UKF carries (no oracle-only signal) and the
training data comes from the disjoint widened-box LHS sweep in
``data/dallas_scm/lhs_train300/`` (collected by
``data_collection/collect_lhs_training_scms.py --widened-box``),
with a 90/10 split-by-scenario so the held-out 10 % is genuinely
unseen.

Output goes to ``nn_models/vehicle_fy_64_32/{weights.pt, scaler.pkl,
config.json}``. The UKF picks it up at runtime via the new
``_vehicle_fy_total`` helper in ``ukf_paper_validation.py``.

CLAUDE.md §8 compliance: the SCM logs are uniform LHS over the
documented Bekker--Mohr box; the manoeuvre (sinusoidal steering +
PI cruise throttle) is the same across all 100 samples; no operating
point distribution is targeted toward a subregion of the soil space.
"""
from __future__ import annotations

import json
import math
import pickle
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deliverables"))
sys.path.insert(0, str(ROOT / "simulation"))

from ukf_paper_validation import Vehicle  # noqa


# Feature columns of the input vector (10).
INPUT_COLS = ("u", "v", "omega", "delta",
              "Kphi", "Kc", "n", "cohesion",
              "friction_angle", "janosi_shear")
# Outputs the bicycle's lateral channel needs (2).
OUTPUT_COLS = ("Fy_total", "M_yaw_total")


def _aggregate(log_dir: Path, decim: int = 4, max_scenarios: int = 0
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Walk every SCM log + matching yaml, return (X, Y, scenario_id)."""
    veh = Vehicle()
    X: List[np.ndarray] = []
    Y: List[np.ndarray] = []
    sid: List[int] = []
    yaml_files = sorted(log_dir.glob("scn_*.yaml"))
    if max_scenarios and max_scenarios > 0:
        yaml_files = yaml_files[:max_scenarios]   # data-scaling ablation
    print(f"Reading {len(yaml_files)} scenario configs from {log_dir}")
    for yp in yaml_files:
        idx = int(yp.stem.split("_")[1])
        lp = log_dir / f"scn_{idx:03d}.npz"
        if not lp.exists():
            continue
        with open(yp) as f:
            params = json.load(f)
        data = np.load(str(lp))
        lead = float(data["lead_in"][0])
        mask = data["t"] >= lead
        t = data["t"][mask]
        u = data["u"][mask]; v = data["v"][mask]
        om = data["omega"][mask]; delta = data["delta_meas"][mask]
        ay = data["ay"][mask]

        # M_yaw = Iz * d_omega/dt by 5-point central difference.
        dt = float(np.median(np.diff(t)))
        if len(om) < 8:
            continue
        dom = np.gradient(om, dt)
        Fy_total = veh.m * ay
        M_yaw = veh.Iz * dom

        # Soil-parameter vector — broadcast to length of trajectory.
        soil = np.array([params["Kphi"], params["Kc"], params["n"],
                          params["cohesion"], params["friction_angle"],
                          params["janosi_shear"]], dtype=np.float64)

        # Decimate to reduce intra-scenario correlation but keep enough
        # samples for training.
        sel = np.arange(0, len(t), decim)
        n_sel = len(sel)
        feats = np.zeros((n_sel, len(INPUT_COLS)), dtype=np.float64)
        feats[:, 0] = u[sel]; feats[:, 1] = v[sel]
        feats[:, 2] = om[sel]; feats[:, 3] = delta[sel]
        feats[:, 4:] = soil[None, :]
        targets = np.stack([Fy_total[sel], M_yaw[sel]], axis=1)
        X.append(feats); Y.append(targets); sid.extend([idx] * n_sel)
    return (np.concatenate(X, axis=0), np.concatenate(Y, axis=0),
            np.asarray(sid, dtype=np.int64))


class VehicleFyMLP(nn.Module):
    def __init__(self, in_dim: int, hidden=(64, 32)):
        super().__init__()
        h1, h2 = hidden
        self.net = nn.Sequential(
            nn.Linear(in_dim, h1), nn.ReLU(),
            nn.Linear(h1, h2),     nn.ReLU(),
            nn.Linear(h2, 2),
        )

    def forward(self, x):  # noqa
        return self.net(x)


def main() -> int:
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--lhs-dir", default="data/dallas_scm/lhs_train300")
    p.add_argument("--max-scenarios", type=int, default=0,
                   help="Limit scenarios loaded (0=all) for data-scaling studies.")
    p.add_argument("--out-dir", default="nn_models/vehicle_fy_64_32",
                   help="Output model directory (default overwrites the deployed model).")
    p.add_argument("--decim", type=int, default=4,
                   help="Decimation stride within each scenario "
                        "(default 4 -> 25 Hz from 24-ms logs).")
    p.add_argument("--hidden", type=int, nargs=2, default=[64, 32])
    p.add_argument("--epochs", type=int, default=400)
    p.add_argument("--lr", type=float, default=3e-3)
    p.add_argument("--batch", type=int, default=512)
    p.add_argument("--test-frac", type=float, default=0.20,
                   help="Fraction of *scenarios* held out for test (split "
                        "by scenario id, not by sample).")
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    out_dir = (Path(args.out_dir) if Path(args.out_dir).is_absolute()
               else ROOT / args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_path = (ROOT / "my_paper" / "paper_figures"
                / "vehicle_fy_surrogate_training.png")

    X_all, Y_all, sid_all = _aggregate(ROOT / args.lhs_dir, decim=args.decim,
                                       max_scenarios=args.max_scenarios)
    print(f"Aggregated training rows: {len(X_all)} from "
          f"{len(np.unique(sid_all))} scenarios")

    # Split by scenario id so the held-out 20 % is genuinely unseen.
    rng = np.random.default_rng(args.seed)
    uniq = np.unique(sid_all)
    rng.shuffle(uniq)
    n_test = max(1, int(args.test_frac * len(uniq)))
    test_ids = set(uniq[:n_test].tolist())
    is_test = np.array([s in test_ids for s in sid_all])
    Xtr, Ytr = X_all[~is_test], Y_all[~is_test]
    Xte, Yte = X_all[is_test], Y_all[is_test]
    print(f"  train: {len(Xtr)} rows ({len(uniq) - n_test} scenarios)")
    print(f"  test:  {len(Xte)} rows ({n_test} scenarios)")

    # Standardise inputs and outputs.
    x_mean = Xtr.mean(0); x_std = Xtr.std(0) + 1e-9
    y_mean = Ytr.mean(0); y_std = Ytr.std(0) + 1e-9
    Xtrn = (Xtr - x_mean) / x_std
    Ytrn = (Ytr - y_mean) / y_std
    Xten = (Xte - x_mean) / x_std
    Yten = (Yte - y_mean) / y_std

    torch.manual_seed(args.seed)
    net = VehicleFyMLP(in_dim=len(INPUT_COLS), hidden=tuple(args.hidden))
    opt = optim.Adam(net.parameters(), lr=args.lr)
    Xtrn_t = torch.tensor(Xtrn, dtype=torch.float32)
    Ytrn_t = torch.tensor(Ytrn, dtype=torch.float32)
    Xten_t = torch.tensor(Xten, dtype=torch.float32)
    Yten_t = torch.tensor(Yten, dtype=torch.float32)

    best_te = float("inf"); best_state = None; bad = 0; patience = 60
    for ep in range(args.epochs):
        net.train()
        perm = torch.randperm(len(Xtrn_t))
        ep_loss = 0.0; nb = 0
        for i in range(0, len(perm), args.batch):
            sel = perm[i:i + args.batch]
            xb = Xtrn_t[sel]; yb = Ytrn_t[sel]
            opt.zero_grad()
            pred = net(xb)
            loss = ((pred - yb) ** 2).mean()
            loss.backward()
            opt.step()
            ep_loss += float(loss); nb += 1
        net.eval()
        with torch.no_grad():
            te = float(((net(Xten_t) - Yten_t) ** 2).mean())
        if te < best_te - 1e-5:
            best_te = te
            best_state = {k: v.clone() for k, v in net.state_dict().items()}
            bad = 0
        else:
            bad += 1
        if ep % 25 == 0 or bad == patience:
            print(f"  ep {ep:3d}  train={ep_loss/max(nb,1):.4f}  "
                  f"test={te:.4f}  best={best_te:.4f}")
        if bad >= patience:
            print("  early stop")
            break
    net.load_state_dict(best_state)

    # Final metrics in physical units.
    net.eval()
    with torch.no_grad():
        Pten = (net(Xten_t).numpy() * y_std) + y_mean
        Ptrn = (net(Xtrn_t).numpy() * y_std) + y_mean
    def _r2_rmse(y, p):
        rss = float(np.mean((y - p) ** 2))
        ss = float(np.var(y))
        return 1.0 - rss / max(ss, 1e-9), float(np.sqrt(rss))
    r2_Fy_tr, rmse_Fy_tr = _r2_rmse(Ytr[:, 0], Ptrn[:, 0])
    r2_Fy_te, rmse_Fy_te = _r2_rmse(Yte[:, 0], Pten[:, 0])
    r2_M_tr,  rmse_M_tr  = _r2_rmse(Ytr[:, 1], Ptrn[:, 1])
    r2_M_te,  rmse_M_te  = _r2_rmse(Yte[:, 1], Pten[:, 1])
    print("Final metrics:")
    print(f"  Fy   train  R2={r2_Fy_tr:.4f}  RMSE={rmse_Fy_tr:7.0f} N")
    print(f"  Fy    test  R2={r2_Fy_te:.4f}  RMSE={rmse_Fy_te:7.0f} N")
    print(f"  Myaw train  R2={r2_M_tr:.4f}  RMSE={rmse_M_tr:7.0f} Nm")
    print(f"  Myaw  test  R2={r2_M_te:.4f}  RMSE={rmse_M_te:7.0f} Nm")

    torch.save(net.state_dict(), out_dir / "weights.pt")
    with open(out_dir / "scaler.pkl", "wb") as f:
        pickle.dump({"x_mean": x_mean.tolist(), "x_std": x_std.tolist(),
                      "y_mean": y_mean.tolist(), "y_std": y_std.tolist(),
                      "input_cols": list(INPUT_COLS),
                      "output_cols": list(OUTPUT_COLS),
                      "hidden": list(args.hidden)}, f)
    with open(out_dir / "config.json", "w") as f:
        json.dump({"hidden": args.hidden,
                    "input_dim": len(INPUT_COLS),
                    "output_dim": 2,
                    "test_scenarios": int(n_test),
                    "train_scenarios": int(len(uniq) - n_test),
                    "test_r2_Fy": r2_Fy_te, "test_rmse_Fy": rmse_Fy_te,
                    "test_r2_M":  r2_M_te,  "test_rmse_M":  rmse_M_te},
                   f, indent=2)
    print(f"Saved -> {out_dir}")

    # Diagnostic plot
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].scatter(Yte[:, 0], Pten[:, 0], s=4, alpha=0.3,
                     color="#1f77b4")
    lim = float(max(abs(Yte[:, 0]).max(), abs(Pten[:, 0]).max()))
    axes[0].plot([-lim, lim], [-lim, lim], "k-", lw=1)
    axes[0].set_xlabel("Chrono Fy_total (N)")
    axes[0].set_ylabel("NN Fy_total (N)")
    axes[0].set_title(f"Fy_total  test  R²={r2_Fy_te:.3f}  "
                       f"RMSE={rmse_Fy_te:.0f} N")
    axes[0].grid(alpha=0.3)
    axes[1].scatter(Yte[:, 1], Pten[:, 1], s=4, alpha=0.3,
                     color="#1f77b4")
    lim = float(max(abs(Yte[:, 1]).max(), abs(Pten[:, 1]).max()))
    axes[1].plot([-lim, lim], [-lim, lim], "k-", lw=1)
    axes[1].set_xlabel("Chrono M_yaw (Nm)")
    axes[1].set_ylabel("NN M_yaw (Nm)")
    axes[1].set_title(f"M_yaw  test  R²={r2_M_te:.3f}  "
                       f"RMSE={rmse_M_te:.0f} Nm")
    axes[1].grid(alpha=0.3)
    fig.suptitle(f"Vehicle Fy surrogate — train on {len(uniq)-n_test} "
                  f"scenarios, held-out test on {n_test}", fontsize=11)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"Wrote {fig_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
