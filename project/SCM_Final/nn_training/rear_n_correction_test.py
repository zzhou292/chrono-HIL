#!/usr/bin/env python3
"""Can a per-axle soil/force correction help the rear axle?

Two cheap, deployment-relevant levers that need NO retrain and NO new feature:
  (1) rear-n scale: feed the rear surrogate a scaled sinkage exponent n
      (multi-pass intuition: rear runs on front-compacted soil), and
  (2) rear-Fy scale: a constant gain on the predicted rear lateral force.

A scalar correction can only remove a systematic BIAS, not variance. So we
first decompose the rear/front Fy error into bias (mean signed error) vs
scatter (std), then sweep a rear-n inference scale and a rear-Fy gain and
report the best achievable rear-Fy RMSE. If the error is bias-dominated, a
correction helps (and we wire it like the existing rear_alpha_scale); if it is
scatter-dominated, no scalar can move it and we say so.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "lhs" / "training_data_rich_tire_frame.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
# exact deployed per-axle layout (no axle_id):
FEATS = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
         "d_slip_ratio", "d_slip_angle", "d_velocity"] + TERRAIN
N_IDX = FEATS.index("bekker_n")
RATES = ["slip_ratio", "slip_angle", "velocity"]
N_SCEN = 1500
SEEDS = (0, 1)
dev = "cuda" if torch.cuda.is_available() else "cpu"
rng0 = np.random.default_rng(0)


def load():
    df = pd.read_csv(DATA)
    scen = df["scenario_id"].unique()
    keep = rng0.choice(scen, min(N_SCEN, len(scen)), replace=False)
    df = df[df["scenario_id"].isin(keep)].copy()
    df = df.sort_values(["scenario_id", "axle_id", "timestep"])
    g = df.groupby(["scenario_id", "axle_id"])
    for c in RATES:
        df["d_" + c] = g[c].diff().fillna(0.0)
    df = df[df["slip_ratio"].between(-1.2, 1.2) & df["slip_angle"].between(-0.7, 0.7)
            & df["velocity"].between(0.25, 20) & df["vertical_load"].between(1000, 10000)
            & df["Fx"].between(-5e4, 5e4) & df["Fy"].between(-5e4, 5e4)]
    return df.reset_index(drop=True)


class MLP(nn.Module):
    def __init__(self, nin):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(nin, 64), nn.Tanh(),
                                 nn.Linear(64, 32), nn.Tanh(), nn.Linear(32, 2))
    def forward(self, x): return self.net(x)


N_SCALES = [0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 1.8]
FY_SCALES = [0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15]


def run(seed):
    torch.manual_seed(seed)
    df = load()
    scen = df["scenario_id"].unique()
    rng = np.random.default_rng(50 + seed)
    te = set(rng.choice(scen, max(1, len(scen) // 5), replace=False))
    tr = df[~df["scenario_id"].isin(te)]; te_df = df[df["scenario_id"].isin(te)]
    Xtr = tr[FEATS].to_numpy(np.float32); ytr = tr[["Fx", "Fy"]].to_numpy(np.float32)
    mx, sx = Xtr.mean(0), Xtr.std(0) + 1e-6
    my, sy = ytr.mean(0), ytr.std(0) + 1e-6
    m = MLP(len(FEATS)).to(dev); opt = torch.optim.Adam(m.parameters(), lr=2e-3)
    Xt = torch.tensor((Xtr - mx) / sx, device=dev); Yt = torch.tensor((ytr - my) / sy, device=dev)
    n = len(Xt); bs = 8192
    for _ in range(120):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            opt.zero_grad(); loss = ((m(Xt[idx]) - Yt[idx]) ** 2).mean(); loss.backward(); opt.step()
    m.eval()

    ax = te_df["axle_id"].to_numpy()
    Xte = te_df[FEATS].to_numpy(np.float32); yte = te_df[["Fx", "Fy"]].to_numpy(np.float32)

    def predict(Xmat):
        with torch.no_grad():
            return m(torch.tensor((Xmat - mx) / sx, device=dev)).cpu().numpy() * sy + my

    base = predict(Xte)
    out = {"seed": seed}
    for axval, axn in [(0, "front"), (1, "rear")]:
        msk = ax == axval
        err = base[msk, 1] - yte[msk, 1]
        out[f"{axn}_bias"] = float(err.mean())
        out[f"{axn}_scatter"] = float(err.std())
        out[f"{axn}_rmse"] = float(np.sqrt((err ** 2).mean()))
    # rear-n inference scale sweep
    rmsk = ax == 1
    out["n_sweep"] = {}
    for s in N_SCALES:
        Xs = Xte.copy(); Xs[rmsk, N_IDX] = Xs[rmsk, N_IDX] * s
        p = predict(Xs)
        out["n_sweep"][s] = float(np.sqrt(((p[rmsk, 1] - yte[rmsk, 1]) ** 2).mean()))
    # rear-Fy constant-gain sweep (on the baseline prediction)
    out["fy_sweep"] = {}
    for s in FY_SCALES:
        pred_rear_fy = base[rmsk, 1] * s
        out["fy_sweep"][s] = float(np.sqrt(((pred_rear_fy - yte[rmsk, 1]) ** 2).mean()))
    return out


def main():
    print(f"device={dev}; rear-axle correction diagnostic")
    res = [run(s) for s in SEEDS]
    def avg(k): return float(np.mean([r[k] for r in res]))
    print("\nFy error decomposition (N, mean over seeds):")
    print(f"{'axle':6s}{'bias':>10s}{'scatter':>10s}{'rmse':>10s}{'bias/rmse':>11s}")
    for axn in ["front", "rear"]:
        b, sc, rm = avg(f"{axn}_bias"), avg(f"{axn}_scatter"), avg(f"{axn}_rmse")
        print(f"{axn:6s}{b:10.1f}{sc:10.1f}{rm:10.1f}{abs(b)/rm:11.2f}")
    base_rear = avg("rear_rmse")
    print(f"\nrear-Fy RMSE vs rear-n inference scale (baseline n-scale=1.0 -> {base_rear:.1f} N):")
    for s in N_SCALES:
        v = float(np.mean([r["n_sweep"][s] for r in res]))
        print(f"  n*{s:<4}: {v:8.1f} N  ({100*(v-base_rear)/base_rear:+.0f}%)")
    print(f"\nrear-Fy RMSE vs constant rear-Fy gain:")
    for s in FY_SCALES:
        v = float(np.mean([r["fy_sweep"][s] for r in res]))
        print(f"  Fy*{s:<5}: {v:8.1f} N  ({100*(v-base_rear)/base_rear:+.0f}%)")


if __name__ == "__main__":
    main()
