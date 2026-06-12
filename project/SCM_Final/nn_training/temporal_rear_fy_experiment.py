#!/usr/bin/env python3
"""Last physical lever for the stubborn REAR-Fy: temporal relaxation.

Ruled out so far for the rear axle (all <=3% on rear-Fy):
  - dynamic/load-transfer context (v_body, yaw_rate, dFz),
  - full causal commands/slip/omega,
  - per-wheel sinkage INCLUDING oracle ground-truth multi-pass.

Remaining hypothesis: lateral tire force has a *relaxation length* -- it lags
the slip angle by ~half a tyre revolution -- and the rear runs in continuously
transient slip. A per-timestep MLP (even with first differences) cannot see that
lag. So we add a short CAUSAL history (lags of slip_angle / slip_ratio / v_body /
yaw_rate) and measure the per-axle effect. All lagged channels are MPC-available
(no oracle, no leakage). If this doesn't move rear-Fy either, the rear gap is
structural variance, not a missing feature.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "lhs" / "training_data_rich_tire_frame.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
RATES = ["slip_ratio", "slip_angle", "velocity", "v_body", "yaw_rate"]
LAGCH = ["slip_angle", "slip_ratio", "v_body", "yaw_rate"]   # relaxation-relevant
LAGS = [1, 2, 4, 8, 16]                                       # 50 Hz -> up to 320 ms
BASE = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
        "steering_angle", "v_body", "yaw_rate", "dFz_lateral_kin",
        "d_slip_ratio", "d_slip_angle", "d_velocity", "d_v_body", "d_yaw_rate", "axle_id"] + TERRAIN
N_SCEN = 1500
SEEDS = (0, 1)
dev = "cuda" if torch.cuda.is_available() else "cpu"
base_rng = np.random.default_rng(0)


def load():
    df = pd.read_csv(DATA)
    scen = df["scenario_id"].unique()
    keep = base_rng.choice(scen, min(N_SCEN, len(scen)), replace=False)
    df = df[df["scenario_id"].isin(keep)].copy()
    df = df.sort_values(["scenario_id", "axle_id", "timestep"])
    g = df.groupby(["scenario_id", "axle_id"])
    for c in RATES:
        df["d_" + c] = g[c].diff().fillna(0.0)
    lag_cols = []
    for c in LAGCH:
        for L in LAGS:
            name = f"{c}_lag{L}"
            df[name] = g[c].shift(L).fillna(method="bfill").fillna(0.0)
            lag_cols.append(name)
    df = df[df["slip_ratio"].between(-1.2, 1.2) & df["slip_angle"].between(-0.7, 0.7)
            & df["velocity"].between(0.25, 20) & df["vertical_load"].between(1000, 10000)
            & df["Fx"].between(-5e4, 5e4) & df["Fy"].between(-5e4, 5e4)]
    return df.reset_index(drop=True), lag_cols


class MLP(nn.Module):
    def __init__(self, nin):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(nin, 64), nn.Tanh(),
                                 nn.Linear(64, 32), nn.Tanh(), nn.Linear(32, 2))
    def forward(self, x): return self.net(x)


def train_eval(df, feats, te_scen, seed):
    torch.manual_seed(seed)
    tr = df[~df["scenario_id"].isin(te_scen)]; te = df[df["scenario_id"].isin(te_scen)]
    Xtr, ytr = tr[feats].to_numpy(np.float32), tr[["Fx", "Fy"]].to_numpy(np.float32)
    Xte, yte = te[feats].to_numpy(np.float32), te[["Fx", "Fy"]].to_numpy(np.float32)
    mx, sx = Xtr.mean(0), Xtr.std(0) + 1e-6
    my, sy = ytr.mean(0), ytr.std(0) + 1e-6
    Xtr_, Xte_ = (Xtr - mx) / sx, (Xte - mx) / sx
    ytr_ = (ytr - my) / sy
    m = MLP(len(feats)).to(dev)
    opt = torch.optim.Adam(m.parameters(), lr=2e-3)
    Xt = torch.tensor(Xtr_, device=dev); Yt = torch.tensor(ytr_, device=dev)
    n = len(Xt); bs = 8192
    for _ in range(120):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            opt.zero_grad(); loss = ((m(Xt[idx]) - Yt[idx]) ** 2).mean(); loss.backward(); opt.step()
    m.eval()
    with torch.no_grad():
        pred = m(torch.tensor(Xte_, device=dev)).cpu().numpy() * sy + my
    ax = te["axle_id"].to_numpy(); out = {}
    for axval, axn in [(0, "front"), (1, "rear")]:
        msk = ax == axval
        out[f"Fx_{axn}"] = float(np.sqrt(np.mean((pred[msk, 0] - yte[msk, 0]) ** 2)))
        out[f"Fy_{axn}"] = float(np.sqrt(np.mean((pred[msk, 1] - yte[msk, 1]) ** 2)))
    out["Fx_all"] = float(np.sqrt(np.mean((pred[:, 0] - yte[:, 0]) ** 2)))
    out["Fy_all"] = float(np.sqrt(np.mean((pred[:, 1] - yte[:, 1]) ** 2)))
    return out


KEYS = ["Fx_front", "Fx_rear", "Fy_front", "Fy_rear", "Fx_all", "Fy_all"]


def main():
    print(f"device={dev}; loading {DATA.name} ...")
    df, lag_cols = load()
    print(f"rows={len(df)}  scenarios={df['scenario_id'].nunique()}  +{len(lag_cols)} lag feats")
    configs = {"BASE (full-causal)": BASE, "+history(relaxation)": BASE + lag_cols}
    scen = df["scenario_id"].unique()
    agg = {k: {m: [] for m in KEYS} for k in configs}
    for seed in SEEDS:
        rng = np.random.default_rng(100 + seed)
        te_scen = set(rng.choice(scen, max(1, len(scen) // 5), replace=False))
        for name, feats in configs.items():
            r = train_eval(df, feats, te_scen, seed)
            for k in KEYS:
                agg[name][k].append(r[k])
    print("\nheld-out RMSE (N), mean over %d seeds:" % len(SEEDS))
    print(f"{'model':22s}" + "".join(f"{h:>10s}" for h in KEYS))
    means = {}
    for name in configs:
        means[name] = {k: float(np.mean(agg[name][k])) for k in KEYS}
        print(f"{name:22s}" + "".join(f"{means[name][k]:10.1f}" for k in KEYS))
    b = means["BASE (full-causal)"]
    for name in configs:
        if name == "BASE (full-causal)":
            continue
        print(f"\nimprovement ({name} vs BASE):")
        for k in KEYS:
            print(f"  {k:10s}: {b[k]:.1f} -> {means[name][k]:.1f} N  ({100*(means[name][k]-b[k])/b[k]:+.0f}%)")


if __name__ == "__main__":
    main()
