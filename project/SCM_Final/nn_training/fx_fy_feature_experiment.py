#!/usr/bin/env python3
"""Does adding causal dynamic/load-transfer context improve the surrogate's
Fx and (rear) Fy prediction?

The deployed surrogate (rate mode) already has kappa-dot/alpha-dot, so the
untested lever is the vehicle dynamic state -- lateral velocity, yaw rate, and
kinematic lateral load transfer -- which the rear axle's force especially
depends on. We train baseline vs augmented MLPs (identical arch, scenario
split) on the existing rich data and report held-out RMSE *per axle*, so the
rear-Fy effect is visible. ax_imu/ay_imu are excluded (force consequences =
leakage); only causal, MPC-available channels are added.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "lhs" / "training_data_rich_tire_frame.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
RATES = ["slip_ratio", "slip_angle", "velocity", "v_body", "yaw_rate", "axle_kappa"]   # for d_* features
BASE = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
        "d_slip_ratio", "d_slip_angle", "d_velocity", "axle_id"] + TERRAIN
ADD = ["v_body", "yaw_rate", "dFz_lateral_kin", "steering_angle", "d_v_body", "d_yaw_rate"]  # causal only
ADD2 = ADD + ["measured_kappa", "axle_kappa", "wheel_omega_axle", "accel_cmd", "jerk_cmd", "d_axle_kappa"]
N_SCEN = 1500   # subset of scenarios for tractable training
rng = np.random.default_rng(0)
torch.manual_seed(0)
dev = "cuda" if torch.cuda.is_available() else "cpu"


def load():
    df = pd.read_csv(DATA)
    scen = df["scenario_id"].unique()
    keep = rng.choice(scen, min(N_SCEN, len(scen)), replace=False)
    df = df[df["scenario_id"].isin(keep)].copy()
    df = df.sort_values(["scenario_id", "axle_id", "timestep"])
    g = df.groupby(["scenario_id", "axle_id"])
    for c in RATES:
        df["d_" + c] = g[c].diff().fillna(0.0)
    # basic physical filter
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


def train_eval(df, feats, label):
    scen = df["scenario_id"].unique()
    te_scen = set(rng.choice(scen, max(1, len(scen) // 5), replace=False))
    tr = df[~df["scenario_id"].isin(te_scen)]
    te = df[df["scenario_id"].isin(te_scen)]
    Xtr, ytr = tr[feats].to_numpy(np.float32), tr[["Fx", "Fy"]].to_numpy(np.float32)
    Xte, yte = te[feats].to_numpy(np.float32), te[["Fx", "Fy"]].to_numpy(np.float32)
    mx, sx = Xtr.mean(0), Xtr.std(0) + 1e-6
    my, sy = ytr.mean(0), ytr.std(0) + 1e-6
    Xtr_, ytr_ = (Xtr - mx) / sx, (ytr - my) / sy
    Xte_ = (Xte - mx) / sx
    m = MLP(len(feats)).to(dev)
    opt = torch.optim.Adam(m.parameters(), lr=2e-3)
    Xt = torch.tensor(Xtr_, device=dev); Yt = torch.tensor(ytr_, device=dev)
    n = len(Xt); bs = 8192
    for ep in range(120):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            opt.zero_grad(); loss = ((m(Xt[idx]) - Yt[idx]) ** 2).mean(); loss.backward(); opt.step()
    m.eval()
    with torch.no_grad():
        pred = m(torch.tensor(Xte_, device=dev)).cpu().numpy() * sy + my
    ax = te["axle_id"].to_numpy()
    out = {"label": label, "nfeat": len(feats)}
    for axval, axname in [(0, "front"), (1, "rear")]:
        msk = ax == axval
        if msk.sum():
            out[f"Fx_{axname}"] = float(np.sqrt(np.mean((pred[msk, 0] - yte[msk, 0]) ** 2)))
            out[f"Fy_{axname}"] = float(np.sqrt(np.mean((pred[msk, 1] - yte[msk, 1]) ** 2)))
    out["Fx_all"] = float(np.sqrt(np.mean((pred[:, 0] - yte[:, 0]) ** 2)))
    out["Fy_all"] = float(np.sqrt(np.mean((pred[:, 1] - yte[:, 1]) ** 2)))
    return out


def main():
    print(f"device={dev}; loading {DATA.name} ...")
    df = load()
    print(f"rows={len(df)}  scenarios={df['scenario_id'].nunique()}  axles={sorted(df['axle_id'].unique())}")
    res = [train_eval(df, BASE, "baseline (rate)"),
           train_eval(df, BASE + ADD, "+dynamic context"),
           train_eval(df, BASE + ADD2, "+full causal")]
    print("\nheld-out RMSE (N), per axle:")
    hdr = ["model", "Fx_front", "Fx_rear", "Fy_front", "Fy_rear", "Fx_all", "Fy_all"]
    print(f"{'model':18s}" + "".join(f"{h:>10s}" for h in hdr[1:]))
    for r in res:
        print(f"{r['label']:18s}" + "".join(f"{r.get(h, float('nan')):10.1f}" for h in hdr[1:]))
    b = res[0]
    for r in res[1:]:
        print(f"\nimprovement ({r['label']} vs baseline):")
        for k in ["Fx_front", "Fx_rear", "Fy_front", "Fy_rear", "Fx_all", "Fy_all"]:
            if k in b and k in r:
                print(f"  {k:10s}: {b[k]:.1f} -> {r[k]:.1f} N  ({100*(r[k]-b[k])/b[k]:+.0f}%)")


if __name__ == "__main__":
    main()
