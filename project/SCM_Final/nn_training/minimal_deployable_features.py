#!/usr/bin/env python3
"""Which dynamic features are worth the acados wiring cost?

The deployed surrogate is per-axle v8_rate: [kappa, alpha, u, Fz, sr,
d_kappa, d_alpha, d_u, soil6] -- NO axle_id (front/rear distinguished only by
their alpha/Fz). To deploy a richer surrogate I must wire each new feature as a
CasADi function of the MPC state. Cost ranking:

  v_body  = v   (MPC state)         -> trivial
  yaw_rate= omega (MPC state)       -> trivial
  steering_angle = delta (state)    -> easy
  dFz_lateral_kin = m*ay*h/track    -> moderate (ay = u*omega + vdot)
  d_v_body, d_yaw_rate = vdot,omegadot*dt -> moderate (dynamics RHS)

This finds where the Fx/Fy gain SATURATES so we wire the fewest features. Trained
on the rich closed-loop data with the EXACT deployed feature layout (per-axle,
no axle_id), held-out per-axle RMSE, multi-seed.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "lhs" / "training_data_rich_tire_frame.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
# deployed layout exactly (per-axle, no axle_id):
DEPLOYED = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
            "d_slip_ratio", "d_slip_angle", "d_velocity"] + TERRAIN
RATES = ["slip_ratio", "slip_angle", "velocity", "v_body", "yaw_rate"]
N_SCEN = 1500
SEEDS = (0, 1, 2)
dev = "cuda" if torch.cuda.is_available() else "cpu"
load_rng = np.random.default_rng(0)


def load():
    df = pd.read_csv(DATA)
    scen = df["scenario_id"].unique()
    keep = load_rng.choice(scen, min(N_SCEN, len(scen)), replace=False)
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


def train_eval(df, feats, te_scen, seed):
    torch.manual_seed(seed)
    tr = df[~df["scenario_id"].isin(te_scen)]; te = df[df["scenario_id"].isin(te_scen)]
    Xtr, ytr = tr[feats].to_numpy(np.float32), tr[["Fx", "Fy"]].to_numpy(np.float32)
    Xte, yte = te[feats].to_numpy(np.float32), te[["Fx", "Fy"]].to_numpy(np.float32)
    mx, sx = Xtr.mean(0), Xtr.std(0) + 1e-6
    my, sy = ytr.mean(0), ytr.std(0) + 1e-6
    Xtr_, Xte_, ytr_ = (Xtr - mx) / sx, (Xte - mx) / sx, (ytr - my) / sy
    m = MLP(len(feats)).to(dev); opt = torch.optim.Adam(m.parameters(), lr=2e-3)
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


CONFIGS = {
    "deployed (rate)": DEPLOYED,
    "+v_body,yaw_rate": DEPLOYED + ["v_body", "yaw_rate"],
    "+steering_angle": DEPLOYED + ["v_body", "yaw_rate", "steering_angle"],
    "+dFz_lat": DEPLOYED + ["v_body", "yaw_rate", "steering_angle", "dFz_lateral_kin"],
    "+d_v,d_yaw (full)": DEPLOYED + ["v_body", "yaw_rate", "steering_angle",
                                     "dFz_lateral_kin", "d_v_body", "d_yaw_rate"],
}
KEYS = ["Fx_front", "Fx_rear", "Fy_front", "Fy_rear", "Fx_all", "Fy_all"]


def main():
    print(f"device={dev}; loading {DATA.name} ...")
    df = load()
    print(f"rows={len(df)}  scenarios={df['scenario_id'].nunique()}")
    scen = df["scenario_id"].unique()
    agg = {name: {k: [] for k in KEYS} for name in CONFIGS}
    for seed in SEEDS:
        rng = np.random.default_rng(200 + seed)
        te_scen = set(rng.choice(scen, max(1, len(scen) // 5), replace=False))
        for name, feats in CONFIGS.items():
            r = train_eval(df, feats, te_scen, seed)
            for k in KEYS:
                agg[name][k].append(r[k])
    print("\nheld-out RMSE (N), mean over %d seeds, per axle:" % len(SEEDS))
    print(f"{'config':20s}" + "".join(f"{h:>10s}" for h in KEYS))
    means = {}
    for name in CONFIGS:
        means[name] = {k: float(np.mean(agg[name][k])) for k in KEYS}
        print(f"{name:20s}" + "".join(f"{means[name][k]:10.1f}" for k in KEYS))
    b = means["deployed (rate)"]
    print("\n% change vs deployed (rate):")
    print(f"{'config':20s}" + "".join(f"{h:>10s}" for h in KEYS))
    for name in CONFIGS:
        if name == "deployed (rate)":
            continue
        print(f"{name:20s}" + "".join(f"{100*(means[name][k]-b[k])/b[k]:+9.0f}%" for k in KEYS))

    # reproducible artifact for paper Table (sec:feature_headroom)
    out_csv = ROOT / "my_paper" / "paper_figures" / "feature_headroom.csv"
    rows = []
    for name in CONFIGS:
        row = {"config": name, **{k: round(means[name][k], 1) for k in KEYS}}
        if name != "deployed (rate)":
            for k in KEYS:
                row[f"{k}_pct"] = round(100 * (means[name][k] - b[k]) / b[k], 1)
        rows.append(row)
    try:
        pd.DataFrame(rows).to_csv(out_csv, index=False)
        print(f"\nwrote {out_csv}")
    except Exception as e:
        print(f"(csv write skipped: {e})")


if __name__ == "__main__":
    main()
