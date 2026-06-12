#!/usr/bin/env python3
"""Does per-wheel SINKAGE fix the rear axle's stubborn force prediction?

Background: causal dynamic/load features cut FRONT-axle Fx/Fy RMSE ~25 % but the
REAR by only ~3 %. Hypothesis: the rear runs on soil the front just compacted
(multi-pass), so its force depends on a soil state no current feature carries.

This trains identical 64-32 MLPs on the sinkage dataset under four feature sets
and reports held-out RMSE *per axle*, multi-seed:

  BASE            -- causal op-point + dynamic state + rates + soil + axle_id.
                     NOTE: BASE already has vertical_load AND the 6 Bekker-Mohr
                     params, so the MLP can derive a *single-pass* Bekker sinkage
                     internally. Anything sinkage adds beyond BASE is therefore
                     the part single-pass physics cannot give -- the multi-pass /
                     transient sinkage.
  +bekker_sink    -- BASE + an explicit DEPLOYABLE single-pass Bekker sinkage
                     proxy z=(p/k_eq)^(1/n) from load+soil (what the controller
                     could compute online). Tests if just handing the MLP the
                     proxy helps.
  +oracle_sink    -- BASE + ground-truth SCM sinkage (upper bound; encodes
                     multi-pass). The gap (+oracle) - (+bekker) IS the
                     multi-pass residual.
  +oracle+front   -- also the front same-side wheel's sinkage (explicit
                     pre-compaction context for the rear).
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "sinkage" / "training_data_sinkage.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
RATES = ["slip_ratio", "slip_angle", "velocity", "v_body", "yaw_rate"]
BASE = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate", "steering_angle",
        "u_body", "v_body", "yaw_rate",
        "d_slip_ratio", "d_slip_angle", "d_velocity", "d_v_body", "d_yaw_rate", "axle_id"] + TERRAIN
SEEDS = (0, 1, 2)
dev = "cuda" if torch.cuda.is_available() else "cpu"

# tire contact-patch constants for the deployable single-pass Bekker proxy
TIRE_B, TIRE_L = 0.30, 0.25   # m (HMMWV tire width, approx contact length)


def add_bekker_sinkage(df):
    n = df["bekker_n"].clip(0.3, 1.4)
    k_eq = df["bekker_Kc"] / TIRE_B + df["bekker_Kphi"]      # Pa/m^n
    p = df["vertical_load"] / (TIRE_B * TIRE_L)              # Pa
    df["bekker_sink"] = (p / k_eq.clip(lower=1.0)).clip(lower=0.0) ** (1.0 / n)
    return df


def load():
    df = pd.read_csv(DATA)
    df = df.sort_values(["scenario_id", "axle_id", "side", "timestep"])
    g = df.groupby(["scenario_id", "axle_id", "side"])
    for c in RATES:
        df["d_" + c] = g[c].diff().fillna(0.0)
    df["d_sinkage"] = g["sinkage"].diff().fillna(0.0)
    df = add_bekker_sinkage(df)
    df = df[df["slip_ratio"].between(-1.2, 1.2) & df["slip_angle"].between(-0.7, 0.7)
            & df["velocity"].between(0.25, 20) & df["vertical_load"].between(500, 12000)
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
    for _ in range(150):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            opt.zero_grad(); loss = ((m(Xt[idx]) - Yt[idx]) ** 2).mean(); loss.backward(); opt.step()
    m.eval()
    with torch.no_grad():
        pred = m(torch.tensor(Xte_, device=dev)).cpu().numpy() * sy + my
    ax = te["axle_id"].to_numpy()
    out = {}
    for axval, axname in [(0, "front"), (1, "rear")]:
        msk = ax == axval
        out[f"Fx_{axname}"] = float(np.sqrt(np.mean((pred[msk, 0] - yte[msk, 0]) ** 2)))
        out[f"Fy_{axname}"] = float(np.sqrt(np.mean((pred[msk, 1] - yte[msk, 1]) ** 2)))
    out["Fx_all"] = float(np.sqrt(np.mean((pred[:, 0] - yte[:, 0]) ** 2)))
    out["Fy_all"] = float(np.sqrt(np.mean((pred[:, 1] - yte[:, 1]) ** 2)))
    return out


CONFIGS = {
    "BASE": BASE,
    "+bekker_sink (deployable)": BASE + ["bekker_sink"],
    "+oracle_sink": BASE + ["sinkage", "d_sinkage"],
    "+oracle+front_sink": BASE + ["sinkage", "d_sinkage", "sinkage_front_same_side"],
}
KEYS = ["Fx_front", "Fx_rear", "Fy_front", "Fy_rear", "Fx_all", "Fy_all"]


def main():
    print(f"device={dev}; loading {DATA.name} ...")
    df = load()
    print(f"rows={len(df)}  scenarios={df['scenario_id'].nunique()}  axles={sorted(df['axle_id'].unique())}")
    scen = df["scenario_id"].unique()
    agg = {name: {k: [] for k in KEYS} for name in CONFIGS}
    for seed in SEEDS:
        rng = np.random.default_rng(seed)
        te_scen = set(rng.choice(scen, max(1, len(scen) // 5), replace=False))
        for name, feats in CONFIGS.items():
            r = train_eval(df, feats, te_scen, seed)
            for k in KEYS:
                agg[name][k].append(r[k])
    print("\nheld-out RMSE (N), mean over %d seeds, per axle:" % len(SEEDS))
    hdr = ["model"] + KEYS
    print(f"{'model':26s}" + "".join(f"{h:>10s}" for h in hdr[1:]))
    means = {}
    for name in CONFIGS:
        means[name] = {k: float(np.mean(agg[name][k])) for k in KEYS}
        print(f"{name:26s}" + "".join(f"{means[name][k]:10.1f}" for k in KEYS))
    b = means["BASE"]
    for name in CONFIGS:
        if name == "BASE":
            continue
        print(f"\nimprovement ({name} vs BASE):")
        for k in KEYS:
            print(f"  {k:10s}: {b[k]:.1f} -> {means[name][k]:.1f} N  ({100*(means[name][k]-b[k])/b[k]:+.0f}%)")


if __name__ == "__main__":
    main()
