#!/usr/bin/env python3
"""Is Fx really 'worse' than Fy, or is it just a units (scale) artifact?

We keep quoting Fx RMSE in Newtons (~510) vs Fy (~410) and call Fx 'bad'. But Fx
(traction/braking) has a far larger dynamic range than Fy (lateral) in normal
driving, so equal-ish Newton RMSE can mean Fx is RELATIVELY more accurate. This
trains the deployed per-axle surrogate and reports, per channel and per axle:
raw RMSE, label std, normalized RMSE (RMSE/std), R^2, and mean|F| -- so the
relative accuracy of Fx vs Fy is unambiguous.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np, pandas as pd, torch, torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "whole_vehicle" / "lhs" / "training_data_rich_tire_frame.csv"
TERRAIN = ["bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction", "janosi_shear"]
FEATS = ["slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
         "d_slip_ratio", "d_slip_angle", "d_velocity"] + TERRAIN
RATES = ["slip_ratio", "slip_angle", "velocity"]
N_SCEN = 1500
dev = "cuda" if torch.cuda.is_available() else "cpu"
rng = np.random.default_rng(0)


def load():
    df = pd.read_csv(DATA)
    scen = df["scenario_id"].unique()
    keep = rng.choice(scen, min(N_SCEN, len(scen)), replace=False)
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


def main():
    df = load()
    print(f"rows={len(df)} scenarios={df['scenario_id'].nunique()}")
    scen = df["scenario_id"].unique()
    te = set(rng.choice(scen, len(scen) // 5, replace=False))
    tr = df[~df["scenario_id"].isin(te)]; te_df = df[df["scenario_id"].isin(te)]
    Xtr = tr[FEATS].to_numpy(np.float32); ytr = tr[["Fx", "Fy"]].to_numpy(np.float32)
    Xte = te_df[FEATS].to_numpy(np.float32); yte = te_df[["Fx", "Fy"]].to_numpy(np.float32)
    mx, sx = Xtr.mean(0), Xtr.std(0) + 1e-6
    my, sy = ytr.mean(0), ytr.std(0) + 1e-6
    torch.manual_seed(0)
    m = MLP(len(FEATS)).to(dev); opt = torch.optim.Adam(m.parameters(), lr=2e-3)
    Xt = torch.tensor((Xtr - mx) / sx, device=dev); Yt = torch.tensor((ytr - my) / sy, device=dev)
    n = len(Xt); bs = 8192
    for _ in range(120):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            opt.zero_grad(); ((m(Xt[idx]) - Yt[idx]) ** 2).mean().backward(); opt.step()
    m.eval()
    with torch.no_grad():
        pred = m(torch.tensor((Xte - mx) / sx, device=dev)).cpu().numpy() * sy + my
    ax = te_df["axle_id"].to_numpy()

    def stats(mask, ch, name):
        t = yte[mask, ch]; p = pred[mask, ch]
        rmse = float(np.sqrt(np.mean((p - t) ** 2)))
        std = float(t.std()); mae = float(np.mean(np.abs(p - t)))
        meanabs = float(np.mean(np.abs(t)))
        r2 = 1.0 - np.sum((p - t) ** 2) / np.sum((t - t.mean()) ** 2)
        rng_ = float(np.percentile(t, 97.5) - np.percentile(t, 2.5))
        return name, rmse, std, rmse / std, float(r2), meanabs, rng_

    print(f"\n{'channel':10s}{'RMSE':>8s}{'std':>8s}{'RMSE/std':>10s}{'R^2':>8s}{'mean|F|':>9s}{'95% range':>11s}")
    for lbl, msk in [("all", np.ones(len(ax), bool)), ("front", ax == 0), ("rear", ax == 1)]:
        for ch, cn in [(0, "Fx"), (1, "Fy")]:
            nm, rmse, std, nr, r2, ma, rg = stats(msk, ch, f"{cn}_{lbl}")
            print(f"{nm:10s}{rmse:8.0f}{std:8.0f}{nr:10.2f}{r2:8.3f}{ma:9.0f}{rg:11.0f}")


if __name__ == "__main__":
    main()
