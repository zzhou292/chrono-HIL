#!/usr/bin/env python3
"""Train a unified two-head whole-vehicle force surrogate.

One shared trunk, two output heads, so a single learned vehicle-dynamics model
serves BOTH the NMPC planner and the Dallas-style UKF estimator (instead of the
two separate nets ``vehicle_rate_64_32_lhs`` and ``vehicle_fy_64_32``):

    input (11):  [u, v, omega, delta, throttle, Kphi, Kc, n, c, phi_rad, k]
      |
    shared trunk (MLP)
      |- HEAD A "control"    -> [Fx_f, Fy_f, Fx_r, Fy_r]      (per-axle body frame)
      |- HEAD B "estimation" -> [Fy_total=m*ay, M_yaw=Iz*dwz] (UKF-measured)

Two design points that matter (both learned the hard way):
* HEAD B targets the quantities the UKF actually MEASURES/propagates
  (Fy_total = m*ay, M_yaw = Iz*d_omega/dt), NOT the raw summed tyre forces.
  The two differ ~15-20 % on firm soil; training Head B on the tyre-force sum
  makes the UKF settle on the wrong n (it over-scales Fy and the filter lowers
  n to match m*ay).
* The plain [state, throttle, soil] input is kept. Feeding slip-ratio / slip-
  angle features in (to sharpen the longitudinal Fx channel) was tried and
  *regressed* the estimation head's UKF accuracy while barely helping rear Fx,
  so it is not used. Longitudinal rear Fx remains the weak channel of the
  unified surrogate.

Trained on widened-box Chrono SCM logs (collect_lhs_training_scms --widened-box),
which log per-axle forces + m*ay-consistent state.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys as _sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

_sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deliverables"))
from ukf_paper_validation import Vehicle as _UKFVehicle  # noqa: E402
_VEH = _UKFVehicle()

# Trunk input is the flat state+throttle+soil vector (identical to the
# estimation-only model that validated in the UKF). Slip ratios enter ONLY at
# the control head, so the trunk + Head B path is unchanged and the UKF
# estimation accuracy is preserved.
INPUT_NAMES = ["u", "v", "omega", "delta", "throttle",
               "Kphi", "Kc", "n", "c", "phi_rad", "k"]
KAPPA_INPUTS = ["kappa_f", "kappa_r"]                  # sign-corrected slip ratios -> Head A only
HEAD_A_NAMES = ["Fx_f", "Fy_f", "Fx_r", "Fy_r"]        # per-axle body-frame tyre force
HEAD_B_NAMES = ["Fy_total_inertial", "Myaw_inertial"]  # m*ay, Iz*dwz (UKF-consistent)

# Calibrated effective rolling radius (estimated from cruise u/omega over the
# widened-box sweep). NOTE: GetSpindleOmega is negative for forward motion, so
# wheel speeds are sign-flipped when forming slip ratio.
WHEEL_R = 0.4563
# Fx is ~62% high-frequency terrain noise (irreducible from smooth state); the
# control head is trained on the low-pass *controllable* Fx so it predicts the
# force the NMPC can act on rather than chasing noise.
FX_SMOOTH_WIN = 15


class TwoHeadSurrogate(nn.Module):
    def __init__(self, n_in: int, n_kappa: int = 2, trunk=(128, 128), head_hidden: int = 64):
        super().__init__()
        layers = []
        d = n_in
        for h in trunk:
            layers += [nn.Linear(d, h), nn.ReLU()]
            d = h
        self.trunk = nn.Sequential(*layers)
        self.head_b = nn.Sequential(nn.Linear(d, head_hidden), nn.ReLU(),
                                    nn.Linear(head_hidden, len(HEAD_B_NAMES)))
        self.head_a = nn.Sequential(nn.Linear(d + n_kappa, head_hidden), nn.ReLU(),
                                    nn.Linear(head_hidden, len(HEAD_A_NAMES)))

    def forward(self, x, kappa):
        z = self.trunk(x)
        # Stop-gradient into the trunk from the control head: the trunk is
        # shaped ONLY by the estimation loss, so Head B (and its UKF accuracy)
        # is identical to the estimation-only model and cannot be perturbed by
        # the control head. Head A adapts to that fixed representation + slip.
        return self.head_a(torch.cat([z.detach(), kappa], dim=1)), self.head_b(z)

    def forward_head_b(self, x):
        return self.head_b(self.trunk(x))


def _smooth(a, w):
    if len(a) < w or w < 2:
        return a
    return np.convolve(a, np.ones(w) / w, mode="same")


def _load_logs(logs_dir: Path, decim: int, max_scenarios: int = 0):
    Xs, XKs, YAs, YBs, sids = [], [], [], [], []
    files = sorted(logs_dir.glob("*.npz"))
    if max_scenarios and max_scenarios > 0:
        files = files[:max_scenarios]   # data-scaling ablation
    if not files:
        raise SystemExit(f"No .npz logs in {logs_dir}")
    needed = ["t", "lead_in", "u", "v", "omega", "delta_meas", "throttle_cmd", "ay",
              "w_fl", "w_fr", "w_rl", "w_rr",
              "Fx_axle_f", "Fy_axle_f", "Fx_axle_r", "Fy_axle_r",
              "soil_Kphi", "soil_Kc", "soil_n", "soil_c", "soil_phi_rad", "soil_k"]
    skipped = 0
    for si, f in enumerate(files):
        d = np.load(str(f), allow_pickle=True)
        if any(k not in d.files for k in needed):
            skipped += 1
            continue
        m = d["t"] >= float(d["lead_in"][0])
        if m.sum() < 50:
            skipped += 1
            continue
        dc = max(1, decim)
        dt_ = float(np.median(np.diff(d["t"][m])))
        # Per-scenario quantities on the full masked series (so smoothing /
        # gradient never leak across scenarios), then decimate.
        dom = np.gradient(d["omega"][m], dt_)
        u_m = d["u"][m]; us = np.maximum(np.abs(u_m), 0.5)
        wf = -0.5 * (d["w_fl"][m] + d["w_fr"][m])   # sign-corrected (fwd -> +)
        wr = -0.5 * (d["w_rl"][m] + d["w_rr"][m])
        kf = (wf * WHEEL_R - u_m) / us
        kr = (wr * WHEEL_R - u_m) / us
        fxf = _smooth(d["Fx_axle_f"][m], FX_SMOOTH_WIN)   # controllable (low-freq) Fx
        fxr = _smooth(d["Fx_axle_r"][m], FX_SMOOTH_WIN)
        sl = slice(None, None, dc)
        soil = np.array([float(d["soil_Kphi"][0]), float(d["soil_Kc"][0]),
                         float(d["soil_n"][0]), float(d["soil_c"][0]),
                         float(d["soil_phi_rad"][0]), float(d["soil_k"][0])])
        nrow = u_m[sl].size
        X = np.empty((nrow, len(INPUT_NAMES)))
        X[:, 0] = u_m[sl]; X[:, 1] = d["v"][m][sl]; X[:, 2] = d["omega"][m][sl]
        X[:, 3] = d["delta_meas"][m][sl]; X[:, 4] = d["throttle_cmd"][m][sl]
        X[:, 5:] = soil[None, :]
        XK = np.column_stack([kf[sl], kr[sl]])
        YA = np.column_stack([fxf[sl], d["Fy_axle_f"][m][sl],
                              fxr[sl], d["Fy_axle_r"][m][sl]])
        YB = np.column_stack([_VEH.m * d["ay"][m][sl], _VEH.Iz * dom[sl]])
        good = (np.all(np.isfinite(X), 1) & np.all(np.isfinite(XK), 1)
                & np.all(np.isfinite(YA), 1) & np.all(np.isfinite(YB), 1))
        Xs.append(X[good]); XKs.append(XK[good]); YAs.append(YA[good]); YBs.append(YB[good])
        sids.append(np.full(int(good.sum()), si))
    print(f"[data] {len(files)} logs ({skipped} skipped), {sum(len(x) for x in Xs)} rows")
    return (np.concatenate(Xs), np.concatenate(XKs),
            np.concatenate(YAs), np.concatenate(YBs), np.concatenate(sids))


def _standardize(a):
    mu = a.mean(0); sd = a.std(0); sd[sd < 1e-9] = 1.0
    return mu, sd


def _r2(pred, true):
    ss_res = np.sum((true - pred) ** 2, 0)
    ss_tot = np.sum((true - true.mean(0)) ** 2, 0); ss_tot[ss_tot < 1e-12] = 1e-12
    return 1.0 - ss_res / ss_tot


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--logs-dir", default="data/dallas_scm/lhs_twohead")
    p.add_argument("--out-dir", default="nn_models/vehicle_twohead_128")
    p.add_argument("--epochs", type=int, default=400)
    p.add_argument("--batch", type=int, default=512)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--decim", type=int, default=2)
    p.add_argument("--max-scenarios", type=int, default=0,
                   help="Limit number of scenarios loaded (0=all) for data-scaling studies.")
    p.add_argument("--val-frac", type=float, default=0.10)
    p.add_argument("--head-b-weight", type=float, default=1.0)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--trunk", type=int, nargs="+", default=[128, 128])
    args = p.parse_args()

    torch.manual_seed(args.seed); np.random.seed(args.seed)
    X, XK, YA, YB, sid = _load_logs(Path(args.logs_dir), args.decim, args.max_scenarios)

    uniq = np.unique(sid); rng = np.random.default_rng(args.seed); rng.shuffle(uniq)
    n_val = max(1, int(len(uniq) * args.val_frac))
    val_ids = set(uniq[:n_val].tolist())
    vm = np.array([s in val_ids for s in sid]); tr = ~vm

    xm, xs = _standardize(X[tr]); km, ks = _standardize(XK[tr])
    am, as_ = _standardize(YA[tr]); bm, bs = _standardize(YB[tr])
    T = lambda a: torch.tensor(a, dtype=torch.float32)
    Xtr = T((X[tr]-xm)/xs); Xva = T((X[vm]-xm)/xs)
    Ktr = T((XK[tr]-km)/ks); Kva = T((XK[vm]-km)/ks)
    Atr = T((YA[tr]-am)/as_); Ava = T((YA[vm]-am)/as_)
    Btr = T((YB[tr]-bm)/bs); Bva = T((YB[vm]-bm)/bs)
    print(f"[split] train={len(Xtr)} val={len(Xva)} ({len(uniq)-n_val}/{n_val} scn)")

    model = TwoHeadSurrogate(len(INPUT_NAMES), len(KAPPA_INPUTS), trunk=tuple(args.trunk))
    opt = torch.optim.Adam(model.parameters(), lr=args.lr); lf = nn.MSELoss()
    n = len(Xtr); best = float("inf"); best_state = None; wB = float(args.head_b_weight)
    for ep in range(args.epochs):
        model.train(); perm = torch.randperm(n)
        for i in range(0, n, args.batch):
            j = perm[i:i+args.batch]; opt.zero_grad()
            pa, pb = model(Xtr[j], Ktr[j])
            (lf(pa, Atr[j]) + wB*lf(pb, Btr[j])).backward(); opt.step()
        model.eval()
        with torch.no_grad():
            pa, pb = model(Xva, Kva); vloss = (lf(pa, Ava)+wB*lf(pb, Bva)).item()
        if vloss < best: best = vloss; best_state = {k: v.clone() for k, v in model.state_dict().items()}
        if ep % 50 == 0 or ep == args.epochs-1: print(f"  ep={ep:3d} val={vloss:.4f}")

    model.load_state_dict(best_state); model.eval()
    with torch.no_grad():
        pa, pb = model(Xva, Kva)
    pa = pa.numpy()*as_+am; pb = pb.numpy()*bs+bm
    r2a = _r2(pa, YA[vm]); r2b = _r2(pb, YB[vm])
    print("\n[held-out R2] control head (per-axle force; Fx on low-pass controllable target):")
    for nm, r in zip(HEAD_A_NAMES, r2a): print(f"    {nm:9} R2={r:.3f}")
    print("[held-out R2] estimation head:")
    for nm, r in zip(HEAD_B_NAMES, r2b): print(f"    {nm:18} R2={r:.3f}")

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), out/"weights.pt")
    with open(out/"scaler.pkl", "wb") as f:
        pickle.dump({"x_mean": xm, "x_std": xs, "xk_mean": km, "xk_std": ks,
                     "ya_mean": am, "ya_std": as_, "yb_mean": bm, "yb_std": bs}, f)
    with open(out/"config.json", "w") as f:
        json.dump({"input_names": INPUT_NAMES, "kappa_inputs": KAPPA_INPUTS,
                   "head_a_names": HEAD_A_NAMES, "head_b_names": HEAD_B_NAMES,
                   "trunk": list(args.trunk), "wheel_r": WHEEL_R,
                   "fx_smooth_win": FX_SMOOTH_WIN, "Lf": _VEH.Lf, "Lr": _VEH.Lr,
                   "head_hidden": 64, "decim": args.decim, "best_val_loss": best,
                   "held_out_r2_control": {n: float(r) for n, r in zip(HEAD_A_NAMES, r2a)},
                   "held_out_r2_estimation": {n: float(r) for n, r in zip(HEAD_B_NAMES, r2b)}},
                  f, indent=2)
    print(f"\n[train] saved to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
