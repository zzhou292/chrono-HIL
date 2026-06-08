#!/usr/bin/env python3
"""Convert widened-box whole-vehicle NPZ logs into a per-axle tire CSV in the
EXACT convention train_variant.py / the acados OCP expect (so the trained model
is a drop-in for the NMPC, like vehicle_rate_64_32_lhs).

Per tick we emit two rows (front axle_id=0, rear axle_id=1) with:
  slip_ratio, slip_angle, velocity, vertical_load (per-wheel), steering_rate,
  bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction(deg),
  janosi_shear, Fx, Fy   (per-wheel; Fy negated to tire frame, matching the
  existing training_data_rich_tire_frame.csv).
"""
from __future__ import annotations
import argparse, csv, glob, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deliverables"))
from ukf_paper_validation import Vehicle  # noqa

VEH = Vehicle(); G = 9.81; H_CG = 0.72; WHEEL_R = 0.4563
COLS = ["scenario_id","timestep","axle_id","slip_ratio","slip_angle","velocity","vertical_load",
        "steering_rate","bekker_Kphi","bekker_Kc","bekker_n","mohr_cohesion",
        "mohr_friction","janosi_shear","Fx","Fy"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--logs-dir", default="data/dallas_scm/lhs_twohead")
    p.add_argument("--out", default="data/whole_vehicle/twohead_peraxle_tire_frame.csv")
    p.add_argument("--decim", type=int, default=2)
    args = p.parse_args()
    files = sorted(glob.glob(str(Path(args.logs_dir) / "*.npz")))
    L = VEH.Lf + VEH.Lr
    Wf = VEH.m * G * VEH.Lr / L; Wr = VEH.m * G * VEH.Lf / L  # static axle loads
    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    nrows = 0
    chk = {"fy_alpha": [], "fx_kappa": []}
    with out.open("w", newline="") as fh:
        w = csv.writer(fh); w.writerow(COLS)
        for si, f in enumerate(files):
            d = np.load(f, allow_pickle=True)
            need = ["t","lead_in","u","v","omega","delta_meas","ax","w_fl","w_fr","w_rl","w_rr",
                    "Fx_axle_f","Fy_axle_f","Fx_axle_r","Fy_axle_r",
                    "soil_Kphi","soil_Kc","soil_n","soil_c","soil_phi_rad","soil_k"]
            if any(k not in d.files for k in need):
                continue
            m = d["t"] >= float(d["lead_in"][0])
            if m.sum() < 50:
                continue
            dc = max(1, args.decim)
            t = d["t"][m]; dt_ = float(np.median(np.diff(t)))
            u = d["u"][m]; v = d["v"][m]; om = d["omega"][m]; de = d["delta_meas"][m]; ax = d["ax"][m]
            us = np.maximum(np.abs(u), 0.5)
            wf = -0.5*(d["w_fl"][m]+d["w_fr"][m]); wr = -0.5*(d["w_rl"][m]+d["w_rr"][m])  # sign-fixed
            kf = (wf*WHEEL_R-u)/us; kr = (wr*WHEEL_R-u)/us
            af = de - np.arctan2(v + VEH.Lf*om, us)
            ar = -np.arctan2(v - VEH.Lr*om, us)
            sr = np.clip(np.gradient(de, dt_), -1.0, 1.0)
            # per-wheel vertical load with longitudinal transfer
            dW = VEH.m*ax*H_CG/L
            FzF = np.clip((Wf - dW)/2.0, 500, None); FzR = np.clip((Wr + dW)/2.0, 500, None)
            # per-wheel tire-frame forces (axle body-frame /2; Fy negated to tire frame)
            FxF = d["Fx_axle_f"][m]/2.0; FyF = -d["Fy_axle_f"][m]/2.0
            FxR = d["Fx_axle_r"][m]/2.0; FyR = -d["Fy_axle_r"][m]/2.0
            Kphi=float(d["soil_Kphi"][0]); Kc=float(d["soil_Kc"][0]); n=float(d["soil_n"][0])
            coh=float(d["soil_c"][0]); phid=math.degrees(float(d["soil_phi_rad"][0])); jan=float(d["soil_k"][0])
            sl = slice(None, None, dc)
            for axle, (k_, a_, Fz_, Fx_, Fy_) in enumerate([(kf,af,FzF,FxF,FyF),(kr,ar,FzR,FxR,FyR)]):
                k_s,a_s,Fz_s,Fx_s,Fy_s,sr_s,u_s = k_[sl],a_[sl],Fz_[sl],Fx_[sl],Fy_[sl],sr[sl],u[sl]
                chk["fy_alpha"].append(np.column_stack([Fy_s,a_s]))
                chk["fx_kappa"].append(np.column_stack([Fx_s,k_s]))
                for i in range(len(k_s)):
                    if not (np.isfinite(k_s[i]) and np.isfinite(a_s[i]) and np.isfinite(Fx_s[i]) and np.isfinite(Fy_s[i])):
                        continue
                    # Unique scenario_id per (scenario, axle) so compute_rates
                    # diffs within a single axle's time series.
                    w.writerow([si*2+axle, i, axle, f"{k_s[i]:.5f}", f"{a_s[i]:.5f}", f"{u_s[i]:.4f}",
                                f"{Fz_s[i]:.1f}", f"{sr_s[i]:.4f}", f"{Kphi:.1f}", f"{Kc:.1f}",
                                f"{n:.4f}", f"{coh:.1f}", f"{phid:.3f}", f"{jan:.5f}",
                                f"{Fx_s[i]:.2f}", f"{Fy_s[i]:.2f}"])
                    nrows += 1
    fa = np.vstack(chk["fy_alpha"]); fk = np.vstack(chk["fx_kappa"])
    print(f"wrote {nrows} rows to {out}")
    print(f"convention check (must match existing CSV): corr(Fy,slip_angle)={np.corrcoef(fa[:,0],fa[:,1])[0,1]:+.3f} "
          f"(target ~ -0.69),  corr(Fx,slip_ratio)={np.corrcoef(fk[:,0],fk[:,1])[0,1]:+.3f} (target ~ +0.58)")


if __name__ == "__main__":
    main()
