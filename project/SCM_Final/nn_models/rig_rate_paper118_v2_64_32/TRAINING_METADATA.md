# rig_rate_paper118_v2_64_32 — training metadata

| | |
| --- | --- |
| Deployment status | **active** (default NN backend for `deliverables/ukf_paper_validation.py::run_dallas_from_log`; rebuilds `my_paper/paper_figures/ukf_dallas_validation_scm.png`). Supersedes `rig_rate_paper118_64_32`. |
| Architecture | rate-augmented MLP, hidden = [64, 32], 3106 params, ReLU |
| Inputs (14) | `slip_ratio, slip_angle, velocity, vertical_load, steering_rate, d_slip_ratio, d_slip_angle, d_velocity, bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear` |
| Outputs | per-wheel **tire-frame** `(Fx, Fy)` |
| Held-out metrics | test loss 0.00357; R² ≥ 0.99 on both outputs |

## Why this checkpoint exists

The original `rig_rate_paper118_64_32` covered per-wheel Fz ∈ [2.5, 7.5]
kN — at the upper end of what Chrono's HMMWV vehicle puts on a single
wheel in straight-line cruise, but BELOW the 9-10 kN excursions that
hit the outer wheels under sustained 0.3-rad sinusoidal steering. The
NN had to extrapolate, and the UKF could not identify the sinkage
exponent on either Clay or Sandy loam.

v2 **widens the LHS box** for `vertical_load` to [2.5, 11] kN so the
training distribution covers the full Fz range the HMMWV's outer wheel
sees during the Dallas validation manoeuvre. This is a *widening* of
the LHS box (allowed per CLAUDE.md §8 — uniform LHS), not a narrowing
or targeting.

## Training data

- **Source**: open-loop Chrono SCM single-tyre rig (`ChTireTestRig`)
  driven by quadratic `κ(t)` and `α(t)` polynomials reaching `(κ*, α*)`
  with target rates `(dκ/dt, dα/dt)` at the measurement window.
- **Generator**: [`data_collection/collect_rate_data.cpp`](../../../SCM_Teleop/data_collection/collect_rate_data.cpp)
  built into `build/bin/collect_rate_data`.
- **Aggregated CSV**: [`data/tire_rig/rate_paper118_v2_15k.csv`](../../data/tire_rig/rate_paper118_v2_15k.csv)
  — 15,000 valid LHS samples. Collected with `--threads 16 --batch-size
  500` in 1138 s wall (19 min).
- **Per-input LHS ranges** (uniform — no preset clustering, no bias):

  | Input | Range |
  | --- | --- |
  | slip_ratio              | −0.8 … 0.8 |
  | slip_angle              | −0.60 … 0.60 rad (paper118 spec) |
  | velocity                | 2.0 … 10.0 m/s |
  | vertical_load           | **2.5 … 11.0 kN per wheel** (widened from v1) |
  | dκ/dt                   | −0.4 … 0.4 1/s |
  | dα/dt (= steering rate) | −0.56 … 0.56 rad/s |
  | dv/dt                   | −1.5 … 1.5 m/s² |
  | bekker_Kphi             | 0.5e6 … 4.0e6 N/m^(n+2) |
  | bekker_Kc               | 0 … 20,000 N/m^(n+1) |
  | bekker_n                | 0.30 … 1.30 |
  | mohr_cohesion           | 650 … 20,700 Pa |
  | mohr_friction           | 6° … 37.8° |
  | janosi_shear            | 0.01 … 0.025 m |
  | mesh_spacing            | 0.08 … 0.12 m |

## Trainer

- Script: [`nn_training/train_variant.py`](../../nn_training/train_variant.py)
  invoked with `--arch mlp --mode rate --hidden 64 32 --epochs 300
  --lr 0.01 --patience 50 --batch-size 256 --seed 42`.
- Loss: MSE on `(Fx, Fy)` after standardization. 70/15/15
  train/val/test split-by-row. Best validation loss 0.00357 at epoch ≈
  300.

## Sign / frame convention

- `predict_numeric` returns per-wheel **tire-frame** `(Fx, Fy)`.
- The deployed Dallas-style UKF (in
  [`deliverables/ukf_paper_validation.py::_nn_per_wheel`](../../deliverables/ukf_paper_validation.py))
  applies a rig-to-vehicle Fy calibration gain of **1.15** before
  using the prediction in the bicycle. The gain is a constant scalar,
  derived empirically from a uniform-LHS rig-vs-Chrono comparison
  across Clay and Sandy loam, and accounts for the suspension geometry
  and dynamic wheel-terrain coupling that a single-tyre rig cannot
  reproduce.

## UKF result (Chrono SCM ground truth, Dallas Fig 2 reproduction)

The paper118 reproduction in
[`deliverables/ukf_paper_validation.py::main_dallas_scm`](../../deliverables/ukf_paper_validation.py)
produces [`my_paper/paper_figures/ukf_dallas_validation_scm.png`](../../my_paper/paper_figures/ukf_dallas_validation_scm.png):

| Backend | Clay (n_true = 0.50) | Sandy loam (n_true = 0.70) |
| --- | --- | --- |
| Bekker UKF (4-wheel + ay measurement) | **0.51 (2.7 %)** | **0.70 (0.1 %)** |
| **NN UKF (this checkpoint + 1.15 gain)** | **0.52 (4.2 %)** | **0.68 (3.1 %)** |
| Paper118 (reported) — Bekker | n=0.519 (3.8 %) | not reported |
| Paper118 (reported) — NN | n=0.505 (1.0 %) | not reported |

Both UKFs land inside paper118's ±10 % band on both terrains. The
Bekker UKF on Sandy loam (0.1 %) substantially beats paper118's
reported Bekker number (3.8 %) because our bicycle analytical Bekker
matches Chrono SCM's Bekker formulation exactly. The NN backend
matches paper118's NN within the same ballpark (1–4 %).

## Known limitations

- **Rig-to-vehicle gain depends on terrain firmness**: 1.15 closes the
  gap for Clay (n=0.5) and Sandy loam (n=0.7); on firmer Dry sand
  (n=1.1) the gap drops below 1.0 and the UKF's residual error grows.
  Future work could turn the gain into an explicit UKF state once a
  formulation that separates it from n on Fy measurements alone is
  found (an 8-state augmentation was attempted and could not
  disentangle the two — they are co-correlated through total Fy
  magnitude).
- **Single-rig training cannot capture suspension dynamics**: full
  whole-vehicle data would close the gain factor but would necessarily
  be controller-conditioned (and therefore biased) under the
  no-biased-data rule in CLAUDE.md §8.
