# rig_rate_64_32 — training metadata

| | |
| --- | --- |
| Deployment status | **active** (retained as the rig-side baseline; not the controller's default) |
| Architecture | rate-augmented MLP, hidden = [64, 32], 3106 params, ReLU |
| Inputs (14) | `slip_ratio, slip_angle, velocity, vertical_load, steering_rate, d_slip_ratio, d_slip_angle, d_velocity, bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear` |
| Outputs | per-wheel **tire-frame** `(Fx, Fy)` — see "Sign convention" below |
| Held-out metrics | R² Fx = 0.733, R² Fy = 0.725, RMSE Fx = 530 N, RMSE Fy = 412 N |

## Training data

- **Source**: Chrono SCM single-tire rig sweeps.
- **Generators**: [`data_collection/collect_rate_data.cpp`](../../data_collection/collect_rate_data.cpp) — sweeps `(κ, α, F_z, θ_soil)` on a fixed test rig while applying a steering-rate excitation that exposes the model to rate-of-change features.
- **Raw CSV**: [`data/tire_rig/rate_v2_100k.csv`](../../data/tire_rig/rate_v2_100k.csv) (~100 k rows, shipped, ≈19 MB). Re-collect via the Chrono rig binary (`collect_rate_data.cpp`) if you need to regenerate it from scratch.
- **Coverage**: open-loop pure-slip, slip angles up to ±0.55 rad, slip ratios up to ±0.30, F_z in [3 kN, 10 kN], terrain parameters LHS-jittered around the canonical clay/dirt/sand presets.

## Trainer

- Script: [`nn_training/train_rate_v2.sh`](../../nn_training/train_rate_v2.sh) → wraps [`nn_training/train_variant.py`](../../nn_training/train_variant.py).
- Hyperparameters: Adam, lr=1e-3, batch=256, ~200 epochs, 70/15/15 train/val/test split.
- Loss: MSE on `(Fx, Fy)` after standardization.

## Sign / frame convention

- **predict_numeric returns per-wheel, tire-frame `Fy`**. To get the axle-level body-frame value the MPC controller uses, multiply by 2 (inner + outer wheel, sharing the same axle slip in the bicycle model) and negate:
  `Fy_axle_body = -2 × Fy_per_wheel_tire`
- This sign flip is implemented inside the controller; pre-existing training data was generated with the tire-frame convention (see `data_collection/collect_closed_loop_data.py` "tire-frame CSV" note).

## Known limitations

- **Saturation gap**: the rig protocol cannot represent post-peak Fy roll-off because there is no contact-patch dynamics on a single-wheel rig. Closed-loop predictions at slip > ~0.20 rad systematically over-predict by ~1 kN. See paper §III-D and `benchmarking/make_fig1_4way.py`.
- The model is retained as a **controlled baseline** for rig-vs-vehicle ablations, not as a deployment candidate.
