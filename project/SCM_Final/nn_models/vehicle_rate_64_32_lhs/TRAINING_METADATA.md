# vehicle_rate_64_32_lhs — training metadata

| | |
| --- | --- |
| Deployment status | **default** (`DEFAULT_NN_MODEL` in the standard MPC stack and all safety filters) |
| Architecture | rate-augmented MLP, hidden = [64, 32], 3106 params, ReLU |
| Inputs (14) | same as `rig_rate_64_32` — `slip_ratio, slip_angle, velocity, vertical_load, steering_rate, d_slip_*, bekker_*, mohr_*, janosi_shear` |
| Outputs | per-wheel **tire-frame** `(Fx, Fy)` (sign flipped at log time — same convention as the rig surrogate) |
| Held-out metrics | R² Fx = 0.733, R² Fy = 0.725, RMSE Fx = 530 N, RMSE Fy = 412 N (split-by-scenario test set) |

## Training data

- **Source**: closed-loop HMMWV sims driven by the previous-generation NMPC + tire NN, run on Chrono::Vehicle.
- **Generator**: [`data_collection/collect_closed_loop_data.py`](../../data_collection/collect_closed_loop_data.py) with `--terrain-randomization lhs` and `--rich` output enabled.
- **Aggregated CSV**: [`data/whole_vehicle/lhs/training_data_rich_tire_frame.csv`](../../data/whole_vehicle/lhs/training_data_rich_tire_frame.csv) — ≈ **1.85 M rows** (front + rear axle per controller tick, ~10 Hz × ~10 s each).
- **Scenarios**: **800** closed-loop runs LHS-sampled jointly over soil parameters AND maneuver axes:
  - terrain: 266 clay / 270 dirt / 264 sand
  - path: 214 sinusoidal / 201 right_left / 200 lane_change / 185 double_lane_change
  - speeds: random uniform in [3, 7] m/s
  - bumpiness: random in {0, 2, 4}
  - rocks: random 0–5
  - Soil params LHS-sampled across the Bekker–Mohr box `TRAINING_RANGES_V6` (no canonical-preset jitter).
- **Bootstrap controller**: the previous-generation `vehicle_rate_64_32_lhs` checkpoint drove the MPC during collection. This means the operating-point distribution is conditioned on what *that* NN considered safe — known under-prediction bias at high-slip peaks. See paper §III for the Pacejka-bootstrap mitigation discussion.

## Trainer

- Script: [`nn_training/train_vehicle_lhs.sh`](../../nn_training/train_vehicle_lhs.sh) → [`nn_training/train_variant.py`](../../nn_training/train_variant.py).
- Hyperparameters: Adam, lr=1e-3, batch=256, ~200 epochs, 70/15/15 split-by-scenario.
- Loss: MSE on `(Fx, Fy)` after standardization.

## Sign / frame convention

- Same as `rig_rate_64_32`: `predict_numeric` returns per-wheel tire-frame `Fy`; controller logs body-frame axle Fy as `-2 × Fy_per_wheel`.

## Strengths and known limitations

- **Strength**: training distribution matches the operating point the deployed MPC actually visits — best R² on LHS-jittered terrains, no canonical-preset overfit.
- **Limitation (under-prediction at transients)**: because the bootstrap controller plans conservatively, the training data contains few extreme-slip examples. The deployed model therefore under-predicts peak lateral force during sharp transients on firm soil (dirt at ≥ 7 m/s in particular). See `benchmarking/make_fig1_4way.py` for per-scenario front-axle RMSE numbers; rig surrogates can match or beat this one in regions where the bootstrap bias bites.
