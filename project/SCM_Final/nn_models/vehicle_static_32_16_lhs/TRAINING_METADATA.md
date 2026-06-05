# vehicle_static_32_16_lhs — training metadata

| | |
| --- | --- |
| Deployment status | **active** (referenced by paper §III as the static-feature whole-vehicle tire baseline; consumed by `benchmarking/make_fig1_4way.py` and `deliverables/cte_vehicle_static_vs_rate.py` for the static-vs-rate CTE comparison). |
| Architecture | static (non-rate) MLP, hidden = [32, 16], ≈ 946 params |
| Inputs (11) | `slip_ratio, slip_angle, velocity, vertical_load, steering_rate, bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear` |
| Outputs (2) | `(Fx, Fy)` — per-wheel, tire-frame longitudinal and lateral force |
| Held-out metrics | Fx R² = 0.71, RMSE = 552 N; Fy R² = 0.71, RMSE = 424 N (split by scenario, seed 42) |

## Training data

- **Source**: the same closed-loop whole-vehicle LHS dataset as
  `vehicle_rate_64_32_lhs` — NMPC-driven Chrono::Vehicle runs,
  LHS-sampled jointly over the Bekker–Mohr soil box and the maneuver
  axes (path / speed / bumpiness / seed), logging the live operating
  point alongside the SCM ground-truth force.
- **Aggregated CSV**: [`data/whole_vehicle/lhs/training_data_tire_frame.csv`](../../data/whole_vehicle/lhs/training_data_tire_frame.csv)
  (≈ 453 MB; the non-"rich" frame — one row per axle per controller
  tick). This is the `--mode static` counterpart of the rate model's
  `training_data_rich_tire_frame.csv`.
- **Generator**: [`data_collection/collect_closed_loop_data.py`](../../data_collection/collect_closed_loop_data.py)
  (`--terrain-randomization lhs`).

## Trainer

- Script: [`nn_training/train_vehicle_lhs.sh`](../../nn_training/train_vehicle_lhs.sh)
  → [`nn_training/train_variant.py`](../../nn_training/train_variant.py),
  invoked as `--mode static --hidden 32 16`.
- Split: by scenario (`split_by_scenario`), `torch_seed = 42`.

## Notes

- "Static" means no rate-of-change features: the model sees the
  instantaneous operating point only, unlike the rate-augmented
  `vehicle_rate_64_32_lhs` (the deployed NMPC tire). It is retained as
  the static baseline in the rig-vs-vehicle / static-vs-rate analyses,
  not as the controller default.
