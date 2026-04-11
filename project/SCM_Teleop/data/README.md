## Datasets in `data/`

This folder is organized so it’s obvious which CSVs are the **same underlying data** vs **derived/resampled** variants.

### `rate_aware/`
- **`rate_aware_timeseries.csv`**: rate-aware *time-series* dataset (has `scenario_id,timestep,...` and time-varying `slip_ratio/slip_angle/velocity`).
- **`rate_aware_timeseries_subsample10.csv`**: same dataset as above, but **subsampled by ~10×** in time (derived/resampling).

### `temporal/`
- **`temporal_v1_timeseries.csv`**: temporal *time-series* dataset (older version).
- **`temporal_v2_timeseries.csv`**: temporal *time-series* dataset (v2).
- **`temporal_v2_static_subsample100.csv`**: **static resampling** derived from `temporal_v2_timeseries.csv` (drops `scenario_id/timestep` and keeps row-wise features + `Fx/Fy`).

### `sweep/`
- **`sweep_static.csv`**: static dataset used for architecture sweeps (row-wise samples, no `scenario_id/timestep`).
- **`sweep_static_subsample10.csv`**: **subsampled** version of `sweep_static.csv` (derived/resampling).

### `steady_state/`
- **`v6_static.csv`**: steady-state static dataset.

### MPC-aligned tire CSV (optional)

Run `acados_mpc_controller_node.py` with `--log-tire-csv /path/to/out.csv` during a Chrono run. Each row uses the **same** slip/α/Fz/δ̇ convention as the MPC (bicycle front axle, mean front-wheel SCM forces). Use with `train_temporal_nn.py` / `train_rate_nn.py` after merging multiple runs with distinct `scenario_id` values (`--log-scenario-id`).

For **parallel** collection (many scenarios, one merged CSV), use `simulation/collect_vehicle_tire_dataset.py`, which calls `launch_decoupled.py` with unique ZMQ ports per job and merges shard files. From `simulation/` with your Chrono conda env active:

`python collect_vehicle_tire_dataset.py --runs 8 --jobs 4 --output ../data/vehicle_mpc/my_run.csv --terrain clay --path double_lane_change --time 25 --speed 5 --lead-in 10` (MPC tire model defaults to **tmeasy**; use `--model nn --nn-model …` if you want NN-in-the-loop collection.)

For **Latin hypercube** soil parameters (same six-dimensional box as `TRAINING_RANGES_V6` / rig `collect_scm_data_fast`, covering clay/sand/dirt preset neighborhoods), add `--terrain-mode lhs --lhs-seed <int>` and keep `--runs` as the number of LHS samples.

