# terrain_window_mlp — training metadata

| | |
| --- | --- |
| Deployment status | **active** (online estimator inside the standard NMPC and the MPPI predictive shield) |
| Architecture | sliding-window MLP, hidden = [64, 64, 64], ReLU, ~10 k params |
| Inputs (26) | hand-crafted statistics over a 4 s sliding window: `u/v/ω/ax/ay` means + stds + p95, wheel-slip mean/std/p95, lateral-grip, yaw-gain, longitudinal-drag, throttle mean + vertical-dynamics block (`az_std, az_p95, pitch_rate_std/p95, roll_rate_std`) |
| Output | scalar Bekker sinkage exponent `n` (clamped to [0.4, 1.3] at inference) |
| Validation RMSE | best **val RMSE = 0.20** (split-by-LHS-cell); per-bin RMSE 0.03–0.07 |
| Held-out cell RMSE | **0.098** on 30 % held-out LHS cells |

## Training data

- **Source**: closed-loop HMMWV traces collected by the broad multi-axis terrain-estimator collector.
- **Generator**: [`data_collection/collect_broad_terrain.py`](../../data_collection/collect_broad_terrain.py).
- **Trace directory**: [`data/terrain_estimator/traces_broad_v7/`](../../data/terrain_estimator/traces_broad_v7/) — **3600 trace CSVs**, one per scenario.
- **Scenario design**: 180 LHS terrain cells (n ∈ [0.40, 1.30]) × 20 scenarios per cell:
  - 4 scripted open-loop with cruise-bias seeds 0..3 (bumpiness = 0)
  - 4 scripted open-loop, cruise-bias 0..3, bumpiness = 4
  - 9 closed-loop NMPC (3 paths × 3 speeds in {4, 6, 8} m/s, bumpiness = 0)
  - 3 closed-loop NMPC, sinusoidal, 3 speeds, bumpiness = 4
- **Each trace**: 20 s of sim at ~83 Hz (the sim node's state-publication rate), 13 raw channels + commanded throttle, sliding 4 s windows with 0.4 s stride.
- **Aggregated windows**: ~133 k 26-feature vectors (after 1.5 s warmup discard).

## Trainer

- Script: [`nn_training/train_terrain_window_mlp.py`](../../nn_training/train_terrain_window_mlp.py).
- Hyperparameters: Adam, lr = 2e-3, batch = 256, 400 epochs, `--normalize-y`, seed = 42.
- Split: 85 % train / 15 % validation by **LHS cell** (no cell appears in both train and val) so the held-out RMSE measures terrain generalization, not just within-cell over-fit.

## Sign / frame convention

- Outputs scalar `n` (no sign convention question). The downstream interpolator (`learned_terrain_estimator._terrain_params_for_n`) maps `n` to the full Bekker-Mohr vector before handing to the MPC.

## Known limitations

- Bimodal switching on canonical clay at low closed-loop speeds — the estimator's planar dynamics signature can be hard to disambiguate between clay (n≈0.5) and dirt (n≈0.7) when the vehicle is bogged at low slip. See paper §IV for the canonical clay closed-loop case (`v_cmd = 7 m/s` → err +0.21; `v_cmd = 6 m/s` → err +0.087).
- Per-n-bin error largest in the lowest bin n ∈ [0.40, 0.55] (median |err| ≈ 0.12 closed-loop). The SCM physics is unsimulable below n ≈ 0.37.
- Backup snapshot (deployed weights + figures at training time): [`experiment_results/v7_baseline_backup/`](../../experiment_results/v7_baseline_backup/).
