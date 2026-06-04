# Active Neural Checkpoints

`nn_models/` is the canonical runtime model directory. It intentionally
keeps only the checkpoints currently referenced by either the deployed
runtime or a live paper claim.

| Directory | Role |
| --- | --- |
| `terrain_window_mlp/` | **Deployed** online terrain estimator (v7 weights — broad-coverage 3600-trace training). See [`TRAINING_METADATA.md`](terrain_window_mlp/TRAINING_METADATA.md). |
| `vehicle_rate_64_32_lhs/` | **Deployed** whole-vehicle rate tire surrogate used by every paper sweep. See [`TRAINING_METADATA.md`](vehicle_rate_64_32_lhs/TRAINING_METADATA.md). |
| `rig_rate_64_32/` | Tire-rig rate baseline retained for rig-vs-vehicle diagnostics and §III ablations. See [`TRAINING_METADATA.md`](rig_rate_64_32/TRAINING_METADATA.md). |
| `vehicle_static_32_16_lhs/` | Static-features vehicle surrogate; retained because §III and `deliverables/cte_vehicle_static_vs_rate.py` actively reference it for the force-RMSE-vs-CTE analysis. |

Each deployed checkpoint ships with a `TRAINING_METADATA.md` describing
its dataset path, generator script, hyperparameters, sign convention,
and known limitations.

## Retired / archived checkpoints

Stale or experimental checkpoints (development snapshots, held-out
ablations, "didn't help" architectural variants) live in:

* `archive/2026-05-25_nn_models_cleanup/` — terrain-estimator history
  (v6, v5 held-out, v7 named snapshot, low-n weighted variant, LSTM
  smoke test, etc.). See that directory's `README.md` for the full
  table.
* `archive/2026-05-23_model_checkpoint_and_root_artifact_cleanup/` —
  earlier static / axle-rate / PIL / joint-estimator checkpoints and
  the former `nn_models_new/` staging tree.

Restore from those archives only when replaying a historical ablation
that explicitly requires the older checkpoint.
