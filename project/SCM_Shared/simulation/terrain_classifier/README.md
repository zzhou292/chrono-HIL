# Terrain Classification Module

Classifies terrain type (**clay**, **sand**, **dirt**) in real time from
vehicle-measurable signals (wheel speeds, IMU, GPS).  Runs as a decoupled
ZMQ subprocess alongside the Chrono simulation and MPC controller.

**Live accuracy: 27/27 on the standard evaluation grid** (3 terrains × 3 paths
× 3 speeds).

---

## Architecture Overview

```
chrono_sim_node ──(VehicleState @ 100 Hz)──► classifier_node
                                                │
                                       FeatureExtractor (1.0 s window, 0.25 s stride)
                                                │
                                       HistGradientBoosting model (26 features)
                                                │
                                       ExponentialSmoother (EMA + running-mean)
                                                │
                                       TerrainEstimate ──► (published on ZMQ port 5557)
```

### Components

| File                       | Purpose                                                          |
| -------------------------- | ---------------------------------------------------------------- |
| `classifier_node.py`       | ZMQ online node: subscribes to state, publishes terrain estimate |
| `feature_extractor.py`     | Sliding-window feature computation (13 base features)            |
| `train_model.py`           | Trains HistGradientBoosting classifier, saves `.pkl` model       |
| `collect_data.py`          | Per-run CSV data collector (subscribes to sim via ZMQ)           |
| `launch_data_collection.py`| Automated multi-run data collection orchestrator                 |
| `messages.py`              | `TerrainEstimate` dataclass and ZMQ serialization                |
| `models/terrain_rf.pkl`    | Trained model bundle (model + scaler + label encoder)            |

---

## End-to-End Pipeline

### Step 1 — Collect Training Data

The data collection grid sweeps over terrains, paths, bumpiness levels, and
speeds.  Each combination launches a full Chrono simulation with the MPC
controller to produce realistic driving excitation.

To reproduce the v10 dataset, run both halves and merge:

```bash
cd project/SCM_Teleop/simulation

# Pacejka MPC half (with sensor noise)
conda run -n sim --no-capture-output \
  python3 -m terrain_classifier.launch_data_collection \
    --runs-per-combo 2 \
    --time 25 \
    --model pacejka \
    --output-dir terrain_classifier/data_v10_pacejka \
    --merged-output terrain_classifier/data_v10_pacejka/training_data_pacejka.csv

# NN MPC half (with sensor noise)
conda run -n sim --no-capture-output \
  python3 -m terrain_classifier.launch_data_collection \
    --runs-per-combo 2 \
    --time 25 \
    --model nn \
    --nn-model paper_v1_mlp_16_4 \
    --output-dir terrain_classifier/data_v10_nn \
    --merged-output terrain_classifier/data_v10_nn/training_data_nn.csv

# Merge into one CSV
python3 -c "
import pandas as pd, pathlib
p = pathlib.Path('terrain_classifier')
df = pd.concat([pd.read_csv(p/'data_v10_pacejka/training_data_pacejka.csv'),
                pd.read_csv(p/'data_v10_nn/training_data_nn.csv')], ignore_index=True)
out = p/'data_v10_merged/training_data_v10.csv'
out.parent.mkdir(exist_ok=True)
df.to_csv(out, index=False)
print(f'Merged {len(df)} samples -> {out}')
"
```

**Default grid** (defined in `launch_data_collection.py`):

| Dimension  | Values                                          |
| ---------- | ----------------------------------------------- |
| Terrain    | `clay`, `sand`, `dirt`                           |
| Path       | `lane_change`, `double_lane_change`, `sinusoidal`|
| Bumpiness  | `0`, `3`, `6`                                    |
| Speed      | `5.0`, `8.0` m/s                                 |
| Reps       | `2` (configurable via `--runs-per-combo`)        |

Total combinations: 3 × 3 × 3 × 2 × 2 = **108 runs**.

Each run produces a CSV in `data/` named
`{terrain}_{path}_b{bump}_v{speed}_r{rep}.csv`, and at the end all per-run
files are merged into `data/training_data.csv`.

**Latest on-disk datasets**:

| File                     | Samples  | Controllers      | Notes                    |
| ------------------------ | -------- | ---------------- | ------------------------ |
| `data/training_data_v5.csv`                        | 21,220   | Pacejka MPC      | No noise, 12 base feats (legacy)  |
| `data/training_data_v7.csv`                        | 29,780   | Pacejka + NN MPC | No noise, 12 base feats (legacy)  |
| `data_v10_merged/training_data_v10.csv` **(active)**| 16,300   | Pacejka + NN MPC | With noise, 13 base feats (`steering_std`) |

The v10 dataset was collected in two halves:
- `data_v10_pacejka/` — 7,980 samples (Pacejka MPC, `--model pacejka`)
- `data_v10_nn/` — 8,320 samples (NN MPC, `--model nn --nn-model paper_v1_mlp_16_4`)

Both halves used the standard grid with sensor noise enabled (no `--no-noise`).
The merged file is `data_v10_merged/training_data_v10.csv`.

### Step 2 — Train the Model

```bash
cd project/SCM_Teleop/simulation

conda run -n sim --no-capture-output \
  python3 -m terrain_classifier.train_model \
    --data terrain_classifier/data_v10_merged/training_data_v10.csv \
    --output terrain_classifier/models/terrain_rf.pkl \
    --classifier gb \
    --n-trees 200 \
    --max-depth 6
```

This:
1. Loads the CSV (13 base features per sample).
2. Drops any legacy `speed_mean`/`speed_std` columns if present.
3. Computes **13 derived features** from the base features:
   - 4 log-transforms of slip means/maxes
   - Front/rear slip ratio, front/rear slip CVs, yaw-sideslip ratio
   - 5 steering-normalized features (slip, ay, yaw rate, yaw accel,
     sideslip per unit steering_std)
4. Standardizes all **26 features** with `StandardScaler`.
5. Trains a `HistGradientBoostingClassifier` and evaluates with 5-fold
   stratified cross-validation.
6. Saves `terrain_rf.pkl` containing:
   - `model` — the trained classifier
   - `scaler` — fitted `StandardScaler`
   - `label_encoder` — fitted `LabelEncoder`
   - `feature_names` — ordered list of all 26 feature names
   - `classes` — `["clay", "dirt", "sand"]`
7. Generates `confusion_matrix.png` and `feature_importance.png` in `models/`.

**Current cross-validation accuracy**: ~90.8%.

### Step 3 — Run the Classifier Online

The classifier runs as a separate process launched by `launch_decoupled.py`:

```bash
cd project/SCM_Teleop/simulation

conda run -n sim --no-capture-output \
  python3 launch_decoupled.py \
    --terrain sand \
    --path lane_change \
    --speed 5 \
    --time 25 \
    --model pacejka \
    --terrain-classifier \
    --no-noise \
    --vis-mode none
```

Or standalone:

```bash
conda run -n sim --no-capture-output \
  python3 -m terrain_classifier.classifier_node \
    --model terrain_classifier/models/terrain_rf.pkl \
    --sim-port 5555 \
    --pub-port 5557
```

The node:
1. Subscribes to `VehicleState` on ZMQ port 5555.
2. Feeds each state through `FeatureExtractor` (1.0 s sliding window, 0.25 s
   stride, min speed 1.0 m/s → ~4 Hz classification rate).
3. Extracts 13 base features, adds 13 derived features, scales, and runs
   `model.predict_proba()`.
4. Smooths the probability vector through a **two-layer smoother**:
   - **Layer 1 (EMA)**: `α=0.3` exponential moving average for short-term
     responsiveness during the first 8 predictions (burn-in).
   - **Layer 2 (running mean)**: After burn-in, accumulates all posterior
     probability vectors and returns the running mean — very stable.
5. Publishes `TerrainEstimate` (class, confidence, per-class probabilities)
   on ZMQ port 5557.

---

## Features

### 13 Base Features (from `FeatureExtractor`)

| #  | Feature              | Source          | Description                                |
| -- | -------------------- | --------------- | ------------------------------------------ |
| 0  | `slip_front_mean`    | Wheel encoders  | Mean |κ| of front axle                      |
| 1  | `slip_front_std`     | Wheel encoders  | Standard deviation of front slip           |
| 2  | `slip_front_max`     | Wheel encoders  | Maximum front slip in window               |
| 3  | `slip_rear_mean`     | Wheel encoders  | Mean |κ| of rear axle                       |
| 4  | `slip_rear_std`      | Wheel encoders  | Standard deviation of rear slip            |
| 5  | `slip_rear_max`      | Wheel encoders  | Maximum rear slip in window                |
| 6  | `yaw_accel_std`      | IMU (gyroscope) | Yaw acceleration vibration                 |
| 7  | `az_std`             | IMU (accel.)    | Vertical acceleration std (roughness)      |
| 8  | `sideslip_ratio_mean`| IMU / GPS       | Mean |v|/max(|u|, 0.5) — lateral dynamics  |
| 9  | `yaw_rate_mean`      | IMU (gyroscope) | Mean |ω_z|                                 |
| 10 | `ax_std`             | IMU (accel.)    | Longitudinal acceleration vibration        |
| 11 | `ay_std`             | IMU (accel.)    | Lateral acceleration vibration             |
| 12 | `steering_std`       | Steering sensor | Std of steering input over window          |

*Note*: `speed_mean` and `speed_std` are computed for logging but
**intentionally excluded** from the ML feature array to prevent the classifier
from using vehicle speed as a shortcut (sand is physically slower due to higher
rolling resistance, which would confound terrain classification).
`steering_std` is kept as a base feature for driving-intensity normalization.

### 13 Derived Features (computed in `train_model.py` and `classifier_node.py`)

| Feature                  | Formula                                         |
| ------------------------ | ----------------------------------------------- |
| `log_slip_front_mean`    | log1p(slip_front_mean)                          |
| `log_slip_rear_mean`     | log1p(slip_rear_mean)                           |
| `log_slip_front_max`     | log1p(slip_front_max)                           |
| `log_slip_rear_max`      | log1p(slip_rear_max)                             |
| `slip_front_rear_ratio`  | slip_front_mean / slip_rear_mean                |
| `slip_front_cv`          | slip_front_std / slip_front_mean                |
| `slip_rear_cv`           | slip_rear_std / slip_rear_mean                  |
| `yaw_sideslip_ratio`     | yaw_rate_mean / sideslip_ratio_mean             |
| `slip_rear_per_steer`    | slip_rear_mean / steering_std                   |
| `ay_per_steer`           | ay_std / steering_std                           |
| `yaw_rate_per_steer`     | yaw_rate_mean / steering_std                    |
| `yaw_accel_per_steer`    | yaw_accel_std / steering_std                    |
| `sideslip_per_steer`     | sideslip_ratio_mean / steering_std              |

The 5 steering-normalized features divide dynamic responses by steering
intensity, making the classifier invariant to how aggressively the driver
is steering — terrain determines the *ratio* of slip/yaw/sideslip per unit
steering input.

---

## Directory Structure

```
terrain_classifier/
├── README.md                      # This file
├── __init__.py                    # Module docstring, exports FeatureExtractor
├── classifier_node.py             # Online classification node (ZMQ)
├── collect_data.py                # Per-run data collection subscriber
├── feature_extractor.py           # Sliding-window feature computation
├── launch_data_collection.py      # Automated multi-run orchestrator
├── messages.py                    # TerrainEstimate message definition
├── train_model.py                 # Model training + evaluation
├── models/
│   ├── terrain_rf.pkl             # Active trained model (26 features, 16,300 samples)
│   ├── confusion_matrix.png       # CV confusion matrix
│   └── feature_importance.png     # Feature importance chart
├── data_v10_pacejka/              # v10 Pacejka half (7,980 samples, with noise)
├── data_v10_nn/                   # v10 NN MPC half (8,320 samples, with noise)
└── data_v10_merged/
    └── training_data_v10.csv      # **Active** merged v10 dataset (16,300 samples)
```

Legacy data (v5, v7, v8, v9, per-run CSVs) has been moved to
`SCM_Teleop/archive/terrain_classifier_data_legacy/`.

---

## Design Decisions

1. **Speed features excluded**: Speed had 39.5% permutation importance in early
   models because sand is physically limited to ~4.4 m/s (SCM rolling
   resistance + MPC ax_max=1.9 m/s²).  Removing speed forced the model to
   learn actual terrain signatures (slip patterns, vibration).

2. **HistGradientBoosting over RandomForest**: Better calibrated probabilities
   and faster training on 20k+ samples.

3. **Two-layer probability smoothing**: Individual window predictions are ~91%
   accurate, but the old single-layer EMA was too reactive — one misclassified
   window could flip the estimate.  The running-mean accumulator after burn-in
   averages all posterior probabilities over the run, converging to the correct
   class.  This took live accuracy from 24/27 to **27/27**.

4. **Bumpiness in training grid**: Including bump=0,3,6 made the classifier
   robust to varied terrain roughness.  Without bumpiness variation, the model
   struggled at extreme roughness levels.

5. **Steering-normalized features**: Dividing dynamic responses by `steering_std`
   makes the classifier invariant to driving intensity.  For the same terrain,
   aggressive steering produces more slip/yaw/sideslip, but the *ratio* per
   unit steering input is a terrain property.  These 5 derived features improved
   CV accuracy from ~89% to ~91%.

6. **Mixed-controller data**: Training on data from both Pacejka MPC and NN MPC
   controllers ensures the classifier generalizes across controller behaviors.
   Pacejka MPC produces smoother trajectories; NN MPC produces more aggressive
   cornering on off-road terrains.
