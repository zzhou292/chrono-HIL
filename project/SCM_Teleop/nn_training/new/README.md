## New experiment pipeline (`nn_training/new/`)

This folder provides a **reproducible** way to retrain and compare:
- **MLP** vs **ResNet**
- **static** vs **temporal (window K)** vs **rate-augmented**

All outputs are compatible with:
- `simulation/nn_tire_model.py` (loads checkpoints + scalers)
- `simulation/acados_mpc_controller_node.py` (runs closed-loop MPC)

### 1) Prepare datasets

Use the rate-aware time-series dataset as the single source of truth:

```bash
python prepare_universal_dataset.py \
  --timeseries ../../data/rate_aware/rate_aware_timeseries.csv \
  --out-dir ../../data/universal \
  --static-subsample 10
```

### 2) Train a single variant

Examples:

```bash
# Static MLP
python train_variant.py --data ../../data/universal/rate_aware_timeseries_static_subsample10.csv \
  --output-dir ../../nn_models/paper_v1_mlp_16_8 --arch mlp --mode static --hidden 16 8

# Temporal ResNet (K=5)
python train_variant.py --data ../../data/rate_aware/rate_aware_timeseries.csv \
  --output-dir ../../nn_models/paper_v1_resnet_temporal_K5_h16_b2 \
  --arch resnet --mode temporal --K 5 --hidden-dim 16 --n-blocks 2

# Rate-augmented ResNet
python train_variant.py --data ../../data/rate_aware/rate_aware_timeseries.csv \
  --output-dir ../../nn_models/paper_v1_resnet_rate_h16_b2 \
  --arch resnet --mode rate --hidden-dim 16 --n-blocks 2
```

### 3) Train a full grid

```bash
python run_experiment_grid.py \
  --timeseries ../../data/rate_aware/rate_aware_timeseries.csv \
  --static ../../data/universal/rate_aware_timeseries_static_subsample10.csv \
  --tag paper_v1 --epochs 50
```

