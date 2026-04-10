# SCM Teleop — Deformable Terrain Vehicle Simulation

MPC-based HMMWV control on **SCM** (Soil Contact Model) deformable terrain using **PyChrono**, **ACADOS** SQP MPC, and **neural-network tire forces** embedded in the optimizer via CasADi.

The NN tire model replaces analytical tire formulations (Pacejka, TMeasy, linear) inside the MPC, allowing the controller to reason about terrain-dependent tire forces on deformable soil parameterized by Bekker/Wong soil properties.

---

## Quick start

```bash
conda activate chrono
cd simulation

# NN tire in MPC on clay double lane change
python launch_decoupled.py --model nn --nn-model paper_v1_mlp_16_4 \
  --terrain clay --path double_lane_change --time 30 --speed 5 --lead-in 10

# Analytical tire (TMeasy) for comparison
python launch_decoupled.py --model tmeasy --terrain clay --path double_lane_change \
  --time 30 --speed 5 --lead-in 10

# Headless (no visualization, no live plot, no CSV output)
python launch_decoupled.py --model nn --nn-model paper_v1_mlp_16_4 \
  --terrain dirt --path lane_change --no-vis --no-plot --no-csv
```

---

## Directory structure

```
SCM_Teleop/
├── simulation/                    # PyChrono sim + ACADOS MPC core
│   ├── launch_decoupled.py        # Main launcher (sim + controller via ZMQ)
│   ├── chrono_sim_node.py         # PyChrono physics → ZMQ state publisher
│   ├── acados_mpc_controller_node.py  # ACADOS MPC → ZMQ control subscriber
│   ├── acados_mpc_solver.py       # ACADOS solver wrapper (SQP/SQP_RTI, HPIPM)
│   ├── nn_tire_model.py           # Load NN checkpoints → CasADi symbolic for MPC
│   ├── analytical_tire_models.py  # Pacejka, TMeasy, linear tire CasADi wrappers
│   ├── param_consistency.py       # SSOT: vehicle geometry, TERRAIN_PRESETS, TRAINING_RANGES_V6
│   └── tire_input_features.py     # MPC-aligned tire feature vector f()
│
├── utilities/                     # Benchmarking, plotting, data collection, solver tools
│   ├── run_paper_benchmarks.py    # Full paper benchmark suite (3 suites, parallel)
│   ├── plot_paper_benchmarks.py   # Generate paper-ready benchmark figures
│   ├── precompile_solvers.py      # Pre-build ACADOS solver cache
│   ├── run_mpc_benchmarks.py      # Per-model MPC comparison (--discover)
│   ├── benchmark_tire_models.py   # NN vs analytical tire comparisons
│   ├── benchmark_nn_variants.py   # Multi-model closed-loop NN benchmark
│   └── generate_reference_paths.py  # Generate reference path CSVs
│
├── nn_training/                   # NN training pipeline
│   ├── new/                       # Unified experiment pipeline (paper_v1)
│   │   ├── train_variant.py       # Single model: MLP/ResNet × static/temporal/rate
│   │   ├── run_experiment_grid.py # 25-model grid launcher
│   │   └── prepare_universal_dataset.py  # Time-series → static subsample
│   ├── evaluate_models.py         # Collect test metrics across all models
│   └── smoke_test_nn_models.py    # Load + CasADi numeric forward-pass check
│
├── nn_models/                     # Trained checkpoints (paper_v1_* + paper_validation_*)
│   ├── paper_v1_mlp_{12_2, 16_4, 16_8, 24_12, 32_16}       # Static MLP (5)
│   ├── paper_v1_resnet_{h8_b2, h16_b2, h16_b4, h32_b2}     # Static ResNet (4)
│   ├── paper_v1_mlp_rate_{16_8, 24_12}                      # Rate MLP (2)
│   ├── paper_v1_resnet_rate_{h16_b2, h32_b2}                # Rate ResNet (2)
│   ├── paper_v1_mlp_temporal_K{3,5,10}_{16_8, 24_12}        # Temporal MLP (6)
│   ├── paper_v1_resnet_temporal_K{3,5,10}_{h16_b2, h32_b2}  # Temporal ResNet (6)
│   └── paper_validation_*                                     # Reproducibility checks (5)
│
├── data/                          # Training datasets
│   ├── temporal/temporal_v1_timeseries.csv      # 1.2M rows, rig time-series
│   └── universal/temporal_v1_timeseries_static_subsample6.csv  # ~200K static rows
│
├── data_collection/           # C++/Python parallel LHS rig data collection
├── paths/                     # Reference path CSVs (lane_change, double_lane_change, sinusoidal)
├── diagnostic_scripts/        # Force comparison & validation tools
├── docs/                      # Pipeline documentation
├── archive/                   # Archived models, data, and scripts from earlier experiments
└── plots/                     # Benchmark output (CSV + JSON + figures)
```

---

## Models

All 25 `paper_v1_*` models are trained on **rig data** collected via `data_collection/collect_temporal_data` with Latin hypercube sampling over 11 parameters (load, slip, velocity, 6 Bekker/Wong soil params) per `param_consistency.TRAINING_RANGES_V6`.

Each checkpoint directory contains:
- `best_terrain_nn.pt` — PyTorch state dict + architecture metadata
- `scalers.pkl` — sklearn `StandardScaler` for inputs (X) and outputs (y)
- `test_metrics.json` — R², RMSE, MAE for Fx and Fy on held-out test set

| Category | Models | Input |
|----------|--------|-------|
| Static MLP | 5 sizes: `[12,2]` to `[32,16]` | 11 features (slip, load, terrain) |
| Static ResNet | 4 configs: h∈{8,16,32}, b∈{2,4} | Same 11 features |
| Rate-augmented | 2 MLP + 2 ResNet | 11 + steering_rate |
| Temporal K=3,5,10 | 6 MLP + 6 ResNet | K×11 sliding window |

Static models achieve R² ≈ 0.81 (Fx) / 0.86 (Fy). Temporal/rate models show minimal improvement due to the steady-state-dominated rig data. Static `paper_v1_mlp_16_4` benchmarks best in closed-loop MPC tracking.

---

## Key components

- **MPC solver:** ACADOS SQP-RTI (static NN, 1 iter) or SQP (analytical/temporal, 3 iters), `PARTIAL_CONDENSING_HPIPM` QP, N=30 horizon at dt=0.1s. QP robustness: adaptive Levenberg-Marquardt, kinematic rollout fallback on QP failure, conditional warm-start.
- **Parameter SSOT:** `simulation/param_consistency.py` — HMMWV geometry, `TERRAIN_PRESETS` (clay/sand/dirt), `TRAINING_RANGES_V6`, LHS helpers.
- **NN in MPC:** `simulation/nn_tire_model.py` — auto-detects static/temporal/rate from checkpoint metadata and builds CasADi symbolic graph.
- **Benchmarking:** `utilities/run_paper_benchmarks.py` runs 252 sims (28 models × 3 terrains × 3 paths). Solver success counts both converged (status 0) and max-iterations (status 2) as success.

---

## Dependencies

```
conda activate chrono
```

- **PyChrono** (vehicle, SCM terrain, Irrlicht visualization)
- **ACADOS** + **CasADi** (3.6.7)
- **PyTorch**, NumPy, pandas, scikit-learn, matplotlib, PyYAML
- **ZeroMQ** (pyzmq) for decoupled sim/controller

---

## Reproducing from scratch

The sections below give step-by-step commands to recollect data, retrain models, and run benchmarks. All commands assume `conda activate chrono` and start from the `SCM_Teleop/` directory.

### Step 1 — Collect rig data (C++)

Build the C++ collector and generate time-series training data:

```bash
# Build (from chrono-HIL build directory)
cd /path/to/chrono-HIL/build
cmake ..
cmake --build . --target collect_temporal_data

# Collect ~1M+ rows of time-series rig data with polynomial slip profiles
# Usage: ./collect_temporal_data <num_samples> <output_csv> [options]
./collect_temporal_data 50000 temporal_v1_timeseries.csv --parallel-only --batch-size 5000

# Move to data directory
mv temporal_v1_timeseries.csv /path/to/SCM_Teleop/data/temporal/
```

The collector samples 11 parameters via Latin hypercube over `TRAINING_RANGES_V6`:
loads (2500–7500 N), slip angles (±0.15 rad), slip ratios (±0.12), velocity (0.5–10.5 m/s),
and 6 Bekker/Wong soil parameters (Kphi, Kc, n, cohesion, friction, janosi).

### Step 2 — Prepare static subsample

Extract a static (row-wise) subset from the time-series for training static and rate models:

```bash
cd nn_training/new

python prepare_universal_dataset.py \
  --timeseries ../../data/temporal/temporal_v1_timeseries.csv \
  --out-dir ../../data/universal \
  --static-subsample 6
```

This creates `data/universal/temporal_v1_timeseries_static_subsample6.csv` (~200K rows).

### Step 3 — Train the full paper_v1 model grid (25 models)

```bash
cd nn_training/new

python run_experiment_grid.py \
  --timeseries ../../data/temporal/temporal_v1_timeseries.csv \
  --static ../../data/universal/temporal_v1_timeseries_static_subsample6.csv \
  --tag paper_v1 \
  --epochs 100 \
  --batch-size 256 \
  --patience 50
```

This launches 25 training runs sequentially (5 static MLP, 4 static ResNet, 4 rate, 12 temporal K∈{3,5,10}) and saves checkpoints under `nn_models/paper_v1_*`.

To train a **single model** instead:

```bash
# Static MLP [16,4]
python train_variant.py \
  --data ../../data/universal/temporal_v1_timeseries_static_subsample6.csv \
  --output-dir ../../nn_models/paper_v1_mlp_16_4 \
  --arch mlp --mode static --hidden 16 4 \
  --epochs 100 --batch-size 256 --lr 0.01 --patience 50

# Temporal ResNet K=5, h=16, blocks=2
python train_variant.py \
  --data ../../data/temporal/temporal_v1_timeseries.csv \
  --output-dir ../../nn_models/paper_v1_resnet_temporal_K5_h16_b2 \
  --arch resnet --mode temporal --K 5 --hidden-dim 16 --n-blocks 2 \
  --epochs 100 --batch-size 256 --lr 0.01 --patience 50

# Rate-augmented MLP [16,8]
python train_variant.py \
  --data ../../data/temporal/temporal_v1_timeseries.csv \
  --output-dir ../../nn_models/paper_v1_mlp_rate_16_8 \
  --arch mlp --mode rate --hidden 16 8 \
  --epochs 100 --batch-size 256 --lr 0.01 --patience 50
```

### Step 4 — Smoke-test trained models

Verify all models load and produce valid CasADi outputs:

```bash
cd nn_training
python smoke_test_nn_models.py --prefix paper_v1_
```

### Step 5 — Pre-compile ACADOS solvers

Build the solver cache before benchmarking to avoid compile overhead during parallel runs:

```bash
cd utilities

# Compile all solvers (analytical + all 25 NN models), 4 workers
python precompile_solvers.py -j 4

# Or compile only analytical baselines
python precompile_solvers.py --only analytical

# Or compile specific models
python precompile_solvers.py --models paper_v1_mlp_16_4 paper_v1_resnet_h16_b2
```

### Step 6 — Run paper benchmarks

Run the full benchmark suite (28 models × 3 terrains × 3 paths = 252 simulations):

```bash
cd utilities

# Full suite, 4 parallel workers
python run_paper_benchmarks.py --suite all -j 4 --speed 5.0 --time 30 --lead-in 10

# Run only analytical baselines (pacejka, tmeasy, linear)
python run_paper_benchmarks.py --suite analytical -j 4

# Run only static NN models
python run_paper_benchmarks.py --suite nn-static -j 4

# Run solver-success suite (all 25 models for success-rate analysis)
python run_paper_benchmarks.py --suite solver-success -j 4

# Dry run to preview jobs without executing
python run_paper_benchmarks.py --suite all --dry-run
```

Results are saved to `simulation/plots/paper_benchmark/paper_benchmark_results.csv`.

### Step 7 — Generate plots

```bash
cd utilities

# Generate all 8 paper-ready figures from benchmark results
python plot_paper_benchmarks.py

# Or specify a custom CSV / output directory
python plot_paper_benchmarks.py --csv ../simulation/plots/paper_benchmark/paper_benchmark_results.csv --out-dir ../paper_figures
```

Plots generated:
1. `solver_success_by_type.png` — Solver success rate by model type
2. `solver_success_heatmap.png` — Success rate heatmap (model × terrain × path)
3. `nn_vs_analytical.png` — Best NN vs Pacejka/TMeasy CTE comparison
4. `cte_vs_solver_success.png` — CTE vs solver success scatter
5. `static_model_comparison.png` — All static NN models CTE comparison
6. `solver_success_vs_K.png` — Temporal window size vs solver success
7. `solve_time_vs_params.png` — Solve time scaling with parameter count
8. `static_path_breakdown.png` — Per-path CTE breakdown for static models

### Step 8 — Run a single simulation

```bash
cd simulation

# NN model with visualization
python launch_decoupled.py \
  --model nn --nn-model paper_v1_mlp_16_4 \
  --terrain clay --path double_lane_change \
  --time 30 --speed 5 --lead-in 10

# Pacejka baseline comparison
python launch_decoupled.py \
  --model pacejka --terrain clay --path double_lane_change \
  --time 30 --speed 5 --lead-in 10

# Headless with CSV logging
python launch_decoupled.py \
  --model nn --nn-model paper_v1_resnet_h16_b2 \
  --terrain dirt --path sinusoidal \
  --time 30 --speed 5 --lead-in 10 \
  --no-vis --plot-dir ../plots/my_run

# Manual driving (requires Logitech G29)
python launch_decoupled.py --manual --terrain clay --speed 5
```

Key `launch_decoupled.py` flags:

| Flag | Default | Description |
|------|---------|-------------|
| `--model` | nn | Tire model: `nn`, `pacejka`, `tmeasy`, `linear` |
| `--nn-model` | v6 | Checkpoint directory under `nn_models/` |
| `--terrain` | sand | `sand`, `clay`, `dirt` |
| `--path` | lane_change | `lane_change`, `double_lane_change`, `sinusoidal` |
| `--time` | 15.0 | Simulation duration (s) |
| `--speed` | 8.0 | Target speed (m/s) |
| `--lead-in` | 0.0 | Straight lead-in before path (m) |
| `--no-vis` | — | Headless mode |
| `--no-plot` | — | Skip live plot |
| `--no-csv` | — | Skip CSV output |
| `--plot-dir` | plots | Output directory for plots/CSV |
| `--bumpiness` | 0 | Terrain roughness 0–10 |
| `--rocks` | 0 | Number of rock obstacles |
| `--safety-filter` | — | Enable DOB-CBF safety filter |
| `--manual` | — | G29 wheel control |

### Collecting closed-loop vehicle data

For collecting MPC-supervised driving data (used for paper_v2 lineage, now archived):

```bash
cd simulation

# 64 scenarios with LHS terrain sampling, 4 parallel workers
python collect_vehicle_tire_dataset.py \
  --runs 64 --jobs 4 \
  --output ../data/vehicle_mpc/lhs_v6.csv \
  --terrain-mode lhs --lhs-seed 42 \
  --path double_lane_change --time 25 --speed 5 --lead-in 10

# Fixed terrain, 12 runs
python collect_vehicle_tire_dataset.py \
  --runs 12 --jobs 4 \
  --output ../data/vehicle_mpc/clay_dlc.csv \
  --terrain clay --path double_lane_change --time 25 --speed 5
```

---

## See also

| Doc | Content |
|-----|---------|
| [Context.md](Context.md) | Long-form context — goals, architecture, known issues |
| [REPORT.md](REPORT.md) | Benchmark results and analysis |
| [simulation/README.md](simulation/README.md) | Sim nodes, launch flags, terrain YAML |
| [nn_training/new/README.md](nn_training/new/README.md) | `train_variant` / `run_experiment_grid` details |
| [data/README.md](data/README.md) | Dataset layout and CSV schema |
| [data_collection/README.md](data_collection/README.md) | Data collection tools |
| [docs/NN_TIRE_FORCE_PIPELINE.md](docs/NN_TIRE_FORCE_PIPELINE.md) | End-to-end pipeline documentation |
