# SCM Teleop - Deformable Terrain Vehicle Simulation

MPC-based vehicle control on SCM (Soil Contact Model) deformable terrain using PyChrono, with neural network tire force prediction and online terrain estimation.

TERRAIN PARAMS:
https://fada.birzeit.edu/bitstream/20.500.11889/7935/1/An%20equivalent%20soil%20mechanics%20formulation%20for%20rigid%20wheels%20in%20deformable%20terrain%2C%20with%20application%20to%20planetary%20exploration%20rovers.pdf


## Quick Start

```bash
# Activate conda environment with PyChrono
conda activate chrono

# Run simulation with NN tire model
cd simulation
python dallas_chrono_demo.py --nn --terrain sand --path sinusoidal
```

## Directory Structure

```
SCM_Teleop/
├── simulation/          # Main simulation scripts (Python/PyChrono)
│   ├── dallas_chrono_demo.py   # Primary MPC vehicle simulation
│   ├── dallas_mpc.py           # MPC controller implementation
│   ├── benchmark_tire_models.py # Tire model comparison benchmark
│   └── param_consistency.py    # Vehicle/terrain parameter definitions
│
├── nn_training/         # NN model training and validation
│   ├── train_terrain_nn.py     # Train tire force NN
│   ├── validate_terrain_nn.py  # Validate trained model
│   └── diagnose_nn_accuracy.py # NN accuracy diagnostics
│
├── terrain_estimation/  # UKF-based terrain parameter estimation
│   ├── terrain_estimator_v3.py # Trajectory matching estimator
│   └── test_estimator_random.py
│
├── terrain_configs/     # YAML terrain configuration files
│   ├── sand.yaml, clay.yaml, dirt.yaml, asphalt.yaml
│   └── training_mean.yaml
│
├── nn_models/           # Trained NN model checkpoints
│   ├── v3/              # Recommended (64x32, 50k samples)
│   ├── v2/, fast/, legacy/
│
├── cpp_collect/         # C++ SCM data collection tools
│   ├── collect_scm_data.cpp      # Original (visualization support)
│   └── collect_scm_data_fast.cpp # Fast parallel (recommended)
│
├── cpp_hil/             # C++ HIL demo
│   └── proj_HIL_scm_teleop.cpp
│
└── docs/                # Documentation
    ├── NN_TIRE_FORCE_PIPELINE.md
    └── API_FIXES.txt
```

## Common Usage

### MPC Vehicle Simulation

```bash
cd simulation

# Basic run with Pacejka tire model
python dallas_chrono_demo.py --linear

# Run with NN tire model on clay terrain
python dallas_chrono_demo.py --nn --terrain clay

# Sinusoidal path with tight turns
python dallas_chrono_demo.py --nn --path sinusoidal --sine-wavelength 25

# Enable terrain estimation
python dallas_chrono_demo.py --nn --ukf --random-terrain

# Headless benchmark run
python dallas_chrono_demo.py --nn --no-vis --time 30
```

### Tire Model Benchmarking

```bash
cd simulation

# Full benchmark (parallel, 20 runs per config)
python benchmark_tire_models.py

# Quick test (3 runs)
python benchmark_tire_models.py --quick

# With measurement noise
python benchmark_tire_models.py --noise
```

### Train New NN Model

```bash
# Generate data using C++ collector (from build directory)
# Use collect_scm_data_fast for parallel collection
./collect_scm_data_fast 50000 scm_data.csv --parallel-only --batch-size 1000

# Or use original (slower, but supports visualization)
./collect_scm_data 5000 scm_data.csv --visualize

# Train model
cd /path/to/SCM_Teleop/nn_training
python train_terrain_nn.py --data /path/to/scm_data.csv --hidden 64 32 --output_dir ../nn_models/new

# Validate
python validate_terrain_nn.py --model ../nn_models/new/best_terrain_nn.pt
```

## Key Components

### MPC Controller
- 8-state bicycle model (Dallas et al.)
- 10Hz control rate, 2.5s horizon
- CasADi/IPOPT nonlinear optimization
- Supports async and multiprocess modes

### Tire Models
- **Pacejka (Linear)**: Magic Formula tire model
- **Neural Network**: Trained on SCM simulation data, terrain-adaptive

### Terrain Types
- **sand**: Soft, low cohesion, deep sinkage
- **clay**: High cohesion, sticky
- **dirt**: Moderate (packed gravel)
- **asphalt**: Hard surface, minimal sinkage

### Path Types
- `lane_change`: Single lane change maneuver
- `double_lane_change`: ISO lane change test
- `sinusoidal`: Continuous sinusoidal path

## Dependencies

- PyChrono with Vehicle, Sensor, Irrlicht modules
- CasADi, IPOPT
- PyTorch
- NumPy, Matplotlib, PyYAML

## See Also

- [simulation/README.md](simulation/README.md) - Detailed simulation script documentation
- [nn_training/README.md](nn_training/README.md) - NN training instructions
- [terrain_configs/README.md](terrain_configs/README.md) - Terrain parameter reference
- [docs/NN_TIRE_FORCE_PIPELINE.md](docs/NN_TIRE_FORCE_PIPELINE.md) - Full pipeline documentation
