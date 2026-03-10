# Simulation Scripts

This folder contains the main simulation and benchmarking scripts for MPC-based vehicle control on deformable terrain using PyChrono.

## Main Scripts

### `dallas_chrono_demo.py` - MPC Vehicle Simulation

Full vehicle simulation with Model Predictive Control (MPC) for path tracking on SCM deformable terrain.

**Basic Usage:**
```bash
# Run with linear (Pacejka) tire model
python dallas_chrono_demo.py --linear

# Run with Neural Network tire model
python dallas_chrono_demo.py --nn

# Run with NN + UKF terrain estimation
python dallas_chrono_demo.py --nn --ukf
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--linear` | - | Use Pacejka (linear) tire model |
| `--nn` | - | Use Neural Network tire model |
| `--both` | - | Compare both models side-by-side |
| `--no-vis` | - | Disable Irrlicht visualization (for batch runs) |
| `--time <s>` | 15.0 | Simulation duration in seconds |
| `--speed <m/s>` | 5.0 | Target vehicle speed |

**Path Options:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--path <type>` | lane_change | Path type: `lane_change`, `double_lane_change`, `slalom`, `sinusoidal` |
| `--sine-amplitude <m>` | 2.0 | Sinusoidal path amplitude |
| `--sine-wavelength <m>` | 30.0 | Sinusoidal path wavelength (lower = tighter turns) |
| `--no-path-reindex` | - | Disable closest-point path re-indexing |

**Terrain Options:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--terrain <preset>` | sand | Terrain preset: `sand`, `clay`, `dirt`, `asphalt` |
| `--terrain-config <path>` | - | Load terrain from YAML config file |
| `--random-terrain` | - | Random soil params within NN training range |
| `--terrain-n <value>` | - | Override sinkage exponent n (1.0-1.4) |
| `--bump <m>` | 0.0 | Terrain bump amplitude (0 = flat) |
| `--bump-wavelength <m>` | 20.0 | Distance between bump peaks |

**MPC Options:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--nn-model <version>` | v3 | NN model: `legacy`, `fast`, `v2`, `v3` |
| `--async` | - | Run MPC in separate thread |
| `--multiprocess` | - | Run MPC in separate process (bypasses GIL) |
| `--ukf` | - | Enable UKF terrain estimation |
| `--ukf-n-init <value>` | 0.7 | UKF initial guess for n |
| `--manual` | - | Manual control with G29 steering wheel |

**Examples:**
```bash
# Sinusoidal path with tight turns on clay terrain
python dallas_chrono_demo.py --nn --path sinusoidal --sine-wavelength 25 --terrain clay

# Headless benchmark on random terrain
python dallas_chrono_demo.py --nn --no-vis --random-terrain --time 30

# Use specific YAML terrain config
python dallas_chrono_demo.py --nn --terrain-config ../terrain_configs/training_mean.yaml
```

---

### `benchmark_tire_models.py` - Tire Model Comparison Benchmark

Compares NN vs Pacejka tire models across terrain presets and path types with statistical analysis.

**Basic Usage:**
```bash
# Full benchmark (parallel, 20 runs per configuration)
python benchmark_tire_models.py

# Quick test (3 runs, shorter simulation)
python benchmark_tire_models.py --quick

# Sequential with visualization
python benchmark_tire_models.py --sequential --vis
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--runs <n>` | 20 | Number of runs per terrain/path/model combination |
| `--time <s>` | 30.0 | Simulation time per run |
| `--rms-start <s>` | 5.0 | Start time for RMS calculation (excludes startup transient) |
| `--rms-end <s>` | 25.0 | End time for RMS calculation |
| `--speed <m/s>` | 5.0 | Target vehicle speed |
| `--workers <n>` | CPU-1 | Number of parallel workers |
| `--sequential` | - | Run sequentially (slower, allows --vis) |
| `--vis` | - | Enable visualization (requires --sequential) |
| `--noise` | - | Enable measurement noise (more realistic) |
| `--no-reindex` | - | Disable closest-point path re-indexing |
| `--output <file>` | benchmark_results.png | Output plot filename |
| `--quick` | - | Quick mode: 3 runs, 15s sim, [3-12]s RMS window |

**Output:**
- `benchmark_results.png` - Error bar comparison plot
- `benchmark_results_heatmap.png` - Heatmap of improvements
- `benchmark_raw_data.npz` - Raw data for further analysis

---

### `benchmark_soil_sweep.py` - Soil Parameter Sweep

Benchmarks controllers across randomly sampled soil parameters within the NN training range.

---

### `param_consistency.py` - Parameter Utilities

Defines vehicle parameters (HMMWV) and NN training ranges. Used by other scripts.

---

### `dallas_mpc.py` - MPC Controller Module

Core MPC implementation using CasADi/IPOPT. Imported by `dallas_chrono_demo.py`.
