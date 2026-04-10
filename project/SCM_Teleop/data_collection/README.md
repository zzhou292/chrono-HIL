# C++ SCM Data Collection

C++ tools for collecting tire force data from SCM deformable terrain using Chrono's ChTireTestRig. Data is used to train neural network tire models.

## Files

| File | Description |
|------|-------------|
| `collect_scm_data.cpp` | Original single-threaded data collector with visualization support |
| `collect_scm_data_fast.cpp` | Optimized parallel collector with OpenMP, subprocess batching, and memory management |
| `collect_temporal_data.cpp` | Time-series data collector for temporal NN training (polynomial slip angle profiles) |

## Building

Both targets are built as part of the SCM_Teleop CMake project:

```bash
cd /path/to/chrono-HIL/build
cmake ..
cmake --build . --target collect_scm_data collect_scm_data_fast collect_temporal_data
```

---

## collect_scm_data (Original)

Single-threaded data collection with optional Irrlicht visualization.

**Usage:**
```bash
./collect_scm_data [num_samples] [output.csv] [--visualize]
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_samples` | 100 | Number of samples to collect |
| `output.csv` | scm_training_data.csv | Output CSV filename |
| `--visualize`, `-v` | off | Enable Irrlicht visualization |

**Example:**
```bash
./collect_scm_data 1000 training_data.csv
./collect_scm_data 10 debug.csv --visualize
```

---

## collect_scm_data_fast (Recommended)

Optimized parallel collector with 5-10x speedup. Supports continuous collection and memory-safe batch processing.

**Usage:**
```bash
./collect_scm_data_fast [num_samples] [output.csv] [options]
```

**Parameters:**

| Parameter | Short | Default | Description |
|-----------|-------|---------|-------------|
| `num_samples` | - | continuous | Number of samples (omit for unlimited) |
| `output.csv` | - | scm_data.csv | Output CSV filename |
| `--accurate` | `-a` | - | Use accurate (slow) settings, single-threaded |
| `--parallel-only` | `-p` | - | Accurate settings with parallelization |
| `--threads N` | `-t N` | auto | Number of OpenMP threads (0=auto) |
| `--batch-size N` | `-b N` | - | Samples per subprocess (prevents memory leaks) |
| `--continuous` | `-c` | - | Run until Ctrl+C |

**Modes:**

| Mode | Timestep | Sim Time | Threads | Use Case |
|------|----------|----------|---------|----------|
| Fast (default) | 5e-4 s | 2.0 s | parallel | Quick data generation |
| Accurate (`-a`) | 2e-4 s | 4.0 s | single | High-fidelity samples |
| Parallel-only (`-p`) | 2e-4 s | 4.0 s | parallel | Best accuracy at speed |

**Examples:**
```bash
# Quick 10k samples in fast mode
./collect_scm_data_fast 10000 fast_data.csv

# Accurate parallel collection with memory batching
./collect_scm_data_fast 50000 training.csv --parallel-only --batch-size 1000

# Continuous collection until interrupted
./collect_scm_data_fast scm_data.csv --continuous

# Specify thread count
./collect_scm_data_fast 5000 data.csv -t 8
```

**Memory Management:**

For large runs (>5000 samples), use `--batch-size` to prevent memory accumulation:

```bash
./collect_scm_data_fast 50000 large_dataset.csv --batch-size 1000
```

Each batch runs in a subprocess that exits after completion, fully releasing memory.

---

## collect_temporal_data (Temporal/Time-Series)

Collects time-series tire force data for temporal NN training. Each scenario runs a 2.0s recording with a polynomial slip angle profile (degree 1-3), producing ~400 timesteps at 5ms intervals. This captures transient tire dynamics (tire relaxation, soil deformation history) that steady-state collection cannot.

**Usage:**
```bash
./collect_temporal_data [num_scenarios] [output.csv] [options]
```

**Parameters:**

| Parameter | Short | Default | Description |
|-----------|-------|---------|-------------|
| `num_scenarios` | - | continuous | Number of scenarios (omit for unlimited) |
| `output.csv` | - | scm_temporal_data.csv | Output CSV filename |
| `--sequential` | `-s` | - | Single-threaded |
| `--threads N` | `-t N` | auto | Number of OpenMP threads (0=auto) |
| `--batch-size N` | `-b N` | - | Subprocess batch size (prevents OOM) |
| `--continuous` | `-c` | - | Run until Ctrl+C |

**Slip angle profiles (per scenario):**

| Degree | Proportion | Dynamics |
|--------|-----------|----------|
| 1 (linear) | 33% | Constant steering rate |
| 2 (quadratic) | 33% | Accelerating/decelerating steering |
| 3 (cubic) | 33% | Rich transient dynamics (jerk) |

**Examples:**
```bash
# Collect 3000 scenarios (recommended minimum for K=10)
./collect_temporal_data 3000 temporal_v2_timeseries.csv --batch-size 200

# Continuous collection
./collect_temporal_data scm_temporal_data.csv --continuous

# Single-threaded for debugging
./collect_temporal_data 10 debug_temporal.csv --sequential
```

**Output:** ~400 rows per scenario. For 3000 scenarios, expect ~1.2M rows.

---

## Output Format

CSV with columns:

| Column | Units | Description |
|--------|-------|-------------|
| `Fz` | N | Normal (vertical) force |
| `slip_angle` | rad | Tire slip angle |
| `longitudinal_slip` | ratio | Slip ratio (-0.12 to 0.12) |
| `camber_angle` | rad | Camber angle |
| `velocity` | m/s | Wheel velocity |
| `bekker_Kphi` | Pa | Bekker friction modulus |
| `bekker_Kc` | Pa | Bekker cohesion modulus |
| `bekker_n` | - | Bekker sinkage exponent |
| `mohr_cohesion` | Pa | Soil cohesion |
| `mohr_friction` | deg | Internal friction angle |
| `janosi_shear` | m | Shear displacement coefficient |
| `Fy` | N | Lateral force (output) |
| `Fx` | N | Longitudinal force (output) |

## Parameter Ranges (Latin Hypercube Sampling)

| Parameter | Min | Max |
|-----------|-----|-----|
| `slip_angle` | -0.6 rad | 0.6 rad |
| `longitudinal_slip` | -0.12 | 0.12 |
| `velocity` | 0.5 m/s | 10.5 m/s |
| `vertical_load` | 2500 N | 7500 N |
| `camber_angle` | -5° | 5° |
| `bekker_Kphi` | 2.0 MPa | 4.0 MPa |
| `bekker_Kc` | 0 | 10 kPa |
| `bekker_n` | 0.3 | 1.3 |
| `mohr_cohesion` | 0 | 5 kPa |
| `mohr_friction` | 25° | 45° |
| `janosi_shear` | 0.01 m | 0.05 m |

## Why C++?

ChTireTestRig is only available in C++ Chrono, not in Python bindings. It provides proper tire-terrain contact simulation for accurate force measurements on deformable terrain.
