# Simulation Scripts

MPC-based vehicle control on SCM deformable terrain using PyChrono.

## Architecture

**Decoupled (ZMQ)** — `chrono_sim_node.py` (physics) + controller node (control) communicate over ZMQ. Launch both via `launch_decoupled.py`.

## Quick Start

```bash
# Decoupled — NN tire model, sinusoidal path, 8 m/s
python launch_decoupled.py --model nn --path sinusoidal --speed 8 --time 30

# Manual driving with G29 wheel
python launch_decoupled.py --manual --path sinusoidal --speed 5 --time 30

# Headless (no visualization)
python launch_decoupled.py --model nn --vis-mode none --time 60
```

## Scripts

| Script | Description |
|--------|-------------|
| `chrono_sim_node.py` | Decoupled sim node — publishes VehicleState, receives ControlCommand |
| `acados_mpc_controller_node.py` | Decoupled ACADOS MPC node (Pacejka/TMeasy/Linear/NN) |
| `launch_decoupled.py` | Launcher for both decoupled nodes |
| `benchmark_tire_models.py` | Batch comparison of NN vs Pacejka across terrains/paths |

## Modules

| Module | Description |
|--------|-------------|
| `acados_mpc_solver.py` | ACADOS SQP-RTI MPC formulation (HPIPM QP solver) |
| `chrono_setup.py` | Vehicle, terrain, and marker setup helpers |
| `param_consistency.py` | Centralized vehicle params, terrain presets, bumpiness levels |
| `path_utils.py` | Path generation (lane change, double lane change, sinusoidal) |
| `reference_path.py` | ReferencePath class with closest-point re-indexing |
| `hil_messages.py` | ZMQ message definitions (VehicleState, ControlCommand, SimStatus) |
| `g29_controller.py` | Logitech G29 steering wheel interface via SDL |
| `terrain_gen.py` | Perlin noise heightmap generation for bumpy terrain |
| `safety/` | DOB-CBF safety filter for obstacle avoidance |
| `sensors/` | Rock obstacle placement utilities |

## CLI Reference

### Controller

| Flag | Default | Description |
|------|---------|-------------|
| `--linear` | — | Use Pacejka tire model |
| `--nn` | — | Use Neural Network tire model |
| `--both` | — | Compare both models side-by-side |
| `--nn-model <dir>` | v3 | NN model directory under `nn_models/` |
| `--speed <m/s>` | 5.0 | Target vehicle speed |
| `--async` | — | Run MPC in a separate thread |
| `--multiprocess` | — | Run MPC in a separate process (bypasses GIL) |
| `--manual` | — | Manual control with G29 steering wheel |

### Path

| Flag | Default | Description |
|------|---------|-------------|
| `--path <type>` | lane_change | `lane_change`, `double_lane_change`, `sinusoidal` |
| `--sine-amplitude <m>` | 2.0 | Sinusoidal path amplitude |
| `--sine-wavelength <m>` | 30.0 | Sinusoidal path wavelength |
| `--no-path-reindex` | — | Disable closest-point path re-indexing |

### Terrain

| Flag | Default | Description |
|------|---------|-------------|
| `--terrain <preset>` | sand | Soil type: `sand`, `clay`, `dirt` |
| `--bumpiness <0-10>` | 0 | Terrain bumpiness (0=flat, 10=extreme) |
| `--terrain-config <yaml>` | — | Custom YAML soil params (overrides `--terrain`) |
| `--random-terrain` | — | Random soil within NN training range |
| `--terrain-n <float>` | — | Override sinkage exponent n |

### Obstacles & Safety

| Flag | Default | Description |
|------|---------|-------------|
| `--rocks <n>` | 0 | Number of rock obstacles |
| `--safety-filter` | — | Enable DOB-CBF safety filter |
| `--cbf-alpha <float>` | 5.0 | CBF aggressiveness |
| `--safety-buffer <m>` | 0.25 | Extra obstacle buffer distance |

### Visualization

| Flag | Default | Description |
|------|---------|-------------|
| `--no-vis` | — | Disable all visualization |
| `--vis-mode <mode>` | irrlicht | `irrlicht`, `sensor`, `both` |
| `--time <s>` | 15.0 | Simulation duration |

## Examples

```bash
# Clay terrain, bumpiness 5, tight sinusoidal
python launch_decoupled.py --model nn --terrain clay --bumpiness 5 --path sinusoidal --sine-wavelength 20

# Obstacle avoidance with safety filter (ground truth obstacles)
python launch_decoupled.py --model nn --rocks 10 --safety-filter

# Decoupled mode with teleop delay compensation
python launch_decoupled.py --model nn --teleop-delay 0.2 --speed 8

# Headless batch benchmark
python benchmark_tire_models.py --quick
```
