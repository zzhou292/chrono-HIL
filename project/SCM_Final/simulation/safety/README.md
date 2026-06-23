# Safety Module — Swappable Safety Filters

Safety-filter flavors share the same `filter(...) -> SafetyFilterResult`
API and are selectable via `--safety-filter --safety-flavor <name>`.
This is an in-process plugin boundary inside `chrono_sim_node.py`: the
sim gathers the latest command, applies optional latency, calls the
selected safety filter, and then passes the filtered command to Chrono.

- **`dob_cbf` (the only shipped flavor):** Single-step DOB-CBF-QP filter.
  This is the most natural human-in-the-loop/shared-control safety
  mechanism because the QP minimizes deviation from the operator command
  while enforcing obstacle constraints.

The predictive **`mppi`** and SLSQP **`nmpc`** shields (and the shared
`surrogate_dynamics.py` / `predictive_shield.py` rollout) were **archived
2026-06-21** to `archive/2026-06-21_mppi_nmpc_removal/`. `make_safety_filter`
and `--safety-flavor` now reject those names with a pointer to the archive.
The registry stays swappable, but DOB-CBF is the shipped instance. The
architecture notes below are retained for the archived shields.

## MPPI / NMPC architecture (archived 2026-06-21)

Inspired by sampling-based MPC for off-road driving (e.g., Williams et al.)
but adapted to a *safety filter* setting: the operator/autonomous command
is the *reference*, and the shield projects onto a safe set defined by the
NN-surrogate rollout cost.

## DOB-CBF architecture

Inspired by the DOB-CBF (Disturbance Observer Control Barrier Function)
from the ROS2 bridge (`DobCBFHelper.py`). The DOB-CBF implementation can
run with the NN tire model enabled or with `--no-safety-nn`, which falls
back to kinematic steering authority and fixed longitudinal acceleration
limits. The NN-on/NN-off ablation isolates how much terrain-aware force
prediction helps the same intent-preserving filter.

### Safety Constraints

1. **Obstacle Avoidance (CBF)**
   - Barrier function: $h(x) = \|p - p_{obs}\|^2 - r_{safe}^2$
   - Second-order CBF condition: $\ddot{h} + \alpha_2 \dot{h} + \alpha_1 h \geq 0$
   - Linearized w.r.t. control inputs (steering rate, acceleration)
   - Multiple obstacles → multiple linear constraints

2. **Terrain-Aware Speed Limiting**
   - Roughness-based: $v_{max} = v_{limit} / (1 + 10 \cdot roughness)$
   - NN traction-based: queries the same neural network as the MPC to find 
     lateral force capacity, then computes max cornering speed
   - Uses 70% safety margin on NN-predicted traction limits

3. **Latency Compensation**
   - Discrete predictor with FIFO buffers (matches CCTA bridge paper)
   - Derivative + proportional feedback gains (k1=0.6, k2=2.0)
   - Compensates actuation delay to filter with actual vehicle state

### QP Formulation

At each step, solves:
$$\min_{u} \|u - u_{desired}\|^2 \quad \text{s.t.} \quad A_{cbf} u \leq b_{cbf}$$

where $u = [\dot{\delta}, a_x]$ (steering rate and longitudinal acceleration).
This yields the minimally-invasive safe control — the driver's input is only 
modified when necessary to prevent constraint violation.

## Usage

```python
from safety import CBFSafetyFilter

cbf = CBFSafetyFilter(
    vehicle_params=params,
    nn_casadi=nn_model,       # Optional: enables NN traction limits
    cbf_alpha=3.0,            # Barrier function gain
    obstacle_buffer=1.0,      # Extra margin around obstacles (m)
    delay_steps=5,            # Latency compensation steps
)

# In loop:
result = cbf.filter(
    desired_steering, desired_throttle, desired_brake,
    vehicle_state, obstacles, terrain_roughness
)

# Use filtered outputs
steering = result.steering
throttle = result.throttle
braking  = result.braking

# Check diagnostics
print(f"Modified: {result.was_modified}, Active: {result.active_constraints}")
print(f"Terrain v_max: {result.v_max_terrain:.1f} m/s")
```

## CLI Flags

```bash
# Enable safety filter with default settings
python launch_decoupled.py --manual --safety-filter

# Custom CBF parameters
python launch_decoupled.py --manual --safety-filter \
    --cbf-alpha 5.0 --safety-buffer 2.0 --delay-steps 10

# Full demo: manual + rocks + safety (ground truth obstacles)
python launch_decoupled.py --manual --rocks 20 \
    --safety-filter --terrain-config terrain_configs/rough_trail.yaml
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cbf_alpha` | 3.0 | CBF barrier gain (higher = more conservative) |
| `cbf_alpha2` | 5.0 | Second-order CBF gain |
| `obstacle_buffer` | 1.0 m | Extra margin around obstacles |
| `vehicle_radius` | 2.5 m | Effective vehicle collision radius |
| `max_speed` | 15.0 m/s | Absolute speed limit |
| `delay_steps` | 5 | Actuation delay for compensator |
| `control_dt` | 0.02 s | Safety filter update period |

## Collision Warning (`collision_warning.py`)

A parallel advisory channel that runs alongside whichever safety
filter is selected (or none). The warning **never modifies commands**
— it emits a discrete severity signal {GREEN, YELLOW, ORANGE, RED}
that downstream HMI code can consume.

Built around the same swappable-factory pattern as the safety
filters:

```python
from safety.collision_warning import make_collision_warning_system

warning = make_collision_warning_system(
    flavor='ttc',
    tire_model_dir='nn_models/rig_rate_64_32',
    reaction_time=0.5,
)
result = warning.evaluate(vehicle_state, obstacles, command_age_s)
# result.severity, result.ttc_s, result.d_stop, result.d_to_lead
```

Default flavor (`'ttc'`) computes required stopping distance as
$d_\mathrm{stop} = v\tau_r + v^2 / (2\,a_b(\hat n))$ with:

- $a_b(\hat n)$ — **analytical** brake deceleration. At init time the
  module loads the deployed rig tire surrogate (`rig_rate_64_32`),
  sweeps braking slip $\kappa \in [-0.4, 0]$ on a per-axle grid for
  each $\hat n \in [0.40, 1.30]$ on a 0.05 step, and tabulates the
  peak available longitudinal force per axle. The live $\hat n$ from
  the terrain estimator indexes this table at runtime. Values run
  from ~2.3 m/s² on soft clay to ~4.7 m/s² on firm sand.
- $\tau_r$ — operator reaction-time budget, inflated by an EMA of
  recent one-way delays plus $k\sigma$ on a sliding-window jitter
  estimator. Higher latency and higher jitter both fire the warning
  earlier.

### CLI

```bash
# Enable the warning alongside any other safety filter
python launch_decoupled.py --manual --collision-warning \
    --cw-tire-model nn_models/rig_rate_64_32 \
    --cw-reaction-time 0.5 \
    --collision-warning-csv warnings.csv
```

Per-tick severity is written to the requested CSV; severity
transitions are also printed to the sim-node console (and, when the
Irrlicht visualiser is active, shown as a text banner). When the
controller is also running with `--terrain-estimator`, the live
$\hat n$ on every `ControlCommand` is forwarded to the warning so the
brake-decel estimate adapts to whatever the estimator currently
believes; otherwise the warning falls back to the `--terrain`
preset's nominal $n$.

Validators:

- `benchmarking/collision_warning_test.py` — terrain × latency sweep
  at fixed throttle into a single rock; verifies lead time grows with
  softer soil and longer delay.
- `benchmarking/brake_test.py` — 27 actual Chrono SCM brake stops;
  the analytical $a_b(\hat n)$ table is validated to within 0.17 m
  mean absolute stopping-distance error.
