# Safety Module — NN-Informed CBF Safety Filter

Control Barrier Function (CBF) safety filter that prevents collisions and 
enforces terrain-aware speed limits during manual or autonomous driving.

## Architecture

Inspired by the DOB-CBF (Disturbance Observer Control Barrier Function) from 
the ROS2 bridge (`DobCBFHelper.py`), adapted for off-road driving with 
deformable terrain.

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
python scm_hmmwv_demo.py --nn --manual --safety-filter

# Custom CBF parameters
python scm_hmmwv_demo.py --nn --manual --safety-filter \
    --cbf-alpha 5.0 --safety-buffer 2.0 --delay-steps 10

# Full demo: manual + lidar + rocks + safety
python scm_hmmwv_demo.py --nn --manual --lidar --rocks 20 \
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
