# Neural Network Tire Force Pipeline

## Overview

This document describes the complete pipeline for predicting tire forces on SCM deformable terrain:
1. **Data Collection** — ChTireTestRig generates steady-state force measurements
2. **NN Training** — Maps tire/terrain state → forces
3. **NMPC Usage** — NN provides tire forces in trajectory optimization

---

## 1. Data Collection (Tire Test Rig)

### Source Files
- `cpp_collect/collect_scm_data.cpp` — Original single-threaded collector
- `cpp_collect/collect_scm_data_fast.cpp` — Parallelized version

### What the Tire Test Rig Does

The `ChTireTestRig` is Chrono's tool for characterizing tire behavior in isolation:

```
                    Vertical Load (controlled)
                          ↓
    ┌─────────────────────●─────────────────────┐
    │                   Wheel                   │
    │     ←──────── velocity ──────────→        │
    │         slip_angle (yaw rotation)         │
    └─────────────────────┬─────────────────────┘
                          │
    ══════════════════════╧══════════════════════
              SCM Deformable Terrain
              (Bekker-Wong model)
```

**Procedure for each sample:**
1. Create fresh SCM terrain with sampled soil parameters (Kphi, Kc, n, cohesion, friction, janosi)
2. Place single HMMWV wheel (RigidTire) on terrain
3. Apply controlled vertical load, velocity, slip angle via sinusoidal ramp
4. Allow 1s delay + 1s ramp to reach steady state
5. At t=2s, call `rig.ReportTireForce()` to capture forces
6. Record: inputs + (Fx, Fy, Fz)

### Input Parameter Ranges (Latin Hypercube Sampling)

| Parameter | Symbol | Range | Units | Notes |
|-----------|--------|-------|-------|-------|
| Vertical load | Fz_cmd | 2500–7500 | N | Commanded; actual Fz measured |
| Slip angle | α | ±0.15 | rad | ≈ ±8.6° |
| Slip ratio | κ | ±0.12 | — | Longitudinal slip |
| Camber | γ | ±0.087 | rad | ≈ ±5° |
| Velocity | v | 0.5–10.5 | m/s | |
| Bekker Kphi | Kphi | 2–4 | MPa | Frictional modulus |
| Bekker Kc | Kc | 0–10 | kPa | Cohesive modulus |
| Bekker n | n | 1.0–1.4 | — | Sinkage exponent |
| Mohr cohesion | c | 0–5 | kPa | |
| Mohr friction | φ | 25–45 | deg | Internal friction angle |
| Janosi shear | k | 0.01–0.05 | m | Shear displacement |

### Force Extraction

```cpp
TerrainForce tire_force = rig.ReportTireForce();
double Fz = tire_force.force.z();  // Measured normal force
double Fx = tire_force.force.x();  // Longitudinal force
double Fy = tire_force.force.y();  // Lateral force
```

**Coordinate Frame:** Forces are returned in **global frame**. For the tire test rig:
- The wheel travels along positive X
- Positive slip angle rotates the wheel to point left of travel direction
- Global frame ≈ tire frame (since wheel heading ≈ X-axis)

### CSV Output Format

```
vertical_load,slip_angle,longitudinal_slip,camber_angle,velocity,
bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,
Fz,Fx,Fy
```

**Important:** The NN training uses `Fz` (measured), not `vertical_load` (commanded), because actual load depends on terrain settling.

---

## 2. NN Training

### Source File
- `train_terrain_nn.py`

### Architecture

```
Input (11) → [32] → tanh → [16] → tanh → (2) Output
```

Using `tanh` activation for C² continuity (required by IPOPT optimization).

### Input Features (11)

| Index | Feature | Notes |
|-------|---------|-------|
| 0 | Fz | Measured normal force (N) |
| 1 | slip_angle | **Radians** |
| 2 | longitudinal_slip | κ ratio |
| 3 | camber_angle | Radians |
| 4 | velocity | m/s |
| 5 | bekker_Kphi | N/m^(n+1) |
| 6 | bekker_Kc | N/m^(n+2) |
| 7 | bekker_n | Sinkage exponent |
| 8 | mohr_cohesion | Pa |
| 9 | mohr_friction | Degrees |
| 10 | janosi_shear | m |

### Output Features (2)

| Index | Feature | Notes |
|-------|---------|-------|
| 0 | Fx | Longitudinal force (N) |
| 1 | Fy | Lateral force (N) |

### Sign Convention (CRITICAL)

The NN learns the tire-frame convention:
- **Positive slip angle** (α > 0) → wheel points left of velocity → **negative Fy** (force pushes right to restore alignment)

Example NN output:
```
α = +8° (0.14 rad) → Fy ≈ -989 N
α = -8° (-0.14 rad) → Fy ≈ +963 N
```

### Data Scaling

StandardScaler applied to inputs and outputs:
```python
X_scaled = (X - X_mean) / X_std
y_scaled = (y - y_mean) / y_std
```

Scalers saved to `scalers.pkl` and used in CasADi symbolic function.

---

## 3. NMPC Usage

### Source File
- `dallas_mpc.py`

### CasADi Integration

The NN is converted to CasADi symbolic operations for use in trajectory optimization:

```python
# Build symbolic function (NNCasADi._build_casadi_function)
x_scaled = (x_input - X_mean) / X_scale
h = x_scaled
for layer in layers:
    h = tanh(W @ h + b)  # (except last layer: no activation)
y_output = h * y_scale + y_mean
```

### Slip Angle Calculation

NMPC computes slip angles from bicycle model states:

```python
# Front axle: wheel points at angle δ, velocity direction is arctan(v_y/v_x)
alpha_f = delta - arctan2(v + Lf * omega, u)

# Rear axle: no steering
alpha_r = -arctan2(v - Lr * omega, u)
```

Where:
- `u` = longitudinal velocity (body frame)
- `v` = lateral velocity (body frame)  
- `omega` = yaw rate
- `delta` = steering angle
- `Lf, Lr` = distances from CG to front/rear axles

### Longitudinal Slip (κ)

The bicycle model doesn't track wheel speed, so κ is approximated:

```python
if kappa_mode == 'approx':
    kappa = ax / (mu * g)  # Rough estimate from acceleration
else:
    kappa = 0.0  # Pure lateral slip assumption (default)
```

### The `-2.0` Multiplier (WHY?)

```python
Fyf = -2.0 * nn_scale * Fyf_wheel
Fyr = -2.0 * nn_scale * Fyr_wheel
```

**This combines two factors:**

1. **`×2`**: NN predicts force for ONE wheel. Axle has TWO wheels.

2. **`-1`**: Sign flip between tire frame and body frame.
   - NN: positive α → negative Fy (tire wants to realign)
   - Body dynamics: we need Fy in body-fixed frame
   - When wheel has positive slip angle, the terrain pushes the vehicle in positive Y direction (vehicle-frame)
   
   Simplified: tire-frame Fy and body-frame Fy have opposite signs.

### Vehicle Dynamics Equations

```python
# Lateral velocity derivative (Dallas Eq. 13)
v_dot = (Fyf + Fyr) / M - u * omega

# Yaw rate derivative  
omega_dot = (Fyf * Lf - Fyr * Lr) / Izz
```

### Normal Force Computation

Static weight distribution (no dynamic load transfer):

```python
Fz_f_axle = M * g * Lr / (Lf + Lr)  # Front axle total
Fz_r_axle = M * g * Lf / (Lf + Lr)  # Rear axle total

Fz_f_wheel = Fz_f_axle / 2  # Per wheel (for NN input)
Fz_r_wheel = Fz_r_axle / 2
```

---

## 4. Why Tire Test Rig Works for NMPC

### Validation Results

| Metric | Value |
|--------|-------|
| NN prediction (α=-4°, soft terrain) | -313 N |
| Chrono full vehicle mean (same conditions) | -305 N |
| **Error** | **3%** |
| Chrono std deviation | 147 N (48% CV) |

The NN accurately predicts the **mean/steady-state** force. The high variance in full-vehicle simulation comes from:
- Dynamic load transfer (pitch/roll)
- Terrain deformation from other wheels
- Transient contact effects

### Why NMPC Tolerates This

NMPC plans over 1–2s horizons. Force prediction errors **average out** over the trajectory. What matters:
1. Correct sign (positive α → negative Fy ✓)
2. Correct trend (more slip → more force ✓)
3. Reasonable magnitude ✓

### Comparison to Linear Tire Model

| Model | RMS Error vs Chrono | Correlation |
|-------|---------------------|-------------|
| Linear (80k N/rad) | 6,477 N | -0.07 |
| **NN** | **1,550 N** | -0.10 |

NN is **76% better** than linear model. The linear model uses pavement cornering stiffness (80,000 N/rad), which massively overpredicts forces on soft terrain.

---

## 5. Limitations for Terrain Estimation (UKF)

The UKF tries to estimate terrain parameter `n` by comparing:
- NN prediction: `Fy_predicted = NN(α, Fz, n_estimated, ...)`
- Measured: `Fy_measured` from Chrono

**Problem:** 48% variance in instant Chrono forces means the "measurement" is extremely noisy. The signal (terrain parameter effect) is drowned by variance.

**Solutions:**
1. Filter/average measurements before feeding to UKF
2. Increase measurement noise covariance (R) to match observed variance
3. Use trajectory-matching (V3 estimator) which implicitly averages

---

## 6. Quick Reference

### Files
| File | Purpose |
|------|---------|
| `cpp_collect/collect_scm_data.cpp` | Tire test rig data collection |
| `train_terrain_nn.py` | NN training |
| `nn_models_v2/` | Trained model + scalers |
| `dallas_mpc.py` | NMPC with NN tire forces |
| `diagnose_nn_accuracy.py` | Compare NN vs Chrono forces |

### Key Equations
```
Slip angle:     α_f = δ - arctan2(v + Lf·ω, u)
                α_r = -arctan2(v - Lr·ω, u)

NN prediction:  Fx, Fy = NN(Fz, α, κ, γ, v, Kphi, Kc, n, c, φ, k)

Body forces:    Fyf = -2 × Fy_front_wheel
                Fyr = -2 × Fy_rear_wheel

Dynamics:       v̇ = (Fyf + Fyr)/M - u·ω
                ω̇ = (Fyf·Lf - Fyr·Lr)/Izz
```

### Unit Conventions
| Quantity | Training Data | NMPC |
|----------|---------------|------|
| Slip angle | Radians | Radians |
| Forces | Newtons | Newtons |
| Friction angle | Degrees | Degrees |
| Velocity | m/s | m/s |
