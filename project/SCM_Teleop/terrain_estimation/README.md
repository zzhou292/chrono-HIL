# Terrain Estimation

UKF-based online terrain parameter estimation modules.

## Scripts

### `terrain_estimator_v3.py` - Trajectory Matching Estimator (Recommended)

Uses trajectory prediction matching rather than force residuals for more robust estimation.

**Features:**
- Estimates sinkage exponent `n` online
- Uses measurement noise covariance tuning
- Exponential moving average smoothing
- Works with NN tire model in MPC

**Usage:**
Imported by `simulation/dallas_chrono_demo.py` when using `--ukf` flag.

---

### `terrain_estimator_v2.py` - Force-Based UKF

Previous version using force residual matching.

---

### `terrain_estimator.py` - Original UKF (Legacy)

First implementation, provided for reference.

---

### `test_estimator_random.py` - Randomized Testing

Test terrain estimator on randomized soil parameters.

**Usage:**
```bash
# Run from terrain_estimation directory
python test_estimator_random.py
```

**Note:** Requires `simulation/dallas_chrono_demo.py` to be runnable with PyChrono.
