# Terrain Parameter Estimation: Alternative Approaches

## Problem Analysis

The Dallas et al. paper worked on a **single terrain type** (sandy loam). Our goal is harder:
estimate `n` across **different terrain classes** (clay, sand, dirt) with varying Bekker parameters.

**Current NN Limitations:**
- Training data: n ∈ [1.0, 1.4], Kphi ∈ [2M, 4M], phi ∈ [25°, 45°]
- Clay (n=0.5) is complete extrapolation
- NN learns correlated patterns, not underlying physics

---

## Approach 1: Expanded Training Data (Recommended First Step)

**Idea**: Retrain the NN with much wider parameter ranges.

```python
# New LHS ranges
EXPANDED_RANGES = {
    "bekker_n": (0.3, 1.5),           # Was 1.0-1.4
    "bekker_Kphi": (0.5e6, 5.0e6),    # Was 2M-4M
    "bekker_Kc": (0.0, 20000.0),      # Was 0-10k
    "mohr_cohesion": (0.0, 10000.0),  # Was 0-5k
    "mohr_friction": (10.0, 45.0),    # Was 25-45
}
```

**Pros**: Simple, uses existing infrastructure
**Cons**: 10k samples may not cover the larger space adequately; may need 50k+ samples

---

## Approach 2: Terrain-Conditional NN

**Idea**: Train a single NN but add terrain "class embeddings" or auxiliary inputs.

```python
# Add normalized terrain "signature" as extra inputs
# Option A: Discrete terrain class (one-hot)
terrain_class = [is_clay, is_sand, is_dirt, is_loam]  # 4 extra inputs

# Option B: Terrain stiffness ratio
K_ratio = Kphi / Kc  # Single dimensionless parameter capturing terrain "type"
```

**Pros**: Single model, captures inter-terrain variations
**Cons**: Requires labeled terrain classes or derived features

---

## Approach 3: Physics-Informed Neural Network (PINN)

**Idea**: Embed Bekker/Mohr-Coulomb equations directly into the loss function.

```python
def physics_loss(model, inputs, outputs):
    # Bekker pressure: σ = (kc/b + kφ) * z^n
    # Mohr-Coulomb shear: τmax = c + σ * tan(φ)
    # Janosi shear: τ = τmax * (1 - e^(-j/k))
    
    # Compute expected tire force from physics
    Fy_physics = compute_bekker_force(inputs)
    Fy_nn = outputs[:, 1]
    
    # Hybrid loss: data + physics
    return mse_data + lambda_physics * mse_physics
```

**Pros**: Forces NN to respect terramechanics equations
**Cons**: Complex implementation; physics equations may not be exact

---

## Approach 4: Adaptive Per-Terrain Estimator

**Idea**: Run UKF/batch separately for each terrain, using terrain-specific priors.

```python
class TerrainAdaptiveEstimator:
    def __init__(self):
        self.estimators = {
            'soft_clay': UKFEstimator(n_prior=0.5, n_std=0.2),
            'medium': UKFEstimator(n_prior=1.0, n_std=0.3),
            'hard_packed': UKFEstimator(n_prior=1.3, n_std=0.2),
        }
    
    def estimate(self, measurements, terrain_hint=None):
        if terrain_hint:
            return self.estimators[terrain_hint].update(measurements)
        else:
            # Run all, pick best by likelihood
            return self.select_best(measurements)
```

**Pros**: Simple, well-suited for known terrain classes
**Cons**: Requires terrain classification or hint

---

## Approach 5: Gaussian Process (GP) Model

**Idea**: Use GP instead of NN - provides uncertainty quantification and graceful extrapolation.

```python
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, Matern

# GP with Matern kernel (good for physical processes)
kernel = Matern(length_scale=1.0, nu=2.5)
gp = GaussianProcessRegressor(kernel=kernel, alpha=1e-2)

# Fit on same training data
gp.fit(X_train, y_train)

# Prediction with uncertainty
Fy_mean, Fy_std = gp.predict(X_test, return_std=True)
```

**Pros**: Uncertainty bounds help detect extrapolation; better with limited data
**Cons**: O(n³) scaling - slow for 10k+ samples; sparse GP needed

---

## Approach 6: Force Residual Estimation (Simpler Alternative)

**Idea**: Skip n estimation entirely. Estimate a force correction factor instead.

```python
# Instead of estimating terrain parameter n, estimate force scale factor
# State = [x, y, ψ, u, v, ω, k_force]  where k_force scales Fy

# Tire force model:
Fy_actual = k_force * Fy_nn_nominal(n=1.0)

# Advantages:
# - Forces are directly observable through vehicle dynamics
# - Doesn't require n to match a specific physical value
# - Works even if NN is miscalibrated
```

**Pros**: Robust to NN errors; directly relates to observable quantities
**Cons**: Loses physical interpretability of n

---

## Approach 7: Multi-Model Ensemble

**Idea**: Train 3 NNs on terrain-specific subsets, blend predictions.

```python
# Model 1: Trained on soft soil data (n < 0.8)
# Model 2: Trained on medium soil data (0.7 < n < 1.2)
# Model 3: Trained on hard soil data (n > 1.0)

def predict_ensemble(inputs):
    y1 = model_soft(inputs)
    y2 = model_medium(inputs)
    y3 = model_hard(inputs)
    
    # Weight by terrain parameter similarity
    w1, w2, w3 = compute_weights(inputs.n)
    return w1 * y1 + w2 * y2 + w3 * y3
```

**Pros**: Each model specializes; overlapping ranges help blending
**Cons**: More complex training; 3x data collection

---

## Approach 8: Direct SCM Physics Integration

**Idea**: Use analytical Bekker equations directly (no NN for estimation).

```python
def bekker_lateral_force(Fz, alpha, v, Kphi, Kc, n, c, phi, k, b=0.26):
    """Analytical Bekker-based lateral force model"""
    # Pressure-sinkage: p = (kc/b + kphi) * z^n
    # Estimate sinkage from Fz equilibrium
    z = estimate_sinkage(Fz, Kphi, Kc, n, b)
    
    # Contact area
    A = estimate_contact_area(z, b)
    
    # Max shear stress (Mohr-Coulomb)
    sigma = (Kc/b + Kphi) * z**n
    tau_max = c + sigma * np.tan(np.radians(phi))
    
    # Lateral force from slip angle
    j = lateral_slip(v, alpha)
    tau = tau_max * (1 - np.exp(-j/k))
    
    return tau * A * np.sin(alpha)
```

**Pros**: Physics-based; generalizes by construction
**Cons**: Simplified compared to SCM; may not match Chrono exactly

---

## Recommended Implementation Path

### Phase 1: Quick Win (1-2 days)
1. **Expand training data** to cover n ∈ [0.3, 1.5], wider Kphi/Kc
2. Retrain existing [12, 2] architecture
3. Re-run estimation experiments

### Phase 2: If Phase 1 Insufficient (1 week)
1. Implement **Force Residual Estimation** (Approach 6)
2. Test on all three terrains
3. Compare with n-estimation results

### Phase 3: Production Quality (2 weeks)
1. Implement **Terrain-Conditional NN** (Approach 2)
2. Add terrain classifier based on force response patterns
3. Validate on mixed-terrain scenarios

---

## Quick Test: Does Wider Training Help?

Before implementing anything complex, test if the narrow n-range is the sole issue:

```bash
# Collect ~5000 samples with n ∈ [0.3, 1.5]
# Modify collect_scm_data_fast.cpp ParameterRanges:
#   bekker_n_min = 0.3
#   bekker_n_max = 1.5

# Retrain and re-run estimation
```

If clay (n=0.5) estimates correctly with new data → the NN capacity is sufficient.
If still fails → need architectural changes (deeper NN, physics loss, etc.)
