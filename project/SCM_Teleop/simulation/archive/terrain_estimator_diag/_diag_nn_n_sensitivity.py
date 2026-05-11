#!/usr/bin/env python3
"""Quick diagnostic: how does NN predicted Fy change as we vary n along the
preset manifold for representative dirt operating conditions?"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from terrain_parameter_estimator import TerrainParameterEstimator, _PRESET_INTERNAL
from param_consistency import terrain_preset_to_internal, get_terrain_preset

model_dir = Path(__file__).parent.parent / "nn_models" / "paper_v2_mlp_16_4"
init_terrain = terrain_preset_to_internal(get_terrain_preset("clay"))
est = TerrainParameterEstimator(model_dir=str(model_dir), initial_terrain=init_terrain)

# Representative dirt conditions from openloop test
# u≈8.8 m/s, Fz_f≈6500, Fz_r≈6000, alpha during turns ~0.15 rad
ops = [
    dict(kappa=0.05, alpha=0.10, u=8.5, Fz=6500.0, sr=0.0, name="left_turn_front"),
    dict(kappa=0.05, alpha=-0.10, u=8.5, Fz=6500.0, sr=0.0, name="right_turn_front"),
    dict(kappa=0.05, alpha=0.05, u=8.5, Fz=6000.0, sr=0.0, name="left_turn_rear"),
]

n_grid = np.linspace(0.3, 1.3, 21)

print("\nNN single-wheel Fy(n) along preset manifold:")
print(f"{'n':>6}", end='')
for op in ops:
    print(f"  {op['name']:>20s}", end='')
print()

for n_val in n_grid:
    print(f"{n_val:6.2f}", end='')
    for op in ops:
        _, fy = est._nn_FxFy(op['kappa'], op['alpha'], op['u'],
                              op['Fz'], op['sr'], n_val)
        print(f"  {fy:20.1f}", end='')
    print()

print("\nTerrain params at each n:")
for n_val in [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]:
    params = est._terrain_params_for_n(n_val)
    print(f"  n={n_val:.2f}: Kphi={params['Kphi']:>9.0f} "
          f"Kc={params['Kc']:>7.0f} c={params['c']:>5.0f} "
          f"phi={params['phi']:5.1f}° k={params['k']:.4f}")

# Now check: what would the NN say if we swept ONLY n holding other terrain params fixed?
print("\n\nNN single-wheel Fy varying ONLY n with clay params otherwise:")
clay_params = _PRESET_INTERNAL["clay"]
print(f"{'n':>6}", end='')
for op in ops:
    print(f"  {op['name']:>20s}", end='')
print()

import numpy as np
for n_val in n_grid:
    print(f"{n_val:6.2f}", end='')
    for op in ops:
        x = np.array([[op['kappa'], op['alpha'], op['u'], op['Fz'], op['sr'],
                        clay_params['Kphi'], clay_params['Kc'], n_val,
                        clay_params['c'], np.radians(clay_params['phi']), clay_params['k']]])
        x_s = (x - est._X_mean) / est._X_scale
        y_s = est._np_nn(x_s)
        y = y_s * est._y_scale + est._y_mean
        print(f"  {float(y[0,1]):20.1f}", end='')
    print()
