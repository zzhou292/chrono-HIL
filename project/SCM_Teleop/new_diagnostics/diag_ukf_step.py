import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT))
sys.path.append(str(PROJECT_ROOT / "simulation"))

from simulation.terrain_parameter_estimator import TerrainParameterEstimator

def test_ukf_step():
    est = TerrainParameterEstimator(
        model_dir=str(PROJECT_ROOT / "nn_models" / "paper_v2_mlp_16_4"),
        update_interval=1,
    )
    
    # Setup state to match t=1.2s approx
    est._x = np.array([0.21, 0.13, 0.68])
    est._P = np.diag([0.05**2, 0.01**2, 0.3**2])
    
    # Let's say it sees v_meas = +0.37, w_meas = +0.19, u = 0.63
    alpha_f = 0.2
    alpha_r = 0.0
    Fz_f = 6500.0
    Fz_r = 6000.0
    
    # We will instrument observe to print sigmas
    print("Initial state:", est._x)
    
    # Pass arbitrary inputs, we just want to see the UKF internal vars
    est._last_alpha_f = alpha_f
    est._last_alpha_r = alpha_r
    est._last_Fz_f = Fz_f
    est._last_Fz_r = Fz_r
    est._last_kappa = 0.0
    est._last_sr = 0.0
    est._meas_x = 0.0
    est._meas_y = 0.0
    est._meas_psi = 0.0
    
    import math
    u_meas = 0.63
    v_lat = 0.37
    omega = 0.19
    est._last_delta = alpha_f + math.atan2(v_lat + est._Lf * omega, max(abs(u_meas), 0.5))
    
    print("last_delta:", est._last_delta)
    
    # run predict
    from simulation.terrain_parameter_estimator import _ukf_sigma_points
    sigmas, Wm, Wc = _ukf_sigma_points(est._x, est._P)
    
    sigmas_pred = np.zeros_like(sigmas)
    for i in range(len(sigmas)):
        sigmas_pred[i] = est._predict_sigma(sigmas[i], u_meas)
        
    print("\nSigmas:")
    for i, s in enumerate(sigmas):
        print(f"  {i}: v={s[0]:+.3f}, w={s[1]:+.3f}, n={s[2]:+.3f}")
        
    print("\nSigmas Pred:")
    for i, sp in enumerate(sigmas_pred):
        print(f"  {i}: v={sp[0]:+.3f}, w={sp[1]:+.3f}, n={sp[2]:+.3f}")
        
    x_pred = np.zeros(3)
    for i in range(len(sigmas)):
        x_pred += Wm[i] * sigmas_pred[i]
        
    y_pred = x_pred[:2]
    print("\nx_pred:", x_pred)
    
    y = np.array([v_lat, omega])
    
    Pyy = est._R.copy()
    Pxy = np.zeros((3, 2))
    for i in range(len(sigmas)):
        dy = sigmas_pred[i, :2] - y_pred
        dx = sigmas_pred[i] - x_pred
        Pyy += Wc[i] * np.outer(dy, dy)
        Pxy += Wc[i] * np.outer(dx, dy)
        
    K = Pxy @ np.linalg.inv(Pyy)
    print("\nPxy:")
    print(Pxy)
    print("\nPyy:")
    print(Pyy)
    print("\nK:")
    print(K)
    
    innovation = y - y_pred
    print("\nInnovation:", innovation)
    
    x_new = x_pred + K @ innovation
    print("\nx_new:", x_new)

if __name__ == "__main__":
    test_ukf_step()
