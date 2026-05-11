import numpy as np
import pickle
import torch
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(PROJECT_ROOT))
sys.path.append(str(PROJECT_ROOT / "simulation"))

from simulation.terrain_parameter_estimator import TerrainParameterEstimator

def test_force_vs_n():
    est = TerrainParameterEstimator(
        model_dir=str(PROJECT_ROOT / "nn_models" / "paper_v2_mlp_16_4")
    )
    
    n_vals = np.linspace(0.3, 1.3, 11)
    
    print("For alpha = 0.2 rad, Fz = 6000 N, u = 5.0 m/s")
    print(f"{'n':>5} | {'Fx':>8} | {'Fy':>8}")
    print("-" * 27)
    for n in n_vals:
        Fx, Fy = est._nn_FxFy(kappa=0.0, alpha=-0.2, u=5.0, Fz=6000.0, sr=0.0, n_val=n)
        print(f"{n:5.2f} | {Fx:8.1f} | {Fy:8.1f}")

if __name__ == "__main__":
    test_force_vs_n()
