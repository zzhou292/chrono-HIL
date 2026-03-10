#!/usr/bin/env python3
"""
Diagnose force mismatch between NN and simulation for different terrain parameters.
This helps understand why the terrain estimator converges to the wrong value.
"""

import numpy as np
import torch
import pickle
import sys
from pathlib import Path

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from nn_training.train_terrain_nn import TerrainNN


def load_nn_model(model_dir: str):
    """Load trained NN and scalers"""
    # Get absolute path relative to script location
    script_dir = Path(__file__).parent
    model_path = script_dir / model_dir
    
    # Load scalers
    with open(model_path / 'scalers.pkl', 'rb') as f:
        scalers = pickle.load(f)
    
    # Load model (use weights_only=False for numpy compatibility)
    model = TerrainNN()
    checkpoint = torch.load(model_path / 'best_terrain_nn.pt', 
                           map_location='cpu', weights_only=False)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    
    return model, scalers['X'], scalers['y']


def predict_force(model, scaler_X, scaler_y,
                  Fz, alpha, Kphi, Kc, n, c, phi, K):
    """Predict lateral force using NN"""
    # Features: Fz, slip_angle, slip_ratio, camber, velocity, Kphi, Kc, n, c, phi, K
    x = np.array([[Fz, alpha, 0.0, 0.0, 5.0, Kphi, Kc, n, c, phi, K]])
    x_scaled = scaler_X.transform(x)
    with torch.no_grad():
        y_scaled = model(torch.FloatTensor(x_scaled))
    y = scaler_y.inverse_transform(y_scaled.numpy())
    return y[0, 0], y[0, 1]  # Fx, Fy


def main():
    # Define terrain presets inline
    TERRAIN_PRESETS = {
        'clay': {
            'bekker_Kphi': 2.1e6,
            'bekker_Kc': 9500,
            'bekker_n': 0.5,
            'mohr_cohesion': 8000,
            'mohr_friction': 13,  # degrees
            'janosi_shear': 0.01,
        },
        'sand': {
            'bekker_Kphi': 1.5e6,
            'bekker_Kc': 600,
            'bekker_n': 1.38,
            'mohr_cohesion': 1000,
            'mohr_friction': 30,  # degrees
            'janosi_shear': 0.025,
        },
        'dirt': {
            'bekker_Kphi': 2e6,
            'bekker_Kc': 1500,
            'bekker_n': 1.1,
            'mohr_cohesion': 2000,
            'mohr_friction': 32,  # degrees
            'janosi_shear': 0.02,
        },
    }
    
    # Load both NN versions
    nn_models = {
        'nn_models (tire rig)': 'nn_models',
        'v5_vehicle (full vehicle)': 'nn_models_v5_vehicle',
    }
    
    # Define test conditions (similar to what estimator uses)
    test_conditions = [
        {'Fz': 5000, 'alpha': 0.05, 'label': 'Light load, small slip'},
        {'Fz': 7000, 'alpha': 0.1, 'label': 'Medium load, medium slip'},
        {'Fz': 10000, 'alpha': 0.15, 'label': 'Heavy load, large slip'},
    ]
    
    # Test terrains
    terrains = ['clay', 'sand', 'dirt']
    
    print("=" * 80)
    print("NN Force Prediction Comparison: Test Rig (v4) vs Full Vehicle (v5)")
    print("=" * 80)
    
    for terrain_name in terrains:
        terrain = TERRAIN_PRESETS[terrain_name]
        n_true = terrain['bekker_n']
        Kphi = terrain['bekker_Kphi']
        Kc = terrain['bekker_Kc']
        c = terrain['mohr_cohesion']
        phi = terrain['mohr_friction']  # degrees
        K = terrain['janosi_shear']
        
        print(f"\n{terrain_name.upper()} (n_true={n_true}):")
        print("-" * 70)
        
        for nn_name, nn_path in nn_models.items():
            try:
                model, scaler_X, scaler_y = load_nn_model(nn_path)
            except Exception as e:
                print(f"  {nn_name}: Failed to load - {e}")
                continue
            
            print(f"\n  {nn_name}:")
            for cond in test_conditions:
                Fz = cond['Fz']
                alpha = cond['alpha']
                
                # Predict with true n
                Fx_true, Fy_true = predict_force(
                    model, scaler_X, scaler_y,
                    Fz, alpha, Kphi, Kc, n_true, c, phi, K
                )
                
                # Predict with n=0.3 (lower bound - what estimator converges to)
                Fx_03, Fy_03 = predict_force(
                    model, scaler_X, scaler_y,
                    Fz, alpha, Kphi, Kc, 0.3, c, phi, K
                )
                
                # Predict with n=1.5 (upper bound)
                Fx_15, Fy_15 = predict_force(
                    model, scaler_X, scaler_y,
                    Fz, alpha, Kphi, Kc, 1.5, c, phi, K
                )
                
                print(f"    {cond['label']}:")
                print(f"      Fy(n=0.3): {Fy_03:8.1f}N | Fy(n={n_true}): {Fy_true:8.1f}N | Fy(n=1.5): {Fy_15:8.1f}N")
                print(f"      Sensitivity: Fy changes by {abs(Fy_15 - Fy_03):.0f}N over n=[0.3, 1.5]")
    
    # Also show what n-sweep looks like for each NN
    print("\n" + "=" * 80)
    print("N-SWEEP: How Fy varies with n for typical conditions")
    print("=" * 80)
    
    n_values = [0.3, 0.5, 0.8, 1.0, 1.2, 1.4]
    Fz = 6000
    alpha = 0.08
    
    for nn_name, nn_path in nn_models.items():
        try:
            model, scaler_X, scaler_y = load_nn_model(nn_path)
        except:
            continue
        
        print(f"\n{nn_name} (Fz={Fz}N, alpha={alpha:.2f}rad):")
        
        for terrain_name in terrains:
            terrain = TERRAIN_PRESETS[terrain_name]
            Kphi = terrain['bekker_Kphi']
            Kc = terrain['bekker_Kc']
            c = terrain['mohr_cohesion']
            phi = terrain['mohr_friction']
            K = terrain['janosi_shear']
            
            forces = []
            for n in n_values:
                _, Fy = predict_force(model, scaler_X, scaler_y, Fz, alpha, Kphi, Kc, n, c, phi, K)
                forces.append(Fy)
            
            print(f"  {terrain_name:5s}: " + " | ".join([f"n={n}→{Fy:7.0f}N" for n, Fy in zip(n_values, forces)]))


if __name__ == "__main__":
    main()
