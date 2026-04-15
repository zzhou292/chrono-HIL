#!/usr/bin/env python3
"""
Validate trained terrain neural network against SCM terrain data
Shows that the NN accurately predicts forces for different soil conditions
"""

import numpy as np
import pandas as pd
import torch
import pickle
import matplotlib.pyplot as plt
from pathlib import Path
import argparse

# Import the NN model class
import sys
sys.path.append(str(Path(__file__).parent))
from train_terrain_nn import TerrainNN


def load_model_and_scalers(model_path, scaler_path):
    """Load trained model and scalers"""
    # Load model
    model = TerrainNN(input_size=11, output_size=2)
    
    # Handle both checkpoint format and state dict format
    checkpoint = torch.load(model_path, weights_only=False)
    if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
        model.load_state_dict(checkpoint['model_state_dict'])
    else:
        model.load_state_dict(checkpoint)
    model.eval()
    
    # Load scalers
    with open(scaler_path, 'rb') as f:
        scalers = pickle.load(f)
    
    return model, scalers['X'], scalers['y']


def predict_forces(model, scaler_X, scaler_y, inputs):
    """Predict Fx, Fy for given inputs"""
    # Scale inputs
    inputs_scaled = scaler_X.transform(inputs.reshape(1, -1))
    
    # Predict
    with torch.no_grad():
        inputs_tensor = torch.FloatTensor(inputs_scaled)
        outputs_scaled = model(inputs_tensor).numpy()
    
    # Inverse scale
    outputs = scaler_y.inverse_transform(outputs_scaled)
    return outputs[0]  # [Fx, Fy]


def validate_on_dataset(model, scaler_X, scaler_y, data_file):
    """Validate model on a CSV dataset"""
    print(f"\n=== Validating on {data_file} ===")
    
    df = pd.read_csv(data_file)
    print(f"Loaded {len(df)} samples")
    
    # Get input columns (use Fz if available)
    load_col = 'Fz' if 'Fz' in df.columns else 'vertical_load'
    input_cols = [
        load_col, 'slip_angle', 'longitudinal_slip', 'camber_angle', 'velocity',
        'bekker_Kphi', 'bekker_Kc', 'bekker_n',
        'mohr_cohesion', 'mohr_friction', 'janosi_shear'
    ]
    
    X = df[input_cols].values
    y_true = df[['Fx', 'Fy']].values
    
    # Predict
    X_scaled = scaler_X.transform(X)
    with torch.no_grad():
        X_tensor = torch.FloatTensor(X_scaled)
        y_pred_scaled = model(X_tensor).numpy()
    y_pred = scaler_y.inverse_transform(y_pred_scaled)
    
    # Compute metrics
    for i, name in enumerate(['Fx', 'Fy']):
        true_vals = y_true[:, i]
        pred_vals = y_pred[:, i]
        
        rmse = np.sqrt(np.mean((true_vals - pred_vals)**2))
        mae = np.mean(np.abs(true_vals - pred_vals))
        
        # R² score
        ss_res = np.sum((true_vals - pred_vals)**2)
        ss_tot = np.sum((true_vals - np.mean(true_vals))**2)
        r2 = 1 - ss_res / ss_tot
        
        print(f"  {name}: RMSE={rmse:.1f}N, MAE={mae:.1f}N, R²={r2:.4f}")
    
    return y_true, y_pred


def plot_validation_results(y_true, y_pred, output_path):
    """Plot predicted vs actual forces"""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    for i, (ax, name) in enumerate(zip(axes, ['Fx (Longitudinal)', 'Fy (Lateral)'])):
        ax.scatter(y_true[:, i], y_pred[:, i], alpha=0.5, s=10)
        
        # Perfect prediction line
        min_val = min(y_true[:, i].min(), y_pred[:, i].min())
        max_val = max(y_true[:, i].max(), y_pred[:, i].max())
        ax.plot([min_val, max_val], [min_val, max_val], 'r--', linewidth=2, label='Perfect')
        
        ax.set_xlabel(f'Actual {name} (N)')
        ax.set_ylabel(f'Predicted {name} (N)')
        ax.set_title(f'{name}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to: {output_path}")
    plt.close()


def demonstrate_terrain_adaptation(model, scaler_X, scaler_y):
    """Show how the NN predicts different forces for different terrains"""
    print("\n=== Terrain Adaptation Demonstration ===")
    print("Showing forces at fixed slip conditions with different soil types:\n")
    
    # Base conditions: moderate slip angle and ratio
    Fz = 5000  # N
    slip_angle = 0.05  # rad (~3 deg)
    slip_ratio = 0.05
    camber = 0
    velocity = 1.0  # m/s
    
    # Different terrain types (from Table II in the reference paper)
    terrains = {
        'Dry Sand': {
            'bekker_Kphi': 1.5e6, 'bekker_Kc': 0, 'bekker_n': 1.1,
            'mohr_cohesion': 200, 'mohr_friction': 28, 'janosi_shear': 0.025
        },
        'Clay (Wet)': {
            'bekker_Kphi': 3e6, 'bekker_Kc': 5000, 'bekker_n': 1.0,
            'mohr_cohesion': 4000, 'mohr_friction': 35, 'janosi_shear': 0.02
        },
        'Hard Soil': {
            'bekker_Kphi': 4e6, 'bekker_Kc': 8000, 'bekker_n': 1.3,
            'mohr_cohesion': 3000, 'mohr_friction': 40, 'janosi_shear': 0.015
        }
    }
    
    print(f"Conditions: Fz={Fz}N, slip_angle={np.rad2deg(slip_angle):.1f}deg, slip_ratio={slip_ratio:.2f}")
    print("-" * 60)
    
    for terrain_name, params in terrains.items():
        inputs = np.array([
            Fz, slip_angle, slip_ratio, camber, velocity,
            params['bekker_Kphi'], params['bekker_Kc'], params['bekker_n'],
            params['mohr_cohesion'], params['mohr_friction'], params['janosi_shear']
        ])
        
        Fx, Fy = predict_forces(model, scaler_X, scaler_y, inputs)
        
        # Calculate friction coefficient estimates
        mu_x = abs(Fx / Fz) if Fz > 0 else 0
        mu_y = abs(Fy / Fz) if Fz > 0 else 0
        
        print(f"{terrain_name:15s}: Fx={Fx:8.1f}N (μx={mu_x:.3f}), Fy={Fy:8.1f}N (μy={mu_y:.3f})")


def main():
    parser = argparse.ArgumentParser(description='Validate terrain neural network')
    parser.add_argument('--model', type=str, default='../nn_models/v3/best_terrain_nn.pt',
                       help='Path to trained model (default: ../nn_models/v3/best_terrain_nn.pt)')
    parser.add_argument('--scaler', type=str, default='../nn_models/v3/scalers.pkl',
                       help='Path to scalers (default: ../nn_models/v3/scalers.pkl)')
    parser.add_argument('--data', type=str, 
                       default='/home/kyle/Documents/chrono-HIL/nbuild/scm_training_data.csv',
                       help='Path to validation data CSV')
    parser.add_argument('--output', type=str, default='nn_validation.png',
                       help='Output plot filename')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("TERRAIN NEURAL NETWORK VALIDATION")
    print("=" * 60)
    
    # Load model
    model, scaler_X, scaler_y = load_model_and_scalers(args.model, args.scaler)
    print(f"Loaded model from {args.model}")
    
    # Validate on dataset
    y_true, y_pred = validate_on_dataset(model, scaler_X, scaler_y, args.data)
    
    # Plot results
    plot_validation_results(y_true, y_pred, args.output)
    
    # Demonstrate terrain adaptation
    demonstrate_terrain_adaptation(model, scaler_X, scaler_y)
    
    print("\n" + "=" * 60)
    print("VALIDATION COMPLETE")
    print("=" * 60)


if __name__ == '__main__':
    main()
