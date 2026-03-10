#!/usr/bin/env python3
"""
Diagnose NN prediction accuracy vs actual Chrono SCM forces.

This helps understand why the UKF fails:
- If NN predictions don't match reality, UKF can't work
- Compares NN Fy predictions to actual terrain contact forces
"""

import sys
import numpy as np
import torch
import pickle
from pathlib import Path

# Import from simulation/ folder (one level up, then into simulation/)
sys.path.insert(0, str(Path(__file__).parent.parent / "simulation"))

import pychrono as chrono
import pychrono.vehicle as veh

from dallas_chrono_demo import setup_chrono_vehicle, setup_scm_terrain
from train_terrain_nn import TerrainNN


def load_nn_model(model_dir='v3'):
    """Load trained NN and scalers."""
    # Models are in SCM_Teleop/nn_models/{version}/ (one level up from nn_training/)
    model_path = Path(__file__).parent.parent / "nn_models" / model_dir / "best_terrain_nn.pt"
    scaler_path = Path(__file__).parent.parent / "nn_models" / model_dir / "scalers.pkl"
    
    # Load checkpoint and detect hidden sizes
    checkpoint = torch.load(model_path, weights_only=False)
    
    # Get hidden sizes from checkpoint if available
    hidden_sizes = checkpoint.get('hidden_sizes', [64, 32])  # v3 default
    print(f"  Loading NN from {model_dir} with hidden_sizes={hidden_sizes}")
    
    model = TerrainNN(input_size=11, output_size=2, hidden_sizes=hidden_sizes)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    
    with open(scaler_path, 'rb') as f:
        scalers = pickle.load(f)
    
    return model, scalers['X'], scalers['y']


def predict_nn_forces(model, scaler_X, scaler_y, Fz, slip_angle_rad, kappa, camber, 
                      velocity, Kphi, Kc, n, cohesion, friction, janosi):
    """Get NN force prediction for given inputs.
    
    Input features (11):
        Fz, slip_angle, longitudinal_slip, camber_angle, velocity,
        bekker_Kphi, bekker_Kc, bekker_n,
        mohr_cohesion, mohr_friction, janosi_shear
        
    Note: Training data uses tire test rig with SAE convention.
    Slip angle and forces may need sign adjustments for vehicle frame.
    """
    X = np.array([[Fz, slip_angle_rad, kappa, camber, velocity,
                   Kphi, Kc, n, cohesion, friction, janosi]])
    X_scaled = scaler_X.transform(X)
    with torch.no_grad():
        y_scaled = model(torch.tensor(X_scaled, dtype=torch.float32)).numpy()
    y = scaler_y.inverse_transform(y_scaled)
    # Sign flip: NN tire frame Fy opposite to Chrono wheel frame Fy
    return y[0, 0], -y[0, 1]  # Fx, -Fy


def compute_slip_angle(u, v, omega, Lf, Lr, delta):
    """Compute front and rear slip angles."""
    eps = 0.1
    if abs(u) < eps:
        return 0.0, 0.0
    alpha_f = np.arctan2(v + Lf * omega, u) - delta
    alpha_r = np.arctan2(v - Lr * omega, u)
    return alpha_f, alpha_r


def run_diagnostic(terrain_preset='sand', sim_time=10.0, use_sinusoidal=True):
    """Run simulation and compare NN vs actual forces."""
    
    print("=" * 70)
    print("NN Force Prediction Accuracy Diagnostic")
    print("=" * 70)
    
    # Load NN
    model, scaler_X, scaler_y = load_nn_model()
    print("✓ NN model loaded")
    
    # Setup simulation
    system, vehicle = setup_chrono_vehicle(visualize=False)
    terrain, terrain_params = setup_scm_terrain(
        system, vehicle=vehicle, visualize=False, terrain_preset=terrain_preset
    )
    
    n_true = terrain_params['n']
    Kphi = terrain_params['Kphi']
    Kc = terrain_params['Kc']
    cohesion = terrain_params['c']       # 'c' not 'cohesion'
    friction = terrain_params['phi']     # 'phi' not 'friction_angle' 
    janosi = terrain_params['k']         # 'k' not 'janosi_shear'
    
    print(f"  Terrain: n={n_true:.2f}, Kphi={Kphi:.2e}, Kc={Kc:.0f}")
    print(f"  cohesion={cohesion:.0f}, friction={friction:.0f}°, janosi={janosi:.3f}")
    print(f"  Terrain preset: {terrain_preset}")
    
    # Vehicle params
    Lf = 1.4  # Front axle to CG
    Lr = 1.5  # Rear axle to CG
    
    # Get wheel bodies for force extraction
    # HMMWV_Full wraps ChWheeledVehicle, use GetVehicle() to access wheel methods
    wheels = []
    inner_vehicle = vehicle.GetVehicle()
    for axle in inner_vehicle.GetAxles():
        for side in [veh.LEFT, veh.RIGHT]:
            wheel = axle.GetWheel(side)
            wheels.append(wheel.GetSpindle())
    
    # Steering input - sinusoidal for excitation
    def get_steering(t):
        if use_sinusoidal and t > 2.0:
            return 0.3 * np.sin(2 * np.pi * 0.25 * (t - 2.0))  # Larger amplitude, slower
        return 0.0
    
    # Simulation
    dt = 0.003
    
    # Data collection
    data = {
        't': [], 'alpha_f': [], 'alpha_r': [],
        'Fy_nn_f': [], 'Fy_nn_r': [],
        'Fy_chrono_f': [], 'Fy_chrono_r': [],
        'Fz_f': [], 'Fz_r': [],
        'Fx_nn_f': [], 'Fx_chrono_f': [],
        'Fy_linear_f': [],  # Linear tire model prediction (MPC default)
    }
    
    t = 0.0
    step = 0
    print(f"\n  Running {sim_time}s simulation...")
    
    while t < sim_time:
        # Steering
        delta = get_steering(t)
        
        # Create driver inputs
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = delta / 0.5  # Normalize to [-1, 1]
        driver_inputs.m_throttle = 0.3 if t < sim_time - 2.0 else 0.0
        driver_inputs.m_braking = 0.0
        
        # Synchronize and advance (vehicle.Advance handles system stepping)
        terrain.Synchronize(t)
        vehicle.Synchronize(t, driver_inputs, terrain)
        
        terrain.Advance(dt)
        vehicle.Advance(dt)
        # Don't call system.DoStepDynamics separately - vehicle.Advance does it
        
        # Sample every 0.1s and print progress
        if step % 333 == 0 and t > 0.5:
            pos = vehicle.GetChassisBody().GetPos()
            print(f"  t={t:.1f}s pos=({pos.x:.1f},{pos.y:.1f}) delta={delta:.3f}")
        
        # Sample for analysis every 0.1s
        if step % 33 == 0 and t > 1.0:
            # Get vehicle state
            chassis = vehicle.GetChassisBody()
            vel_global = chassis.GetPosDt()
            omega_global = chassis.GetAngVelLocal()
            rot = chassis.GetRot()
            
            # Transform to body frame
            vel_body = rot.GetInverse().Rotate(vel_global)
            u = vel_body.x
            v = vel_body.y
            omega = omega_global.z
            
            # Debug first sample
            if len(data['t']) == 0:
                print(f"  First sample: u={u:.2f}, v={v:.2f}, omega={omega:.3f}, delta={delta:.3f}")
            
            # Slip angles (delta is steering wheel input normalized, not actual wheel angle)
            # Approximate wheel angle as delta * max_steer_angle (about 30 degrees)
            wheel_steer = delta * 0.5  # radians (approx 30 deg max)
            alpha_f, alpha_r = compute_slip_angle(u, v, omega, Lf, Lr, wheel_steer)
            
            # Get actual forces from terrain (sum front wheels, sum rear wheels)
            # Wheels: 0,1 = front left/right, 2,3 = rear left/right
            Fy_chrono_f = 0.0
            Fy_chrono_r = 0.0
            Fz_f = 0.0
            Fz_r = 0.0
            Fx_chrono_f = 0.0
            
            for i, spindle in enumerate(wheels):
                # GetContactForceBody requires output vectors
                force = chrono.ChVector3d()
                torque = chrono.ChVector3d()
                terrain.GetContactForceBody(spindle, force, torque)
                # Transform to WHEEL frame (not body frame!) - NN was trained with tire frame forces
                spindle_rot = spindle.GetRot()
                force_wheel = spindle_rot.GetInverse().Rotate(force)
                
                if i < 2:  # Front wheels
                    Fy_chrono_f += force_wheel.y
                    Fz_f += abs(force_wheel.z)
                    Fx_chrono_f += force_wheel.x
                else:  # Rear wheels
                    Fy_chrono_r += force_wheel.y
                    Fz_r += abs(force_wheel.z)
            
            # NN predictions (per wheel, then sum)
            # NN inputs: Fz, slip_angle, kappa, camber, velocity, Kphi, Kc, n, cohesion, friction, janosi
            Fz_per_wheel = Fz_f / 2 if Fz_f > 0 else 5000
            velocity = np.sqrt(u**2 + v**2)  # Vehicle speed
            camber = 0.0  # Assume zero camber
            kappa = 0.0   # Assume zero longitudinal slip
            
            Fx_nn_f, Fy_nn_f = predict_nn_forces(
                model, scaler_X, scaler_y,
                Fz_per_wheel, alpha_f, kappa, camber, velocity,  # alpha_f in RADIANS (not degrees!)
                Kphi, Kc, n_true, cohesion, friction, janosi
            )
            Fy_nn_f *= 2  # Two front wheels
            Fx_nn_f *= 2
            
            Fz_per_wheel_r = Fz_r / 2 if Fz_r > 0 else 5000
            _, Fy_nn_r = predict_nn_forces(
                model, scaler_X, scaler_y,
                Fz_per_wheel_r, alpha_r, kappa, camber, velocity,  # alpha_r in RADIANS
                Kphi, Kc, n_true, cohesion, friction, janosi
            )
            Fy_nn_r *= 2  # Two rear wheels
            
            # Store
            data['t'].append(t)
            data['alpha_f'].append(np.degrees(alpha_f))
            data['alpha_r'].append(np.degrees(alpha_r))
            data['Fy_nn_f'].append(Fy_nn_f)
            data['Fy_nn_r'].append(Fy_nn_r)
            data['Fy_chrono_f'].append(Fy_chrono_f)
            data['Fy_chrono_r'].append(Fy_chrono_r)
            data['Fz_f'].append(Fz_f)
            data['Fz_r'].append(Fz_r)
            data['Fx_nn_f'].append(Fx_nn_f)
            data['Fx_chrono_f'].append(Fx_chrono_f)
            
            # Also compute linear tire model prediction (MPC default)
            # MPC uses Cf = Cr = 80000 N/rad
            Cf_linear = 80000.0
            Fy_linear_f = Cf_linear * alpha_f  # alpha already in radians
            data['Fy_linear_f'].append(Fy_linear_f)
        
        t += dt
        step += 1
    
    # Analysis
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    
    Fy_nn_f = np.array(data['Fy_nn_f'])
    Fy_nn_r = np.array(data['Fy_nn_r'])
    Fy_chrono_f = np.array(data['Fy_chrono_f'])
    Fy_chrono_r = np.array(data['Fy_chrono_r'])
    alpha_f = np.array(data['alpha_f'])
    Fz_f = np.array(data['Fz_f'])
    Fy_linear_f = np.array(data['Fy_linear_f'])
    
    # Print training ranges for reference
    print("\nTraining ranges (from collect_scm_data.cpp):")
    print("  slip_angle: [-0.15, 0.15] rad = [-8.6, 8.6]°")
    print("  Fz (commanded): [3000, 15000] N")
    print("  Kphi: [2e6, 4e6] N/m^(n+1)")
    print("  n: [1.0, 1.4]")
    
    print(f"\nActual input ranges in this simulation:")
    print(f"  slip_angle: [{np.min(alpha_f):.1f}°, {np.max(alpha_f):.1f}°] (radians: [{np.radians(np.min(alpha_f)):.3f}, {np.radians(np.max(alpha_f)):.3f}])")
    print(f"  Fz per wheel: [{np.min(Fz_f/2):.0f}, {np.max(Fz_f/2):.0f}] N")
    print(f"  Kphi: {Kphi:.2e} ({'IN' if 2e6 <= Kphi <= 4e6 else 'OUT OF'} range)")
    print(f"  n: {n_true:.2f} ({'IN' if 1.0 <= n_true <= 1.4 else 'OUT OF'} range)")
    
    # Filter to samples with significant slip
    mask = np.abs(alpha_f) > 0.5  # More than 0.5 degree slip
    
    # Also check in-distribution samples
    alpha_f_rad = np.radians(alpha_f)
    Fz_per_wheel = Fz_f / 2
    in_dist_mask = (np.abs(alpha_f_rad) <= 0.15) & (Fz_per_wheel >= 3000) & (Fz_per_wheel <= 15000) & (np.abs(alpha_f) > 0.5)
    print(f"\nSamples IN training distribution: {in_dist_mask.sum()} / {len(alpha_f)}")
    print(f"  (|alpha| <= 0.15 rad AND Fz in [3000, 15000] N)")
    
    # Compute correlation for in-distribution samples
    if in_dist_mask.sum() > 5:
        corr_in_dist = np.corrcoef(Fy_nn_f[in_dist_mask], Fy_chrono_f[in_dist_mask])[0,1]
        print(f"  Correlation (in-dist): {corr_in_dist:.3f}")
    
    print(f"\nSlip angle range: {np.min(alpha_f):.2f}° to {np.max(alpha_f):.2f}°")
    print(f"Samples with |α| > 0.5°: {mask.sum()} / {len(alpha_f)}")
    
    if mask.sum() > 5:
        # Front axle comparison
        err_f = Fy_chrono_f[mask] - Fy_nn_f[mask]
        err_r = Fy_chrono_r[mask] - Fy_nn_r[mask]
        
        print(f"\nFront Axle Fy (|α| > 1°, {mask.sum()} samples):")
        print(f"  NN mean:     {np.mean(Fy_nn_f[mask]):+.0f} N")
        print(f"  Chrono mean: {np.mean(Fy_chrono_f[mask]):+.0f} N")
        print(f"  Error mean:  {np.mean(err_f):+.0f} N")
        print(f"  Error RMS:   {np.sqrt(np.mean(err_f**2)):.0f} N")
        print(f"  Correlation: {np.corrcoef(Fy_nn_f[mask], Fy_chrono_f[mask])[0,1]:.3f}")
        
        print(f"\nRear Axle Fy (|α| > 1°):")
        print(f"  NN mean:     {np.mean(Fy_nn_r[mask]):+.0f} N")
        print(f"  Chrono mean: {np.mean(Fy_chrono_r[mask]):+.0f} N")
        print(f"  Error mean:  {np.mean(err_r):+.0f} N")
        print(f"  Error RMS:   {np.sqrt(np.mean(err_r**2)):.0f} N")
        
        # Sign check
        sign_match_f = np.mean(np.sign(Fy_nn_f[mask]) == np.sign(Fy_chrono_f[mask]))
        print(f"\nSign agreement (front): {sign_match_f*100:.0f}%")
        
        # Scale factor
        if np.std(Fy_chrono_f[mask]) > 100:
            scale = np.mean(Fy_chrono_f[mask]) / np.mean(Fy_nn_f[mask]) if abs(np.mean(Fy_nn_f[mask])) > 10 else np.nan
            print(f"Scale factor (Chrono/NN): {scale:.2f}")
        
        # Sample comparison
        print(f"\nSample comparisons (α, Fy_NN, Fy_Chrono):")
        indices = np.where(mask)[0]
        for i in indices[::max(1, len(indices)//5)][:5]:
            print(f"  α={alpha_f[i]:+.1f}°: NN={Fy_nn_f[i]:+.0f}N, Chrono={Fy_chrono_f[i]:+.0f}N")
    else:
        print("Not enough samples with significant slip angle")
    
    # Vertical load analysis
    print(f"\nVertical loads:")
    print(f"  Front: mean={np.mean(data['Fz_f']):.0f} N, std={np.std(data['Fz_f']):.0f} N")
    print(f"  Rear:  mean={np.mean(data['Fz_r']):.0f} N, std={np.std(data['Fz_r']):.0f} N")
    
    # =========================================================================
    # KEY COMPARISON: NN vs Linear vs Reality
    # This answers "why does NMPC work better with NN if NN values are wrong?"
    # =========================================================================
    print("\n" + "=" * 70)
    print("NN vs LINEAR TIRE MODEL vs CHRONO (MPC uses Cf=80000 N/rad)")
    print("=" * 70)
    
    if mask.sum() > 5:
        # RMS errors
        rms_nn = np.sqrt(np.mean((Fy_nn_f[mask] - Fy_chrono_f[mask])**2))
        rms_linear = np.sqrt(np.mean((Fy_linear_f[mask] - Fy_chrono_f[mask])**2))
        
        print(f"\nFront Axle Fy RMS Error vs Chrono ({mask.sum()} samples):")
        print(f"  NN model:     {rms_nn:.0f} N")
        print(f"  Linear model: {rms_linear:.0f} N")
        print(f"  NN improvement: {(rms_linear - rms_nn) / rms_linear * 100:+.1f}%" if rms_linear > 0 else "")
        
        # Correlations
        corr_nn = np.corrcoef(Fy_nn_f[mask], Fy_chrono_f[mask])[0, 1]
        corr_linear = np.corrcoef(Fy_linear_f[mask], Fy_chrono_f[mask])[0, 1]
        
        print(f"\nCorrelation with Chrono:")
        print(f"  NN model:     {corr_nn:.3f}")
        print(f"  Linear model: {corr_linear:.3f}")
        
        # Sample comparison
        print(f"\nSample comparisons (α, Chrono, NN, Linear):")
        indices = np.where(mask)[0]
        for i in indices[::max(1, len(indices)//5)][:5]:
            print(f"  α={alpha_f[i]:+5.1f}°: Chrono={Fy_chrono_f[i]:+6.0f}N, NN={Fy_nn_f[i]:+6.0f}N, Linear={Fy_linear_f[i]:+6.0f}N")
    
    # Check if NN captures the TREND even if absolute values are wrong
    # This matters for NMPC - it needs d(Fy)/d(alpha) to be correct, not Fy itself
    print("\n" + "=" * 70)
    print("TREND ANALYSIS (does NN capture directional behavior?)")
    print("=" * 70)
    
    # Compute delta(Fy) / delta(alpha) for consecutive samples
    if len(alpha_f) > 10:
        d_alpha = np.diff(alpha_f)
        d_Fy_nn = np.diff(Fy_nn_f)
        d_Fy_chrono = np.diff(Fy_chrono_f)
        
        # Filter to samples where alpha changed meaningfully
        alpha_change_mask = np.abs(d_alpha) > 0.2  # > 0.2 deg change
        
        if alpha_change_mask.sum() > 5:
            # Check if dFy/dalpha has same sign
            nn_gradient = d_Fy_nn[alpha_change_mask] / d_alpha[alpha_change_mask]
            chrono_gradient = d_Fy_chrono[alpha_change_mask] / d_alpha[alpha_change_mask]
            
            gradient_sign_match = np.mean(np.sign(nn_gradient) == np.sign(chrono_gradient))
            gradient_corr = np.corrcoef(nn_gradient, chrono_gradient)[0, 1]
            
            print(f"Gradient dFy/dα analysis ({alpha_change_mask.sum()} samples with |Δα| > 0.2°):")
            print(f"  NN gradient mean:     {np.mean(nn_gradient):.0f} N/deg")
            print(f"  Chrono gradient mean: {np.mean(chrono_gradient):.0f} N/deg")
            print(f"  Gradient sign agreement: {gradient_sign_match*100:.0f}%")
            print(f"  Gradient correlation: {gradient_corr:.3f}")
            
            # Rolling average correlation - does smoothed NN match smoothed Chrono?
            window = 5
            if len(Fy_nn_f) > window * 2:
                Fy_nn_smooth = np.convolve(Fy_nn_f, np.ones(window)/window, mode='valid')
                Fy_chrono_smooth = np.convolve(Fy_chrono_f, np.ones(window)/window, mode='valid')
                smooth_corr = np.corrcoef(Fy_nn_smooth, Fy_chrono_smooth)[0, 1]
                print(f"\nSmoothed Fy correlation (window={window}): {smooth_corr:.3f}")
    
    return data


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--terrain', type=str, default='sand',
                       choices=['sand', 'clay', 'dirt', 'asphalt'],
                       help='Terrain preset')
    parser.add_argument('--time', type=float, default=10.0)
    args = parser.parse_args()
    
    run_diagnostic(terrain_preset=args.terrain, sim_time=args.time)
