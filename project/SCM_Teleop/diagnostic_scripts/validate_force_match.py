#!/usr/bin/env python3
"""
Direct validation: Run SCM simulation with specific terrain and compare
actual tire forces vs NN predictions vs bicycle model estimates.

This will show exactly where the mismatch is.
"""

import numpy as np
import torch
import pickle
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from nn_training.train_terrain_nn import TerrainNN


def load_nn_model(model_dir: str):
    """Load trained NN and scalers"""
    script_dir = Path(__file__).parent
    model_path = script_dir / model_dir
    
    with open(model_path / 'scalers.pkl', 'rb') as f:
        scalers = pickle.load(f)
    
    model = TerrainNN()
    checkpoint = torch.load(model_path / 'best_terrain_nn.pt', 
                           map_location='cpu', weights_only=False)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    
    return model, scalers['X'], scalers['y']


def predict_force(model, scaler_X, scaler_y, Fz, alpha, Kphi, Kc, n, c, phi, K):
    """Predict forces using NN"""
    x = np.array([[Fz, alpha, 0.0, 0.0, 5.0, Kphi, Kc, n, c, phi, K]])
    x_scaled = scaler_X.transform(x)
    with torch.no_grad():
        y_scaled = model(torch.FloatTensor(x_scaled))
    y = scaler_y.inverse_transform(y_scaled.numpy())
    return y[0, 0], y[0, 1]  # Fx, Fy


def run_single_cornering_test():
    """Run a single cornering simulation and extract tire forces"""
    import pychrono as chrono
    import pychrono.vehicle as veh
    
    # Set data paths
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Use CLAY terrain (n=0.5)
    terrain_params = {
        'bekker_Kphi': 2.1e6,
        'bekker_Kc': 9500,
        'bekker_n': 0.5,
        'mohr_cohesion': 8000,
        'mohr_friction': 13,  # degrees
        'janosi_shear': 0.01,
    }
    
    # Create vehicle
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.6), chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()
    
    sys = hmmwv.GetSystem()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    
    # Disable visualization
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_NONE)
    
    # Create SCM terrain
    scm_terrain = veh.SCMTerrain(sys)
    scm_terrain.SetSoilParameters(
        terrain_params['bekker_Kphi'],
        terrain_params['bekker_Kc'],
        terrain_params['bekker_n'],
        terrain_params['mohr_cohesion'],
        np.radians(terrain_params['mohr_friction']),
        terrain_params['janosi_shear'],
        2e8, 3e4
    )
    scm_terrain.AddMovingPatch(
        hmmwv.GetChassisBody(),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(6, 3, 1)
    )
    scm_terrain.SetPlotType(veh.SCMTerrain.PLOT_NONE, 0, 0)
    scm_terrain.Initialize(100.0, 30.0, 0.08)
    
    # Vehicle params
    m = 2000  # kg (approximate HMMWV mass)
    Lf = 1.689
    Lr = 1.689
    
    # Simulation
    results = []
    step_size = 3e-3
    t = 0.0
    
    print("Running simulation...")
    while t < 10.0:
        # Driving: accelerate then turn left
        if t < 2.0:
            steer, throttle = 0.0, 0.5
        elif t < 4.0:
            steer, throttle = 0.2 * (t - 2.0) / 2.0, 0.4  # ramp up steering
        else:
            steer, throttle = 0.2, 0.4  # hold turn
        
        inputs = veh.DriverInputs()
        inputs.m_steering = steer
        inputs.m_throttle = throttle
        inputs.m_braking = 0.0
        
        scm_terrain.Synchronize(t)
        hmmwv.Synchronize(t, inputs, scm_terrain)
        scm_terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        
        # Sample every 0.5s after t=3s
        if t > 3.0 and int(t * 1000) % 500 == 0:
            chassis = hmmwv.GetChassisBody()
            vel_global = chassis.GetPosDt()
            acc_global = chassis.GetPosDt2()
            rot = chassis.GetRot()
            rot_inv = rot.GetInverse()
            
            vel_body = rot_inv.Rotate(vel_global)
            acc_body = rot_inv.Rotate(acc_global)
            omega_vec = chassis.GetAngVelLocal()
            
            u = vel_body.x
            v = vel_body.y
            omega = omega_vec.z
            a_y = acc_body.y
            
            if u > 2.0:
                # Compute slip angles
                delta = steer * 0.5
                alpha_f = np.arctan2(v + Lf * omega, max(u, 0.5)) - delta
                alpha_r = np.arctan2(v - Lr * omega, max(u, 0.5))
                
                # Get actual tire forces (use GetForce method)
                front_left = hmmwv.GetVehicle().GetTire(0, veh.LEFT)
                front_right = hmmwv.GetVehicle().GetTire(0, veh.RIGHT)
                rear_left = hmmwv.GetVehicle().GetTire(1, veh.LEFT)
                rear_right = hmmwv.GetVehicle().GetTire(1, veh.RIGHT)
                
                # Get wheel states for forces
                ws_fl = hmmwv.GetVehicle().GetWheel(0, veh.LEFT).GetState()
                ws_fr = hmmwv.GetVehicle().GetWheel(0, veh.RIGHT).GetState()
                ws_rl = hmmwv.GetVehicle().GetWheel(1, veh.LEFT).GetState()
                ws_rr = hmmwv.GetVehicle().GetWheel(1, veh.RIGHT).GetState()
                
                try:
                    # Get tire force from wheel state
                    FL_force = front_left.GetTireForce()
                    FR_force = front_right.GetTireForce()
                    RL_force = rear_left.GetTireForce()
                    RR_force = rear_right.GetTireForce()
                    
                    # Transform to body frame
                    Fyf_actual = (rot_inv.Rotate(FL_force.force).y +
                                  rot_inv.Rotate(FR_force.force).y)
                    Fyr_actual = (rot_inv.Rotate(RL_force.force).y +
                                  rot_inv.Rotate(RR_force.force).y)
                    
                    Fzf = (rot_inv.Rotate(FL_force.force).z +
                           rot_inv.Rotate(FR_force.force).z)
                    Fzr = (rot_inv.Rotate(RL_force.force).z +
                           rot_inv.Rotate(RR_force.force).z)
                except:
                    # Fallback - use estimated vertical load from vehicle weight
                    Fzf = 10000  # ~5000 per front tire
                    Fzr = 10000  # ~5000 per rear tire
                    Fyf_actual = 0
                    Fyr_actual = 0
                
                # Validate values
                if abs(Fzf) > 1e8 or abs(Fyf_actual) > 1e8:
                    print(f"Skipping t={t:.1f} due to invalid forces")
                    continue
                
                # Bicycle model estimate (from steady-state assumption)
                Fyf_bm = m * a_y * (Lr / (Lf + Lr))
                Fyr_bm = m * a_y * (Lf / (Lf + Lr))
                
                results.append({
                    't': t,
                    'u': u,
                    'a_y': a_y,
                    'alpha_f': alpha_f,
                    'alpha_r': alpha_r,
                    'Fzf': Fzf,
                    'Fzr': Fzr,
                    'Fyf_actual': Fyf_actual,
                    'Fyr_actual': Fyr_actual,
                    'Fyf_bm': Fyf_bm,
                    'Fyr_bm': Fyr_bm,
                })
        
        t += step_size
    
    return results, terrain_params


def main():
    # Load NN
    print("Loading NN model...")
    model, scaler_X, scaler_y = load_nn_model('nn_models_v5_vehicle')
    
    # Run simulation
    results, terrain_params = run_single_cornering_test()
    
    if not results:
        print("No results collected!")
        return
    
    print("\n" + "=" * 80)
    print("FORCE COMPARISON: Actual SCM vs Bicycle Model vs NN Prediction")
    print("=" * 80)
    print(f"Terrain: Clay (n={terrain_params['bekker_n']}, phi={terrain_params['mohr_friction']}°)")
    print()
    
    print(f"{'Time':>5} | {'u':>5} | {'a_y':>6} | {'α_f':>6} | {'Fzf':>7} | "
          f"{'Fyf_act':>8} | {'Fyf_bm':>8} | {'Fyf_nn':>8} | {'err_bm':>7} | {'err_nn':>7}")
    print("-" * 100)
    
    for r in results[:10]:  # First 10 samples
        # NN prediction (per-axle)
        _, Fyf_nn_single = predict_force(
            model, scaler_X, scaler_y,
            r['Fzf'] / 2,  # per tire
            r['alpha_f'],
            terrain_params['bekker_Kphi'],
            terrain_params['bekker_Kc'],
            terrain_params['bekker_n'],
            terrain_params['mohr_cohesion'],
            terrain_params['mohr_friction'],
            terrain_params['janosi_shear']
        )
        Fyf_nn = 2 * Fyf_nn_single  # Two front tires
        
        err_bm = r['Fyf_bm'] - r['Fyf_actual']
        err_nn = Fyf_nn - r['Fyf_actual']
        
        print(f"{r['t']:5.1f} | {r['u']:5.1f} | {r['a_y']:6.2f} | "
              f"{np.degrees(r['alpha_f']):6.2f} | {r['Fzf']:7.0f} | "
              f"{r['Fyf_actual']:8.0f} | {r['Fyf_bm']:8.0f} | {Fyf_nn:8.0f} | "
              f"{err_bm:7.0f} | {err_nn:7.0f}")
    
    # Summary statistics
    print("\n" + "=" * 80)
    print("SUMMARY:")
    
    all_actual = [r['Fyf_actual'] for r in results]
    all_bm = [r['Fyf_bm'] for r in results]
    all_nn = []
    for r in results:
        _, Fyf_nn_single = predict_force(
            model, scaler_X, scaler_y,
            r['Fzf'] / 2, r['alpha_f'],
            terrain_params['bekker_Kphi'], terrain_params['bekker_Kc'],
            terrain_params['bekker_n'], terrain_params['mohr_cohesion'],
            terrain_params['mohr_friction'], terrain_params['janosi_shear']
        )
        all_nn.append(2 * Fyf_nn_single)
    
    rmse_bm = np.sqrt(np.mean([(bm - act)**2 for bm, act in zip(all_bm, all_actual)]))
    rmse_nn = np.sqrt(np.mean([(nn - act)**2 for nn, act in zip(all_nn, all_actual)]))
    
    print(f"Actual Fyf range: {min(all_actual):.0f} to {max(all_actual):.0f} N")
    print(f"Bicycle model RMSE: {rmse_bm:.0f} N")
    print(f"NN prediction RMSE: {rmse_nn:.0f} N")
    print(f"Scale factor (mean NN/actual): {np.mean(all_nn) / np.mean(all_actual):.2f}")


if __name__ == "__main__":
    main()
