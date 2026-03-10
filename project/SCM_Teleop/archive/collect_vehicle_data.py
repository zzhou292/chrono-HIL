#!/usr/bin/env python3
"""
Full Vehicle SCM Data Collection for NN Tire Model Training

Unlike the tire test rig, this collects data from actual vehicle simulation:
- Real tire loads from vehicle weight and load transfer
- Natural slip angles from cornering maneuvers
- Forces that match what the estimator sees

This should produce training data that better matches the estimator's environment.
"""

import pychrono as chrono
import pychrono.vehicle as veh
import numpy as np
import multiprocessing as mp
from multiprocessing import Pool, Value
import csv
import argparse
import time
import sys
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class TerrainParams:
    """Bekker soil parameters for SCM terrain"""
    bekker_Kphi: float
    bekker_Kc: float
    bekker_n: float
    mohr_cohesion: float
    mohr_friction: float  # degrees
    janosi_shear: float


@dataclass
class TireMeasurement:
    """Single tire measurement sample"""
    time: float
    axle: int  # 0=front, 1=rear
    side: int  # 0=left, 1=right
    slip_angle: float  # radians
    slip_ratio: float
    camber: float  # radians
    velocity: float  # m/s
    Fz: float  # vertical force
    Fx: float  # longitudinal force
    Fy: float  # lateral force
    # Terrain params
    bekker_Kphi: float
    bekker_Kc: float
    bekker_n: float
    mohr_cohesion: float
    mohr_friction: float
    janosi_shear: float


class TerrainRanges:
    """Parameter ranges for random terrain generation"""
    bekker_Kphi_min = 0.5e6
    bekker_Kphi_max = 4.0e6
    bekker_Kc_min = 0
    bekker_Kc_max = 20000
    bekker_n_min = 0.3
    bekker_n_max = 1.5
    mohr_cohesion_min = 0
    mohr_cohesion_max = 10000
    mohr_friction_min = 10  # degrees
    mohr_friction_max = 45  # degrees
    janosi_shear_min = 0.005
    janosi_shear_max = 0.05


def generate_random_terrain(rng: np.random.Generator) -> TerrainParams:
    """Generate random terrain parameters within ranges"""
    r = TerrainRanges
    return TerrainParams(
        bekker_Kphi=rng.uniform(r.bekker_Kphi_min, r.bekker_Kphi_max),
        bekker_Kc=rng.uniform(r.bekker_Kc_min, r.bekker_Kc_max),
        bekker_n=rng.uniform(r.bekker_n_min, r.bekker_n_max),
        mohr_cohesion=rng.uniform(r.mohr_cohesion_min, r.mohr_cohesion_max),
        mohr_friction=rng.uniform(r.mohr_friction_min, r.mohr_friction_max),
        janosi_shear=rng.uniform(r.janosi_shear_min, r.janosi_shear_max),
    )


def run_vehicle_simulation(
    terrain: TerrainParams,
    sample_idx: int,
    sim_duration: float = 15.0,
    step_size: float = 3e-3,
    mesh_spacing: float = 0.1,
    verbose: bool = False,
) -> List[TireMeasurement]:
    """Run a single vehicle simulation and collect tire data"""
    
    measurements = []
    
    # Set data paths
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Create HMMWV vehicle (it creates its own system internally)
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.6), chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)  # RIGID for SCM interaction
    hmmwv.Initialize()
    
    # Get system from vehicle
    sys = hmmwv.GetSystem()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(100)
    
    # Disable visualization 
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_NONE)
    
    # Create SCM terrain
    scm_terrain = veh.SCMTerrain(sys)
    scm_terrain.SetSoilParameters(
        terrain.bekker_Kphi,
        terrain.bekker_Kc,
        terrain.bekker_n,
        terrain.mohr_cohesion,
        np.radians(terrain.mohr_friction),  # Convert to radians!
        terrain.janosi_shear,
        2e8,  # elastic stiffness
        3e4,  # damping
    )
    
    # Add moving patch for efficiency
    scm_terrain.AddMovingPatch(
        hmmwv.GetChassisBody(),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(6, 3, 1),
    )
    
    scm_terrain.SetPlotType(veh.SCMTerrain.PLOT_NONE, 0, 0)
    scm_terrain.Initialize(100.0, 30.0, mesh_spacing)
    
    # Vehicle parameters
    Lf = 1.689  # front axle to CG
    Lr = 1.689  # rear axle to CG
    
    # Simulation loop
    t = 0.0
    sample_count = 0
    sample_interval = 100  # Every 100 steps
    
    while t < sim_duration:
        # Generate driving inputs: acceleration then cornering
        if t < 2.0:
            steer = 0.0
            throttle = 0.5
        elif t < 5.0:
            steer = 0.3 * (t - 2.0) / 3.0
            throttle = 0.35
        elif t < 8.0:
            steer = 0.3
            throttle = 0.35
        elif t < 11.0:
            steer = 0.3 - 0.6 * (t - 8.0) / 3.0
            throttle = 0.35
        else:
            steer = -0.3
            throttle = 0.35
        
        # Apply inputs
        inputs = veh.DriverInputs()
        inputs.m_steering = steer
        inputs.m_throttle = throttle
        inputs.m_braking = 0.0
        
        # Synchronize and advance
        scm_terrain.Synchronize(t)
        hmmwv.Synchronize(t, inputs, scm_terrain)
        scm_terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        
        # Sample tire data periodically after initial settling
        if t > 3.0 and (sample_count % sample_interval) == 0:
            # Get chassis state
            chassis = hmmwv.GetChassisBody()
            vel_global = chassis.GetPosDt()
            rot = chassis.GetRot()
            rot_inv = rot.GetInverse()
            vel_body = rot_inv.Rotate(vel_global)
            omega_vec = chassis.GetAngVelLocal()
            
            u = vel_body.x  # forward velocity
            v = vel_body.y  # lateral velocity
            omega = omega_vec.z  # yaw rate
            delta = steer * 0.5  # steering angle (approximate ratio)
            
            # Skip if vehicle too slow
            if u >= 2.0:
                # Get tire forces for all 4 wheels
                vehicle = hmmwv.GetVehicle()
                
                for axle in range(2):
                    for side in range(2):
                        side_enum = veh.LEFT if side == 0 else veh.RIGHT
                        tire = vehicle.GetTire(axle, side_enum)
                        
                        # Get tire force
                        tire_force = tire.ReportTireForce(scm_terrain)
                        force_local = rot_inv.Rotate(tire_force.force)
                        
                        # Compute slip angle
                        L = Lf if axle == 0 else -Lr
                        wheel_delta = delta if axle == 0 else 0.0
                        
                        eps = 0.5
                        u_safe = max(u, eps)
                        alpha = np.arctan2(v + L * omega, u_safe) - wheel_delta
                        
                        # Store measurement
                        m = TireMeasurement(
                            time=t,
                            axle=axle,
                            side=side,
                            slip_angle=alpha,
                            slip_ratio=0.0,
                            camber=0.0,
                            velocity=u,
                            Fz=force_local.z,
                            Fx=force_local.x,
                            Fy=force_local.y,
                            bekker_Kphi=terrain.bekker_Kphi,
                            bekker_Kc=terrain.bekker_Kc,
                            bekker_n=terrain.bekker_n,
                            mohr_cohesion=terrain.mohr_cohesion,
                            mohr_friction=terrain.mohr_friction,
                            janosi_shear=terrain.janosi_shear,
                        )
                        
                        # Filter for reasonable samples
                        if m.Fz > 1000 and abs(m.Fy) < 20000 and abs(alpha) > 0.01:
                            measurements.append(m)
        
        sample_count += 1
        t += step_size
    
    if verbose:
        print(f"Sample {sample_idx}: collected {len(measurements)} tire measurements "
              f"(n={terrain.bekker_n:.2f}, phi={terrain.mohr_friction:.1f}°)")
    
    return measurements


def run_single_sim_wrapper(args):
    """Wrapper for multiprocessing"""
    sample_idx, seed = args
    rng = np.random.default_rng(seed)
    terrain = generate_random_terrain(rng)
    
    try:
        measurements = run_vehicle_simulation(
            terrain=terrain,
            sample_idx=sample_idx,
            sim_duration=15.0,
            step_size=3e-3,
            mesh_spacing=0.1,
            verbose=False,
        )
        return measurements
    except Exception as e:
        print(f"Warning: Simulation {sample_idx} failed: {e}")
        return []


def main():
    parser = argparse.ArgumentParser(
        description="Collect tire force data from full vehicle SCM simulation"
    )
    parser.add_argument("-n", type=int, default=100, help="Number of simulations")
    parser.add_argument("-o", type=str, default="vehicle_tire_data.csv", help="Output CSV")
    parser.add_argument("-j", type=int, default=4, help="Parallel processes")
    parser.add_argument("-t", type=float, default=15.0, help="Simulation duration (s)")
    parser.add_argument("--seed", type=int, default=12345, help="Random seed")
    parser.add_argument("-v", action="store_true", help="Verbose output")
    args = parser.parse_args()
    
    print("=" * 66)
    print("Full Vehicle SCM Data Collection")
    print("=" * 66)
    print(f"Simulations: {args.n}")
    print(f"Processes: {args.j}")
    print(f"Duration each: {args.t}s")
    print(f"Output: {args.o}")
    print("=" * 66)
    
    # Open output file
    csv_file = open(args.o, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "vertical_load", "slip_angle", "longitudinal_slip", "camber_angle", "velocity",
        "bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction",
        "janosi_shear", "mesh_spacing", "Fz", "Fx", "Fy"
    ])
    
    start_time = time.time()
    all_measurements = []
    
    # Prepare simulation arguments
    sim_args = [(i, args.seed + i * 1000) for i in range(args.n)]
    
    if args.j > 1:
        # Multiprocessing
        with Pool(args.j) as pool:
            for i, measurements in enumerate(pool.imap_unordered(run_single_sim_wrapper, sim_args)):
                all_measurements.extend(measurements)
                
                # Write measurements to CSV
                for m in measurements:
                    writer.writerow([
                        m.Fz,  # vertical_load
                        m.slip_angle,
                        m.slip_ratio,
                        m.camber,
                        m.velocity,
                        m.bekker_Kphi,
                        m.bekker_Kc,
                        m.bekker_n,
                        m.mohr_cohesion,
                        m.mohr_friction,
                        m.janosi_shear,
                        0.1,  # mesh_spacing
                        m.Fz,
                        m.Fx,
                        m.Fy,
                    ])
                csv_file.flush()
                
                # Progress
                done = i + 1
                if done % 10 == 0 or done == args.n:
                    elapsed = time.time() - start_time
                    rate = done / elapsed
                    remaining = (args.n - done) / rate if rate > 0 else 0
                    print(f"\rProgress: {done}/{args.n} ({100*done//args.n}%) | "
                          f"Samples: {len(all_measurements)} | ETA: {int(remaining)}s", 
                          end="", flush=True)
    else:
        # Single process
        for i, (sample_idx, seed) in enumerate(sim_args):
            rng = np.random.default_rng(seed)
            terrain = generate_random_terrain(rng)
            measurements = run_vehicle_simulation(
                terrain=terrain,
                sample_idx=sample_idx,
                sim_duration=args.t,
                verbose=args.v,
            )
            all_measurements.extend(measurements)
            
            for m in measurements:
                writer.writerow([
                    m.Fz, m.slip_angle, m.slip_ratio, m.camber, m.velocity,
                    m.bekker_Kphi, m.bekker_Kc, m.bekker_n, m.mohr_cohesion,
                    m.mohr_friction, m.janosi_shear, 0.1, m.Fz, m.Fx, m.Fy,
                ])
            csv_file.flush()
            
            if args.v or (i + 1) % 10 == 0:
                print(f"Progress: {i+1}/{args.n}")
    
    csv_file.close()
    
    total_time = time.time() - start_time
    
    print()
    print("=" * 66)
    print("Collection complete!")
    print(f"  Total simulations: {args.n}")
    print(f"  Total tire samples: {len(all_measurements)}")
    print(f"  Time: {total_time:.1f}s")
    print(f"  Rate: {args.n/total_time:.2f} sims/s")
    print(f"  Output: {args.o}")
    print("=" * 66)


if __name__ == "__main__":
    main()
