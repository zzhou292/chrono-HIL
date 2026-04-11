#!/usr/bin/env python3
"""
Temporal SCM Data Collection for NN Tire Model Training (Python port)

Records time-series of tire forces under time-varying slip angle profiles.
Each scenario produces multiple timestep records (instead of a single
steady-state measurement), enabling the NN to learn transient tire dynamics
such as tire relaxation and soil deformation history.

Slip angle profiles: polynomial (degree 1-3) to create diverse transients.
Recording rate: configurable (default 5ms = 200Hz).

Output CSV columns:
  scenario_id, timestep, slip_ratio, slip_angle, velocity, vertical_load,
  steering_rate, bekker_Kphi, ..., janosi_shear, mesh_spacing, Fz, Fx, Fy

Usage:
  python collect_temporal_data.py [num_scenarios] [output.csv] [options]
  python collect_temporal_data.py --visualize          # 1 scenario with Irrlicht
  python collect_temporal_data.py 3000 data.csv -t 6   # 3000 scenarios, 6 workers
  python collect_temporal_data.py -c                    # continuous mode
"""

import argparse
import csv
import io
import math
import os
import signal
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Simulation parameters (must match C++ version exactly)
# ---------------------------------------------------------------------------
STEP_SIZE = 5e-4
T_DELAY = 0.5            # Settling time before recording
T_RECORD_DURATION = 2.0  # Duration of force recording
T_END = T_DELAY + T_RECORD_DURATION
RECORD_DT = 0.005        # 5 ms recording interval (200 Hz)
SOLVER_ITERS = 50
TIRE_RADIUS = 0.47

CSV_HEADER = (
    "scenario_id,timestep,slip_ratio,slip_angle,velocity,vertical_load,"
    "steering_rate,bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,"
    "mohr_friction,janosi_shear,mesh_spacing,Fz,Fx,Fy\n"
)

# ---------------------------------------------------------------------------
# Parameter ranges (reference paper Table I)
# ---------------------------------------------------------------------------
RANGES = {
    "slip_angle_base": (-0.40, 0.40),
    "slip_ratio":      (-1.0, 1.0),
    "velocity":        (2.0, 10.0),
    "vertical_load":   (2500.0, 7500.0),
    "sr_base":         (-0.56, 0.56),
    "sr_accel":        (-1.5, 1.5),
    "sr_jerk":         (-3.0, 3.0),
    "bekker_Kphi":     (0.5e6, 4.0e6),
    "bekker_Kc":       (0.0, 20000.0),
    "bekker_n":        (0.3, 1.3),
    "mohr_cohesion":   (650.0, 20700.0),
    "mohr_friction":   (6.0, 37.8),
    "janosi_shear":    (0.01, 0.025),
    "mesh_spacing":    (0.05, 0.15),
}


# ---------------------------------------------------------------------------
# Sample parameters dataclass
# ---------------------------------------------------------------------------
@dataclass
class SampleParams:
    slip_ratio: float = 0.0
    velocity: float = 5.0
    vertical_load: float = 4000.0
    profile_degree: int = 1
    poly_a0: float = 0.0
    poly_a1: float = 0.0
    poly_a2: float = 0.0
    poly_a3: float = 0.0
    bekker_Kphi: float = 2e6
    bekker_Kc: float = 5000.0
    bekker_n: float = 0.8
    mohr_cohesion: float = 5000.0
    mohr_friction: float = 20.0   # degrees internally
    janosi_shear: float = 0.015
    mesh_spacing: float = 0.10

    def alpha(self, t: float) -> float:
        return self.poly_a0 + self.poly_a1 * t + self.poly_a2 * t**2 + self.poly_a3 * t**3

    def steering_rate(self, t: float) -> float:
        return self.poly_a1 + 2.0 * self.poly_a2 * t + 3.0 * self.poly_a3 * t**2


# ---------------------------------------------------------------------------
# LHS sampling
# ---------------------------------------------------------------------------
def _lhs_dim(n: int, lo: float, hi: float, rng: np.random.Generator) -> np.ndarray:
    perm = rng.permutation(n)
    u = rng.uniform(0.0, 1.0, n)
    return lo + ((perm + u) / n) * (hi - lo)


def generate_lhs_samples(n: int, seed: int = 42) -> List[SampleParams]:
    rng = np.random.default_rng(seed)
    R = RANGES

    alpha_bases = _lhs_dim(n, *R["slip_angle_base"], rng)
    slip_ratios = _lhs_dim(n, *R["slip_ratio"], rng)
    velocities  = _lhs_dim(n, *R["velocity"], rng)
    vert_loads  = _lhs_dim(n, *R["vertical_load"], rng)
    sr_bases    = _lhs_dim(n, *R["sr_base"], rng)
    sr_accels   = _lhs_dim(n, *R["sr_accel"], rng)
    sr_jerks    = _lhs_dim(n, *R["sr_jerk"], rng)
    bk_Kphis    = _lhs_dim(n, *R["bekker_Kphi"], rng)
    bk_Kcs      = _lhs_dim(n, *R["bekker_Kc"], rng)
    bk_ns       = _lhs_dim(n, *R["bekker_n"], rng)
    mc_cohes    = _lhs_dim(n, *R["mohr_cohesion"], rng)
    mc_fricts   = _lhs_dim(n, *R["mohr_friction"], rng)
    j_shears    = _lhs_dim(n, *R["janosi_shear"], rng)
    meshes      = _lhs_dim(n, *R["mesh_spacing"], rng)

    samples = []
    for i in range(n):
        s = SampleParams(
            slip_ratio=slip_ratios[i],
            velocity=velocities[i],
            vertical_load=vert_loads[i],
            bekker_Kphi=bk_Kphis[i],
            bekker_Kc=bk_Kcs[i],
            bekker_n=bk_ns[i],
            mohr_cohesion=mc_cohes[i],
            mohr_friction=mc_fricts[i],
            janosi_shear=j_shears[i],
            mesh_spacing=meshes[i],
        )
        # Profile degree: cycle 1, 2, 3
        s.profile_degree = 1 + (i % 3)
        s.poly_a0 = alpha_bases[i]
        s.poly_a1 = sr_bases[i]
        s.poly_a2 = sr_accels[i] / 2.0 if s.profile_degree >= 2 else 0.0
        s.poly_a3 = sr_jerks[i] / 6.0  if s.profile_degree >= 3 else 0.0

        # Clamp polynomial to keep alpha within [-0.6, 0.6]
        ALPHA_LIMIT = 0.6
        max_alpha = max(
            abs(s.alpha(T_DELAY + k * T_RECORD_DURATION / 20.0))
            for k in range(21)
        )
        if max_alpha > ALPHA_LIMIT:
            scale = ALPHA_LIMIT / max_alpha * 0.95
            s.poly_a0 *= scale
            s.poly_a1 *= scale
            s.poly_a2 *= scale
            s.poly_a3 *= scale

        samples.append(s)
    return samples


# ---------------------------------------------------------------------------
# Run one scenario — returns list of (time, slip_angle, steering_rate, Fz, Fx, Fy)
# ---------------------------------------------------------------------------
def collect_one_scenario(
    params: SampleParams,
    scenario_id: int,
    visualize: bool = False,
) -> List[Tuple[float, float, float, float, float, float]]:
    """Run one ChTireTestRig scenario and return timestep records."""
    import pychrono.core as chrono
    import pychrono.vehicle as veh

    records = []
    try:
        system = chrono.ChSystemNSC()
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
        solver = chrono.CastToChIterativeSolverVI(system.GetSolver())
        solver.SetMaxIterations(SOLVER_ITERS)

        data_path = chrono.GetChronoDataPath()
        wheel = veh.ReadWheelJSON(data_path + "vehicle/hmmwv/wheel/HMMWV_Wheel.json")
        tire  = veh.ReadTireJSON(data_path + "vehicle/hmmwv/tire/HMMWV_RigidTire.json")
        tire.SetStepsize(STEP_SIZE)

        rig = veh.ChTireTestRig(wheel, tire, system)
        rig.SetGravitationalAcceleration(9.8)
        rig.SetNormalLoad(params.vertical_load)
        rig.SetCamberAngle(0.0)
        rig.SetTireStepsize(STEP_SIZE)
        rig.SetTireCollisionType(veh.ChTire.CollisionType_FOUR_POINTS)
        rig.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

        # SCM terrain (use the convenience overload with individual params)
        rig.SetTerrainSCM(
            params.bekker_Kphi,
            params.bekker_Kc,
            params.bekker_n,
            params.mohr_cohesion,
            params.mohr_friction,
            params.janosi_shear,
            params.mesh_spacing,
            200.0,   # terrain_length
            1.0,     # terrain_width
        )

        # Motion functions
        ang_speed = (params.velocity / TIRE_RADIUS) * (1.0 + params.slip_ratio)

        # Build slip angle function via interpolation points
        n_pts = 500
        times = [T_END * k / n_pts for k in range(n_pts + 1)]
        values = [params.alpha(t) for t in times]
        rig.SetSlipAngleInterp(times, values)

        rig.SetConstantLongSpeed(params.velocity)
        rig.SetConstantAngSpeed(ang_speed)
        rig.SetTimeDelay(T_DELAY)
        rig.Initialize(veh.ChTireTestRig.Mode_TEST)

        # Visualization (optional)
        vis = None
        if visualize:
            try:
                import pychrono.irrlicht as irr
                vis = irr.ChVisualSystemIrrlicht()
                vis.AttachSystem(system)
                vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
                vis.SetWindowSize(1200, 600)
                vis.SetWindowTitle(f"Temporal Data - Scenario {scenario_id}")
                vis.Initialize()
                vis.AddLogo()
                vis.AddSkyBox()
                vis.AddCamera(chrono.ChVector3d(1.0, 2.5, 1.0))
                vis.AddLightDirectional()
            except ImportError:
                print("Warning: pychrono.irrlicht not available, running without visualization")
                vis = None

        # Simulate and record
        t = 0.0
        t_next_record = T_DELAY
        render_step = 1.0 / 60.0
        next_render = 0.0
        t_vis_end = T_END + 3.0 if visualize else T_END

        while t < t_vis_end:
            # Render
            if vis is not None and t >= next_render:
                loc = rig.GetPos()
                vis.UpdateCamera(
                    loc + chrono.ChVector3d(1.5, 3.0, 1.0), loc
                )
                if not vis.Run():
                    break
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                next_render += render_step

            rig.Advance(STEP_SIZE)
            t += STEP_SIZE

            if t >= t_next_record and t <= T_END:
                force = rig.ReportTireForce()
                fv = force.force
                records.append((
                    t,
                    params.alpha(t),
                    params.steering_rate(t),
                    fv.z, fv.x, fv.y,
                ))
                t_next_record += RECORD_DT

        system.Clear()

    except Exception as e:
        print(f"Scenario {scenario_id} failed: {e}", file=sys.stderr)
        records = []

    return records


# ---------------------------------------------------------------------------
# Format one scenario's records as CSV text
# ---------------------------------------------------------------------------
def format_records(scenario_id: int, params: SampleParams, records) -> str:
    if not records:
        return ""
    mohr_friction_rad = math.radians(params.mohr_friction)
    buf = io.StringIO()
    for i, (t, sa, sr, fz, fx, fy) in enumerate(records):
        buf.write(
            f"{scenario_id},{i},{params.slip_ratio:.6f},{sa:.6f},"
            f"{params.velocity:.6f},{params.vertical_load:.6f},{sr:.6f},"
            f"{params.bekker_Kphi:.6f},{params.bekker_Kc:.6f},{params.bekker_n:.6f},"
            f"{params.mohr_cohesion:.6f},{mohr_friction_rad:.6f},"
            f"{params.janosi_shear:.6f},{params.mesh_spacing:.6f},"
            f"{fz:.6f},{fx:.6f},{fy:.6f}\n"
        )
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Worker function for multiprocessing (top-level for pickling)
# ---------------------------------------------------------------------------
def _worker(args):
    """Run a single scenario in a subprocess. Returns (scenario_id, csv_text)."""
    params_dict, scenario_id = args
    p = SampleParams(**params_dict)
    recs = collect_one_scenario(p, scenario_id, visualize=False)
    return scenario_id, format_records(scenario_id, p, recs)


# ---------------------------------------------------------------------------
# Global stop flag
# ---------------------------------------------------------------------------
_stop = False


def _signal_handler(signum, frame):
    global _stop
    _stop = True
    print("\nInterrupted — finishing current batch...")


# ---------------------------------------------------------------------------
# Main collection routines
# ---------------------------------------------------------------------------
def collect_with_workers(
    n_samples: int,
    output_file: str,
    num_threads: int,
    batch_size: int,
):
    """Fixed-count collection with ProcessPoolExecutor."""
    global _stop
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    samples = generate_lhs_samples(n_samples)
    timesteps_per = int(T_RECORD_DURATION / RECORD_DT)

    print(f"\n=== Temporal SCM Data Collection (Python) ===")
    print(f"Scenarios: {n_samples}")
    print(f"Workers: {num_threads}")
    print(f"Recording: every {RECORD_DT*1000:.0f}ms over {T_RECORD_DURATION}s = ~{timesteps_per} timesteps/scenario")

    # Warm-up: run first scenario single-threaded
    print("Warm-up scenario 0...")
    recs = collect_one_scenario(samples[0], 0)
    warm_text = format_records(0, samples[0], recs)

    with open(output_file, "w") as f:
        f.write(CSV_HEADER)
        if warm_text:
            f.write(warm_text)

    t0 = time.time()
    success = 1 if warm_text else 0
    completed = 1

    # Prepare work items (serialize params as dict for pickling)
    work = [
        (vars(samples[i]), i) for i in range(1, n_samples)
    ]

    with open(output_file, "a") as f:
        with ProcessPoolExecutor(max_workers=num_threads) as executor:
            # Submit in batches to allow graceful shutdown
            for batch_start in range(0, len(work), batch_size):
                if _stop:
                    break
                batch = work[batch_start:batch_start + batch_size]
                futures = {executor.submit(_worker, w): w for w in batch}
                for future in as_completed(futures):
                    if _stop:
                        break
                    try:
                        sid, csv_text = future.result()
                        if csv_text:
                            f.write(csv_text)
                            f.flush()
                            success += 1
                    except Exception as e:
                        print(f"Worker error: {e}", file=__import__('sys').stderr)
                    completed += 1
                    if completed % 10 == 0 or completed == n_samples:
                        elapsed = time.time() - t0
                        rate = completed / elapsed if elapsed > 0 else 0
                        eta = (n_samples - completed) / rate if rate > 0 else 0
                        print(
                            f"Progress: {completed}/{n_samples} "
                            f"({100.0*completed/n_samples:.1f}%) "
                            f"{rate:.2f} scen/s ETA: {eta:.0f}s"
                        )

    elapsed = time.time() - t0
    print(f"\n=== Collection Complete ===")
    print(f"Time: {elapsed:.1f}s")
    print(f"Scenarios: {success}/{completed}")
    print(f"Output: {output_file}")


def collect_sequential(
    n_samples: int,
    output_file: str,
):
    """Single-threaded collection."""
    global _stop
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    samples = generate_lhs_samples(n_samples)
    timesteps_per = int(T_RECORD_DURATION / RECORD_DT)

    print(f"\n=== Temporal SCM Data Collection (Python, sequential) ===")
    print(f"Scenarios: {n_samples}")
    print(f"Recording: every {RECORD_DT*1000:.0f}ms over {T_RECORD_DURATION}s = ~{timesteps_per} timesteps/scenario")

    t0 = time.time()
    success = 0
    with open(output_file, "w") as f:
        f.write(CSV_HEADER)
        for i in range(n_samples):
            if _stop:
                break
            recs = collect_one_scenario(samples[i], i)
            text = format_records(i, samples[i], recs)
            if text:
                f.write(text)
                f.flush()
                success += 1
            done = i + 1
            if done % 10 == 0 or done == n_samples:
                elapsed = time.time() - t0
                rate = done / elapsed if elapsed > 0 else 0
                eta = (n_samples - done) / rate if rate > 0 else 0
                print(
                    f"Progress: {done}/{n_samples} "
                    f"({100.0*done/n_samples:.1f}%) "
                    f"{rate:.2f} scen/s ETA: {eta:.0f}s"
                )

    elapsed = time.time() - t0
    print(f"\n=== Collection Complete ===")
    print(f"Time: {elapsed:.1f}s")
    print(f"Scenarios: {success}/{n_samples}")
    print(f"Output: {output_file}")


def collect_continuous(
    output_file: str,
    num_threads: int,
):
    """Continuous collection until Ctrl+C."""
    global _stop
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"\n=== Temporal SCM Data Collection (Python, continuous) ===")
    print(f"Workers: {num_threads}")
    print(f"Press Ctrl+C to stop.")

    # Warm-up
    warm = SampleParams()
    recs = collect_one_scenario(warm, 0)
    with open(output_file, "w") as f:
        f.write(CSV_HEADER)
        text = format_records(0, warm, recs)
        if text:
            f.write(text)

    t0 = time.time()
    completed = 1
    success = 1 if recs else 0
    batch_num = 0
    rng = np.random.default_rng()

    with open(output_file, "a") as f:
        while not _stop:
            # Generate a random batch
            batch_sz = max(num_threads, 1)
            batch_samples = []
            for _ in range(batch_sz):
                p = SampleParams()
                R = RANGES
                for attr, key in [
                    ("slip_ratio", "slip_ratio"), ("velocity", "velocity"),
                    ("vertical_load", "vertical_load"),
                    ("poly_a0", "slip_angle_base"), ("poly_a1", "sr_base"),
                    ("bekker_Kphi", "bekker_Kphi"), ("bekker_Kc", "bekker_Kc"),
                    ("bekker_n", "bekker_n"), ("mohr_cohesion", "mohr_cohesion"),
                    ("mohr_friction", "mohr_friction"), ("janosi_shear", "janosi_shear"),
                    ("mesh_spacing", "mesh_spacing"),
                ]:
                    lo, hi = R[key]
                    setattr(p, attr, rng.uniform(lo, hi))
                p.profile_degree = rng.integers(1, 4)
                p.poly_a2 = rng.uniform(*R["sr_accel"]) / 2.0 if p.profile_degree >= 2 else 0.0
                p.poly_a3 = rng.uniform(*R["sr_jerk"]) / 6.0 if p.profile_degree >= 3 else 0.0
                # Clamp
                max_a = max(abs(p.alpha(T_DELAY + k * T_RECORD_DURATION / 20.0)) for k in range(21))
                if max_a > 0.6:
                    sc = 0.57 / max_a
                    p.poly_a0 *= sc; p.poly_a1 *= sc; p.poly_a2 *= sc; p.poly_a3 *= sc
                batch_samples.append(p)

            if num_threads > 1:
                work = [(vars(batch_samples[i]), completed + i) for i in range(batch_sz)]
                with ProcessPoolExecutor(max_workers=num_threads) as executor:
                    futures = [executor.submit(_worker, w) for w in work]
                    for fut in as_completed(futures):
                        if _stop:
                            break
                        try:
                            sid, csv_text = fut.result()
                            if csv_text:
                                f.write(csv_text)
                                f.flush()
                                success += 1
                        except Exception:
                            pass
                        completed += 1
            else:
                for i, p in enumerate(batch_samples):
                    if _stop:
                        break
                    recs = collect_one_scenario(p, completed)
                    text = format_records(completed, p, recs)
                    if text:
                        f.write(text)
                        f.flush()
                        success += 1
                    completed += 1

            if completed % 10 < batch_sz:
                elapsed = time.time() - t0
                rate = completed / elapsed if elapsed > 0 else 0
                print(f"Scenarios: {completed} (ok: {success}) {rate:.2f} scen/s")

    elapsed = time.time() - t0
    print(f"\n=== Collection Complete ===")
    print(f"Time: {elapsed:.1f}s")
    print(f"Scenarios: {success}/{completed}")
    print(f"Output: {output_file}")


def run_visualize(output_file: str, scenario_idx: int = 0):
    """Run 1 scenario with Irrlicht visualization."""
    n_needed = scenario_idx + 1
    samples = generate_lhs_samples(n_needed)
    s = samples[scenario_idx]
    print(f"\n=== Visualization Mode ===")
    print(f"Scenario {scenario_idx} (degree {s.profile_degree})")
    print(f"  slip_ratio={s.slip_ratio:.3f}  velocity={s.velocity:.1f}  load={s.vertical_load:.0f}")
    print(f"  alpha(t=0.5)={math.degrees(s.alpha(T_DELAY)):+.1f} deg  alpha(t=2.5)={math.degrees(s.alpha(T_END)):+.1f} deg")
    recs = collect_one_scenario(s, scenario_idx, visualize=True)
    print(f"Recorded {len(recs)} timesteps")
    if recs:
        with open(output_file, "w") as f:
            f.write(CSV_HEADER)
            f.write(format_records(scenario_idx, s, recs))
        print(f"Output: {output_file}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Collect time-series tire force data for temporal NN training.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            f"Each scenario records ~{int(T_RECORD_DURATION/RECORD_DT)} timesteps "
            f"at {RECORD_DT*1000:.0f}ms intervals.\n\n"
            "Slip angle profiles:\n"
            "  Degree 1 (33%%): linear ramp (constant steering rate)\n"
            "  Degree 2 (33%%): quadratic (accelerating steering)\n"
            "  Degree 3 (33%%): cubic (rich transient dynamics)\n"
        ),
    )
    parser.add_argument("num_scenarios", nargs="?", type=int, default=None,
                        help="Number of scenarios (omit for continuous mode)")
    parser.add_argument("output", nargs="?", default="scm_temporal_data.csv",
                        help="Output CSV file (default: scm_temporal_data.csv)")
    parser.add_argument("-v", "--visualize", action="store_true",
                        help="Run 1 scenario with Irrlicht visualization")
    parser.add_argument("-s", "--sequential", action="store_true",
                        help="Single-threaded (no multiprocessing)")
    parser.add_argument("-t", "--threads", type=int, default=0,
                        help="Number of worker processes (0=auto)")
    parser.add_argument("-b", "--batch-size", type=int, default=0,
                        help="Batch size for submission (0=all at once)")
    parser.add_argument("-c", "--continuous", action="store_true",
                        help="Continuous mode (Ctrl+C to stop)")
    parser.add_argument("--scenario", type=int, default=0,
                        help="Scenario index to visualize (default: 0)")

    args = parser.parse_args()

    # Determine thread count
    if args.threads <= 0:
        args.threads = max(os.cpu_count() - 2, 1)
    if args.batch_size <= 0:
        args.batch_size = max(args.threads * 4, 20)

    if args.visualize:
        run_visualize(args.output, args.scenario)
        return

    if args.continuous or args.num_scenarios is None:
        collect_continuous(args.output, 1 if args.sequential else args.threads)
        return

    n = args.num_scenarios
    if args.sequential:
        collect_sequential(n, args.output)
    else:
        collect_with_workers(n, args.output, args.threads, args.batch_size)


if __name__ == "__main__":
    main()
