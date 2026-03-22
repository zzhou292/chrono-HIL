#!/usr/bin/env python3
"""
Launch script for decoupled Chrono simulation + MPC controller.
================================================================

Starts two processes:
  1. chrono_sim_node.py   — PyChrono HMMWV simulation (publishes state, receives commands)
  2. mpc_controller_node.py — MPC controller (receives state, publishes commands)

Usage:
    # Default (NN model, sand terrain, lane change, irrlicht visualization)
    python launch_decoupled.py

    # Sinusoidal path on clay, headless, 30s
    python launch_decoupled.py --path sinusoidal --terrain clay --no-vis --time 30

    # Sensor-only visualization (driver POV camera)
    python launch_decoupled.py --vis-mode sensor

    # Both irrlicht chase cam and sensor driver POV camera
    python launch_decoupled.py --vis-mode both

    # TMeasy MPC tire model
    python launch_decoupled.py --model tmeasy

    # Pacejka MPC tire model with sensor visualization
    python launch_decoupled.py --model pacejka --vis-mode sensor

    # Simple linear tire model, no delay compensation
    python launch_decoupled.py --model linear --no-delay-comp

    # Remote controller (sim on this machine, controller elsewhere)
    python launch_decoupled.py --ctrl-host 192.168.1.50
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(
        description="Launch decoupled Chrono sim + MPC controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --path sinusoidal --terrain clay --time 20
  %(prog)s --model linear --no-vis
  %(prog)s --model pacejka                   # Pacejka Magic Formula MPC
  %(prog)s --model tmeasy                    # TMeasy MPC tire model
  %(prog)s --vis-mode sensor                 # Driver POV via Chrono Sensor
  %(prog)s --vis-mode both                   # Irrlicht + Sensor simultaneously
  %(prog)s --sim-only          # Only start the sim node (controller started separately)
  %(prog)s --ctrl-only         # Only start the controller node
""",
    )

    # Shared args
    p.add_argument("--time", type=float, default=15.0, help="Simulation time (s)")
    p.add_argument("--speed", type=float, default=8.0, help="Target speed (m/s)")
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--terrain-config", type=str, default=None)
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "sinusoidal"])
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=0.0,
                   help="Straight lead-in distance (m) before path starts")
    p.add_argument("--no-vis", action="store_true", help="Headless simulation (alias for --vis-mode none)")
    p.add_argument("--vis-mode", default=None,
                   choices=["irrlicht", "sensor", "both", "none"],
                   help="Visualization mode: irrlicht, sensor (driver POV), both, or none")
    p.add_argument("--no-rt",  action="store_true",
                   help="Disable real-time pacing (fast-forward; breaks MPC sync)")
    p.add_argument("--no-noise", action="store_true",
                   help="Disable sensor noise (noise ON by default)")

    # Controller-specific
    p.add_argument("--model", default="nn",
                   choices=["nn", "pacejka", "tmeasy", "linear"],
                   help="MPC tire model: nn, pacejka (Magic Formula), tmeasy, or linear")
    p.add_argument("--nn-model", default="v6")
    p.add_argument("--kappa", default="zero", choices=["zero", "approx"])
    p.add_argument("--no-lat-transfer", action="store_true")
    p.add_argument("--no-delay-comp", action="store_true")
    p.add_argument("--no-path-reindex", action="store_true")
    p.add_argument("--rms-time-start", type=float, default=2.0,
                   help="Start time for RMS calculation (s)")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip generating end-of-run plots")
    p.add_argument("--no-csv", action="store_true",
                   help="Skip diagnostic CSV output")
    p.add_argument("--plot-dir", default="plots",
                   help="Directory for output plots")

    # Terrain bumps
    p.add_argument("--bump", type=float, default=0.0)
    p.add_argument("--bump-wavelength", type=float, default=20.0)
    p.add_argument("--bump-octaves", type=int, default=4)
    p.add_argument("--bump-seed", type=int, default=12345)
    p.add_argument("--bump-max-slope", type=float, default=0.3)

    # Network
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--ctrl-host", default="localhost",
                   help="Host running the controller (for sim to subscribe)")

    # Mode
    p.add_argument("--sim-only", action="store_true",
                   help="Only launch the simulation node")
    p.add_argument("--ctrl-only", action="store_true",
                   help="Only launch the controller node")

    args = p.parse_args()

    script_dir = Path(__file__).parent

    # Resolve vis mode: --no-vis is shorthand for --vis-mode none
    vis_mode = args.vis_mode
    if vis_mode is None:
        vis_mode = 'none' if args.no_vis else 'irrlicht'

    # ---- Build command lines ----
    sim_cmd = [
        sys.executable, str(script_dir / "chrono_sim_node.py"),
        "--time", str(args.time),
        "--speed", str(args.speed),
        "--terrain", args.terrain,
        "--path", args.path,
        "--sine-amplitude", str(args.sine_amplitude),
        "--sine-wavelength", str(args.sine_wavelength),
        "--lead-in", str(args.lead_in),
        "--sim-port", str(args.sim_port),
        "--ctrl-host", args.ctrl_host,
        "--ctrl-port", str(args.ctrl_port),
        "--bump", str(args.bump),
        "--bump-wavelength", str(args.bump_wavelength),
        "--bump-octaves", str(args.bump_octaves),
        "--bump-seed", str(args.bump_seed),
        "--bump-max-slope", str(args.bump_max_slope),
        "--vis-mode", vis_mode,
    ]
    if args.no_rt:
        sim_cmd.append("--no-rt")
    if args.no_noise:
        sim_cmd.append("--no-noise")
    if args.terrain_config:
        sim_cmd.extend(["--terrain-config", args.terrain_config])

    ctrl_cmd = [
        sys.executable, str(script_dir / "mpc_controller_node.py"),
        "--model", args.model,
        "--nn-model", args.nn_model,
        "--kappa", args.kappa,
        "--path", args.path,
        "--speed", str(args.speed),
        "--terrain", args.terrain,
        "--time", str(args.time),
        "--sine-amplitude", str(args.sine_amplitude),
        "--sine-wavelength", str(args.sine_wavelength),
        "--lead-in", str(args.lead_in),
        "--sim-host", "localhost",
        "--sim-port", str(args.sim_port),
        "--ctrl-port", str(args.ctrl_port),
        "--rms-time-start", str(args.rms_time_start),
        "--plot-dir", args.plot_dir,
    ]
    if args.no_delay_comp:
        ctrl_cmd.append("--no-delay-comp")
    if args.no_lat_transfer:
        ctrl_cmd.append("--no-lat-transfer")
    if args.no_path_reindex:
        ctrl_cmd.append("--no-path-reindex")
    if args.no_plot:
        ctrl_cmd.append("--no-plot")
    if args.no_csv:
        ctrl_cmd.append("--no-csv")

    # ---- Launch ----
    procs = []

    def cleanup(signum=None, frame=None):
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            proc.wait(timeout=5)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        if args.ctrl_only:
            print(f"[launch] Starting controller only")
            print(f"  cmd: {' '.join(ctrl_cmd)}")
            proc = subprocess.Popen(ctrl_cmd)
            procs.append(proc)
            proc.wait()
        elif args.sim_only:
            print(f"[launch] Starting simulation only")
            print(f"  cmd: {' '.join(sim_cmd)}")
            proc = subprocess.Popen(sim_cmd)
            procs.append(proc)
            proc.wait()
        else:
            # Start controller first (it will wait for config from sim)
            print(f"[launch] Starting controller...")
            ctrl_proc = subprocess.Popen(ctrl_cmd)
            procs.append(ctrl_proc)

            # Brief delay, then start simulation
            time.sleep(0.5)
            print(f"[launch] Starting simulation...")
            sim_proc = subprocess.Popen(sim_cmd)
            procs.append(sim_proc)

            # Wait for simulation to finish
            sim_proc.wait()
            print("[launch] Simulation finished. Waiting for controller...")

            # Give controller a moment to process the stop signal
            ctrl_proc.wait(timeout=10)

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    main()
