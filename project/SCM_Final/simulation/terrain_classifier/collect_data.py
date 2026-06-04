#!/usr/bin/env python3
"""
Training Data Collector for Terrain Classification
====================================================

Subscribes to the Chrono simulation via ZMQ, extracts vehicle-measurable
features over sliding windows, and writes labeled CSV rows.  The terrain
label comes from the SimStatus config message broadcast at startup.

Usage:
    # Run alongside chrono_sim_node.py (must be started first):
    python -m terrain_classifier.collect_data --sim-host localhost --sim-port 5555 \\
        --output training_data.csv --append

    # Automated: loop over terrain presets (use with launch_data_collection.py)
    python -m terrain_classifier.collect_data --terrain-label sand --output data/sand_run1.csv

Produces a CSV with columns = FeatureVector.feature_names() + ["terrain_label"].
"""

import argparse
import csv
import sys
import time
from pathlib import Path

# Ensure parent is on the path for sibling imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQSubscriber, sim_sub_endpoint, ctrl_sub_endpoint,
)
from terrain_classifier.feature_extractor import FeatureExtractor, FeatureVector


def collect(args):
    print("=" * 60)
    print("Terrain Classifier — Training Data Collector")
    print("=" * 60)

    endpoint = sim_sub_endpoint(args.sim_host, args.sim_port)
    sub = ZMQSubscriber(endpoint)
    print(f"  Subscribed to state: {endpoint}")

    ctrl_endpoint = ctrl_sub_endpoint(args.ctrl_host, args.ctrl_port)
    ctrl_sub = ZMQSubscriber(ctrl_endpoint)
    print(f"  Subscribed to ctrl:  {ctrl_endpoint}")

    extractor = FeatureExtractor(
        window_sec=args.window,
        stride_sec=args.stride,
        min_speed=args.min_speed,
    )

    terrain_label = args.terrain_label  # may be overridden by SimStatus config
    feature_names = FeatureVector.feature_names()

    # Open CSV output
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = out_path.exists() and args.append
    mode = "a" if args.append else "w"
    csv_file = open(out_path, mode, newline="")
    writer = csv.writer(csv_file)

    # Write header (only if new file or not appending)
    if not file_exists:
        writer.writerow(feature_names + ["terrain_label"])
        csv_file.flush()

    print(f"  Output: {out_path} ({'append' if args.append else 'overwrite'})")
    print(f"  Window: {args.window}s, stride: {args.stride}s, min_speed: {args.min_speed} m/s")

    last_steering = 0.0
    sample_count = 0
    msg_count = 0
    config_received = False

    print("  Waiting for simulation data...")

    try:
        while True:
            # Poll control commands (non-blocking) to track steering
            ctrl_result = ctrl_sub.recv(timeout_ms=0)
            if ctrl_result is not None:
                _, ctrl_msg = ctrl_result
                if isinstance(ctrl_msg, ControlCommand):
                    last_steering = ctrl_msg.steering

            result = sub.recv(timeout_ms=200)
            if result is None:
                continue

            topic, msg = result
            msg_count += 1

            # Pick up terrain label from config message
            if isinstance(msg, SimStatus) and msg.event == "config":
                cfg = msg.config or {}
                preset = cfg.get("terrain_preset", "")
                if preset:
                    terrain_label = preset
                    print(f"  [config] Terrain label from sim: '{terrain_label}'")
                config_received = True
                continue

            if isinstance(msg, ControlCommand):
                last_steering = msg.steering
                continue

            if not isinstance(msg, VehicleState):
                continue

            # Push state through feature extractor
            fv = extractor.push(msg, steering=last_steering)
            if fv is None:
                continue

            if terrain_label is None:
                # Still waiting for label — skip but warn once
                if sample_count == 0:
                    print("  WARNING: No terrain label set. Use --terrain-label "
                          "or wait for SimStatus config.")
                continue

            # Write feature row
            row = fv.to_array().tolist() + [terrain_label]
            writer.writerow(row)
            sample_count += 1

            if sample_count % 20 == 0:
                csv_file.flush()
                print(f"  [t={fv.timestamp:.1f}s] Samples collected: {sample_count}  "
                      f"(slip_f={fv.slip_front_mean:.4f})")

            # Check for sim stop
            if isinstance(msg, SimStatus) and msg.event == "stop":
                print("  Simulation stopped.")
                break

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        csv_file.close()
        sub.close()
        ctrl_sub.close()
        print(f"\n  Collection complete: {sample_count} feature samples "
              f"({msg_count} messages) → {out_path}")


def main():
    p = argparse.ArgumentParser(
        description="Collect labeled terrain classification training data from sim")

    p.add_argument("--sim-host", default="localhost")
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-host", default="localhost")
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--output", "-o", default="terrain_classifier/data/training_data.csv")
    p.add_argument("--append", action="store_true",
                   help="Append to existing CSV instead of overwriting")
    p.add_argument("--terrain-label", default=None,
                   help="Override terrain label (auto-detected from sim config if omitted)")
    p.add_argument("--window", type=float, default=1.0,
                   help="Feature window duration (s)")
    p.add_argument("--stride", type=float, default=0.25,
                   help="Feature emission interval (s)")
    p.add_argument("--min-speed", type=float, default=1.0,
                   help="Minimum speed to collect features (m/s)")

    args = p.parse_args()
    collect(args)


if __name__ == "__main__":
    main()
