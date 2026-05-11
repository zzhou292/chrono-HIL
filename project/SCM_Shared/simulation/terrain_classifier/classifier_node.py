#!/usr/bin/env python3
"""
Terrain Classifier Node (ZMQ, Decoupled)
==========================================

Runs online terrain classification, decoupled from the simulation via ZMQ.
Subscribes to VehicleState, publishes TerrainEstimate.

Architecture mirrors the MPC controller node:
  - Subscribes to VehicleState on sim port (default 5555)
  - Publishes TerrainEstimate on its own port (default 5557)
  - Runs at ~4 Hz (one classification per stride window, default 0.25s)

Usage:
    python -m terrain_classifier.classifier_node \\
        --model terrain_classifier/models/terrain_rf.pkl \\
        --sim-host localhost --sim-port 5555 --pub-port 5557
"""

import argparse
import pickle
import sys
import time as wall_time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_sub_endpoint,
)
from terrain_classifier.feature_extractor import FeatureExtractor
from terrain_classifier.messages import TerrainEstimate, terrain_pub_endpoint


class ExponentialSmoother:
    """Smooths class probability vectors with EMA to reduce flickering.

    Uses a two-layer approach:
      1. EMA on raw probabilities (responsive, short-term)
      2. Running mean of all post-burn-in probabilities (robust, long-term)

    The final output blends both: after *burn_in* predictions, the running
    mean dominates, making the estimate very stable for the common case
    where terrain does not change within a run.
    """

    def __init__(self, alpha: float = 0.3, n_classes: int = 3,
                 burn_in: int = 8):
        self.alpha = alpha
        self.burn_in = burn_in
        self._ema = np.ones(n_classes) / n_classes  # uniform prior
        self._accum = np.zeros(n_classes)  # sum of post-burn-in probs
        self._count = 0
        self._total = 0

    def update(self, probs: np.ndarray) -> np.ndarray:
        self._total += 1
        # Layer 1: EMA (always active)
        self._ema = self.alpha * probs + (1 - self.alpha) * self._ema
        self._ema /= self._ema.sum()

        # Layer 2: running mean after burn-in
        if self._total > self.burn_in:
            self._accum += probs
            self._count += 1
            mean_probs = self._accum / self._count
            mean_probs /= mean_probs.sum()
            return mean_probs.copy()
        return self._ema.copy()

    def reset(self):
        n = len(self._ema)
        self._ema = np.ones(n) / n
        self._accum = np.zeros(n)
        self._count = 0
        self._total = 0


def compute_derived_features(X_base: np.ndarray) -> np.ndarray:
    """Compute derived features matching those added in train_model.py.

    Args:
        X_base: (1, N_base) array of base feature values.
                Column order (13 base features):
                  0=slip_front_mean, 1=slip_front_std, 2=slip_front_max,
                  3=slip_rear_mean,  4=slip_rear_std,  5=slip_rear_max,
                  6=yaw_accel_std,   7=az_std,
                  8=sideslip_ratio_mean, 9=yaw_rate_mean,
                  10=ax_std, 11=ay_std,
                  12=steering_std

    Returns:
        (1, N_base+13) array with base + derived features concatenated.
    """
    eps = 1e-6

    # Log transforms
    log_slip_f = np.log1p(X_base[:, 0:1])
    log_slip_r = np.log1p(X_base[:, 3:4])
    log_slip_f_max = np.log1p(X_base[:, 2:3])
    log_slip_r_max = np.log1p(X_base[:, 5:6])

    # Ratios
    slip_fr_ratio = X_base[:, 0:1] / np.maximum(X_base[:, 3:4], eps)
    slip_f_cv = X_base[:, 1:2] / np.maximum(X_base[:, 0:1], eps)
    slip_r_cv = X_base[:, 4:5] / np.maximum(X_base[:, 3:4], eps)
    yaw_slip_ratio = X_base[:, 9:10] / np.maximum(X_base[:, 8:9], eps)

    # Steering-normalized (driving-intensity invariant)
    steer = np.maximum(X_base[:, 12:13], 0.01)
    slip_per_steer = X_base[:, 3:4] / steer
    ay_per_steer = X_base[:, 11:12] / steer
    yaw_rate_per_steer = X_base[:, 9:10] / steer
    yaw_accel_per_steer = X_base[:, 6:7] / steer
    sideslip_per_steer = X_base[:, 8:9] / steer

    derived = np.hstack([
        log_slip_f, log_slip_r, log_slip_f_max, log_slip_r_max,
        slip_fr_ratio, slip_f_cv, slip_r_cv, yaw_slip_ratio,
        slip_per_steer, ay_per_steer, yaw_rate_per_steer,
        yaw_accel_per_steer, sideslip_per_steer,
    ])
    return np.hstack([X_base, derived])


def run_classifier(args):
    print("=" * 60)
    print("Terrain Classifier Node (Decoupled)")
    print("=" * 60)

    # ---- Load model ----
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"ERROR: Model file not found: {model_path}")
        print("Run train_model.py first to produce a trained model.")
        sys.exit(1)

    with open(model_path, "rb") as f:
        bundle = pickle.load(f)

    model = bundle["model"]
    le = bundle["label_encoder"]
    scaler = bundle["scaler"]
    classes = bundle["classes"]
    feature_names = bundle["feature_names"]
    print(f"  Model loaded: {model_path}")
    print(f"  Classes: {classes}")
    print(f"  Features: {len(feature_names)}")

    # ---- ZMQ setup ----
    state_sub = ZMQSubscriber(sim_sub_endpoint(args.sim_host, args.sim_port))
    ctrl_sub = ZMQSubscriber(ctrl_sub_endpoint(args.ctrl_host, args.ctrl_port))
    est_pub = ZMQPublisher(terrain_pub_endpoint(args.pub_port))
    print(f"  Subscribing to state: tcp://{args.sim_host}:{args.sim_port}")
    print(f"  Subscribing to ctrl:  tcp://{args.ctrl_host}:{args.ctrl_port}")
    print(f"  Publishing estimates on port {args.pub_port}")

    # ---- Feature extractor ----
    extractor = FeatureExtractor(
        window_sec=args.window,
        stride_sec=args.stride,
        min_speed=args.min_speed,
    )

    # ---- Probability smoother ----
    smoother = ExponentialSmoother(alpha=args.ema_alpha, n_classes=len(classes))

    last_steering = 0.0
    classify_count = 0
    msg_count = 0
    terrain_label = "unknown"
    ground_truth = None  # from SimStatus config, for logging

    print(f"  Window: {args.window}s, stride: {args.stride}s, EMA alpha: {args.ema_alpha}")
    print("  Waiting for simulation data...")

    try:
        while True:
            # Poll control commands (non-blocking) to track steering
            ctrl_result = ctrl_sub.recv(timeout_ms=0)
            if ctrl_result is not None:
                _, ctrl_msg = ctrl_result
                if isinstance(ctrl_msg, ControlCommand):
                    last_steering = ctrl_msg.steering

            result = state_sub.recv(timeout_ms=200)
            if result is None:
                continue

            topic, msg = result
            msg_count += 1

            if isinstance(msg, SimStatus):
                if msg.event == "config":
                    cfg = msg.config or {}
                    ground_truth = cfg.get("terrain_preset", None)
                    print(f"  [config] Ground truth terrain: {ground_truth}")
                elif msg.event == "stop":
                    print("  Simulation stopped.")
                    break
                continue

            if isinstance(msg, ControlCommand):
                last_steering = msg.steering
                continue

            if not isinstance(msg, VehicleState):
                continue

            # Push through feature extractor
            fv = extractor.push(msg, steering=last_steering)
            if fv is None:
                continue

            # Classify
            X = fv.to_array().reshape(1, -1)
            # Add derived features if model expects them
            if len(feature_names) > X.shape[1]:
                X = compute_derived_features(X)
            X_scaled = scaler.transform(X)
            proba = model.predict_proba(X_scaled)[0]

            # Smooth probabilities
            proba_smooth = smoother.update(proba)
            pred_idx = np.argmax(proba_smooth)
            terrain_label = classes[pred_idx]
            confidence = proba_smooth[pred_idx]

            # Build probability dict
            prob_dict = {cls: float(proba_smooth[i]) for i, cls in enumerate(classes)}

            # Publish estimate
            est = TerrainEstimate(
                time=msg.time,
                wall_time=wall_time.time(),
                terrain_class=terrain_label,
                confidence=float(confidence),
                probabilities=prob_dict,
            )
            est_pub.send(est)
            classify_count += 1

            if classify_count % 8 == 0:  # ~2s at 4 Hz stride
                gt_str = f" (GT: {ground_truth})" if ground_truth else ""
                prob_str = "  ".join(f"{c}={prob_dict[c]:.2f}" for c in classes)
                correct = "OK" if terrain_label == ground_truth else "MISS"
                print(f"  [t={fv.timestamp:.1f}s] {terrain_label} "
                      f"({confidence:.0%}){gt_str} [{correct}]  |  {prob_str}")

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        state_sub.close()
        ctrl_sub.close()
        est_pub.close()

        # Summary
        if classify_count > 0 and ground_truth:
            print(f"\n  Classifications: {classify_count}")
            print(f"  Final estimate: {terrain_label}")
            print(f"  Ground truth:   {ground_truth}")


def main():
    p = argparse.ArgumentParser(description="Online terrain classifier node (ZMQ)")

    p.add_argument("--model", "-m", default="terrain_classifier/models/terrain_rf.pkl",
                   help="Path to trained model pickle")
    p.add_argument("--sim-host", default="localhost")
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-host", default="localhost")
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--pub-port", type=int, default=5557,
                   help="Port to publish TerrainEstimate messages")
    p.add_argument("--window", type=float, default=1.0, help="Feature window (s)")
    p.add_argument("--stride", type=float, default=0.25, help="Classification interval (s)")
    p.add_argument("--min-speed", type=float, default=1.0, help="Min speed for classification (m/s)")
    p.add_argument("--ema-alpha", type=float, default=0.3,
                   help="EMA smoothing alpha (0=full smooth, 1=no smooth)")

    args = p.parse_args()
    run_classifier(args)


if __name__ == "__main__":
    main()
