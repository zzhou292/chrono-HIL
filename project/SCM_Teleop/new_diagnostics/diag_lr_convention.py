#!/usr/bin/env python3
"""
Check left-right per-wheel sign conventions in the Chrono sim.
Tests whether left and right wheels have the same or opposite alpha/Fy signs.
"""
import argparse, math, subprocess, sys, time
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "simulation"))
from simulation.hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)
from simulation.param_consistency import TERRAIN_PRESETS

def main():
    sim_script = PROJECT_ROOT / "simulation" / "chrono_sim_node.py"
    sim_cmd = [
        sys.executable, str(sim_script),
        "--time", "12", "--speed", "5",
        "--terrain", "dirt", "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", "7894", "--ctrl-host", "localhost", "--ctrl-port", "7895",
        "--no-wait-for-controller",
    ]
    sim_proc = subprocess.Popen(sim_cmd)
    time.sleep(2)
    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", 7894))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(7895))
    
    rows = []
    seq = 0
    try:
        while True:
            result = state_sub.recv(timeout_ms=500)
            if result is None: continue
            topic, msg = result
            if isinstance(msg, SimStatus):
                if msg.event == "stop": break
                continue
            if not isinstance(msg, VehicleState): continue
            
            t, u = msg.time, msg.u
            steer = 0.5 * math.sin(2*math.pi*t/3)
            cmd = ControlCommand(time=t, wall_time=time.time(), seq=seq,
                                 steering=steer, throttle=0.5, braking=0.0,
                                 delta=0.0, acceleration=0.0, delta_dot=0.0, jerk=0.0)
            ctrl_pub.send(cmd); seq += 1
            if abs(u) < 1.0: continue
            tf = msg.tire_forces or {}
            if not tf: continue
            try:
                row = {
                    't': t, 'u': u, 'ay': msg.ay, 'omega': msg.omega,
                    'fl_alpha': float(tf['front_left_slip_angle']),
                    'fr_alpha': float(tf['front_right_slip_angle']),
                    'rl_alpha': float(tf['rear_left_slip_angle']),
                    'rr_alpha': float(tf['rear_right_slip_angle']),
                    'fl_Fy': float(tf['front_left_Fy']),
                    'fr_Fy': float(tf['front_right_Fy']),
                    'rl_Fy': float(tf['rear_left_Fy']),
                    'rr_Fy': float(tf['rear_right_Fy']),
                    'fl_Fz': float(tf['front_left_Fz']),
                    'fr_Fz': float(tf['front_right_Fz']),
                }
                rows.append(row)
            except (KeyError, TypeError):
                continue
    finally:
        state_sub.close(); ctrl_pub.close()
        sim_proc.terminate(); sim_proc.wait(timeout=5)

    if not rows:
        print("No data!"); return
    
    import pandas as pd
    df = pd.DataFrame(rows)
    
    print(f"\n{'='*80}")
    print(f"LEFT-RIGHT COMPARISON ({len(df)} samples)")
    print(f"{'='*80}")
    
    # Alpha correlation
    print(f"\n--- Front Alpha ---")
    print(f"  FL alpha: mean={df['fl_alpha'].mean():.4f}, std={df['fl_alpha'].std():.4f}")
    print(f"  FR alpha: mean={df['fr_alpha'].mean():.4f}, std={df['fr_alpha'].std():.4f}")
    print(f"  Correlation(FL, FR): {df['fl_alpha'].corr(df['fr_alpha']):.4f}")
    print(f"  Mean(FL+FR): {(df['fl_alpha'] + df['fr_alpha']).mean():.4f}")
    print(f"  Mean(FL-FR): {(df['fl_alpha'] - df['fr_alpha']).mean():.4f}")
    
    # Force correlation
    print(f"\n--- Front Fy (body frame) ---")
    print(f"  FL Fy: mean={df['fl_Fy'].mean():.0f}, std={df['fl_Fy'].std():.0f}")
    print(f"  FR Fy: mean={df['fr_Fy'].mean():.0f}, std={df['fr_Fy'].std():.0f}")
    print(f"  Correlation(FL, FR): {df['fl_Fy'].corr(df['fr_Fy']):.4f}")
    print(f"  FL+FR (axle): mean={df['fl_Fy'].mean()+df['fr_Fy'].mean():.0f}")
    
    # Fy vs alpha relationship
    print(f"\n--- Fy vs alpha (per-wheel correlation) ---")
    print(f"  Corr(FL alpha, FL Fy): {df['fl_alpha'].corr(df['fl_Fy']):.4f}")
    print(f"  Corr(FR alpha, FR Fy): {df['fr_alpha'].corr(df['fr_Fy']):.4f}")
    print(f"  Corr(FL alpha, FR Fy): {df['fl_alpha'].corr(df['fr_Fy']):.4f}")
    print(f"  Corr(FR alpha, FL Fy): {df['fr_alpha'].corr(df['fl_Fy']):.4f}")
    
    # Look at specific timestep during a turn
    print(f"\n--- Sample timesteps (high |ay| moments) ---")
    high_ay = df[abs(df['ay']) > 2.0].head(5)
    for _, r in high_ay.iterrows():
        print(f"  t={r['t']:.2f} ay={r['ay']:+.2f} omega={r['omega']:+.3f}: "
              f"FL(alpha={r['fl_alpha']:+.3f}, Fy={r['fl_Fy']:+.0f}) "
              f"FR(alpha={r['fr_alpha']:+.3f}, Fy={r['fr_Fy']:+.0f}) "
              f"FL_Fz={r['fl_Fz']:.0f} FR_Fz={r['fr_Fz']:.0f}")
    
    # Check: during a LEFT turn (omega>0), what signs do things have?
    left_turn = df[df['omega'] > 0.1]
    right_turn = df[df['omega'] < -0.1]
    if len(left_turn) > 5:
        print(f"\n--- During LEFT turns (omega>0.1, {len(left_turn)} pts) ---")
        print(f"  FL alpha mean: {left_turn['fl_alpha'].mean():+.4f}")
        print(f"  FR alpha mean: {left_turn['fr_alpha'].mean():+.4f}")
        print(f"  FL Fy mean: {left_turn['fl_Fy'].mean():+.0f}")
        print(f"  FR Fy mean: {left_turn['fr_Fy'].mean():+.0f}")
        print(f"  ay mean: {left_turn['ay'].mean():+.2f}")
    if len(right_turn) > 5:
        print(f"\n--- During RIGHT turns (omega<-0.1, {len(right_turn)} pts) ---")
        print(f"  FL alpha mean: {right_turn['fl_alpha'].mean():+.4f}")
        print(f"  FR alpha mean: {right_turn['fr_alpha'].mean():+.4f}")
        print(f"  FL Fy mean: {right_turn['fl_Fy'].mean():+.0f}")
        print(f"  FR Fy mean: {right_turn['fr_Fy'].mean():+.0f}")
        print(f"  ay mean: {right_turn['ay'].mean():+.2f}")


if __name__ == "__main__":
    main()
