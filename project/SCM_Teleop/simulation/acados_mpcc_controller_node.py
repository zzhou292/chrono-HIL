"""
acados_mpcc_controller_node.py
==============================

Minimal MPCC controller node for the decoupled HMMWV/SCM stack.

This is a focused, self-contained alternative to
``acados_mpc_controller_node.py``: same ZMQ message contract
(``VehicleState`` in, ``ControlCommand`` out), but the underlying
optimizer is :class:`AcadosMPCC` (path-progress contouring control)
rather than the standard reference-tracking MPC.  The difference
matters for the *speed channel*: MPCC drops the externally-prescribed
``v_ref`` and lets the optimizer pick its own path-progress velocity,
subject to a curvature-derived soft cap (handled by the path provider).

The deliberately-stripped feature set (no online terrain estimator,
no GP residual adapter, no online dynamics correction, no rate /
GRU tire variants — just static-MLP NN) makes the code small enough
to audit and easy to compare against the standard MPC head-to-head.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np

# Repo root on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from acados_mpcc_solver import AcadosMPCC
from hil_messages import (
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    ctrl_pub_endpoint, sim_sub_endpoint,
)
from live_debug_plotter import LiveDebugPlotter
from nn_tire_model import load_nn_tire_model
from param_consistency import (
    HMMWV_MAX_STEER_ANGLE_RAD,
    get_terrain_preset, terrain_preset_to_internal,
)
from reference_path import ReferencePath, generate_path_waypoints


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

def yaw_from_quat(e0: float, e1: float, e2: float, e3: float) -> float:
    """Extract yaw (rotation about Z) from Chrono quaternion (e0=w)."""
    return math.atan2(
        2 * (e0 * e3 + e1 * e2),
        1 - 2 * (e2 * e2 + e3 * e3),
    )


def build_augmented_z0(msg: VehicleState, ref: ReferencePath,
                       delta_state: float, ax_state: float) -> np.ndarray:
    """Map a ``VehicleState`` to the 9-D MPCC augmented state."""
    psi = yaw_from_quat(msg.quat_e0, msg.quat_e1, msg.quat_e2, msg.quat_e3)
    theta = ref.theta_at_xy(msg.x_cg, msg.y_cg)
    return np.array([
        msg.x_cg, msg.y_cg, psi,
        max(float(msg.u), 0.0), float(msg.v), float(msg.omega),
        float(ax_state),
        float(delta_state),
        float(theta),
    ])


def decode_chrono_inputs(delta_next: float, ax_next: float,
                          ax_min: float, ax_max: float
                          ) -> tuple[float, float, float]:
    """Convert (δ, ax) — predicted state at the NEXT MPC sample — into
    the Chrono driver's normalised ``(steering, throttle, braking)``
    triple.  Identical mapping the standard MPC uses, kept local so
    this file has no dependency on the heavier controller node.
    """
    steering = float(np.clip(delta_next / HMMWV_MAX_STEER_ANGLE_RAD, -1.0, 1.0))
    if ax_next >= 0:
        throttle = float(np.clip(ax_next / max(ax_max, 0.1), 0.0, 1.0))
        braking = 0.0
    else:
        throttle = 0.0
        braking = float(np.clip(ax_next / min(ax_min, -0.1), 0.0, 1.0))
    return steering, throttle, braking


# ----------------------------------------------------------------------------
# Main run loop
# ----------------------------------------------------------------------------

def run(args):
    # ---- Terrain + NN
    terrain_preset = get_terrain_preset(args.terrain)
    terrain_params = terrain_preset_to_internal(terrain_preset)
    nn_model_dir = Path(__file__).resolve().parent.parent / 'nn_models' / args.nn_model
    if not nn_model_dir.exists():
        print(f"NN model dir not found: {nn_model_dir}", file=sys.stderr)
        sys.exit(2)
    nn = load_nn_tire_model(str(nn_model_dir), terrain_params)

    # ---- Reference path
    x_pts, y_pts = generate_path_waypoints(
        args.path,
        lead_in=args.lead_in,
        sine_amplitude=args.sine_amplitude,
        sine_wavelength=args.sine_wavelength,
    )
    ref = ReferencePath(
        x_pts, y_pts,
        v_target=args.speed,
        friction_angle_deg=float(terrain_preset['friction_angle']),
    )
    print(f"  Path: {args.path}  ({len(x_pts)} waypoints, "
          f"s_max={ref.s_max:.1f} m)")

    # ---- MPCC solver
    print(f"  Building MPCC solver (N={args.N}, dt={args.dt}) ...")
    t0 = time.time()
    mpcc = AcadosMPCC(
        nn_tire_model=nn,
        terrain_params=terrain_params,
        dt=args.dt, N=args.N,
        w_contour=args.w_contour,
        w_lag=args.w_lag,
        w_progress=args.w_progress,
        w_delta_dot=args.w_delta_dot,
        w_speed_cap=args.w_speed_cap,
        friction_ellipse=args.friction_ellipse,
        vtheta_max=args.vtheta_max,
        verbose=False,
    )
    print(f"  Built in {time.time() - t0:.1f}s")

    # ---- ZMQ
    sub = ZMQSubscriber(sim_sub_endpoint(args.ctrl_host, args.sim_port),
                         ['vehicle_state', 'sim_status'])
    pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))
    print(f"  Subscribed to state at {args.ctrl_host}:{args.sim_port}; "
          f"publishing control on {args.ctrl_port}")

    # Send periodic neutral commands until we get the first VehicleState
    # back.  ZMQ pub-sub drops messages sent before the subscriber
    # connects, so a single ready ping isn't reliable — the standard
    # MPC node does the same thing (periodic resending), and chrono's
    # config_msg / handshake loop expects this.
    def _ready_ping(seq_no: int = 0):
        pub.send(ControlCommand(time=0.0, wall_time=time.time(), seq=seq_no,
                                  steering=0.0, throttle=0.0, braking=0.0,
                                  delta=0.0, acceleration=0.0,
                                  delta_dot=0.0, jerk=0.0))

    print('  Sending ready pings until first VehicleState arrives...')
    last_ping = 0.0
    first_state = None
    while first_state is None:
        now = time.time()
        if now - last_ping >= 0.3:
            _ready_ping()
            last_ping = now
        recv = sub.recv(timeout_ms=100)
        if recv is None:
            continue
        _topic, msg = recv
        if isinstance(msg, VehicleState):
            first_state = msg
        # ignore SimStatus / other during handshake
    print(f'  First state arrived at sim t={first_state.time:.2f}s; entering MPCC loop')

    # ---- Live matplotlib debug window (re-uses the standard MPC's
    #      LiveDebugPlotter — we just feed it MPCC's path samples in the
    #      slots labelled x_ref/y_ref/v_ref, which works because the
    #      plotter treats them as a generic reference curve).
    _live_plotter = None
    if args.live_plot:
        _live_plotter = LiveDebugPlotter(ref, update_every=args.live_plot_every)
        print(f'  Live plotter on, redraw every {args.live_plot_every} solves')

    # ---- Diagnostic CSV (paper-comparison KPIs)
    csv_writer = None
    csv_file = None
    if args.diag_csv:
        csv_path = Path(args.diag_csv).expanduser().resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'time', 'x', 'y', 'psi', 'u', 'v', 'omega',
            'theta', 'pos_err', 'e_lat', 'v_max_path',
            'delta_cmd', 'ax_cmd', 'vtheta_cmd',
            'steering', 'throttle', 'braking',
            'solve_time_ms', 'mpc_cost',
        ])
        print(f'  Diagnostic CSV: {csv_path}')

    # ---- Run loop
    seq = 0
    delta_applied = 0.0   # last actually-commanded δ (post rate limit)
    ax_applied = 0.0      # last actually-commanded ax (post rate limit)
    theta_warm = None     # warm-start θ trajectory
    solve_ms = []
    last_msg_time = None  # for real control-loop dt
    print('  Entering MPCC control loop...')
    stopped = False
    while not stopped:
        recv = sub.recv(timeout_ms=2000)
        if recv is None:
            print('  [MPCC] no state for 2 s; continuing to wait')
            continue
        _topic, msg = recv
        if isinstance(msg, SimStatus):
            if msg.event == 'stop':
                print('  [MPCC] received stop event; exiting')
                stopped = True
            continue
        if not isinstance(msg, VehicleState):
            continue

        # Build augmented state.  We pass the *previously applied* δ and ax
        # (after rate-limiting in the actual control loop dt), not the
        # optimizer's predicted next-step state — those drift apart at
        # control_dt << mpcc.dt and the divergence is what manifested as
        # ±MAX_STEER chatter on the front wheels.
        z0 = build_augmented_z0(msg, ref, delta_applied, ax_applied)

        # Warm-start θ trajectory
        if theta_warm is None or theta_warm.shape != (mpcc.N + 1,):
            theta_warm = z0[mpcc.ITHETA] + np.arange(mpcc.N + 1) * args.dt * args.speed
        else:
            # Shift the previous warm-start forward by one sample and
            # anchor it to the current vehicle θ to avoid drift.
            theta_warm = np.concatenate([theta_warm[1:],
                                          theta_warm[-1:] + args.dt * args.speed])
            theta_warm[0] = z0[mpcc.ITHETA]

        path_xy, path_psi = ref.sample_at_theta(theta_warm)
        v_max_stage = ref.v_max_at_theta(theta_warm)

        t_solve_start = time.time()
        try:
            u0, z_pred, info = mpcc.solve(z0, path_xy, path_psi, v_max_stage)
        except Exception as e:
            print(f'  [MPCC] solve failed at t={msg.time:.2f}: {e}')
            continue
        solve_ms.append(info['solve_time_ms'])

        # Update warm-start θ for next solve from the optimizer's
        # predicted θ trajectory
        theta_warm = z_pred[:, mpcc.ITHETA].copy()

        # Apply the first control (δ̇, jx) over the *real* control-loop
        # interval, not the solver's internal dt.  The state-message rate
        # (~83 Hz) is much faster than mpcc.dt (10 Hz, 0.1 s), so naively
        # taking z_pred[1, δ] as the command allows the optimizer to jump
        # δ by up to δ̇_max · mpcc.dt per outer step — which is ~7× the
        # physical δ̇ budget per control step and shows up as ±MAX_STEER
        # jitter on the wheels.  This is the same fix the standard MPC
        # uses (acados_mpc_controller_node.py:1136 "Post-MPC rate limiter").
        if last_msg_time is None:
            ctrl_dt = mpcc.dt
        else:
            ctrl_dt = max(float(msg.time - last_msg_time), 1e-4)
        last_msg_time = msg.time

        delta_dot_cmd = float(u0[0])
        jx_cmd = float(u0[1])

        max_delta_step = mpcc.delta_dot_max * ctrl_dt
        max_ax_step = mpcc.jx_max * ctrl_dt
        delta_next = float(np.clip(
            delta_applied + delta_dot_cmd * ctrl_dt,
            delta_applied - max_delta_step,
            delta_applied + max_delta_step))
        ax_next = float(np.clip(
            ax_applied + jx_cmd * ctrl_dt,
            ax_applied - max_ax_step,
            ax_applied + max_ax_step))
        # Respect the solver's hard actuator bounds too.
        delta_next = float(np.clip(delta_next, -mpcc.delta_max, mpcc.delta_max))
        ax_next = float(np.clip(ax_next, mpcc.ax_min, mpcc.ax_max))

        # End-of-path stop
        if ref.is_complete(threshold=1.5):
            delta_next = 0.0
            ax_next = mpcc.ax_min  # full brake

        steering, throttle, braking = decode_chrono_inputs(
            delta_next, ax_next, mpcc.ax_min, mpcc.ax_max)

        # Track the actually-applied actuator so the next solve's z0
        # reflects what the simulator received.
        delta_applied = delta_next
        ax_applied = ax_next

        seq += 1
        cmd = ControlCommand(
            time=msg.time, wall_time=time.time(), seq=seq,
            steering=steering, throttle=throttle, braking=braking,
            delta=delta_next, acceleration=ax_next,
            delta_dot=float(u0[0]), jerk=float(u0[1]),
            solve_time_ms=float(info['solve_time_ms']),
            mpc_cost=float(info['cost']),
        )
        pub.send(cmd)

        # Closest-point projection: shared between live plot and CSV diag.
        cp = ref.closest_point_on_path(msg.x_cg, msg.y_cg) \
            if (_live_plotter is not None or csv_writer is not None) else None

        if _live_plotter is not None:
            # The plotter wants per-stage (x_ref, y_ref, v_ref) of length N+1.
            # For MPCC there is no time-parametrised reference — we substitute
            # the path samples at the predicted θ trajectory (path_xy) and
            # the soft v_max cap (v_max_stage).
            _live_plotter.update(
                z0=z0,
                Z_opt=z_pred.T,        # plotter expects (NX, N+1); we have (N+1, NX)
                x_ref=path_xy[:, 0],
                y_ref=path_xy[:, 1],
                v_ref=v_max_stage,
                sim_time=float(msg.time),
                u_meas=float(msg.u),
                steering_angle=delta_applied,
                ax_state=ax_applied,
                mpc_cost=float(info['cost']),
                crosstrack_err=float(cp['e_lat']),
            )

        if csv_writer is not None:
            v_max_now = float(ref.v_max_at_theta(np.array([z0[mpcc.ITHETA]]))[0])
            csv_writer.writerow([
                f'{msg.time:.4f}', f'{msg.x_cg:.4f}', f'{msg.y_cg:.4f}',
                f'{z0[mpcc.IPSI]:.4f}', f'{msg.u:.4f}', f'{msg.v:.4f}',
                f'{msg.omega:.4f}', f'{z0[mpcc.ITHETA]:.4f}',
                f'{cp["pos_err"]:.4f}', f'{cp["e_lat"]:.4f}',
                f'{v_max_now:.4f}',
                f'{delta_next:.4f}', f'{ax_next:.4f}', f'{float(u0[2]):.4f}',
                f'{steering:.4f}', f'{throttle:.4f}', f'{braking:.4f}',
                f'{info["solve_time_ms"]:.3f}', f'{info["cost"]:.4f}',
            ])

        if seq % 20 == 0:
            print(f"  [MPCC #{seq:4d}] t={msg.time:5.2f}s "
                  f"pos=({msg.x_cg:6.2f},{msg.y_cg:+6.2f}) "
                  f"u={msg.u:4.2f} θ={z0[mpcc.ITHETA]:5.1f} "
                  f"vθ={u0[2]:4.2f}  "
                  f"steer={steering:+.2f} thr={throttle:.2f} brk={braking:.2f} "
                  f"solve={info['solve_time_ms']:.1f}ms")

    if csv_file is not None:
        csv_file.close()
    if _live_plotter is not None:
        _live_plotter.close()

    print('\n  --- MPCC controller exited ---')
    if solve_ms:
        print(f"  Solve time: mean={np.mean(solve_ms):.1f}ms "
              f"p90={np.percentile(solve_ms, 90):.1f}ms "
              f"max={np.max(solve_ms):.1f}ms  (n={len(solve_ms)})")


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--nn-model', default='closed_loop_v1_mlp_32_16',
                   help='Tire surrogate to embed inside the OCP.')
    p.add_argument('--terrain', default='clay',
                   choices=['clay', 'sand', 'dirt'])
    p.add_argument('--path', default='sinusoidal',
                   choices=['sinusoidal', 'lane_change',
                             'double_lane_change', 'right_left'])
    p.add_argument('--speed', type=float, default=5.0,
                   help='Cap for the curvature-derived speed limit. '
                        'MPCC will pick speeds below this naturally.')
    p.add_argument('--lead-in', type=float, default=5.0)
    p.add_argument('--sine-amplitude', type=float, default=2.0)
    p.add_argument('--sine-wavelength', type=float, default=30.0)
    # OCP geometry
    p.add_argument('--N', type=int, default=20,
                   help='Horizon steps.')
    p.add_argument('--dt', type=float, default=0.1)
    # Cost weights (paper-tunable knobs)
    p.add_argument('--w-contour', type=float, default=3000.0)
    p.add_argument('--w-lag', type=float, default=2000.0)
    p.add_argument('--w-progress', type=float, default=0.5)
    p.add_argument('--w-delta-dot', type=float, default=80.0)
    p.add_argument('--w-speed-cap', type=float, default=300.0)
    p.add_argument('--friction-ellipse', action='store_true',
                   help='Enable NN-derived hard friction-ellipse constraints.')
    p.add_argument('--vtheta-max', type=float, default=5.0)
    # ZMQ
    p.add_argument('--sim-port', type=int, default=5555)
    p.add_argument('--ctrl-port', type=int, default=5556)
    p.add_argument('--ctrl-host', default='localhost')
    # Diagnostics
    p.add_argument('--diag-csv', default='',
                   help='Path to write per-step CSV with KPI fields (for benchmark scripts).')
    p.add_argument('--live-plot', action='store_true',
                   help='Open a live matplotlib debug window during the run.')
    p.add_argument('--live-plot-every', type=int, default=5,
                   help='Redraw the live plot every N solves.')
    args = p.parse_args()
    print('=' * 60)
    print('ACADOS MPCC Controller Node')
    print('=' * 60)
    run(args)


if __name__ == '__main__':
    main()
