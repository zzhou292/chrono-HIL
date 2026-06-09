#!/usr/bin/env python3
"""Paper experiment: online terrain estimator under a spatial soil transition.

One thing tested: when the plant soil changes type partway across the patch
(one Bekker--Mohr preset blends into another along the +x driving direction,
applied per contact location through PyChrono's ``SCMTerrain`` soil callback),
how quickly and how accurately does the deployed sliding-window terrain
estimator track the new sinkage exponent ``n``, and how does the NMPC's
tracking hold up while the estimate is catching up?

Because the soil field is a deterministic function of position, the exact
ground-truth ``n(x)`` is reconstructed from the *same* blend function the
simulator used (``simulation/spatial_terrain.py``), evaluated at the vehicle's
logged front-axle position -- there is no separate oracle log to keep in sync.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from itertools import permutations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    DEFAULT_NN_MODEL,
    PROJECT_ROOT,
    RunResult,
    launch_and_collect,
    save_summary_markdown,
    timestamped_result_dir,
    write_manifest,
)

# Import the *same* soil field the simulator applies, so ground truth and
# physics can never drift apart.
SIM_DIR = PROJECT_ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))
from spatial_terrain import SpatialTransitionSpec, local_n_at  # noqa: E402

# Open-loop excitation mode imports (controller removed; this script drives the
# sim with scripted sinusoidal steer + sinusoidal throttle and runs the
# estimator in-process on the streamed state, mirroring
# open_loop_terrain_estimator_benchmark.py).
from hil_messages import (  # noqa: E402
    ControlCommand, SimStatus, VehicleState, ZMQPublisher, ZMQSubscriber,
    ctrl_pub_endpoint, sim_sub_endpoint,
)
from learned_terrain_estimator import LearnedTerrainEstimator  # noqa: E402
from param_consistency import get_terrain_preset, terrain_preset_to_internal  # noqa: E402
from tire_input_features import (  # noqa: E402
    VehicleGeometry, compute_bicycle_operating_point, kappa_from_wheel_speed,
)

TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}
PRESETS = ("clay", "dirt", "sand")

# Number of (x, n_hat) samples stored per run so the overlay figure can be
# regenerated from results.csv alone (survives the publish merge).
TRACE_POINTS = 90


def all_ordered_pairs() -> list[str]:
    """Every ordered pair of distinct presets, e.g. ``clay_to_sand``."""
    return [f"{a}_to_{b}" for a, b in permutations(PRESETS, 2)]


def parse_transition(label: str) -> tuple[str, str]:
    start, _, end = label.partition("_to_")
    if start not in TRUE_N or end not in TRUE_N:
        raise ValueError(f"Bad transition {label!r}; use e.g. clay_to_sand")
    return start, end


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--transitions", nargs="+", default=all_ordered_pairs(),
                   help="Soil transitions as <start>_to_<end> (default: all "
                        "ordered pairs of distinct presets).")
    p.add_argument("--excitation", choices=["closed_loop", "open_loop"],
                   default="closed_loop",
                   help="closed_loop: NMPC tracks the path and the estimator "
                        "runs inside the controller (paper figure). open_loop: "
                        "no controller -- scripted sinusoidal steer + sinusoidal "
                        "throttle, estimator run in-process on the state stream.")
    p.add_argument("--ol-throttle", type=float, default=0.75,
                   help="Open-loop mean throttle (modulated by a 0.25 Hz sine).")
    p.add_argument("--ol-steer-amp", type=float, default=0.60,
                   help="Open-loop wide-steer amplitude (rad-equiv command).")
    p.add_argument("--ol-steer-period", type=float, default=3.0,
                   help="Open-loop wide-steer period (s).")
    p.add_argument("--paths", nargs="+", default=["sinusoidal"],
                   help="Sinusoidal by default: the estimator needs lateral "
                        "excitation to read the soil.")
    p.add_argument("--speeds", nargs="+", type=float, default=[5.0])
    # The deployed window-MLP estimator is trained on bumpiness {0,4}; stay
    # within that envelope (cf. terrain_estimator_benchmark.py).
    p.add_argument("--bumpiness", nargs="+", type=int, default=[0])
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--base-seed", type=int, default=720)
    p.add_argument("--transition-x", type=float, default=45.0,
                   help="Center of the soil transition, in terrain x (m).")
    p.add_argument("--transition-width", type=float, default=2.0,
                   help="Full width of the linear soil blend (m); 0 = hard step.")
    p.add_argument("--time", type=float, default=24.0,
                   help="Run long enough for the vehicle to cross the boundary "
                        "and the estimate to settle on the new soil.")
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--pre-window-m", type=float, default=22.0,
                   help="Length of the pre-transition averaging window (m), "
                        "ending a small margin before the blend.")
    p.add_argument("--settle-margin-m", type=float, default=25.0,
                   help="Distance past the transition before the post-window "
                        "starts (lets the estimate settle).")
    p.add_argument("--response-frac", type=float, default=0.63,
                   help="Fraction of the n step the estimate must reach to "
                        "count as 'responded' (settling-distance metric).")
    p.add_argument("--metric-start", type=float, default=8.0)
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--learned-terrain-model-dir", default=None,
                   help="Optional terrain estimator checkpoint to forward to "
                        "the controller.")
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--base-port", type=int, default=9600)
    p.add_argument("--workers", type=int, default=6,
                   help="Chrono workers after one cache-prewarm run.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: one transition, one seed, short.")
    return p.parse_args()


@dataclass(frozen=True)
class TransitionTask:
    idx: int
    total: int
    transition: str
    start_preset: str
    end_preset: str
    transition_x: float
    transition_width: float
    path: str
    speed: float
    bumpiness: int
    seed: int
    sim_port: int
    ctrl_port: int
    run_dir: str
    sim_time: float
    lead_in: float
    timeout: float
    metric_start: float
    pre_window_m: float
    settle_margin_m: float
    response_frac: float
    sine_amplitude: float
    sine_wavelength: float
    learned_terrain_model_dir: str | None
    unique_build_dir: bool
    excitation: str
    ol_throttle: float
    ol_steer_amp: float
    ol_steer_period: float


def _run_one(task: TransitionTask) -> dict:
    if task.excitation == "open_loop":
        return _run_one_open_loop(task)
    if task.unique_build_dir:
        os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    extra = [
        "--terrain-estimator",
        "--terrain-estimator-mode", "n",
        "--terrain-estimator-backend", "fused",
        "--te-verbose",
        "--terrain-transition",
        "--terrain-start", task.start_preset,
        "--terrain-end", task.end_preset,
        "--transition-x", str(task.transition_x),
        "--transition-width", str(task.transition_width),
        "--sine-amplitude", str(task.sine_amplitude),
        "--sine-wavelength", str(task.sine_wavelength),
    ]
    if task.learned_terrain_model_dir:
        extra += ["--learned-terrain-model-dir", task.learned_terrain_model_dir]
    res = launch_and_collect(
        experiment="terrain_transition_benchmark",
        variant=task.transition,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=DEFAULT_NN_MODEL,
        # Plant starts on the start preset; the callback blends to the end one.
        terrain=task.start_preset,
        path=task.path,
        speed=task.speed,
        bumpiness=task.bumpiness,
        seed=task.seed,
        run_dir=Path(task.run_dir),
        sim_port=task.sim_port,
        ctrl_port=task.ctrl_port,
        sim_time=task.sim_time,
        timeout=task.timeout,
        rocks=0,
        lead_in=task.lead_in,
        extra_args=extra,
        metric_start=task.metric_start,
    )
    return result_row(res, task)


def _yaw_from_quat(msg: VehicleState) -> float:
    return math.atan2(
        2 * (msg.quat_e0 * msg.quat_e3 + msg.quat_e1 * msg.quat_e2),
        1 - 2 * (msg.quat_e2 ** 2 + msg.quat_e3 ** 2),
    )


def _command_at_ol(t_rel: float, task: TransitionTask) -> tuple[float, float, float]:
    """Open-loop excitation: sinusoidal throttle + two-frequency sinusoidal
    steer (Buzhardt-Tallapragada style), independent of the soil so any
    estimate change is attributable to the estimator, not a controller
    reaction. Mirrors open_loop_terrain_estimator_benchmark._command_at.
    """
    ramp = min(max(t_rel / 1.0, 0.0), 1.0)
    thr_mean = float(task.ol_throttle)
    thr_amp = 0.30 * thr_mean
    throttle = float(ramp * (thr_mean + thr_amp * math.sin(2.0 * math.pi * t_rel / 4.0)))
    phi_seed = 0.07 * task.seed
    steer_wide = float(task.ol_steer_amp *
                       math.sin(2.0 * math.pi * (t_rel + phi_seed) / task.ol_steer_period))
    steer_fast = float(0.15 * math.sin(2.0 * math.pi * t_rel / 1.0))
    steer = max(-1.0, min(1.0, steer_wide + steer_fast))
    return throttle, steer, 0.0


def _run_one_open_loop(task: TransitionTask) -> dict:
    """Drive the sim open-loop across the spatial transition, running the
    deployed estimator in-process on the streamed state. Writes a diag CSV
    with the columns transition_metrics() expects (x_fa_true, n_terrain_est,
    sim_time, terrain_update_applied, u_true)."""
    run_dir = Path(task.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    diag_csv = run_dir / "open_loop_diag.csv"
    log_path = run_dir / "run.log"
    sim_cmd = [
        sys.executable, "-u", str(SIM_DIR / "chrono_sim_node.py"),
        "--time", str(task.sim_time + 6.0),
        "--speed", "5",
        "--terrain", task.start_preset,
        "--path", task.path,
        "--vis-mode", "none",
        "--sim-port", str(task.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(task.ctrl_port),
        "--bumpiness", str(task.bumpiness),
        "--no-wait-for-controller",
        "--terrain-transition",
        "--terrain-start", task.start_preset,
        "--terrain-end", task.end_preset,
        "--transition-x", str(task.transition_x),
        "--transition-width", str(task.transition_width),
    ]

    model_dir = (Path(task.learned_terrain_model_dir).expanduser().resolve()
                 if task.learned_terrain_model_dir
                 else PROJECT_ROOT / "nn_models" / "terrain_window_mlp")
    estimator = LearnedTerrainEstimator(
        model_dir=str(model_dir),
        initial_terrain=terrain_preset_to_internal(get_terrain_preset("dirt")),
        update_interval=10, verbose=False, window_size=50, min_excitation=0.0,
    )
    geom = VehicleGeometry.from_hmmwv_defaults()
    applied_n = float(terrain_preset_to_internal(get_terrain_preset("dirt"))["n"])
    applied_phi = float(terrain_preset_to_internal(get_terrain_preset("dirt"))["phi"])
    confidence = 0.0
    seq = 0
    t0_sim = None
    prev_delta = None
    prev_t = None
    rows: list[dict] = []
    status = "ok"
    notes = ""
    proc = None
    state_sub = None
    ctrl_pub = None
    try:
        with log_path.open("w") as log:
            proc = subprocess.Popen(sim_cmd, cwd=str(PROJECT_ROOT), stdout=log,
                                    stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", task.sim_port))
        ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(task.ctrl_port))
        time.sleep(1.5)
        deadline = time.time() + task.timeout
        timeout_count = 0
        while time.time() < deadline:
            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    status = "timeout_no_state"
                    break
                continue
            timeout_count = 0
            _, msg = res
            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue
            if t0_sim is None:
                t0_sim = float(msg.time)
            t_rel = float(msg.time) - t0_sim
            throttle, steer, brake = _command_at_ol(t_rel, task)
            ctrl_pub.send(ControlCommand(
                time=float(msg.time), wall_time=time.time(), seq=seq,
                steering=steer, throttle=throttle, braking=brake,
                delta=float(msg.steering_angle), acceleration=float(msg.ax),
                delta_dot=0.0, jerk=0.0,
            ))
            seq += 1

            dt = max(float(msg.time) - prev_t, 1e-4) if prev_t is not None else 1e-2
            sr = ((float(msg.steering_angle) - prev_delta) / dt
                  if prev_delta is not None else 0.0)
            prev_delta = float(msg.steering_angle)
            prev_t = float(msg.time)

            measured_kappa = kappa_from_wheel_speed(
                msg.wheel_omega_fl, msg.wheel_omega_fr,
                msg.wheel_omega_rl, msg.wheel_omega_rr, msg.u,
            )
            terrain_mu = max(math.tan(math.radians(applied_phi)), 0.1)
            kappa, alpha_f, alpha_r, u_safe, Fz_f, Fz_r = compute_bicycle_operating_point(
                float(msg.steering_angle), float(msg.u), float(msg.v),
                float(msg.omega), float(msg.ax), geom=geom, kappa_mode="measured",
                terrain_mu=terrain_mu, measured_kappa=measured_kappa,
            )
            update_applied = 0
            omega_dot = estimator.estimate_omega_dot(float(msg.omega), float(msg.time))
            if omega_dot is not None:
                estimator.observe(
                    kappa=float(kappa), alpha_f=float(alpha_f), alpha_r=float(alpha_r),
                    u=float(u_safe), Fz_f=float(Fz_f), Fz_r=float(Fz_r), sr=float(sr),
                    ay_imu=float(msg.ay), omega_dot=float(omega_dot),
                    omega=float(msg.omega), v_lateral=float(msg.v),
                    x_pos=float(msg.x_cg), y_pos=float(msg.y_cg), psi=_yaw_from_quat(msg),
                    ax_cmd=float(msg.ax), sim_time=float(t_rel),
                    wheel_omegas=(float(msg.wheel_omega_fl), float(msg.wheel_omega_fr),
                                  float(msg.wheel_omega_rl), float(msg.wheel_omega_rr)),
                    ax_imu=float(msg.ax),
                    az_imu=float(getattr(msg, "az", 0.0)),
                    roll_rate=float(getattr(msg, "omega_x", 0.0)),
                    pitch_rate=float(getattr(msg, "omega_y", 0.0)),
                    throttle_cmd=float(throttle),
                )
            if estimator.should_update():
                params, confidence = estimator.estimate()
                applied_n = float(params["n"])
                applied_phi = float(params["phi"])
                update_applied = 1
            # Front-axle x to match the closed-loop diag convention.
            x_fa = float(msg.x_cg) + geom.Lf * math.cos(_yaw_from_quat(msg))
            rows.append({
                "sim_time": f"{t_rel:.4f}",
                "x_fa_true": f"{x_fa:.5f}",
                "u_true": f"{float(msg.u):.6f}",
                "steering": f"{steer:.6f}",
                "throttle": f"{throttle:.6f}",
                "n_terrain_est": f"{applied_n:.6f}",
                "n_terrain_estimator": f"{estimator.get_bekker_n():.6f}",
                "terrain_confidence": f"{confidence:.6f}",
                "terrain_update_applied": int(update_applied),
            })
            if t_rel >= task.sim_time:
                break
        else:
            status = "timeout"
    except Exception as exc:  # noqa: BLE001
        status = "error"
        notes = repr(exc)
    finally:
        for c in (state_sub, ctrl_pub):
            try:
                c.close()
            except Exception:
                pass
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=8)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    if len(rows) < 100 and status == "ok":
        status = "too_few_samples"
    if rows:
        with diag_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    mean_speed = math.nan
    if rows:
        u = pd.to_numeric(pd.Series([r["u_true"] for r in rows]), errors="coerce")
        mean_speed = float(u.mean())
    row = {
        "experiment": "terrain_transition_ol_benchmark",
        "variant": task.transition,
        "transition": task.transition,
        "start_preset": task.start_preset,
        "end_preset": task.end_preset,
        "transition_x": task.transition_x,
        "transition_width": task.transition_width,
        "distribution": "transition_ol",
        "case_label": task.transition,
        "terrain": task.start_preset,
        "path": task.path,
        "speed_mps": task.speed,
        "bumpiness": task.bumpiness,
        "seed": task.seed,
        "status": status,
        "run_dir": str(run_dir),
        "diag_csv": str(diag_csv) if diag_csv.exists() else "",
        "rms_cte_m": math.nan,  # no path-tracking loop in open loop
        "mean_speed_mps": mean_speed,
        "excitation_source": "open_loop_command",
        "notes": notes,
    }
    row.update(transition_metrics(row["diag_csv"], task))
    return row


def _spec_of(task: TransitionTask) -> SpatialTransitionSpec:
    return SpatialTransitionSpec(
        start_preset=task.start_preset,
        end_preset=task.end_preset,
        transition_x=task.transition_x,
        transition_width=task.transition_width,
    )


def transition_metrics(diag_csv: str, task: TransitionTask) -> dict:
    """Tracking metrics + a downsampled (x, n_hat, n_true) trace.

    All distances are relative to ``transition_x``. The estimate is the
    controller's logged ``n_terrain_est`` (the smoothed, deployed estimate),
    and ground truth is the analytic soil field at the logged front-axle x.
    """
    empty = {
        "n_est_pre": math.nan, "n_est_post": math.nan,
        "n_true_start": TRUE_N[task.start_preset],
        "n_true_end": TRUE_N[task.end_preset],
        "pre_abs_err": math.nan, "post_abs_err": math.nan,
        "track_rmse": math.nan,
        "response_dist_m": math.nan, "response_time_s": math.nan,
        "crossed_transition": 0,
        "trace_x_json": "[]", "trace_nhat_json": "[]", "trace_ntrue_json": "[]",
    }
    if not diag_csv:
        return empty
    try:
        df = pd.read_csv(diag_csv)
    except (pd.errors.EmptyDataError, pd.errors.ParserError, FileNotFoundError):
        return empty
    if df.empty or "n_terrain_est" not in df.columns or "x_fa_true" not in df.columns:
        return empty

    spec = _spec_of(task)
    xc = task.transition_x
    t = pd.to_numeric(df.get("sim_time", pd.Series(np.arange(len(df)))), errors="coerce")
    x = pd.to_numeric(df["x_fa_true"], errors="coerce")
    n_est = pd.to_numeric(df["n_terrain_est"], errors="coerce")
    good = np.isfinite(x) & np.isfinite(n_est) & np.isfinite(t)
    t, x, n_est = t[good].to_numpy(), x[good].to_numpy(), n_est[good].to_numpy()
    if x.size < 5:
        return empty

    n_true = np.array([local_n_at(float(xi), spec) for xi in x])
    n_start, n_end = TRUE_N[task.start_preset], TRUE_N[task.end_preset]

    # Pre-transition window: a stretch ending just before the blend zone.
    pre_hi = xc - max(task.transition_width, 2.0)
    pre_lo = pre_hi - task.pre_window_m
    pre = (x >= pre_lo) & (x <= pre_hi)
    # Post-transition window: settled, well past the boundary, to the end.
    post = x >= (xc + task.settle_margin_m)
    n_est_pre = float(n_est[pre].mean()) if pre.any() else math.nan
    n_est_post = float(n_est[post].mean()) if post.any() else math.nan

    crossed = bool((x > xc).any())
    track_rmse = float(np.sqrt(np.mean((n_est - n_true) ** 2)))

    # Settling distance: after crossing xc, how far until the estimate covers
    # response_frac of the (pre-estimate -> true-end) gap.
    response_dist = math.nan
    response_time = math.nan
    base = n_est_pre if math.isfinite(n_est_pre) else (n_est[x <= xc][-1] if (x <= xc).any() else n_start)
    target = base + task.response_frac * (n_end - base)
    after = x > xc
    if after.any() and abs(n_end - base) > 1e-6:
        xa, ta, na = x[after], t[after], n_est[after]
        if n_end >= base:
            hit = np.where(na >= target)[0]
        else:
            hit = np.where(na <= target)[0]
        if hit.size:
            j = hit[0]
            response_dist = float(xa[j] - xc)
            response_time = float(ta[j] - ta[0])

    # Downsample a trace for the overlay figure (ordered by x).
    order = np.argsort(x)
    xs, nh, ntru = x[order], n_est[order], n_true[order]
    if xs.size > TRACE_POINTS:
        sel = np.linspace(0, xs.size - 1, TRACE_POINTS).round().astype(int)
        xs, nh, ntru = xs[sel], nh[sel], ntru[sel]

    return {
        "n_est_pre": n_est_pre, "n_est_post": n_est_post,
        "n_true_start": n_start, "n_true_end": n_end,
        "pre_abs_err": abs(n_est_pre - n_start) if math.isfinite(n_est_pre) else math.nan,
        "post_abs_err": abs(n_est_post - n_end) if math.isfinite(n_est_post) else math.nan,
        "track_rmse": track_rmse,
        "response_dist_m": response_dist, "response_time_s": response_time,
        "crossed_transition": int(crossed),
        "trace_x_json": json.dumps([round(float(v), 3) for v in xs]),
        "trace_nhat_json": json.dumps([round(float(v), 4) for v in nh]),
        "trace_ntrue_json": json.dumps([round(float(v), 4) for v in ntru]),
    }


def result_row(res: RunResult, task: TransitionTask) -> dict:
    row = asdict(res)
    row.pop("extra", None)
    row.update({
        "transition": task.transition,
        "start_preset": task.start_preset,
        "end_preset": task.end_preset,
        "transition_x": task.transition_x,
        "transition_width": task.transition_width,
        # Give the publish dedup key well-defined values.
        "distribution": "transition",
        "case_label": task.transition,
    })
    row.update(transition_metrics(res.diag_csv, task))
    return row


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------
def _load_traces(sub: pd.DataFrame):
    xs, nhs = [], []
    for _, r in sub.iterrows():
        try:
            x = np.asarray(json.loads(r.get("trace_x_json", "[]")), dtype=float)
            nh = np.asarray(json.loads(r.get("trace_nhat_json", "[]")), dtype=float)
        except (TypeError, ValueError):
            continue
        if x.size and x.size == nh.size:
            xs.append(x)
            nhs.append(nh)
    return xs, nhs


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    src = str(ok.get("excitation_source", pd.Series(["closed_loop_mpc"])).iloc[0])
    exc_label = ("open-loop (scripted sine steer + sine throttle)"
                 if src == "open_loop_command" else "closed-loop NMPC")

    transitions = list(ok["transition"].dropna().unique())
    transitions.sort(key=lambda lbl: (TRUE_N.get(parse_transition(lbl)[0], 0.0),
                                       TRUE_N.get(parse_transition(lbl)[1], 0.0)))

    # ---- Figure 1: n_hat(x) overlay vs ground-truth step, one panel each ----
    ncol = min(3, len(transitions))
    nrow = int(math.ceil(len(transitions) / ncol)) if ncol else 1
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.4 * nrow),
                             squeeze=False)
    for k, lbl in enumerate(transitions):
        ax = axes[k // ncol][k % ncol]
        sub = ok[ok["transition"] == lbl]
        start, end = parse_transition(lbl)
        xc = float(sub["transition_x"].iloc[0])
        width = float(sub["transition_width"].iloc[0])
        spec = SpatialTransitionSpec(start, end, xc, width)
        xs, nhs = _load_traces(sub)
        for x, nh in zip(xs, nhs):
            ax.plot(x, nh, color="#c1432a", alpha=0.35, lw=1.0)
        # mean estimate on a common x-grid
        if xs:
            grid = np.linspace(min(x.min() for x in xs), max(x.max() for x in xs), 160)
            stack = np.vstack([np.interp(grid, x, nh, left=np.nan, right=np.nan)
                               for x, nh in zip(xs, nhs)])
            mean_nh = np.nanmean(stack, axis=0)
            ax.plot(grid, mean_nh, color="#7a1d10", lw=2.2, label=r"$\hat{n}$ (mean)")
            gtru = np.array([local_n_at(float(xi), spec) for xi in grid])
            ax.plot(grid, gtru, color="black", lw=1.8, ls="--", label=r"true $n(x)$")
        ax.axvline(xc, color="#3a6ea5", lw=1.0, alpha=0.7)
        ax.set_title(f"{start} $\\to$ {end}  "
                     f"($n$: {TRUE_N[start]:.1f}$\\to${TRUE_N[end]:.1f})", fontsize=10)
        ax.set_xlabel("vehicle x (m)")
        ax.set_ylabel(r"Bekker $n$")
        ax.set_ylim(0.35, 1.25)
        ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(frameon=False, fontsize=8, loc="best")
    for k in range(len(transitions), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle(f"Online terrain estimate across a spatial soil transition -- "
                 f"{exc_label} (blue line = soil boundary)", fontsize=12)
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_transition_traces.png", dpi=220)
    plt.close(fig)

    # ---- Figure 2: pre/post tracking error + settling distance per transition ----
    agg = ok.groupby("transition", sort=False).agg(
        pre_err=("pre_abs_err", "mean"),
        post_err=("post_abs_err", "mean"),
        resp_dist=("response_dist_m", "mean"),
        resp_dist_std=("response_dist_m", "std"),
        track_rmse=("track_rmse", "mean"),
        n_runs=("status", "count"),
    ).reindex(transitions).reset_index()

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13.0, 4.4))
    y = np.arange(len(agg))
    h = 0.38
    axA.barh(y - h / 2, agg["pre_err"], height=h, color="#5b8fb9",
             label="before transition")
    axA.barh(y + h / 2, agg["post_err"], height=h, color="#d08c60",
             label="after (settled)")
    axA.set_yticks(y)
    axA.set_yticklabels([lbl.replace("_to_", r" $\to$ ") for lbl in agg["transition"]],
                        fontsize=9)
    axA.invert_yaxis()
    axA.set_xlabel(r"Mean $|\hat{n}-n_{\mathrm{true}}|$")
    axA.set_title("Estimate recovers on the new soil")
    axA.grid(axis="x", alpha=0.3)
    axA.legend(frameon=False, fontsize=8, loc="lower right")
    for yi, (pre, post) in enumerate(zip(agg["pre_err"], agg["post_err"])):
        if math.isfinite(pre):
            axA.text(pre + 0.003, yi - h / 2, f"{pre:.3f}", va="center", fontsize=7.5)
        if math.isfinite(post):
            axA.text(post + 0.003, yi + h / 2, f"{post:.3f}", va="center", fontsize=7.5)

    axB.barh(y, agg["resp_dist"], color="#6a8d4f",
             xerr=agg["resp_dist_std"].fillna(0.0), capsize=3)
    axB.set_yticks(y)
    axB.set_yticklabels([lbl.replace("_to_", r" $\to$ ") for lbl in agg["transition"]],
                        fontsize=9)
    axB.invert_yaxis()
    axB.set_xlabel("Settling distance past boundary (m)")
    axB.set_title(f"Distance to {int(round(100*float(ok['response_frac'].iloc[0]))) if 'response_frac' in ok.columns else 63}% of new $n$")
    axB.grid(axis="x", alpha=0.3)
    for yi, d in enumerate(agg["resp_dist"]):
        if math.isfinite(d):
            axB.text(d + 0.4, yi, f"{d:.1f} m", va="center", fontsize=8)

    fig.suptitle(f"Terrain-estimator response to a mid-run soil change -- {exc_label}",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_transition_response.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.transitions = ["clay_to_sand"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 14.0)
        args.transition_x = min(args.transition_x, 22.0)
        args.settle_margin_m = min(args.settle_margin_m, 12.0)

    prefix = ("terrain_transition_ol_benchmark" if args.excitation == "open_loop"
              else "terrain_transition_benchmark")
    out_dir = timestamped_result_dir(prefix)
    description = ("Online terrain estimator tracking a spatial soil transition "
                   f"(per-location SCM soil callback), {args.excitation} excitation, "
                   "sensor noise enabled.")
    write_manifest(out_dir, args, description)
    print(f"Output: {out_dir}")

    learned_model_dir = (
        str(Path(args.learned_terrain_model_dir).resolve())
        if args.learned_terrain_model_dir else None
    )
    unique_build_dir = args.workers > 1

    tasks: list[TransitionTask] = []
    idx = 0
    combos = [(tr, p, sp, b)
              for tr in args.transitions
              for p in args.paths
              for sp in args.speeds
              for b in args.bumpiness]
    total = len(combos) * args.seeds
    for tr, path, speed, bump in combos:
        start, end = parse_transition(tr)
        for seed_i in range(args.seeds):
            seed = args.base_seed + seed_i
            sim_port = args.base_port + 2 * idx
            run_dir = out_dir / "raw" / (
                f"{idx:04d}_{tr}_{path}_v{speed:g}_b{bump}_s{seed}"
            )
            tasks.append(TransitionTask(
                idx=idx, total=total, transition=tr,
                start_preset=start, end_preset=end,
                transition_x=args.transition_x,
                transition_width=args.transition_width,
                path=path, speed=speed, bumpiness=bump, seed=seed,
                sim_port=sim_port, ctrl_port=sim_port + 1,
                run_dir=str(run_dir), sim_time=args.time, lead_in=args.lead_in,
                timeout=args.timeout, metric_start=args.metric_start,
                pre_window_m=args.pre_window_m,
                settle_margin_m=args.settle_margin_m,
                response_frac=args.response_frac,
                sine_amplitude=args.sine_amplitude,
                sine_wavelength=args.sine_wavelength,
                learned_terrain_model_dir=learned_model_dir,
                unique_build_dir=unique_build_dir,
                excitation=args.excitation,
                ol_throttle=args.ol_throttle,
                ol_steer_amp=args.ol_steer_amp,
                ol_steer_period=args.ol_steer_period,
            ))
            idx += 1

    rows: list[dict] = []

    def record(row: dict) -> None:
        row["response_frac"] = args.response_frac
        rows.append(row)
        pd.DataFrame(rows).to_csv(out_dir / "results.csv", index=False)
        resp = row["response_dist_m"]
        resp_str = f"{resp:.1f}m" if isinstance(resp, (int, float)) and math.isfinite(resp) else "n/a"
        print(
            f"[{len(rows)}/{total}] {row['transition']} "
            f"{row['path']} v={row['speed_mps']:g} b={row['bumpiness']} "
            f"seed={row['seed']} {row['status']}: "
            f"pre|err|={row['pre_abs_err']:.3f} post|err|={row['post_abs_err']:.3f} "
            f"resp={resp_str} rms_cte={row['rms_cte_m']:.3f}"
        )

    if tasks:
        print(f"Prewarm: {tasks[0].transition} on port {tasks[0].sim_port}")
        record(_run_one(tasks[0]))
        remaining = tasks[1:]
        if remaining and args.workers > 1:
            with ProcessPoolExecutor(max_workers=args.workers) as ex:
                futures = {ex.submit(_run_one, task): task for task in remaining}
                for fut in as_completed(futures):
                    record(fut.result())
        else:
            for task in remaining:
                record(_run_one(task))

    results_csv = out_dir / "results.csv"
    pd.DataFrame(rows).to_csv(results_csv, index=False)
    if rows:
        summary = pd.DataFrame(rows).groupby("transition", sort=False).agg(
            n_runs=("variant", "count"),
            n_ok=("status", lambda s: int((s == "ok").sum())),
            pre_abs_err_mean=("pre_abs_err", "mean"),
            post_abs_err_mean=("post_abs_err", "mean"),
            track_rmse_mean=("track_rmse", "mean"),
            response_dist_m_mean=("response_dist_m", "mean"),
            response_dist_m_std=("response_dist_m", "std"),
            rms_cte_m_mean=("rms_cte_m", "mean"),
        ).reset_index()
        summary.to_csv(out_dir / "summary_by_transition.csv", index=False)
        save_summary_markdown(
            out_dir,
            "Terrain Transition Benchmark",
            summary,
            [
                "Soil changes type at x={:.0f} m via a per-location SCM callback "
                "(blend width {:.0f} m).".format(args.transition_x, args.transition_width),
                "Ground-truth n(x) is the analytic soil field evaluated at the "
                "logged front-axle position; the estimate is the controller's "
                "deployed sliding-window n.",
                "Settling distance = metres past the boundary until the estimate "
                "covers {:.0f}% of the n step.".format(100 * args.response_frac),
            ],
        )
        plot_figures(results_csv, out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
