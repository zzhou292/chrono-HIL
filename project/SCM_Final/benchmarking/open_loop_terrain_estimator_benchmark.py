#!/usr/bin/env python3
"""Open-loop online terrain-estimator diagnostic.

This is intentionally separate from ``terrain_estimator_benchmark.py``.
The paper/presentation estimator figure there is the deployed closed-loop
case: NMPC tracks a sinusoidal path and the estimator updates inside the
controller.  This diagnostic removes the controller and drives Chrono with a
scripted sinusoidal steering command plus fixed throttle, while running the
same learned estimator online on the streamed VehicleState messages.
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
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PROJECT_ROOT, timestamped_result_dir, write_manifest  # noqa: E402
from terrain_estimator_benchmark import (  # noqa: E402
    DIST_LABELS, TRUE_N, TRUE_PHI, _case_label, generate_ood_terrains,
)

SIM_DIR = PROJECT_ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))
from hil_messages import (  # noqa: E402
    ControlCommand, SimStatus, VehicleState, ZMQPublisher, ZMQSubscriber,
    ctrl_pub_endpoint, sim_sub_endpoint,
)
from learned_terrain_estimator import (  # noqa: E402
    BlendedLearnedTerrainEstimator, HybridJointLearnedTerrainEstimator,
    LearnedTerrainEstimator,
)
from param_consistency import get_terrain_preset, terrain_preset_to_internal  # noqa: E402
from tire_input_features import (  # noqa: E402
    VehicleGeometry, compute_bicycle_operating_point, kappa_from_wheel_speed,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--distributions", nargs="+", default=["id", "ood"],
                   choices=["id", "ood"])
    p.add_argument("--terrains", nargs="+", default=["clay", "dirt", "sand"],
                   choices=["clay", "dirt", "sand"])
    p.add_argument("--ood-terrains", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--base-seed", type=int, default=710)
    p.add_argument("--bumpiness", nargs="+", type=int, default=[0])
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--metric-start", type=float, default=8.0)
    p.add_argument("--throttles", nargs="+", type=float, default=[0.75])
    p.add_argument("--steer-amps", nargs="+", type=float, default=[0.60])
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--te-update-interval", type=int, default=10)
    p.add_argument("--te-min-confidence", type=float, default=0.3)
    p.add_argument("--learned-terrain-model-dir", default=None)
    p.add_argument("--base-port", type=int, default=43000)
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--quick", action="store_true")
    return p.parse_args()


@dataclass(frozen=True)
class OpenLoopTask:
    idx: int
    total: int
    distribution: str
    terrain: str
    case_label: str
    true_n: float
    true_phi: float
    terrain_config: str
    throttle: float
    steer_amp: float
    steer_period: float
    bumpiness: int
    seed: int
    sim_port: int
    ctrl_port: int
    run_dir: str
    sim_time: float
    metric_start: float
    timeout: float
    te_update_interval: int
    te_min_confidence: float
    learned_terrain_model_dir: str | None


def _estimator_class(model_dir: Path):
    blend_cfg = model_dir / "blend.json"
    if not blend_cfg.exists():
        return LearnedTerrainEstimator
    try:
        cfg = json.loads(blend_cfg.read_text())
    except Exception:
        cfg = {}
    if cfg.get("type") == "hybrid_joint":
        return HybridJointLearnedTerrainEstimator
    return BlendedLearnedTerrainEstimator


def _make_estimator(task: OpenLoopTask):
    model_dir = (
        Path(task.learned_terrain_model_dir).expanduser().resolve()
        if task.learned_terrain_model_dir
        else PROJECT_ROOT / "nn_models" / "terrain_window_mlp"
    )
    initial = terrain_preset_to_internal(get_terrain_preset("dirt"))
    cls = _estimator_class(model_dir)
    return cls(
        model_dir=str(model_dir),
        initial_terrain=initial,
        update_interval=task.te_update_interval,
        verbose=False,
        window_size=50,
        min_excitation=0.0,
    )


def _yaw_from_quat(msg: VehicleState) -> float:
    return math.atan2(
        2 * (msg.quat_e0 * msg.quat_e3 + msg.quat_e1 * msg.quat_e2),
        1 - 2 * (msg.quat_e2 ** 2 + msg.quat_e3 ** 2),
    )


def _command_at(t_rel: float, task: OpenLoopTask) -> tuple[float, float, float]:
    """Buzhardt-Tallapragada (2024) style excitation: time-varying throttle
    plus a superposition of a slow/wide and a fast/small steering sine.

      throttle(t) = ramp(t) · [mean + amp·sin(2π·t/4)]
      steer(t)    = wide·sin(2π·t/T_slow + φ_seed) + small·sin(2π·t/T_fast)

    The varied throttle ensures longitudinal-accel transients (key for
    discriminating soft/hard soils via wheel slip + sinkage), and the
    two-frequency steering exercises both quasi-static cornering (slow)
    and transient yaw response (fast).
    """
    ramp = min(max(t_rel / 1.0, 0.0), 1.0)
    thr_mean = float(task.throttle)
    thr_amp = 0.30 * thr_mean
    throttle = float(ramp * (thr_mean + thr_amp *
                              math.sin(2.0 * math.pi * t_rel / 4.0)))
    phi_seed = 0.07 * task.seed
    steer_wide = float(task.steer_amp *
                       math.sin(2.0 * math.pi * (t_rel + phi_seed) /
                                task.steer_period))
    steer_fast = float(0.15 *
                       math.sin(2.0 * math.pi * t_rel / 1.0))
    steer = max(-1.0, min(1.0, steer_wide + steer_fast))
    return throttle, steer, 0.0


def _metrics_from_diag(diag_csv: Path, true_n: float, metric_start: float) -> dict:
    df = pd.read_csv(diag_csv)
    if df.empty:
        raise ValueError("empty open-loop diagnostic CSV")
    t = pd.to_numeric(df["sim_time"], errors="coerce")
    tail = np.isfinite(t) & (t >= metric_start)
    if not tail.any():
        tail = np.isfinite(t)
    n_est = pd.to_numeric(df["n_terrain_est"], errors="coerce")
    finite = np.isfinite(n_est)
    tail_n = tail & finite
    update = pd.to_numeric(df["terrain_update_applied"], errors="coerce").fillna(0) > 0
    speed = pd.to_numeric(df["u_true"], errors="coerce")
    speed_tail = speed[tail] if len(speed) == len(df) else speed
    final_n = float(n_est[finite].iloc[-1]) if finite.any() else math.nan
    tail_mean = float(n_est[tail_n].mean()) if tail_n.any() else math.nan
    return {
        "n_samples": int(len(df)),
        "mean_speed_mps": float(np.nanmean(speed_tail)) if len(speed_tail) else math.nan,
        "p95_speed_mps": float(np.nanpercentile(speed_tail, 95)) if len(speed_tail) else math.nan,
        "n_est_final": final_n,
        "n_est_mean_tail": tail_mean,
        "n_abs_err_final": abs(final_n - true_n) if math.isfinite(final_n) else math.nan,
        "n_abs_err_tail": abs(tail_mean - true_n) if math.isfinite(tail_mean) else math.nan,
        "terrain_update_count": int(update.sum()),
        "first_update_time_s": float(t[update].iloc[0]) if update.any() else math.nan,
    }


def _run_one(task: OpenLoopTask) -> dict:
    run_dir = Path(task.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    diag_csv = run_dir / "open_loop_diag.csv"
    log_path = run_dir / "run.log"
    sim_cmd = [
        sys.executable, "-u", str(SIM_DIR / "chrono_sim_node.py"),
        "--time", str(task.sim_time + 6.0),
        "--speed", "5",
        "--terrain", task.terrain,
        "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(task.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(task.ctrl_port),
        "--bumpiness", str(task.bumpiness),
        "--no-wait-for-controller",
    ]
    if task.terrain_config:
        sim_cmd += ["--terrain-config", task.terrain_config]

    proc = None
    rows: list[dict] = []
    estimator = _make_estimator(task)
    geom = VehicleGeometry.from_hmmwv_defaults()
    default_params = terrain_preset_to_internal(get_terrain_preset("dirt"))
    applied_n = float(default_params["n"])
    applied_phi = float(default_params["phi"])
    confidence = 0.0
    seq = 0
    t0_sim = None
    prev_delta = None
    prev_t = None
    status = "ok"
    notes = ""

    try:
        with log_path.open("w") as log:
            proc = subprocess.Popen(
                sim_cmd, cwd=str(PROJECT_ROOT), stdout=log, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
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
            throttle, steer, brake = _command_at(t_rel, task)
            cmd = ControlCommand(
                time=float(msg.time),
                wall_time=time.time(),
                seq=seq,
                steering=steer,
                throttle=throttle,
                braking=brake,
                delta=float(msg.steering_angle),
                acceleration=float(msg.ax),
                delta_dot=0.0,
                jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1

            dt = (
                max(float(msg.time) - prev_t, 1e-4)
                if prev_t is not None else 1e-2
            )
            sr = (
                (float(msg.steering_angle) - prev_delta) / dt
                if prev_delta is not None else 0.0
            )
            prev_delta = float(msg.steering_angle)
            prev_t = float(msg.time)

            measured_kappa = kappa_from_wheel_speed(
                msg.wheel_omega_fl, msg.wheel_omega_fr,
                msg.wheel_omega_rl, msg.wheel_omega_rr, msg.u,
            )
            terrain_mu = max(math.tan(math.radians(applied_phi)), 0.1)
            kappa, alpha_f, alpha_r, u_safe, Fz_f, Fz_r = (
                compute_bicycle_operating_point(
                    float(msg.steering_angle),
                    float(msg.u),
                    float(msg.v),
                    float(msg.omega),
                    float(msg.ax),
                    geom=geom,
                    kappa_mode="measured",
                    terrain_mu=terrain_mu,
                    measured_kappa=measured_kappa,
                )
            )

            update_applied = 0
            omega_dot = estimator.estimate_omega_dot(float(msg.omega), float(msg.time))
            if omega_dot is not None:
                estimator.observe(
                    kappa=float(kappa),
                    alpha_f=float(alpha_f),
                    alpha_r=float(alpha_r),
                    u=float(u_safe),
                    Fz_f=float(Fz_f),
                    Fz_r=float(Fz_r),
                    sr=float(sr),
                    ay_imu=float(msg.ay),
                    omega_dot=float(omega_dot),
                    omega=float(msg.omega),
                    v_lateral=float(msg.v),
                    x_pos=float(msg.x_cg),
                    y_pos=float(msg.y_cg),
                    psi=_yaw_from_quat(msg),
                    ax_cmd=float(msg.ax),
                    sim_time=float(t_rel),
                    wheel_omegas=(
                        float(msg.wheel_omega_fl),
                        float(msg.wheel_omega_fr),
                        float(msg.wheel_omega_rl),
                        float(msg.wheel_omega_rr),
                    ),
                    ax_imu=float(msg.ax),
                    throttle_cmd=float(throttle),
                )
            if estimator.should_update():
                params, confidence = estimator.estimate()
                if confidence >= task.te_min_confidence:
                    applied_n = float(params["n"])
                    applied_phi = float(params["phi"])
                    update_applied = 1

            rows.append({
                "sim_time": f"{t_rel:.4f}",
                "seq": seq,
                "u_true": f"{float(msg.u):.6f}",
                "v_true": f"{float(msg.v):.6f}",
                "omega": f"{float(msg.omega):.6f}",
                "ax": f"{float(msg.ax):.6f}",
                "ay": f"{float(msg.ay):.6f}",
                "steering": f"{steer:.6f}",
                "throttle": f"{throttle:.6f}",
                "steering_angle": f"{float(msg.steering_angle):.6f}",
                "n_terrain_est": f"{applied_n:.6f}",
                "n_terrain_estimator": f"{estimator.get_bekker_n():.6f}",
                "terrain_confidence": f"{confidence:.6f}",
                "terrain_update_applied": int(update_applied),
            })
            if t_rel >= task.sim_time:
                break
        else:
            status = "timeout"
    except Exception as exc:
        status = "error"
        notes = repr(exc)
    finally:
        try:
            state_sub.close()
            ctrl_pub.close()
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
                proc.wait(timeout=5)

    if len(rows) < 100 and status == "ok":
        status = "too_few_samples"
    if rows:
        with diag_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    row = {
        "distribution": task.distribution,
        "case_label": task.case_label,
        "terrain": task.terrain,
        "terrain_config": task.terrain_config,
        "path": "sinusoidal",
        "excitation_source": "open_loop_command",
        "true_n": task.true_n,
        "true_phi_deg": task.true_phi,
        "status": status,
        "run_dir": str(run_dir),
        "diag_csv": str(diag_csv) if diag_csv.exists() else "",
        "open_loop_throttle": task.throttle,
        "steer_amp": task.steer_amp,
        "steer_period": task.steer_period,
        "bumpiness": task.bumpiness,
        "seed": task.seed,
        "notes": notes,
    }
    if status == "ok":
        row.update(_metrics_from_diag(diag_csv, task.true_n, task.metric_start))
    return row


def _display_dist(value: str) -> str:
    return DIST_LABELS.get(value, value)


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(exist_ok=True)
    summary = ok.groupby("distribution", sort=False).agg(
        n_err=("n_abs_err_tail", "mean"),
        n_runs=("status", "count"),
        mean_speed=("mean_speed_mps", "mean"),
    ).reset_index()
    summary["display"] = summary["distribution"].map(_display_dist)
    case = ok.groupby(["distribution", "case_label", "true_n"], sort=False).agg(
        n_err=("n_abs_err_tail", "mean"),
        mean_speed=("mean_speed_mps", "mean"),
        runs=("status", "count"),
    ).reset_index()
    case["display"] = [
        _case_label(c, d) for c, d in zip(case["case_label"], case["distribution"])
    ]
    case = case.sort_values(["true_n", "distribution", "display"]).reset_index(drop=True)

    colors = {"id": "#5b8fb9", "ood": "#d08c60"}
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.3))
    x = np.arange(len(summary))
    axes[0].bar(x, summary["n_err"], color=[colors.get(d, "#777") for d in summary["distribution"]])
    for i, row in summary.iterrows():
        axes[0].text(i, row["n_err"] + 0.004, f"{row['n_err']:.3f}\nN={int(row['n_runs'])}",
                     ha="center", va="bottom", fontsize=8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(summary["display"])
    axes[0].set_ylabel("Mean tail |n error|")
    axes[0].set_title("Aggregate error")
    axes[0].grid(axis="y", alpha=0.3)
    axes[0].set_ylim(0.0, float(summary["n_err"].max()) * 1.35)

    y = np.arange(len(case))
    axes[1].barh(y, case["n_err"], color=[colors.get(d, "#777") for d in case["distribution"]])
    axes[1].set_yticks(y)
    axes[1].set_yticklabels([
        f"{r.display}  n={r.true_n:.2f}, ubar={r.mean_speed:.2f} m/s"
        for r in case.itertuples()
    ], fontsize=8)
    axes[1].invert_yaxis()
    axes[1].set_xlabel("Mean tail |n error|")
    axes[1].set_title("Per-terrain error")
    axes[1].grid(axis="x", alpha=0.3)
    for yi, row in zip(y, case.itertuples()):
        axes[1].text(row.n_err + 0.003, yi, f"{row.n_err:.3f}",
                     va="center", ha="left", fontsize=8)
    fig.suptitle("Online n-only terrain estimator (open-loop commanded sine steering)")
    fig.tight_layout()
    fig.savefig(fig_dir / "open_loop_terrain_estimator_summary.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.distributions = ["id", "ood"]
        args.terrains = ["clay"]
        args.ood_terrains = 1
        args.seeds = 1
        args.bumpiness = [0]
        args.throttles = [args.throttles[0]]
        args.steer_amps = [args.steer_amps[0]]
        args.time = min(args.time, 10.0)
        args.workers = 1
    out_dir = timestamped_result_dir("open_loop_terrain_estimator_benchmark")
    write_manifest(out_dir, args, "Open-loop commanded online terrain-estimator diagnostic.")

    cases = []
    if "id" in args.distributions:
        for terrain in args.terrains:
            cases.append({
                "distribution": "id",
                "terrain": terrain,
                "case_label": terrain,
                "true_n": TRUE_N[terrain],
                "true_phi": TRUE_PHI[terrain],
                "terrain_config": "",
            })
    if "ood" in args.distributions:
        manifest = generate_ood_terrains(out_dir, args.ood_terrains, args.base_seed)
        for _, r in manifest.iterrows():
            cases.append({
                "distribution": "ood",
                "terrain": str(r["preset_proxy"]),
                "case_label": str(r["label"]),
                "true_n": float(r["true_n"]),
                "true_phi": float(r["friction_angle"]),
                "terrain_config": str(r["yaml"]),
            })

    tasks: list[OpenLoopTask] = []
    idx = 0
    for case in cases:
        for throttle in args.throttles:
            for steer_amp in args.steer_amps:
                for bump in args.bumpiness:
                    for s in range(args.seeds):
                        seed = args.base_seed + s
                        idx += 1
                        run_name = (
                            f"{idx:04d}_{case['distribution']}_{case['case_label']}"
                            f"_ol_thr{int(round(throttle * 100)):02d}"
                            f"_amp{int(round(steer_amp * 100)):02d}_b{bump}_s{seed}"
                        )
                        tasks.append(OpenLoopTask(
                            idx=idx,
                            total=0,
                            distribution=case["distribution"],
                            terrain=case["terrain"],
                            case_label=case["case_label"],
                            true_n=case["true_n"],
                            true_phi=case["true_phi"],
                            terrain_config=case["terrain_config"],
                            throttle=throttle,
                            steer_amp=steer_amp,
                            steer_period=args.steer_period,
                            bumpiness=bump,
                            seed=seed,
                            sim_port=args.base_port + 2 * idx,
                            ctrl_port=args.base_port + 2 * idx + 1,
                            run_dir=str(out_dir / "raw" / run_name),
                            sim_time=args.time,
                            metric_start=args.metric_start,
                            timeout=args.timeout,
                            te_update_interval=args.te_update_interval,
                            te_min_confidence=args.te_min_confidence,
                            learned_terrain_model_dir=args.learned_terrain_model_dir,
                        ))
    total = len(tasks)
    tasks = [OpenLoopTask(**{**asdict(t), "total": total}) for t in tasks]
    print(f"[open-loop terrain estimator] {total} runs -> {out_dir}")

    rows: list[dict] = []
    results_csv = out_dir / "results.csv"
    if tasks:
        first = _run_one(tasks[0])
        rows.append(first)
        print(f"[1/{total}] {first['distribution']}:{first['case_label']} {first['status']} "
              f"err={first.get('n_abs_err_tail', math.nan):.3f} "
              f"ubar={first.get('mean_speed_mps', math.nan):.2f}")
        pd.DataFrame(rows).to_csv(results_csv, index=False)
        remaining = tasks[1:]
        workers = max(1, min(int(args.workers), len(remaining) or 1))
        if remaining:
            with ProcessPoolExecutor(max_workers=workers) as ex:
                futs = {ex.submit(_run_one, task): task for task in remaining}
                for fut in as_completed(futs):
                    row = fut.result()
                    rows.append(row)
                    print(f"[{len(rows)}/{total}] {row['distribution']}:{row['case_label']} "
                          f"{row['status']} err={row.get('n_abs_err_tail', math.nan):.3f} "
                          f"ubar={row.get('mean_speed_mps', math.nan):.2f}",
                          flush=True)
                    pd.DataFrame(rows).to_csv(results_csv, index=False)
    df = pd.DataFrame(rows)
    df.to_csv(results_csv, index=False)
    summary = df.groupby("distribution", sort=False).agg(
        n_runs=("status", "count"),
        n_ok=("status", lambda s: int((s == "ok").sum())),
        n_abs_err_tail_mean=("n_abs_err_tail", "mean"),
        n_abs_err_tail_std=("n_abs_err_tail", "std"),
        mean_speed_mps_mean=("mean_speed_mps", "mean"),
        first_update_time_s_mean=("first_update_time_s", "mean"),
    ).reset_index()
    summary.to_csv(out_dir / "summary_by_distribution.csv", index=False)
    plot_figures(results_csv, out_dir)
    print(summary.to_string(index=False))
    print(f"[done] {out_dir}")


if __name__ == "__main__":
    main()
