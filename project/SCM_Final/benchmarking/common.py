#!/usr/bin/env python3
"""Shared helpers for reproducible SCM_Teleop paper experiments."""

from __future__ import annotations

import csv
import ast
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = PROJECT_ROOT / "simulation"
LAUNCHER = SIM_DIR / "launch_decoupled.py"
LOGS_DIR = PROJECT_ROOT / "logs"
RESULTS_ROOT = Path(__file__).resolve().parent / "results"
# DEFAULT_NN_MODEL is the active whole-vehicle surrogate used by paper
# sweeps that do not explicitly set --nn-model.
DEFAULT_NN_MODEL = "vehicle_rate_64_32_lhs"

PATHS = ("sinusoidal", "lane_change", "right_left")
TERRAINS = ("clay", "dirt", "sand")
SPEEDS = (5.0, 7.0, 9.0)
BUMPS = (0, 4, 8)

PATH_ROCK_ZONES: dict[str, dict[str, tuple[float, float]]] = {
    # 'straight' is the HIL boulder field: rocks span the full width (both
    # sides, well beyond any swerve) over the whole course, so there is no
    # clean lateral bypass -- you thread a route, you don't go around.
    "straight": {"x": (10.0, 98.0), "y": (-16.0, 16.0)},
    "sinusoidal": {"x": (12.0, 50.0), "y": (-3.0, 3.0)},
    "lane_change": {"x": (15.0, 50.0), "y": (-1.0, 4.0)},
    "double_lane_change": {"x": (15.0, 60.0), "y": (-1.0, 4.0)},
    "right_left": {"x": (10.0, 22.0), "y": (-3.0, 3.0)},
}

RX_SIM_COMPLETE = re.compile(
    r"Simulation complete:\s*([\d.]+)s in ([\d.]+)s\s*\(RT factor ([\d.]+)x\)"
)
RX_COLLISIONS = re.compile(r"Hard collisions:\s*(\d+)\s+Near misses:\s*(\d+)")


@dataclass
class RunResult:
    experiment: str
    variant: str
    controller_mode: str
    mpc_model: str
    nn_model: str
    terrain: str
    path: str
    speed_mps: float
    bumpiness: int
    seed: int
    run_dir: str
    status: str = "ok"
    rc: int = 0
    wall_s: float = math.nan
    sim_s: float = math.nan
    rt_factor: float = math.nan
    diag_csv: str = ""
    collision_csv: str = ""
    shield_csv: str = ""
    n_samples: int = 0
    rms_cte_m: float = math.nan
    max_abs_cte_m: float = math.nan
    mean_abs_cte_m: float = math.nan
    mean_speed_mps: float = math.nan
    p95_speed_mps: float = math.nan
    speed_ratio: float = math.nan
    mean_solve_ms: float = math.nan
    p99_solve_ms: float = math.nan
    progress_m: float = math.nan
    final_x_m: float = math.nan
    final_y_m: float = math.nan
    collisions: int = 0
    near_misses: int = 0
    min_clearance_m: float = math.nan
    intervention_rate_pct: float = math.nan
    mean_abs_dsteer: float = math.nan
    mean_abs_dthrottle: float = math.nan
    notes: str = ""
    extra: dict[str, Any] = field(default_factory=dict)


def ensure_runtime_env() -> None:
    os.environ.setdefault("ACADOS_SOURCE_DIR", "/home/ksha/Documents/sbel/acados")


def timestamped_result_dir(prefix: str) -> Path:
    # Second-resolution timestamps collide when two runs start in the same
    # second (e.g. a fast-failing sub-run, or back-to-back launches). Append a
    # numeric suffix on collision instead of raising FileExistsError.
    base = f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}"
    out = RESULTS_ROOT / base
    n = 1
    while out.exists():
        out = RESULTS_ROOT / f"{base}_{n}"
        n += 1
    out.mkdir(parents=True, exist_ok=False)
    (out / "raw").mkdir()
    (out / "figures").mkdir()
    return out


def write_manifest(out_dir: Path, args: Any, description: str) -> None:
    rows = [
        ("created_at", datetime.now().isoformat(timespec="seconds")),
        ("project_root", str(PROJECT_ROOT)),
        ("description", description),
        ("command", " ".join(sys.argv)),
    ]
    for k, v in sorted(vars(args).items()):
        rows.append((k, repr(v)))
    with (out_dir / "manifest.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["key", "value"])
        writer.writerows(rows)


def base_launch_args(
    *,
    terrain: str,
    path: str,
    speed: float,
    sim_time: float,
    bumpiness: int,
    seed: int,
    run_dir: Path,
    sim_port: int,
    ctrl_port: int,
    lead_in: float = 5.0,
    rocks: int = 0,
    no_plot: bool = True,
) -> list[str]:
    cmd = [
        sys.executable, "-u", str(LAUNCHER),
        "--terrain", terrain,
        "--path", path,
        "--speed", str(speed),
        "--time", str(sim_time),
        "--lead-in", str(lead_in),
        "--bumpiness", str(bumpiness),
        "--rocks", str(rocks),
        "--rock-seed", str(seed),
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--plot-dir", str(run_dir),
        "--no-vis",
    ]
    if no_plot:
        cmd.append("--no-plot")
    if rocks > 0:
        zone = PATH_ROCK_ZONES.get(path, PATH_ROCK_ZONES["sinusoidal"])
        cmd += [
            "--rock-zone-x", str(zone["x"][0]), str(zone["x"][1]),
            "--rock-zone-y", str(zone["y"][0]), str(zone["y"][1]),
            "--rock-size", "0.8", "1.8",
        ]
    return cmd


def run_process(cmd: list[str], run_dir: Path, timeout: float) -> tuple[int, float, str]:
    ensure_runtime_env()
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "run.log"
    t0 = time.time()
    try:
        # Give each run a PRIVATE log directory so concurrent workers never
        # share the global ``logs/`` collision / shield / CBF logs (which
        # races on truncation and cross-contaminates collision counts).  The
        # sim and safety filters honour ``HIL_RUN_LOG_DIR`` when set.
        _env = os.environ.copy()
        _env["HIL_RUN_LOG_DIR"] = str(run_dir)
        # Pin numpy/BLAS to a single thread per run.  The MPPI shield's
        # vectorized rollout is many tiny (≤14×K) matmuls; multi-threaded
        # OpenBLAS adds pure thread-spawn overhead there, and with several
        # parallel workers the default 24-thread pool oversubscribes the box
        # ~6x and inflates each MPPI run from ~100 s to ~350 s (some then hit
        # the wall-clock timeout).  Single-threaded BLAS is both faster for
        # these shapes and contention-free.  acados (BLASFEO, no OpenMP) and
        # Chrono's own OpenMP threads are unaffected by these vars.
        _env.setdefault("OPENBLAS_NUM_THREADS", "1")
        _env.setdefault("MKL_NUM_THREADS", "1")
        _env.setdefault("NUMEXPR_NUM_THREADS", "1")
        with log_path.open("w") as f:
            proc = subprocess.run(
                cmd,
                cwd=str(PROJECT_ROOT),
                stdout=f,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                env=_env,
            )
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        rc = -9
        with log_path.open("a") as f:
            f.write(f"\nTIMEOUT after {timeout:.1f}s\n")
    return rc, time.time() - t0, log_path.read_text(errors="replace")


def find_diag_csv(run_dir: Path, controller_mode: str, created_after: float) -> Path | None:
    candidates = [
        p for p in run_dir.rglob("diag_*.csv")
        if p.stat().st_mtime >= created_after - 2.0
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def copy_global_log(name: str, run_dir: Path, created_after: float) -> Path | None:
    src = LOGS_DIR / name
    if not src.exists() or src.stat().st_mtime < created_after - 2.0:
        return None
    dst = run_dir / name
    shutil.copy2(src, dst)
    return dst


def _float_series(df: pd.DataFrame, key: str) -> np.ndarray:
    if key not in df.columns:
        return np.asarray([], dtype=float)
    return pd.to_numeric(df[key], errors="coerce").to_numpy(dtype=float)


def _finite(v: np.ndarray) -> np.ndarray:
    return v[np.isfinite(v)]


def parse_diag_csv(path: Path, controller_mode: str, speed: float, metric_start: float = 2.0) -> dict[str, float]:
    # Defensive: a Chrono run can leave behind a header-only or zero-byte
    # diag CSV when the controller dies mid-init. ``pd.read_csv`` raises
    # ``EmptyDataError`` in that case, which would otherwise kill the whole
    # sweep instead of just marking the run as `no_diag`.
    try:
        df = pd.read_csv(path)
    except (pd.errors.EmptyDataError, pd.errors.ParserError):
        return {"n_samples": 0}
    if df.empty:
        return {"n_samples": 0}
    t = _float_series(df, "sim_time")
    mask = np.ones(len(df), dtype=bool)
    if t.size == len(df) and np.any(np.isfinite(t)):
        mask = np.isfinite(t) & (t >= metric_start)
        if not mask.any():
            mask = np.isfinite(t)

    cte = _float_series(df, "crosstrack_err")
    u = _float_series(df, "u_true")
    if u.size == 0:
        u = _float_series(df, "u_meas")
    solve = _float_series(df, "solve_time_ms")
    x = _float_series(df, "x_fa_true")
    if x.size == 0:
        x = _float_series(df, "x_fa_meas")
    y = _float_series(df, "y_fa_true")
    if y.size == 0:
        y = _float_series(df, "y_fa_meas")
    extra = {}

    cte_m = _finite(cte[mask]) if cte.size == len(df) else _finite(cte)
    u_m = _finite(u[mask]) if u.size == len(df) else _finite(u)
    solve_m = _finite(solve)

    progress = math.nan
    final_x = math.nan
    final_y = math.nan
    if x.size == len(df) and y.size == len(df):
        xf = _finite(x)
        yf = _finite(y)
        if len(xf):
            final_x = float(xf[-1])
        if len(yf):
            final_y = float(yf[-1])
        good = np.isfinite(x) & np.isfinite(y)
        if np.count_nonzero(good) >= 2:
            progress = float(np.sum(np.hypot(np.diff(x[good]), np.diff(y[good]))))
    out = {
        "n_samples": int(len(df)),
        "rms_cte_m": float(np.sqrt(np.mean(cte_m ** 2))) if len(cte_m) else math.nan,
        "max_abs_cte_m": float(np.max(np.abs(cte_m))) if len(cte_m) else math.nan,
        "mean_abs_cte_m": float(np.mean(np.abs(cte_m))) if len(cte_m) else math.nan,
        "mean_speed_mps": float(np.mean(u_m)) if len(u_m) else math.nan,
        "p95_speed_mps": float(np.percentile(u_m, 95)) if len(u_m) else math.nan,
        "speed_ratio": float(np.mean(u_m) / speed) if len(u_m) and speed > 1e-6 else math.nan,
        "mean_solve_ms": float(np.mean(solve_m)) if len(solve_m) else math.nan,
        "p99_solve_ms": float(np.percentile(solve_m, 99)) if len(solve_m) else math.nan,
        "progress_m": progress,
        "final_x_m": final_x,
        "final_y_m": final_y,
    }
    out.update(extra)
    return out


def parse_collision_csv(path: Path | None) -> dict[str, float | int]:
    if path is None or not path.exists():
        return {}
    # Zero-collision runs leave a header-only / zero-byte log; older sweeps
    # additionally produced ragged rows when the sim aborted mid-frame.
    # Be defensive: a missing parser-friendly CSV is "no collisions logged",
    # never an exception that fails the whole worker.
    try:
        df = pd.read_csv(path)
    except (pd.errors.EmptyDataError, pd.errors.ParserError):
        return {"collisions": 0, "near_misses": 0}
    if df.empty:
        return {"collisions": 0, "near_misses": 0}
    hit_ids: set[int] = set()
    near_ids: set[int] = set()
    clearances = []
    # Coerce the integer columns once: pandas reads NaN-bearing columns as
    # float64 and `int(NaN)` raises ValueError, which would otherwise tear
    # down the whole worker. nan -> 0 (no event) is the correct fallback.
    def _i(v, default=0):
        try:
            f = float(v)
        except (TypeError, ValueError):
            return default
        if not math.isfinite(f):
            return default
        return int(f)
    for _, row in df.iterrows():
        rid = _i(row.get("rock_id", -1), -1)
        if _i(row.get("is_collision", 0)) == 1 and rid >= 0:
            hit_ids.add(rid)
        if _i(row.get("is_near_miss", 0)) == 1 and rid >= 0:
            near_ids.add(rid)
        d = float(row.get("dist_2d", math.nan))
        hard = float(row.get("hard_margin", math.nan))
        if math.isfinite(d) and math.isfinite(hard):
            clearances.append(d - hard)
    return {
        "collisions": len(hit_ids),
        "near_misses": len(near_ids),
        "min_clearance_m": float(np.min(clearances)) if clearances else math.nan,
    }


def parse_shield_csv(path: Path | None) -> dict[str, float]:
    if path is None or not path.exists():
        return {}
    try:
        df = pd.read_csv(path)
    except (pd.errors.EmptyDataError, pd.errors.ParserError):
        return {}
    if df.empty:
        return {}
    required = {"steer_in", "steer_out", "throttle_in", "throttle_out"}
    if not required.issubset(df.columns):
        return {}
    return {
        "mean_abs_dsteer": float((df["steer_out"] - df["steer_in"]).abs().mean()),
        "mean_abs_dthrottle": float((df["throttle_out"] - df["throttle_in"]).abs().mean()),
    }


def parse_sim_diag_csv(path: Path | None, metric_start: float = 2.0) -> dict[str, float | int]:
    if path is None or not path.exists():
        return {}
    try:
        df = pd.read_csv(path)
    except (pd.errors.EmptyDataError, pd.errors.ParserError):
        return {}
    if df.empty:
        return {}
    out: dict[str, float | int] = {}
    t = pd.to_numeric(df.get("time", pd.Series(np.zeros(len(df)))), errors="coerce")
    mask = np.isfinite(t) & (t >= metric_start)
    if not mask.any():
        mask = np.isfinite(t)
    if "nearest_clearance_m" in df.columns:
        clearance = pd.to_numeric(df["nearest_clearance_m"], errors="coerce")
        c = clearance[mask] if len(clearance) == len(df) else clearance
        if len(c) and np.isfinite(c).any():
            out["min_clearance_m"] = float(np.nanmin(c))
    for col, key in [("collisions", "collisions"), ("near_misses", "near_misses")]:
        if col in df.columns:
            vals = pd.to_numeric(df[col], errors="coerce")
            vals = vals[np.isfinite(vals)]
            if len(vals):
                # int(NaN) raises; the dropna above protects us, but if the
                # column was all-NaN we already skipped it.
                out[key] = int(vals.iloc[-1])
    return out


def parse_log_summary(text: str) -> dict[str, float | int]:
    out: dict[str, float | int] = {}
    if m := RX_SIM_COMPLETE.search(text):
        out["sim_s"] = float(m.group(1))
        out["rt_factor"] = float(m.group(3))
    if m := RX_COLLISIONS.search(text):
        out["collisions"] = int(m.group(1))
        out["near_misses"] = int(m.group(2))
    if "[SAFETY]" in text:
        m = re.search(r"\[SAFETY\]\s*Calls:\s*(\d+),\s*Interventions:\s*(\d+)\s*\(([\d.]+)%\)", text)
        if m:
            out["intervention_rate_pct"] = float(m.group(3))
    return out


def launch_and_collect(
    *,
    experiment: str,
    variant: str,
    controller_mode: str,
    mpc_model: str,
    nn_model: str,
    terrain: str,
    path: str,
    speed: float,
    bumpiness: int,
    seed: int,
    run_dir: Path,
    sim_port: int,
    ctrl_port: int,
    sim_time: float,
    timeout: float,
    rocks: int = 0,
    lead_in: float = 5.0,
    extra_args: list[str] | None = None,
    metric_start: float = 2.0,
) -> RunResult:
    created_after = time.time()
    # Logs now land in the per-run ``run_dir`` (via HIL_RUN_LOG_DIR set in
    # run_process), so there is no shared global log to clear — and the old
    # ``if exists(): unlink()`` on the global path raced across workers
    # (FileNotFoundError when two workers cleared it concurrently).

    cmd = base_launch_args(
        terrain=terrain, path=path, speed=speed, sim_time=sim_time,
        bumpiness=bumpiness, seed=seed, run_dir=run_dir,
        sim_port=sim_port, ctrl_port=ctrl_port, lead_in=lead_in, rocks=rocks,
    )
    # MPCC is archived (2026-05-23). Only the standard NMPC remains.
    if controller_mode != "standard":
        raise ValueError(
            f"controller_mode={controller_mode!r}: only 'standard' is supported "
            "since MPCC was archived in 2026-05-23."
        )
    cmd += ["--model", mpc_model, "--nn-model", nn_model, "--rms-time-start", str(metric_start)]
    if extra_args:
        cmd += list(extra_args)
    sim_diag = run_dir / "sim_diag.csv"
    if rocks > 0:
        cmd += ["--sim-diag-csv", str(sim_diag)]

    rc, wall_s, text = run_process(cmd, run_dir, timeout)
    diag = find_diag_csv(run_dir, controller_mode, created_after)
    # The sim wrote its collision / shield logs directly into run_dir
    # (HIL_RUN_LOG_DIR), so read them per-run instead of copying a global file.
    _coll = run_dir / "collision_log.csv"
    collision = _coll if (rocks > 0 and _coll.exists()) else None
    shield = None
    for name in ("mppi_shield_log.csv", "nmpc_shield_log.csv", "cbf_filter_log.csv"):
        p = run_dir / name
        if p.exists():
            shield = p

    result = RunResult(
        experiment=experiment,
        variant=variant,
        controller_mode=controller_mode,
        mpc_model=mpc_model,
        nn_model=nn_model,
        terrain=terrain,
        path=path,
        speed_mps=speed,
        bumpiness=bumpiness,
        seed=seed,
        run_dir=str(run_dir),
        rc=rc,
        wall_s=wall_s,
        status="ok" if rc == 0 else f"exit_{rc}",
        diag_csv=str(diag) if diag else "",
        collision_csv=str(collision) if collision else "",
        shield_csv=str(shield) if shield else "",
    )
    for k, v in parse_log_summary(text).items():
        setattr(result, k, v)
    if diag is not None:
        diag_metrics = parse_diag_csv(diag, controller_mode, speed, metric_start)
        for k, v in diag_metrics.items():
            if hasattr(result, k):
                setattr(result, k, v)
            else:
                result.extra[k] = v
    elif rc == 0:
        result.status = "no_diag"
    for k, v in parse_collision_csv(collision).items():
        setattr(result, k, v)
    result.extra["sim_diag_csv"] = str(sim_diag) if sim_diag.exists() else ""
    for k, v in parse_sim_diag_csv(sim_diag if sim_diag.exists() else None, metric_start).items():
        current = getattr(result, k, math.nan)
        if k in ("collisions", "near_misses"):
            if collision is None:
                setattr(result, k, v)
        elif not math.isfinite(float(current)):
            setattr(result, k, v)
    for k, v in parse_shield_csv(shield).items():
        setattr(result, k, v)
    return result


def write_results_csv(path: Path, results: list[RunResult]) -> None:
    rows = []
    for r in results:
        d = asdict(r)
        extra = d.pop("extra", {}) or {}
        d.update({f"extra_{k}": v for k, v in extra.items()})
        rows.append(d)
    pd.DataFrame(rows).to_csv(path, index=False)


def summarize_by_variant(results: list[RunResult], metrics: list[str]) -> pd.DataFrame:
    df = pd.DataFrame([asdict(r) for r in results])
    if df.empty:
        return df
    agg: dict[str, Any] = {
        "n_runs": ("variant", "count"),
        "n_ok": ("status", lambda s: int((s == "ok").sum())),
    }
    for m in metrics:
        agg[f"{m}_mean"] = (m, "mean")
        agg[f"{m}_std"] = (m, "std")
    return df.groupby("variant", sort=False).agg(**agg).reset_index()


def save_summary_markdown(out_dir: Path, title: str, summary: pd.DataFrame, notes: list[str]) -> None:
    lines = [f"# {title}", ""]
    lines.extend(notes)
    lines.append("")
    if not summary.empty:
        lines.append("```csv")
        lines.append(summary.to_csv(index=False).strip())
        lines.append("```")
    (out_dir / "summary.md").write_text("\n".join(lines))


# ---------------------------------------------------------------------------
# Paper figure helpers
# ---------------------------------------------------------------------------

PAPER_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]


def _import_plotting():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text)).strip("_")


DISPLAY_LABELS = {
    "pacejka": "Pacejka",
    "pacejka_static": "Pacejka static",
    "tmeasy": "TMeasy",
    "tmeasy_static": "TMeasy static",
    "closed_loop_mlp": "Vehicle NN",
    "closed_loop_v1_mlp_32_16": "Vehicle NN",
    "closed_loop_v2_rate_mlp": "Vehicle NN rate",
    "closed_loop_v2_both_axles_rate_32_16": "Vehicle NN rate",
    "closed_loop_v3_axle_rate_mlp": "Vehicle NN axle-rate",
    "closed_loop_v3_axle_rate_64_32": "Vehicle NN axle-rate",
    "vehicle_rate_64_32_lhs": "Vehicle NN rate",
    "vehicle_rate_32_16_lhs": "Vehicle NN rate",
    "vehicle_static_32_16_lhs": "Vehicle NN static",
    "nn_static": "NN static prior",
    "nn_estimator": "NN live n estimator",
    "nn_wrong_prior": "NN wrong prior",
    "nn_v3_static": "NN static prior",
    "nn_v3_estimator": "NN live n estimator",
    "nn_v3_wrong_prior": "NN wrong prior",
    "rig_static": "Rig NN static",
    "rig_rate": "Rig NN rate",
    "rig_static_lg": "Rig NN static",
    "rig_rate_lg": "Rig NN rate",
    "rig_rate_xl": "Rig NN rate",
    "vehicle_static": "Vehicle NN static",
    "vehicle_rate": "Vehicle NN rate",
    "vehicle_static_lhs": "Vehicle NN static",
    "vehicle_rate_lhs": "Vehicle NN rate",
    "vehicle_rate_xl_lhs": "Vehicle NN rate",
    "pil": "PIL tire NN",
}


def _label(text: str) -> str:
    value = str(text)
    return DISPLAY_LABELS.get(value, value.replace("_", " "))


def _path_or_none(value: Any) -> Path | None:
    if value is None:
        return None
    try:
        if isinstance(value, float) and math.isnan(value):
            return None
    except TypeError:
        pass
    s = str(value)
    if not s or s == "nan":
        return None
    p = Path(s)
    return p if p.exists() else None


def _read_csv_if_exists(value: Any, **kwargs) -> pd.DataFrame | None:
    p = _path_or_none(value)
    if p is None:
        return None
    try:
        return pd.read_csv(p, **kwargs)
    except Exception:
        return None


def _manifest_args(out_dir: Path) -> dict[str, Any]:
    manifest = out_dir / "manifest.csv"
    if not manifest.exists():
        return {}
    try:
        rows = pd.read_csv(manifest)
    except Exception:
        return {}
    if not {"key", "value"}.issubset(rows.columns):
        return {}
    out: dict[str, Any] = {}
    for _, row in rows.iterrows():
        key = str(row["key"])
        value = row["value"]
        try:
            out[key] = ast.literal_eval(str(value))
        except Exception:
            out[key] = value
    return out


def _nominal_reference_xy(out_dir: Path, path_name: str) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        if str(SIM_DIR) not in sys.path:
            sys.path.insert(0, str(SIM_DIR))
        from reference_path import generate_path_waypoints
    except Exception:
        return None

    args = _manifest_args(out_dir)
    lead_in = float(args.get("lead_in", 5.0))
    sine_amplitude = float(args.get("sine_amplitude", 2.0))
    sine_wavelength = float(args.get("sine_wavelength", 30.0))
    try:
        x, y = generate_path_waypoints(
            path_name,
            lead_in=lead_in,
            sine_amplitude=sine_amplitude,
            sine_wavelength=sine_wavelength,
            ds=0.25,
        )
        return np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    except Exception:
        return None


def _variant_colors(variants: list[str]) -> dict[str, str]:
    return {v: PAPER_COLORS[i % len(PAPER_COLORS)] for i, v in enumerate(variants)}


def _numeric_column(df: pd.DataFrame, col: str) -> np.ndarray:
    if col not in df.columns:
        return np.asarray([], dtype=float)
    return pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)


_NN_MODEL_CACHE: dict[tuple[str, str], Any] = {}


def _terrain_internal(terrain: str) -> dict[str, float] | None:
    try:
        if str(SIM_DIR) not in sys.path:
            sys.path.insert(0, str(SIM_DIR))
        from param_consistency import get_terrain_preset, terrain_preset_to_internal
        return terrain_preset_to_internal(get_terrain_preset(str(terrain)))
    except Exception:
        return None


def _load_nn_model_for_row(row: pd.Series):
    model_id = str(row.get("nn_model", "") or "")
    terrain = str(row.get("terrain", "") or "")
    if not model_id or not terrain:
        return None, None
    key = (model_id, terrain)
    if key in _NN_MODEL_CACHE:
        return _NN_MODEL_CACHE[key], _terrain_internal(terrain)
    try:
        if str(SIM_DIR) not in sys.path:
            sys.path.insert(0, str(SIM_DIR))
        from nn_tire_model import load_nn_tire_model
        tp = _terrain_internal(terrain)
        if tp is None:
            return None, None
        model_dir = PROJECT_ROOT / "nn_models" / model_id
        if not model_dir.exists():
            return None, None
        model = load_nn_tire_model(model_dir, tp)
        _NN_MODEL_CACHE[key] = model
        return model, tp
    except Exception:
        return None, None


def _force_arrays_for_row(row: pd.Series, diag: pd.DataFrame, axle: str) -> tuple[np.ndarray, np.ndarray] | None:
    actual_col = f"actual_Fy_{axle}"
    pred_col = f"pred_Fy_{axle}"
    if actual_col not in diag.columns:
        return None
    actual = _numeric_column(diag, actual_col)

    # For NN rows, recompute predictions from logged operating-point features.
    # Older result folders may contain pred_Fy_* generated before loader fixes;
    # recomputing keeps figures tied to the current code/model contract.
    if str(row.get("mpc_model", "")).lower() == "nn":
        req = ["kappa_diag", "u_safe_diag", "Fz_f_mean", "Fz_r_mean", "alpha_f", "alpha_r"]
        if not set(req).issubset(diag.columns):
            return None
        model, tp = _load_nn_model_for_row(row)
        if model is None or tp is None:
            return None
        alpha = _numeric_column(diag, "alpha_f" if axle == "front" else "alpha_r")
        fz = _numeric_column(diag, "Fz_f_mean" if axle == "front" else "Fz_r_mean")
        u = _numeric_column(diag, "u_safe_diag")
        kappa = _numeric_column(diag, "kappa_diag")
        sr = _numeric_column(diag, "sr_diag") if axle == "front" and "sr_diag" in diag.columns else np.zeros(len(diag))
        pred = np.full(len(diag), np.nan, dtype=float)
        n = min(len(actual), len(alpha), len(fz), len(u), len(kappa), len(sr))
        for i in range(n):
            vals = (alpha[i], fz[i], u[i], kappa[i], sr[i])
            if not all(np.isfinite(v) for v in vals):
                continue
            try:
                _, fy = model.predict_numeric(
                    alpha[i], fz[i], u[i],
                    kappa=kappa[i],
                    n_terrain=tp["n"],
                    steering_rate=sr[i] if axle == "front" else 0.0,
                    terrain_params=tp,
                )
                pred[i] = -2.0 * fy
            except Exception:
                pred[i] = np.nan
        return actual, pred

    if pred_col not in diag.columns:
        return None
    return actual, _numeric_column(diag, pred_col)


def plot_metric_distribution_grid(
    results_csv: Path,
    out_dir: Path,
    specs: list[tuple[str, str, str]],
    filename: str,
    title: str,
) -> None:
    """Plot per-run points plus mean/std for selected metrics.

    ``specs`` entries are ``(column, ylabel, direction_hint)``.
    """
    plt = _import_plotting()
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    variants = list(dict.fromkeys(ok["variant"].astype(str)))
    colors = _variant_colors(variants)
    n = len(specs)
    ncols = min(3, n)
    nrows = int(math.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 3.6 * nrows), squeeze=False)
    x = np.arange(len(variants), dtype=float)

    rng = np.random.default_rng(7)
    for ax, (metric, ylabel, hint) in zip(axes.flat, specs):
        for i, variant in enumerate(variants):
            vals = pd.to_numeric(ok.loc[ok["variant"] == variant, metric], errors="coerce")
            vals = vals[np.isfinite(vals)]
            if vals.empty:
                continue
            jitter = rng.normal(0.0, 0.035, size=len(vals))
            ax.scatter(
                np.full(len(vals), x[i]) + jitter,
                vals,
                s=22,
                alpha=0.55,
                color=colors[variant],
                edgecolors="none",
            )
            mean = float(vals.mean())
            std = float(vals.std(ddof=1)) if len(vals) > 1 else 0.0
            ax.errorbar(
                [x[i]], [mean], yerr=[[std], [std]],
                fmt="o", color="black", ecolor="black", capsize=4,
                markersize=5, zorder=5,
            )
        ax.set_xticks(x)
        ax.set_xticklabels([_label(v) for v in variants], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(hint, fontsize=10)
        ax.grid(axis="y", alpha=0.25)

    for ax in axes.flat[n:]:
        ax.set_visible(False)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(fig_dir / filename, dpi=240)
    plt.close(fig)


def _trajectory_columns(diag: pd.DataFrame) -> tuple[str | None, str | None]:
    for x_col, y_col in [
        ("x_fa_true", "y_fa_true"),
        ("x_fa_meas", "y_fa_meas"),
        ("x", "y"),
    ]:
        if x_col in diag.columns and y_col in diag.columns:
            return x_col, y_col
    return None, None


def _time_column(diag: pd.DataFrame) -> str | None:
    for col in ("sim_time", "time", "wall_time"):
        if col in diag.columns:
            return col
    return None


def _thin_xy(x: np.ndarray, y: np.ndarray, max_points: int = 800) -> tuple[np.ndarray, np.ndarray]:
    good = np.isfinite(x) & np.isfinite(y)
    x = x[good]
    y = y[good]
    if len(x) > max_points:
        idx = np.linspace(0, len(x) - 1, max_points).astype(int)
        x = x[idx]
        y = y[idx]
    return x, y


def _unique_rocks(collision_csv: Any) -> pd.DataFrame:
    cdf = _read_csv_if_exists(collision_csv)
    if cdf is None or cdf.empty:
        return pd.DataFrame(columns=["rock_id", "rock_x", "rock_y", "rock_r"])
    cols = ["rock_id", "rock_x", "rock_y", "rock_r"]
    if not set(cols).issubset(cdf.columns):
        return pd.DataFrame(columns=cols)
    return cdf[cols].drop_duplicates("rock_id").sort_values("rock_id")


def plot_trajectory_overlays(
    results_csv: Path,
    out_dir: Path,
    *,
    filename_prefix: str = "trajectory_overlay",
    max_scenarios: int = 4,
    max_variants: int = 8,
) -> None:
    """Create trajectory-vs-reference overlays from existing diag CSVs."""
    plt = _import_plotting()
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty or "diag_csv" not in ok.columns:
        return
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    ok = ok[ok["diag_csv"].map(lambda p: _path_or_none(p) is not None)].copy()
    if ok.empty:
        return

    ok["scenario"] = (
        ok["terrain"].astype(str) + "/" + ok["path"].astype(str)
        + "/v" + ok["speed_mps"].astype(str) + "/b" + ok["bumpiness"].astype(str)
    )
    scenario_score = ok.groupby("scenario")["rms_cte_m"].max().sort_values(ascending=False)
    scenarios = list(scenario_score.index[:max_scenarios])
    variants = list(dict.fromkeys(ok["variant"].astype(str)))[:max_variants]
    colors = _variant_colors(variants)

    for scenario in scenarios:
        sub_all = ok[ok["scenario"] == scenario].copy()
        if sub_all.empty:
            continue
        fig, ax = plt.subplots(figsize=(8.6, 5.4))
        ref_plotted = False
        rock_plotted = False
        extent_x: list[float] = []
        extent_y: list[float] = []
        path_name = str(sub_all.iloc[0]["path"])
        nominal_ref = _nominal_reference_xy(out_dir, path_name)
        if nominal_ref is not None:
            xr, yr = nominal_ref
            ax.plot(xr, yr, "k--", lw=2.0, label="reference", alpha=0.8)
            ref_plotted = True
        for variant in variants:
            rows = sub_all[sub_all["variant"] == variant].sort_values(["seed", "rms_cte_m"])
            if rows.empty:
                continue
            row = rows.iloc[0]
            diag = _read_csv_if_exists(row["diag_csv"])
            if diag is None or diag.empty:
                continue
            x_col, y_col = _trajectory_columns(diag)
            if x_col is None or y_col is None:
                continue
            x, y = _thin_xy(_numeric_column(diag, x_col), _numeric_column(diag, y_col))
            if len(x) < 2:
                continue
            extent_x.extend([float(np.nanmin(x)), float(np.nanmax(x))])
            extent_y.extend([float(np.nanmin(y)), float(np.nanmax(y))])
            ax.plot(x, y, lw=1.8, color=colors[variant], label=_label(variant), alpha=0.9)

            if not ref_plotted and {"x_ref_0", "y_ref_0"}.issubset(diag.columns):
                xr, yr = _thin_xy(_numeric_column(diag, "x_ref_0"), _numeric_column(diag, "y_ref_0"))
                if len(xr) >= 2:
                    # Fallback only: x_ref_0 is a per-solve recovery/blended
                    # reference point, not the nominal path.  Prefer the
                    # reconstructed path above whenever possible.
                    ax.plot(xr, yr, "k--", lw=2.0, label="reference samples", alpha=0.8)
                    ref_plotted = True

            if not rock_plotted and "collision_csv" in row.index:
                rocks = _unique_rocks(row["collision_csv"])
                for _, r in rocks.iterrows():
                    extent_x.extend([
                        float(r["rock_x"]) - float(r["rock_r"]) - 1.5,
                        float(r["rock_x"]) + float(r["rock_r"]) + 1.5,
                    ])
                    extent_y.extend([
                        float(r["rock_y"]) - float(r["rock_r"]) - 1.5,
                        float(r["rock_y"]) + float(r["rock_r"]) + 1.5,
                    ])
                    circ = plt.Circle(
                        (float(r["rock_x"]), float(r["rock_y"])),
                        float(r["rock_r"]) + 1.5,
                        facecolor="none",
                        edgecolor="#444444",
                        lw=1.0,
                        alpha=0.65,
                    )
                    ax.add_patch(circ)
                if not rocks.empty:
                    ax.scatter(rocks["rock_x"], rocks["rock_y"], s=18, c="#444444", marker="x", label="rocks")
                    rock_plotted = True

        ax.set_aspect("equal", adjustable="box")
        if extent_x and extent_y:
            xmin, xmax = min(extent_x), max(extent_x)
            ymin, ymax = min(extent_y), max(extent_y)
            xpad = max(4.0, 0.08 * (xmax - xmin + 1e-6))
            ypad = max(2.0, 0.15 * (ymax - ymin + 1e-6))
            ax.set_xlim(xmin - xpad, xmax + xpad)
            ax.set_ylim(ymin - ypad, ymax + ypad)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(f"Trajectory vs reference: {scenario}")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, ncols=2)
        fig.tight_layout()
        fig.savefig(fig_dir / f"{filename_prefix}_{_slug(scenario)}.png", dpi=240)
        plt.close(fig)


def collect_force_prediction_metrics(results_csv: Path, out_dir: Path) -> pd.DataFrame:
    """Compute actual-vs-predicted lateral-force metrics from diag CSVs."""
    rows: list[dict[str, Any]] = []
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    for _, row in ok.iterrows():
        diag = _read_csv_if_exists(row.get("diag_csv"))
        if diag is None or diag.empty:
            continue
        for axle, actual_col, pred_col in [
            ("front", "actual_Fy_front", "pred_Fy_front"),
            ("rear", "actual_Fy_rear", "pred_Fy_rear"),
        ]:
            arrays = _force_arrays_for_row(row, diag, axle)
            if arrays is None:
                continue
            actual, pred = arrays
            good = np.isfinite(actual) & np.isfinite(pred)
            if np.count_nonzero(good) < 10:
                continue
            a = actual[good]
            p = pred[good]
            err = p - a
            denom = np.sum((a - np.mean(a)) ** 2)
            r2 = 1.0 - float(np.sum(err ** 2) / denom) if denom > 1e-9 else math.nan
            rows.append({
                "variant": row.get("variant", ""),
                "terrain": row.get("terrain", ""),
                "path": row.get("path", ""),
                "speed_mps": row.get("speed_mps", math.nan),
                "bumpiness": row.get("bumpiness", math.nan),
                "seed": row.get("seed", math.nan),
                "axle": axle,
                "n": int(len(a)),
                "mae_N": float(np.mean(np.abs(err))),
                "rmse_N": float(np.sqrt(np.mean(err ** 2))),
                "bias_N": float(np.mean(err)),
                "r2": r2,
                "actual_std_N": float(np.std(a)),
                "diag_csv": row.get("diag_csv", ""),
            })
    out = pd.DataFrame(rows)
    if not out.empty:
        out.to_csv(out_dir / "force_prediction_metrics.csv", index=False)
    return out


def plot_force_prediction_figures(
    results_csv: Path,
    out_dir: Path,
    *,
    max_points_per_variant_axle: int = 2500,
) -> None:
    """Create predicted-vs-actual lateral-force plots from diag CSVs."""
    plt = _import_plotting()
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    metrics = collect_force_prediction_metrics(results_csv, out_dir)
    if metrics.empty:
        return

    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    variants = list(dict.fromkeys(ok["variant"].astype(str)))
    colors = _variant_colors(variants)
    rng = np.random.default_rng(11)

    samples: list[pd.DataFrame] = []
    for _, row in ok.iterrows():
        diag = _read_csv_if_exists(row.get("diag_csv"))
        if diag is None or diag.empty:
            continue
        for axle in ["front", "rear"]:
            arrays = _force_arrays_for_row(row, diag, axle)
            if arrays is None:
                continue
            actual, pred = arrays
            sub = pd.DataFrame({
                "variant": str(row["variant"]),
                "axle": axle,
                "actual": actual,
                "pred": pred,
            }).dropna()
            if not sub.empty:
                samples.append(sub)
    if not samples:
        return
    force = pd.concat(samples, ignore_index=True)
    sampled = []
    for (variant, axle), sub in force.groupby(["variant", "axle"], sort=False):
        n = min(len(sub), max_points_per_variant_axle)
        idx = rng.choice(sub.index.to_numpy(), size=n, replace=False) if len(sub) > n else sub.index
        sampled.append(sub.loc[idx])
    force_s = pd.concat(sampled, ignore_index=True)

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2), sharex=True, sharey=True)
    for ax, axle in zip(axes, ["front", "rear"]):
        sub_ax = force_s[force_s["axle"] == axle]
        if sub_ax.empty:
            ax.set_visible(False)
            continue
        for variant, sub in sub_ax.groupby("variant", sort=False):
            ax.scatter(
                sub["actual"], sub["pred"],
                s=7, alpha=0.22, color=colors.get(variant, "#444444"),
                label=_label(variant),
            )
        lim_vals = pd.concat([sub_ax["actual"], sub_ax["pred"]]).to_numpy(dtype=float)
        lim_vals = lim_vals[np.isfinite(lim_vals)]
        if len(lim_vals):
            lo, hi = np.percentile(lim_vals, [1, 99])
            pad = 0.08 * max(1.0, hi - lo)
            lo -= pad
            hi += pad
            ax.plot([lo, hi], [lo, hi], "k--", lw=1.2, alpha=0.8)
            ax.set_xlim(lo, hi)
            ax.set_ylim(lo, hi)
        ax.set_title(f"{axle.capitalize()} axle Fy")
        ax.set_xlabel("Chrono actual Fy (N)")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Model predicted Fy (N)")
    axes[-1].legend(fontsize=8, loc="best")
    fig.suptitle("Predicted vs actual lateral tire force")
    fig.tight_layout()
    fig.savefig(fig_dir / "force_predicted_vs_actual_scatter.png", dpi=240)
    plt.close(fig)

    present_variants = [v for v in variants if v in set(force_s["variant"])]
    if present_variants:
        fig, axes = plt.subplots(
            len(present_variants), 2,
            figsize=(10.6, max(3.0, 2.6 * len(present_variants))),
            squeeze=False,
            sharex=True,
            sharey=True,
        )
        all_vals = pd.concat([force_s["actual"], force_s["pred"]]).to_numpy(dtype=float)
        all_vals = all_vals[np.isfinite(all_vals)]
        if len(all_vals):
            lo, hi = np.percentile(all_vals, [1, 99])
            pad = 0.08 * max(1.0, hi - lo)
            lo -= pad
            hi += pad
        else:
            lo, hi = -1.0, 1.0
        for r, variant in enumerate(present_variants):
            for c, axle in enumerate(["front", "rear"]):
                ax = axes[r, c]
                sub = force_s[(force_s["variant"] == variant) & (force_s["axle"] == axle)]
                if sub.empty:
                    ax.set_visible(False)
                    continue
                ax.scatter(
                    sub["actual"], sub["pred"],
                    s=5, alpha=0.18, color=colors.get(variant, "#444444"),
                    edgecolors="none",
                )
                ax.plot([lo, hi], [lo, hi], "k--", lw=0.9, alpha=0.7)
                ax.set_xlim(lo, hi)
                ax.set_ylim(lo, hi)
                mrow = metrics[(metrics["variant"].astype(str) == variant) & (metrics["axle"] == axle)]
                if not mrow.empty:
                    mae = float(mrow["mae_N"].mean())
                    r2 = float(mrow["r2"].mean())
                    ax.text(
                        0.03, 0.95, f"MAE {mae:.0f} N\nR² {r2:.2f}",
                        transform=ax.transAxes, va="top", ha="left", fontsize=8,
                        bbox=dict(facecolor="white", edgecolor="none", alpha=0.75, pad=2.5),
                    )
                if r == 0:
                    ax.set_title(f"{axle.capitalize()} axle")
                if c == 0:
                    ax.set_ylabel(f"{_label(variant)}\npredicted Fy (N)")
                if r == len(present_variants) - 1:
                    ax.set_xlabel("Chrono actual Fy (N)")
                ax.grid(alpha=0.22)
        fig.suptitle("Predicted vs actual Fy by model")
        fig.tight_layout()
        fig.savefig(fig_dir / "force_predicted_vs_actual_by_model.png", dpi=240)
        plt.close(fig)

    summary = metrics.groupby(["variant", "axle"], sort=False).agg(
        mae=("mae_N", "mean"),
        rmse=("rmse_N", "mean"),
        r2=("r2", "mean"),
    ).reset_index()
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.3))
    for ax, metric, ylabel in [
        (axes[0], "mae", "MAE (N)"),
        (axes[1], "rmse", "RMSE (N)"),
        (axes[2], "r2", "R²"),
    ]:
        labels = []
        values = []
        colors_bar = []
        for variant in variants:
            for axle in ["front", "rear"]:
                sub = summary[(summary["variant"] == variant) & (summary["axle"] == axle)]
                if sub.empty:
                    continue
                labels.append(f"{_label(variant)}\n{axle}")
                values.append(float(sub[metric].iloc[0]))
                colors_bar.append(colors.get(variant, "#444444"))
        ax.bar(np.arange(len(values)), values, color=colors_bar, alpha=0.85)
        ax.set_xticks(np.arange(len(values)))
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("Force prediction error by tire model")
    fig.tight_layout()
    fig.savefig(fig_dir / "force_prediction_error_summary.png", dpi=240)
    plt.close(fig)

    # Time-series examples for the highest-CTE scenario.  This gives the paper
    # an inspectable view of whether errors are bias-like or transient.
    ok2 = ok.copy()
    ok2["scenario"] = (
        ok2["terrain"].astype(str) + "/" + ok2["path"].astype(str)
        + "/v" + ok2["speed_mps"].astype(str) + "/b" + ok2["bumpiness"].astype(str)
    )
    scenario = ok2.groupby("scenario")["rms_cte_m"].max().sort_values(ascending=False).index[0]
    example_rows = []
    for variant in variants[:4]:
        rows = ok2[(ok2["variant"].astype(str) == variant) & (ok2["scenario"] == scenario)].sort_values("seed")
        if not rows.empty:
            example_rows.append(rows.iloc[0])
    if example_rows:
        fig, axes = plt.subplots(len(example_rows), 1, figsize=(10.5, 2.6 * len(example_rows)), sharex=True)
        if len(example_rows) == 1:
            axes = [axes]
        for ax, row in zip(axes, example_rows):
            diag = _read_csv_if_exists(row.get("diag_csv"))
            if diag is None or diag.empty:
                continue
            t_col = _time_column(diag)
            if t_col is None:
                t = np.arange(len(diag), dtype=float)
            else:
                t = _numeric_column(diag, t_col)
            for actual_col, pred_col, color, label_prefix in [
                ("actual_Fy_front", "pred_Fy_front", "#1f77b4", "front"),
                ("actual_Fy_rear", "pred_Fy_rear", "#ff7f0e", "rear"),
            ]:
                axle = "front" if label_prefix == "front" else "rear"
                arrays = _force_arrays_for_row(row, diag, axle)
                if arrays is None:
                    continue
                actual, pred = arrays
                good = np.isfinite(t) & np.isfinite(actual) & np.isfinite(pred)
                if np.count_nonzero(good) < 2:
                    continue
                idx = np.where(good)[0]
                if len(idx) > 700:
                    idx = idx[np.linspace(0, len(idx) - 1, 700).astype(int)]
                ax.plot(t[idx], actual[idx], color=color, lw=1.4, alpha=0.85, label=f"{label_prefix} actual")
                ax.plot(t[idx], pred[idx], color=color, lw=1.2, alpha=0.85, linestyle="--", label=f"{label_prefix} predicted")
            ax.set_ylabel("Fy (N)")
            ax.set_title(f"{_label(row['variant'])}: {scenario}")
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8, ncols=2)
        axes[-1].set_xlabel("time (s)")
        fig.suptitle("Predicted vs actual Fy over time")
        fig.tight_layout()
        fig.savefig(fig_dir / "force_prediction_timeseries_examples.png", dpi=240)
        plt.close(fig)
