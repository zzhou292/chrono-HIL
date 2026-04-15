#!/usr/bin/env python3
"""
Run paired baseline vs live residual-adaptive closed-loop simulations and summarize.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

UTIL_DIR = Path(__file__).parent
PROJECT_ROOT = UTIL_DIR.parent
SIM_DIR = PROJECT_ROOT / "simulation"


DEFAULT_SCENARIOS = [
    {"name": "clay_right_left", "terrain": "clay", "path": "right_left", "speed": 6.0},
    {"name": "clay_sinusoidal", "terrain": "clay", "path": "sinusoidal", "speed": 6.0},
    {"name": "dirt_right_left", "terrain": "dirt", "path": "right_left", "speed": 6.0},
]


def _find_run_dir(created_after: float, root: Path, terrain: str, path_type: str):
    if not root.is_dir():
        return None
    cands = sorted(
        [
            d
            for d in root.iterdir()
            if d.is_dir()
            and terrain in d.name
            and path_type in d.name
            and d.stat().st_mtime > created_after
        ],
        key=lambda d: d.stat().st_mtime,
        reverse=True,
    )
    return cands[0] if cands else None


def _parse_diag(csv_path: Path) -> dict:
    rows = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    if not rows:
        return {}

    def to_arr(key):
        out = []
        for rr in rows:
            v = rr.get(key, "")
            try:
                out.append(float(v))
            except Exception:
                out.append(np.nan)
        return np.asarray(out, dtype=float)

    cte = np.abs(to_arr("crosstrack_err"))
    hdg = to_arr("heading_err_deg")
    t = to_arr("sim_time")
    solve_ms = to_arr("solve_time_ms")
    status = [rr.get("solver_status", "") for rr in rows]

    mask = t >= 2.0
    if np.any(mask):
        cte_use = cte[mask]
        hdg_use = hdg[mask]
    else:
        cte_use = cte
        hdg_use = hdg

    success_count = sum(1 for s in status if s in ("0", "2"))
    out = {
        "n_rows": len(rows),
        "rms_cte_m": float(np.sqrt(np.nanmean(cte_use**2))),
        "mean_abs_cte_m": float(np.nanmean(cte_use)),
        "rms_heading_deg": float(np.sqrt(np.nanmean(hdg_use**2))),
        "success_pct": float(100.0 * success_count / max(1, len(status))),
        "mean_solve_ms": float(np.nanmean(solve_ms)),
    }

    if "residual_du_pred" in rows[0]:
        du = np.abs(to_arr("residual_du_pred"))
        dv = np.abs(to_arr("residual_dv_pred"))
        dw = np.abs(to_arr("residual_domega_pred"))
        up = to_arr("residual_updates")
        ul = to_arr("residual_last_loss")
        ut = to_arr("residual_last_update_ms")
        out.update(
            {
                "mean_abs_residual_du_pred": float(np.nanmean(du)),
                "mean_abs_residual_dv_pred": float(np.nanmean(dv)),
                "mean_abs_residual_domega_pred": float(np.nanmean(dw)),
                "final_residual_updates": int(np.nanmax(up)) if np.isfinite(up).any() else 0,
                "final_residual_loss": float(ul[~np.isnan(ul)][-1]) if np.any(~np.isnan(ul)) else float("nan"),
                "mean_residual_update_ms": float(np.nanmean(ut)),
            }
        )

    return out


def _run_case(args, scenario: dict, mode: str, sim_port: int, ctrl_port: int, out_root: Path) -> dict:
    tag = f"{scenario['name']}__{mode}"
    plot_dir = out_root / mode / scenario["name"]
    plot_dir.mkdir(parents=True, exist_ok=True)
    stdout_log = plot_dir / f"{tag}_launch_stdout.log"
    stderr_log = plot_dir / f"{tag}_launch_stderr.log"

    cmd = [
        sys.executable,
        str(SIM_DIR / "launch_decoupled.py"),
        "--path",
        scenario["path"],
        "--terrain",
        scenario["terrain"],
        "--time",
        str(args.time),
        "--speed",
        str(scenario["speed"]),
        "--lead-in",
        str(args.lead_in),
        "--model",
        "nn",
        "--nn-model",
        args.nn_model,
        "--sim-port",
        str(sim_port),
        "--ctrl-port",
        str(ctrl_port),
        "--plot-dir",
        str(plot_dir),
        "--no-vis",
        "--no-plot",
    ]

    if scenario["path"] == "sinusoidal":
        cmd += ["--sine-amplitude", str(args.sine_amplitude), "--sine-wavelength", str(args.sine_wavelength)]

    if mode == "residual":
        cmd += [
            "--residual-adapt",
            "--residual-checkpoint",
            str(Path(args.checkpoint).expanduser().resolve()),
            "--residual-correction-gain",
            str(args.residual_correction_gain),
            "--residual-online-lr",
            str(args.residual_online_lr),
            "--residual-online-epochs",
            str(args.residual_online_epochs),
            "--residual-online-batch-size",
            str(args.residual_online_batch_size),
            "--residual-update-interval",
            str(args.residual_update_interval),
            "--residual-buffer-size",
            str(args.residual_buffer_size),
            "--residual-warmup-samples",
            str(args.residual_warmup_samples),
            "--residual-clip-u",
            str(args.residual_clip_u),
            "--residual-clip-v",
            str(args.residual_clip_v),
            "--residual-clip-omega",
            str(args.residual_clip_omega),
        ]
        if args.residual_no_online:
            cmd.append("--residual-no-online")

    print(f"\n[{tag}] ports {sim_port}/{ctrl_port}")
    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(SIM_DIR),
            capture_output=True,
            text=True,
            timeout=float(args.timeout_s),
        )
    except subprocess.TimeoutExpired as ex:
        _out = ex.stdout or b""
        _err = ex.stderr or b""
        stdout_log.write_text(_out.decode("utf-8", errors="replace") if isinstance(_out, bytes) else _out)
        stderr_log.write_text(_err.decode("utf-8", errors="replace") if isinstance(_err, bytes) else _err)
        wall = time.time() - t0
        print(f"  TIMEOUT after {args.timeout_s:.0f}s")
        return {
            "scenario": scenario["name"],
            "terrain": scenario["terrain"],
            "path": scenario["path"],
            "mode": mode,
            "status": "timeout",
            "wall_s": round(wall, 1),
            "stdout_log": str(stdout_log),
            "stderr_log": str(stderr_log),
        }

    stdout_log.write_text(proc.stdout or "")
    stderr_log.write_text(proc.stderr or "")
    wall = time.time() - t0
    if proc.returncode != 0:
        err = (proc.stderr or "")[-500:]
        print(f"  FAILED exit={proc.returncode}")
        if err:
            print(f"  stderr tail: {err[:220]}")
        return {
            "scenario": scenario["name"],
            "terrain": scenario["terrain"],
            "path": scenario["path"],
            "mode": mode,
            "status": f"exit_{proc.returncode}",
            "wall_s": round(wall, 1),
            "stdout_log": str(stdout_log),
            "stderr_log": str(stderr_log),
        }

    run_dir = _find_run_dir(t0, plot_dir, scenario["terrain"], scenario["path"])
    if run_dir is None:
        print("  NO OUTPUT DIR")
        return {
            "scenario": scenario["name"],
            "terrain": scenario["terrain"],
            "path": scenario["path"],
            "mode": mode,
            "status": "no_output",
            "wall_s": round(wall, 1),
        }
    diags = sorted(run_dir.glob("diag_*.csv"))
    if not diags:
        print("  NO DIAG CSV")
        return {
            "scenario": scenario["name"],
            "terrain": scenario["terrain"],
            "path": scenario["path"],
            "mode": mode,
            "status": "no_csv",
            "wall_s": round(wall, 1),
            "run_dir": str(run_dir),
            "stdout_log": str(stdout_log),
            "stderr_log": str(stderr_log),
        }
    metrics = _parse_diag(diags[-1])
    if not metrics:
        print("  EMPTY DIAG CSV")
        return {
            "scenario": scenario["name"],
            "terrain": scenario["terrain"],
            "path": scenario["path"],
            "mode": mode,
            "status": "empty_csv",
            "wall_s": round(wall, 1),
            "run_dir": str(run_dir),
            "diag_csv": str(diags[-1]),
            "stdout_log": str(stdout_log),
            "stderr_log": str(stderr_log),
        }
    print(
        f"  OK rms_cte={metrics.get('rms_cte_m', float('nan')):.3f}m "
        f"solve={metrics.get('mean_solve_ms', float('nan')):.2f}ms "
        f"success={metrics.get('success_pct', float('nan')):.1f}% "
        f"rows={metrics.get('n_rows', 0)}"
    )
    out = {
        "scenario": scenario["name"],
        "terrain": scenario["terrain"],
        "path": scenario["path"],
        "mode": mode,
        "status": "ok",
        "wall_s": round(wall, 1),
        "run_dir": str(run_dir),
        "diag_csv": str(diags[-1]),
        "stdout_log": str(stdout_log),
        "stderr_log": str(stderr_log),
    }
    out.update(metrics)
    return out


def _write_outputs(results: list[dict], out_root: Path) -> tuple[Path, Path, Path]:
    out_root.mkdir(parents=True, exist_ok=True)
    json_path = out_root / "residual_live_benchmark_results.json"
    csv_path = out_root / "residual_live_benchmark_results.csv"
    md_path = out_root / "residual_live_benchmark_summary.md"
    json_path.write_text(json.dumps(results, indent=2))

    fields = sorted({k for r in results for k in r.keys()})
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            w.writerow(r)

    by_scenario = {}
    for r in results:
        by_scenario.setdefault(r["scenario"], {})[r["mode"]] = r
    lines = [
        "# Residual Live Benchmark Summary",
        "",
        "| Scenario | Base RMS CTE (m) | Residual RMS CTE (m) | Delta CTE % | Base Solve (ms) | Residual Solve (ms) | Delta Solve % | Base Success % | Residual Success % |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for sc in sorted(by_scenario.keys()):
        base = by_scenario[sc].get("baseline")
        res = by_scenario[sc].get("residual")
        if not base or not res or base.get("status") != "ok" or res.get("status") != "ok":
            lines.append(f"| {sc} | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |")
            continue
        b_cte = float(base.get("rms_cte_m", np.nan))
        r_cte = float(res.get("rms_cte_m", np.nan))
        b_ms = float(base.get("mean_solve_ms", np.nan))
        r_ms = float(res.get("mean_solve_ms", np.nan))
        b_ok = float(base.get("success_pct", np.nan))
        r_ok = float(res.get("success_pct", np.nan))
        d_cte = 100.0 * (r_cte - b_cte) / max(abs(b_cte), 1e-9)
        d_ms = 100.0 * (r_ms - b_ms) / max(abs(b_ms), 1e-9)
        lines.append(
            f"| {sc} | {b_cte:.3f} | {r_cte:.3f} | {d_cte:+.1f} | "
            f"{b_ms:.2f} | {r_ms:.2f} | {d_ms:+.1f} | {b_ok:.1f} | {r_ok:.1f} |"
        )
    md_path.write_text("\n".join(lines) + "\n")
    return json_path, csv_path, md_path


def main():
    p = argparse.ArgumentParser(description="Run baseline vs residual-adaptive live benchmark")
    p.add_argument("--checkpoint", required=True, help="Path to residual_model.pt")
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--lead-in", type=float, default=10.0)
    p.add_argument("--nn-model", default="paper_v1_mlp_16_4")
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--base-port", type=int, default=7600)
    p.add_argument("--out-root", default=str(PROJECT_ROOT / "simulation" / "plots" / "residual_live_benchmark"))
    p.add_argument("--residual-no-online", action="store_true")
    p.add_argument("--residual-correction-gain", type=float, default=1.0)
    p.add_argument("--residual-online-lr", type=float, default=2e-4)
    p.add_argument("--residual-online-epochs", type=int, default=8)
    p.add_argument("--residual-online-batch-size", type=int, default=256)
    p.add_argument("--residual-update-interval", type=int, default=5)
    p.add_argument("--residual-buffer-size", type=int, default=4096)
    p.add_argument("--residual-warmup-samples", type=int, default=128)
    p.add_argument("--residual-clip-u", type=float, default=0.25)
    p.add_argument("--residual-clip-v", type=float, default=0.25)
    p.add_argument("--residual-clip-omega", type=float, default=0.08)
    p.add_argument("--timeout-s", type=float, default=180.0,
                   help="Per-run timeout passed to launch_decoupled subprocess")
    args = p.parse_args()

    ckpt = Path(args.checkpoint).expanduser().resolve()
    if not ckpt.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt}")

    out_root = Path(args.out_root).expanduser().resolve()
    results = []
    port = int(args.base_port)
    for scenario in DEFAULT_SCENARIOS:
        for mode in ("baseline", "residual"):
            r = _run_case(args, scenario, mode, sim_port=port, ctrl_port=port + 1, out_root=out_root)
            results.append(r)
            port += 10

    ok_count = sum(1 for r in results if r.get("status") == "ok")
    print(f"\nCompleted {ok_count}/{len(results)} runs")
    j, c, m = _write_outputs(results, out_root)
    print(f"JSON: {j}")
    print(f"CSV:  {c}")
    print(f"MD:   {m}")


if __name__ == "__main__":
    main()
