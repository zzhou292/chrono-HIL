#!/usr/bin/env python3
"""Orchestrate repeatable non-human paper experiment sweeps.

This runner does not define new experiments.  It launches the one-question
scripts with explicit paper matrices so high-speed, multi-terrain, multi-path,
multi-bumpiness, multi-seed coverage is repeatable.
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_ROOT = SCRIPT_DIR / "results"


@dataclass(frozen=True)
class SuiteCommand:
    name: str
    argv: list[str]
    estimated_runs: int
    note: str


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--tier", choices=["smoke", "pilot", "paper", "stress"], default="pilot",
                   help="smoke: quick syntax checks; pilot: manageable high-speed matrix; "
                        "paper: broad final matrix; stress: high-speed/bumpy safety stress tests.")
    p.add_argument("--only", nargs="+", default=[],
                   help="Subset names: tire_models, mpcc, safety, dob_cbf_ablation, "
                        "autonomous_obstacle_tire, terrain_estimator, latency_profile, latency_compensation.")
    p.add_argument("--dry-run", action="store_true", help="Print commands without running them.")
    p.add_argument("--continue-on-error", action="store_true")
    p.add_argument("--base-port", type=int, default=20000,
                   help="First port block. Each experiment gets a separated block.")
    p.add_argument("--port-stride", type=int, default=3500,
                   help="Port spacing between experiment blocks. Default 3500 keeps "
                        "all 12 experiment blocks (base_port..base_port+11*stride) inside "
                        "the 65535 ceiling at base_port=20000.")
    p.add_argument("--latency-profile-json",
                   default=str(ROOT / "config" / "latency_profiles" /
                               "5g_nhits_youtube_ul_scm_youtube_ul_smoke.json"))
    p.add_argument("--train-5g", action="store_true",
                   help="Retrain/export the N-HiTS 5G traffic checkpoint before latency sweeps.")
    p.add_argument("--no-publish", action="store_true",
                   help="Skip the final publish_paper_figures step that copies canonical "
                        "figures into my_paper/paper_figures/.")
    return p.parse_args()


def tier_matrix(tier: str) -> dict[str, list[str] | int | float]:
    if tier == "smoke":
        return {
            "terrains": ["clay"], "paths": ["sinusoidal"], "speeds": ["5"],
            "bumps": ["0"], "seeds": 1, "time": 8.0,
        }
    if tier == "pilot":
        return {
            "terrains": ["clay", "sand"],
            "paths": ["sinusoidal", "lane_change"],
            "speeds": ["5", "7"],
            "bumps": ["0", "4"],
            "seeds": 2,
            "time": 12.0,
        }
    if tier == "stress":
        return {
            "terrains": ["clay", "dirt", "sand"],
            "paths": ["sinusoidal", "lane_change", "right_left"],
            "speeds": ["7", "9"],
            "bumps": ["4", "8"],
            "seeds": 5,
            "time": 15.0,
        }
    return {
        "terrains": ["clay", "dirt", "sand"],
        "paths": ["sinusoidal", "lane_change", "right_left"],
        "speeds": ["5", "7", "9"],
        "bumps": ["0", "4", "8"],
        "seeds": 5,
        "time": 15.0,
    }


def count(*groups: list[str] | int) -> int:
    total = 1
    for group in groups:
        total *= group if isinstance(group, int) else len(group)
    return total


def base_args(m: dict[str, list[str] | int | float]) -> list[str]:
    return [
        "--terrains", *m["terrains"],
        "--paths", *m["paths"],
        "--speeds", *m["speeds"],
        "--bumpiness", *m["bumps"],
        "--seeds", str(m["seeds"]),
        "--time", str(m["time"]),
    ]


def python_cmd(script: str, *args: str) -> list[str]:
    return [sys.executable, "-u", str(SCRIPT_DIR / script), *args]


def build_commands(args: argparse.Namespace) -> list[SuiteCommand]:
    m = tier_matrix(args.tier)
    terrain_count = len(m["terrains"])
    path_count = len(m["paths"])
    speed_count = len(m["speeds"])
    bump_count = len(m["bumps"])
    seeds = int(m["seeds"])
    common = base_args(m)
    commands: list[SuiteCommand] = []
    ports = {name: args.base_port + i * args.port_stride for i, name in enumerate([
        "tire_models", "mpcc", "safety", "dob_cbf_ablation",
        "autonomous_obstacle_tire", "terrain_estimator", "latency_compensation",
        "throttle_dob_ablation", "mppi_seed_ablation", "safety_planner_aware",
        "tire_model_with_estimator_ablation", "sigma_gate_ablation",
    ])}
    if max(ports.values()) + args.port_stride - 1 > 65535:
        raise SystemExit(
            f"Port plan exceeds 65535. Lower --base-port or --port-stride. Plan: {ports}"
        )

    if args.train_5g:
        commands.append(SuiteCommand(
            "train_5g",
            python_cmd("train_5g_nhits.py", "--skip-train", "--experiment-id", "scm_youtube_ul_smoke"),
            0,
            "Re-export trained N-HiTS 5G trace/profile; omit --skip-train manually for retraining.",
        ))

    if args.tier == "smoke":
        smoke_scripts = [
            ("tire_models", "mpc_tire_model_sweep.py"),
            ("mpcc", "mpcc_vs_mpc_speed_tracking.py"),
            ("safety", "safety_filter_sweep.py"),
            ("dob_cbf_ablation", "dob_cbf_nn_ablation.py"),
            ("autonomous_obstacle_tire", "autonomous_obstacle_tire_model_sweep.py"),
            ("terrain_estimator", "terrain_estimator_benchmark.py"),
            ("throttle_dob_ablation", "throttle_dob_ablation.py"),
            ("mppi_seed_ablation", "mppi_seed_ablation.py"),
        ]
        for i, (name, script) in enumerate(smoke_scripts):
            commands.append(SuiteCommand(name, python_cmd(script, "--quick", "--base-port", str(ports.get(name, args.base_port + i * 1000))), 1, "Quick smoke run."))
        commands.append(SuiteCommand(
            "latency_profile",
            python_cmd("latency_profile_figure.py", "--profile-json", args.latency_profile_json),
            0,
            "Latency profile raw data/figures.",
        ))
        commands.append(SuiteCommand(
            "latency_compensation",
            python_cmd("latency_compensation_sweep.py", "--quick", "--base-port", str(ports["latency_compensation"]),
                       "--latency-profile-json", args.latency_profile_json),
            2,
            "Quick latency profile closed-loop smoke run.",
        ))
        return filter_commands(commands, args.only)

    tire_models = ["pacejka", "tmeasy", "closed_loop_v2_rate_mlp", "closed_loop_v3_axle_rate_mlp"]
    commands.append(SuiteCommand(
        "tire_models",
        python_cmd("mpc_tire_model_sweep.py", "--models", *tire_models, *common,
                   "--base-port", str(ports["tire_models"])),
        count(tire_models, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Tracking/speed/runtime by tire model.",
    ))

    mpcc_variants = ["standard_mpc_soft_speed", "standard_mpc_overspeed_cap",
                     "mpcc_balanced_tracking", "mpcc_less_speed_cap"]
    commands.append(SuiteCommand(
        "mpcc",
        python_cmd("mpcc_vs_mpc_speed_tracking.py", "--variants", *mpcc_variants, *common,
                   "--base-port", str(ports["mpcc"])),
        count(mpcc_variants, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "MPCC versus retuned standard MPC at high speed.",
    ))

    safety_flavors = ["none", "dob_cbf", "mppi", "nmpc"]
    commands.append(SuiteCommand(
        "safety",
        python_cmd("safety_filter_sweep.py", "--flavors", *safety_flavors, *common,
                   "--base-port", str(ports["safety"])),
        count(safety_flavors, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Obstacle safety shield comparison (planner blind to obstacles).",
    ))

    # Planner-aware variant: lets the NMPC's in-horizon softplus barriers do
    # their share. Comparing this against the planner-blind safety sweep above
    # is the abstract's "two-layer obstacle-avoidance stack" evidence.
    commands.append(SuiteCommand(
        "safety_planner_aware",
        python_cmd("safety_filter_sweep.py", "--flavors", *safety_flavors,
                   "--blind-and-aware", "--output-suffix", "planner_aware",
                   *common, "--base-port", str(ports["safety_planner_aware"])),
        count(safety_flavors, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds) * 2,
        "NMPC in-horizon barrier ablation: same shields, planner-aware vs planner-blind.",
    ))

    dob_variants = ["no_filter", "dob_cbf_nn", "dob_cbf_no_nn"]
    commands.append(SuiteCommand(
        "dob_cbf_ablation",
        python_cmd("dob_cbf_nn_ablation.py", "--variants", *dob_variants, *common,
                   "--base-port", str(ports["dob_cbf_ablation"])),
        count(dob_variants, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "DOB-CBF NN usage ablation.",
    ))

    commands.append(SuiteCommand(
        "autonomous_obstacle_tire",
        python_cmd("autonomous_obstacle_tire_model_sweep.py", "--models", *tire_models,
                   "--safety-flavor", "mppi", "--mpc-blind-obstacles", *common,
                   "--base-port", str(ports["autonomous_obstacle_tire"])),
        count(tire_models, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Autonomous obstacle avoidance by MPC tire model under fixed MPPI shield.",
    ))

    terrain_speeds = ["5", "7"] if args.tier != "stress" else ["7", "9"]
    terrain_paths = ["sinusoidal"]
    terrain_cases = terrain_count + 6
    commands.append(SuiteCommand(
        "terrain_estimator",
        python_cmd("terrain_estimator_benchmark.py", "--distributions", "id", "ood",
                   "--terrains", *m["terrains"], "--paths", *terrain_paths,
                   "--speeds", *terrain_speeds, "--bumpiness", *m["bumps"],
                   "--seeds", str(seeds), "--ood-terrains", "6", "--time", "20",
                   "--metric-start", "8", "--base-port", str(ports["terrain_estimator"])),
        count(terrain_cases, terrain_paths, terrain_speeds, m["bumps"], seeds),
        "Terrain estimator under excited sinusoidal maneuvers.",
    ))

    commands.append(SuiteCommand(
        "latency_profile",
        python_cmd("latency_profile_figure.py", "--profile-json", args.latency_profile_json),
        0,
        "Latency profile raw samples and figures.",
    ))

    latency_filters = ["none", "dob_cbf", "mppi"]
    commands.append(SuiteCommand(
        "latency_compensation",
        python_cmd("latency_compensation_sweep.py", "--filters", *latency_filters,
                   "--latency-profile-json", args.latency_profile_json, *common,
                   "--base-port", str(ports["latency_compensation"])),
        count(latency_filters, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "5G-profile command/camera latency robustness.",
    ))

    # Throttle-DOB ablation: same standard MPC, NN tire model, no obstacles;
    # toggles --dob-ki/--dob-max to zero so we can measure how much of the
    # speed-tracking story the asymmetric DOB actually owns.
    dob_variants_ablation = ["dob_on", "dob_off"]
    commands.append(SuiteCommand(
        "throttle_dob_ablation",
        python_cmd("throttle_dob_ablation.py", "--variants", *dob_variants_ablation,
                   *common, "--base-port", str(ports["throttle_dob_ablation"])),
        count(dob_variants_ablation, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "Asymmetric throttle DOB on vs off.",
    ))

    # MPPI seed-trajectory ablation: same shield, planner-blind MPC, rocks on;
    # toggles --mppi-no-seeds so we can attribute collision-rate improvement to
    # the seed trajectories vs pure Gaussian sampling.
    mppi_seed_variants = ["mppi_with_seeds", "mppi_no_seeds"]
    commands.append(SuiteCommand(
        "mppi_seed_ablation",
        python_cmd("mppi_seed_ablation.py", "--variants", *mppi_seed_variants,
                   *common, "--base-port", str(ports["mppi_seed_ablation"])),
        count(mppi_seed_variants, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "MPPI seed-trajectory ablation.",
    ))

    # Tire model x live terrain estimator: tests the abstract's
    # "order-of-magnitude over Pacejka and TMeasy" claim, which the static
    # tire-model sweep cannot speak to.  Overrides ``common``'s --time and
    # passes --metric-start=8 so the KPI window starts after the estimator
    # has had time to settle (this is also why the figures differ from the
    # static sweep: same vehicle, different observation window).
    estimator_variants = ["pacejka_static", "tmeasy_static", "nn_v3_static",
                          "nn_v3_estimator"]
    commands.append(SuiteCommand(
        "tire_model_with_estimator_ablation",
        python_cmd("tire_model_with_estimator_ablation.py",
                   "--variants", *estimator_variants, *common,
                   "--time", "20.0", "--metric-start", "8.0",
                   "--base-port", str(ports["tire_model_with_estimator_ablation"])),
        count(estimator_variants, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "Tire model with live terrain estimator on vs static params.",
    ))

    # Sigma-gate ablation: tests the abstract's claim that estimator
    # disagreement on phi tightens the shield's friction-cone gate.  Now
    # that the live (n, phi, sigma_phi) channel is wired controller->shield,
    # this sweep is meaningful (it was a no-op before because the shield
    # always saw sigma_phi = 0).
    sigma_gate_variants = ["sigma_inflate", "sigma_tighten", "sigma_both",
                           "sigma_off", "no_live_terrain"]
    commands.append(SuiteCommand(
        "sigma_gate_ablation",
        python_cmd("sigma_gate_ablation.py", "--variants", *sigma_gate_variants,
                   *common, "--base-port", str(ports["sigma_gate_ablation"])),
        count(sigma_gate_variants, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "MPPI shield sigma-gate ablation (estimator -> phi_sigma -> shield).",
    ))

    return filter_commands(commands, args.only)


def filter_commands(commands: list[SuiteCommand], only: list[str]) -> list[SuiteCommand]:
    if not only:
        return commands
    wanted = set(only)
    unknown = wanted - {cmd.name for cmd in commands}
    if unknown:
        raise SystemExit(f"Unknown --only names: {', '.join(sorted(unknown))}")
    return [cmd for cmd in commands if cmd.name in wanted]


def write_suite_manifest(suite_dir: Path, args: argparse.Namespace, commands: list[SuiteCommand]) -> None:
    with (suite_dir / "suite_manifest.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "estimated_runs", "note", "command"])
        for cmd in commands:
            writer.writerow([cmd.name, cmd.estimated_runs, cmd.note, " ".join(cmd.argv)])
    with (suite_dir / "suite_args.txt").open("w") as f:
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v!r}\n")


def main() -> None:
    args = parse_args()
    os.environ.setdefault("ACADOS_SOURCE_DIR", "/home/ksha/Documents/sbel/acados")
    commands = build_commands(args)
    suite_dir = RESULTS_ROOT / f"paper_suite_{args.tier}_{datetime.now():%Y%m%d_%H%M%S}"
    suite_dir.mkdir(parents=True, exist_ok=False)
    write_suite_manifest(suite_dir, args, commands)

    total_runs = sum(cmd.estimated_runs for cmd in commands)
    print(f"Suite: {args.tier}  commands={len(commands)}  estimated Chrono runs={total_runs}")
    print(f"Manifest: {suite_dir / 'suite_manifest.csv'}")
    for cmd in commands:
        print(f"\n[{cmd.name}] estimated_runs={cmd.estimated_runs}")
        print(" ".join(cmd.argv))
        if args.dry_run:
            continue
        rc = subprocess.run(cmd.argv, cwd=str(ROOT), env=os.environ.copy()).returncode
        if rc != 0 and not args.continue_on_error:
            raise SystemExit(rc)

    if args.dry_run or args.no_publish:
        return
    publish_cmd = [sys.executable, "-u", str(SCRIPT_DIR / "publish_paper_figures.py"),
                   "--suite-dir", str(suite_dir)]
    print(f"\n[publish_paper_figures]")
    print(" ".join(publish_cmd))
    subprocess.run(publish_cmd, cwd=str(ROOT), env=os.environ.copy())


if __name__ == "__main__":
    main()
