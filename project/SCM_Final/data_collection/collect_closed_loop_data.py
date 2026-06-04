#!/usr/bin/env python3
"""
collect_closed_loop_data.py
============================

Run many randomised whole-vehicle closed-loop chrono simulations in parallel
and aggregate the per-tick tire-force training rows into one CSV that is
directly consumable by ``nn_training/train_variant.py`` (static or temporal
mode).

Why
---
The existing ``paper_v2_*`` surrogates were trained on a tire test rig
(constant slip, measure steady-state Fy).  In a *closed-loop* HMMWV
driving on SCM terrain the slip changes at tens of Hz and sinkage /
suspension transients dominate the instantaneous Fy.  When
``new_diagnostics/validate_surrogate_vs_chrono.py`` is run against a
clay closed-loop diag CSV, the static surrogate's per-wheel Fy
correlation with chrono ground truth is ~0.1 — barely better than
random.  This script collects the data needed to retrain a surrogate
that *does* match closed-loop physics.

Pipeline
--------
The pipeline launches N independent ``launch_decoupled.py`` runs in
parallel batches.  Each run:

* picks a terrain (clay / sand / dirt) and can optionally randomise its
  Bekker/Mohr parameters using preset jitter or Latin-hypercube samples;
* picks a path (sinusoidal / lane_change / double_lane_change) with
  randomised geometry (sinusoidal amplitude/wavelength);
* picks a speed in ``[--speed-min, --speed-max]``;
* picks a bumpiness level;
* optionally adds rocks for richer slip-angle variation;
* runs the chrono sim + acados MPC with ``--log-tire-csv`` writing
  front- and rear-axle rows per controller tick, tagged with unique
  ``scenario_id`` streams.

After all batches finish the per-scenario CSVs are concatenated into
``training_data.csv`` (body-frame Chrono labels) and
``training_data_tire_frame.csv`` (Fy sign flipped for the NN tire
surrogate convention).  Train on the tire-frame CSV unless you also
change the MPC/safety sign bridge.

Sample sizing
-------------
At the default ``--time 10`` and ~100 Hz controller rate, each
scenario yields ~2000 rows because front and rear axle streams are
logged separately.  ``--scenarios 100`` therefore gives ~200k rows;
``--scenarios 1000`` gives ~2M rows.  With 8 parallel workers
(default) a 1k-scenario run finishes in roughly 1k * 15 s / 8 ≈
30 minutes on a single workstation.

Usage
-----
::

    conda activate sim
    export ACADOS_SOURCE_DIR=/path/to/acados
    python data_collection/collect_closed_loop_data.py \\
        --scenarios 120 --workers 6 --time 10 --output data/whole_vehicle/lhs

The result is ``data/whole_vehicle/lhs/training_data.csv``,
``data/whole_vehicle/lhs/training_data_tire_frame.csv``, and per-run logs
in ``data/whole_vehicle/lhs/per_run/``.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / 'simulation'
LAUNCH = SIM / 'launch_decoupled.py'
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

from param_consistency import (  # noqa: E402
    TRAINING_RANGES_V6,
    generate_lhs_terrain_yaml_dicts,
    get_terrain_preset,
)


PATHS = ['sinusoidal', 'lane_change', 'double_lane_change', 'right_left']
TERRAINS = ['clay', 'sand', 'dirt']


@dataclass
class Scenario:
    scenario_id: int
    terrain: str
    path: str
    speed: float
    sine_amplitude: float
    sine_wavelength: float
    rocks: int
    rock_seed: int
    rock_zone_x: tuple
    rock_zone_y: tuple
    bumpiness: int
    lead_in: float
    sim_time: float
    sim_port: int
    ctrl_port: int
    terrain_config: dict | None = None


def make_scenarios(args, rng: random.Random) -> list[Scenario]:
    scenarios = []
    paths = args.paths or PATHS
    terrains = args.terrains or TERRAINS
    bumpiness_levels = args.bumpiness_levels or [0]
    lhs_configs = (
        generate_lhs_terrain_yaml_dicts(args.scenarios, seed=args.seed)
        if args.terrain_randomization == 'lhs'
        else None
    )
    for i in range(args.scenarios):
        terrain = rng.choice(terrains)
        path = rng.choice(paths)
        speed = round(rng.uniform(args.speed_min, args.speed_max), 2)
        sine_amp = round(rng.uniform(1.0, 3.0), 2)
        sine_wave = round(rng.uniform(20.0, 40.0), 1)
        rocks = rng.choice(args.rock_choices)
        rock_seed = rng.randint(0, 10_000)
        bumpiness = int(rng.choice(bumpiness_levels))
        terrain_config = None
        if args.terrain_randomization == 'jitter':
            terrain_config = jitter_terrain_config(
                terrain,
                rng,
                frac=float(args.terrain_jitter_frac),
                scenario_id=args.first_scenario_id + i,
            )
        elif args.terrain_randomization == 'lhs':
            terrain_config = dict(lhs_configs[i])
            terrain_config['description'] = (
                f"LHS closed-loop terrain scn {args.first_scenario_id + i}"
            )
        # rock zone matches the lateral envelope of the path
        if path == 'sinusoidal':
            zone_y = (-sine_amp - 1, sine_amp + 1)
        else:
            zone_y = (-1.0, 4.0)
        zone_x = (12.0, 50.0)
        scenarios.append(Scenario(
            scenario_id=args.first_scenario_id + i,
            terrain=terrain,
            path=path,
            speed=speed,
            sine_amplitude=sine_amp,
            sine_wavelength=sine_wave,
            rocks=rocks,
            rock_seed=rock_seed,
            rock_zone_x=zone_x,
            rock_zone_y=zone_y,
            bumpiness=bumpiness,
            lead_in=args.lead_in,
            sim_time=args.time,
            # Ports: 9000 + 4*i so concurrent runs don't collide
            sim_port=args.base_port + 4 * i,
            ctrl_port=args.base_port + 4 * i + 1,
            terrain_config=terrain_config,
        ))
    return scenarios


def _clamp_range(key: str, value: float) -> float:
    lo, hi = TRAINING_RANGES_V6[key]
    if key == 'mohr_friction':
        lo = np.degrees(lo)
        hi = np.degrees(hi)
    return float(np.clip(value, lo, hi))


def jitter_terrain_config(name: str, rng: random.Random, frac: float, scenario_id: int) -> dict:
    """Return a small random perturbation around a named soil preset."""
    preset = get_terrain_preset(name)
    cfg = {
        'description': f"{name} jittered closed-loop scn {scenario_id}",
        'Kphi': _clamp_range('bekker_Kphi', preset['Kphi'] * rng.uniform(1 - frac, 1 + frac)),
        'Kc': _clamp_range('bekker_Kc', preset['Kc'] * rng.uniform(1 - frac, 1 + frac)),
        'n': _clamp_range('bekker_n', preset['n'] + rng.uniform(-frac, frac) * max(preset['n'], 0.2)),
        'cohesion': _clamp_range('mohr_cohesion', preset['cohesion'] * rng.uniform(1 - frac, 1 + frac)),
        'friction_angle': _clamp_range(
            'mohr_friction',
            preset['friction_angle'] + rng.uniform(-frac, frac) * max(preset['friction_angle'], 1.0),
        ),
        'janosi_shear': _clamp_range('janosi_shear', preset['janosi_shear'] * rng.uniform(1 - frac, 1 + frac)),
        'elastic_stiffness': float(preset.get('elastic_stiffness', 2e8)),
        'damping': float(preset.get('damping', 3e4)),
    }
    return cfg


def run_scenario(scn: Scenario, args, out_root: Path) -> dict:
    """Run one scenario; return {ok, log_csv, error}."""
    run_dir = out_root / 'per_run' / f"scn_{scn.scenario_id:05d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    tire_csv = run_dir / 'tire.csv'
    rich_tire_csv = run_dir / 'tire_rich.csv'
    sim_log = run_dir / 'sim.log'
    terrain_config_path = None
    if scn.terrain_config is not None:
        terrain_config_path = run_dir / 'terrain.yaml'
        terrain_config_path.write_text(json.dumps(scn.terrain_config, indent=2))

    cmd = [
        sys.executable, '-u', str(LAUNCH),
        '--model', args.mpc_model,
    ]
    # Pass --nn-model only when the MPC needs a learned tire surrogate.
    if args.mpc_model == 'nn':
        cmd.extend(['--nn-model', args.nn_model])
    cmd.extend([
        '--terrain', scn.terrain,
        '--path', scn.path,
        '--speed', str(scn.speed),
        '--time', str(scn.sim_time),
        '--lead-in', str(scn.lead_in),
        '--sine-amplitude', str(scn.sine_amplitude),
        '--sine-wavelength', str(scn.sine_wavelength),
        '--rocks', str(scn.rocks),
        '--rock-seed', str(scn.rock_seed),
        '--rock-zone-x', str(scn.rock_zone_x[0]), str(scn.rock_zone_x[1]),
        '--rock-zone-y', str(scn.rock_zone_y[0]), str(scn.rock_zone_y[1]),
        '--bumpiness', str(scn.bumpiness),
        '--no-vis', '--no-plot', '--no-csv',
        '--sim-port', str(scn.sim_port),
        '--ctrl-port', str(scn.ctrl_port),
        '--log-tire-csv', str(tire_csv),
        '--log-rich-tire-csv', str(rich_tire_csv),
        '--log-scenario-id', str(scn.scenario_id),
    ])
    if args.no_sensor_noise:
        cmd.append('--no-noise')
    if terrain_config_path is not None:
        cmd.extend(['--terrain-config', str(terrain_config_path)])

    t0 = time.time()
    try:
        with open(sim_log, 'w') as f:
            env = os.environ.copy()
            if args.unique_acados_build_dir:
                env['ACADOS_UNIQUE_BUILD_DIR'] = '1'
            proc = subprocess.run(
                cmd, stdout=f, stderr=subprocess.STDOUT,
                timeout=args.timeout, env=env,
            )
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        rc = -1
    wall_s = time.time() - t0

    rows = 0
    rich_rows = 0
    if tire_csv.exists():
        try:
            with open(tire_csv) as f:
                rows = sum(1 for _ in f) - 1  # minus header
        except Exception:
            rows = 0
    if rich_tire_csv.exists():
        try:
            with open(rich_tire_csv) as f:
                rich_rows = sum(1 for _ in f) - 1
        except Exception:
            rich_rows = 0

    return {
        'scenario_id': scn.scenario_id,
        'ok': rc == 0,
        'rc': rc,
        'wall_s': wall_s,
        'rows': rows,
        'rich_rows': rich_rows,
        'tire_csv': str(tire_csv),
        'rich_tire_csv': str(rich_tire_csv),
        'terrain_config_path': str(terrain_config_path) if terrain_config_path else '',
        'config': asdict(scn),
    }


def _worker(scn: Scenario, args, out_root: Path) -> dict:
    return run_scenario(scn, args, out_root)


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('--scenarios', type=int, default=120,
                   help='Number of distinct sims to run.')
    p.add_argument('--workers', type=int, default=6,
                   help='Parallel worker processes.  Each spawns one '
                        'launch_decoupled.py = ~2 CPU cores + ~3 GB RAM.')
    p.add_argument('--time', type=float, default=10.0,
                   help='Sim time per scenario (s).')
    p.add_argument('--lead-in', type=float, default=5.0)
    p.add_argument('--speed-min', type=float, default=3.0)
    p.add_argument('--speed-max', type=float, default=7.0)
    p.add_argument('--paths', nargs='+', choices=PATHS, default=None)
    p.add_argument('--terrains', nargs='+', choices=TERRAINS, default=None)
    p.add_argument('--bumpiness-levels', type=int, nargs='+', default=[0],
                   help='Bumpiness levels sampled per scenario.')
    p.add_argument('--rock-choices', type=int, nargs='+', default=[0, 0, 3, 5],
                   help='Rock counts sampled per scenario.')
    p.add_argument('--terrain-randomization',
                   choices=['presets', 'jitter', 'lhs'], default='presets',
                   help='Soil-parameter distribution: named presets, jittered '
                        'presets, or Latin-hypercube v6 terrain samples.')
    p.add_argument('--terrain-jitter-frac', type=float, default=0.10,
                   help='Relative perturbation for --terrain-randomization jitter.')
    p.add_argument('--no-sensor-noise', action='store_true',
                   help='Disable sensor noise during collection. Noise is ON by default.')
    p.add_argument('--mpc-model', default='nn',
                   choices=['nn', 'pacejka', 'pacejka-oracle', 'tmeasy'],
                   help='Which tire model the MPC uses while driving the '
                        'collection. Switching this changes the operating-'
                        'point distribution that gets logged (the Chrono '
                        'ground-truth Fy logged in --log-tire-csv is the '
                        'same physics regardless of MPC choice). Use '
                        '"pacejka" to bootstrap an alternative training '
                        'distribution that breaks the NN-self-loop bias '
                        'discussed in §III.')
    p.add_argument('--nn-model', default='vehicle_rate_64_32_lhs',
                   help='Bootstrap surrogate for the controller during '
                        'data collection.  Only used when --mpc-model nn. '
                        'Shapes the distribution of slip/throttle commands '
                        'we observe, not the chrono ground-truth Fy we log.')
    p.add_argument('--first-scenario-id', type=int, default=1000,
                   help='Starting scenario_id; bumped per scenario.')
    p.add_argument('--base-port', type=int, default=19000)
    p.add_argument('--timeout', type=float, default=180.0,
                   help='Per-scenario wall-clock timeout (s).')
    p.add_argument('--no-prebuild-cache', dest='prebuild_cache',
                   action='store_false',
                   help='Skip the serial first scenario used to avoid cold acados codegen races.')
    p.set_defaults(prebuild_cache=True)
    p.add_argument('--shared-acados-build-dir', dest='unique_acados_build_dir',
                   action='store_false',
                   help='Allow controller processes to share acados cache dirs. '
                        'Faster when cache is warm, but unsafe for parallel cold builds.')
    p.set_defaults(unique_acados_build_dir=True)
    p.add_argument('--output', type=Path, default=None,
                   help='Output dir (default: data/closed_loop_<ts>).')
    p.add_argument('--seed', type=int, default=42)
    args = p.parse_args()

    if not os.environ.get('ACADOS_SOURCE_DIR'):
        print('WARNING: ACADOS_SOURCE_DIR is not set in the environment',
              file=sys.stderr)

    out_root = args.output or (REPO / 'data' /
                                f'closed_loop_{datetime.now():%Y%m%d_%H%M%S}')
    out_root.mkdir(parents=True, exist_ok=True)
    print(f'Output dir: {out_root}', flush=True)

    rng = random.Random(args.seed)
    scenarios = make_scenarios(args, rng)

    print(f'Spawning {len(scenarios)} scenarios across {args.workers} workers, '
          f'~{args.time:g} s sim each', flush=True)

    t_total = time.time()
    completed = []
    remaining = scenarios
    done_count = 0
    if args.prebuild_cache and args.workers > 1 and scenarios:
        # A cold acados build writes into a shared cache directory keyed by the
        # NN model and solver options. If several workers hit codegen at once,
        # CasADi/acados can race on generated files. Running one full scenario
        # first warms the cache and still contributes data.
        print('Prebuilding/warming solver cache with the first scenario ...', flush=True)
        r = run_scenario(scenarios[0], args, out_root)
        completed.append(r)
        done_count = 1
        remaining = scenarios[1:]
        ok = '✓' if r['ok'] else '✗'
        print(f'  [{done_count:4d}/{len(scenarios)}] {ok} scn={r["scenario_id"]:5d}  '
              f'rows={r["rows"]:>5d} rich={r.get("rich_rows", 0):>5d}  wall={r["wall_s"]:5.1f}s',
              flush=True)

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_worker, scn, args, out_root): scn for scn in remaining}
        for fut in as_completed(futs):
            done_count += 1
            try:
                r = fut.result()
            except Exception as e:
                r = {'scenario_id': futs[fut].scenario_id, 'ok': False,
                     'rc': -1, 'wall_s': 0.0, 'rows': 0, 'error': str(e)}
            completed.append(r)
            ok = '✓' if r['ok'] else '✗'
            print(f'  [{done_count:4d}/{len(scenarios)}] {ok} scn={r["scenario_id"]:5d}  '
                  f'rows={r["rows"]:>5d} rich={r.get("rich_rows", 0):>5d}  wall={r["wall_s"]:5.1f}s',
                  flush=True)

    # ---- Aggregate ------------------------------------------------------
    print('\nAggregating per-scenario CSVs ...', flush=True)
    frames = []
    rich_frames = []
    for r in completed:
        if not r['ok'] or r['rows'] == 0:
            continue
        try:
            df = pd.read_csv(r['tire_csv'])
            frames.append(df)
        except Exception as e:
            print(f'  skip scn={r["scenario_id"]}: {e}')
        if r.get('rich_rows', 0) > 0:
            try:
                rich_frames.append(pd.read_csv(r['rich_tire_csv']))
            except Exception as e:
                print(f'  skip rich scn={r["scenario_id"]}: {e}')
    if not frames:
        print('No data collected — check sim.log files in per_run/.', file=sys.stderr)
        sys.exit(1)
    big = pd.concat(frames, ignore_index=True)
    out_csv = out_root / 'training_data.csv'
    big.to_csv(out_csv, index=False)
    tire_frame = big.copy()
    tire_frame['Fy'] = -tire_frame['Fy']
    tire_frame_csv = out_root / 'training_data_tire_frame.csv'
    tire_frame.to_csv(tire_frame_csv, index=False)
    rich_csv = None
    rich_tire_frame_csv = None
    if rich_frames:
        rich = pd.concat(rich_frames, ignore_index=True)
        rich_csv = out_root / 'training_data_rich.csv'
        rich.to_csv(rich_csv, index=False)
        rich_tire = rich.copy()
        rich_tire['Fy'] = -rich_tire['Fy']
        rich_tire_frame_csv = out_root / 'training_data_rich_tire_frame.csv'
        rich_tire.to_csv(rich_tire_frame_csv, index=False)

    # Summary
    print('\nDone.')
    print(f'  Total rows:     {len(big):,}')
    print(f'  Total scenarios:{len([r for r in completed if r["ok"]])}')
    print(f'  Wall clock:     {time.time() - t_total:.0f} s')
    print(f'  Body-frame CSV: {out_csv}')
    print(f'  Tire-frame CSV: {tire_frame_csv}')
    if rich_csv is not None:
        print(f'  Rich body CSV:  {rich_csv}')
        print(f'  Rich tire CSV:  {rich_tire_frame_csv}')
    print(f'  Per-run logs:   {out_root / "per_run"}')

    # Also save a manifest with per-scenario configs for reproducibility
    pd.DataFrame([r['config'] for r in completed if r['ok']]).to_csv(
        out_root / 'manifest.csv', index=False)


if __name__ == '__main__':
    main()
