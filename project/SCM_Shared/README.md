# SCM_Shared

Curated paper bundle extracted from `SCM_Teleop/`.

This folder is intended to be the shareable, self-contained project root for
the active paper scope:

- differentiable tire-model MPC benchmarks
- sliding-window terrain estimation (`n`, plus joint `n`/`phi` experiments)
- random-soil and closed-loop terrain-estimator validation
- dynamics-GP process-model experiments
- paper assets and figure-generation scripts

It intentionally omits the heavy historical clutter from the original repo,
especially old archives and `simulation/plots/`.

## Layout

```text
SCM_Shared/
├── simulation/      core Chrono + ACADOS runtime
├── utilities/       data collection, benchmarks, paper figure drivers
├── test_suite/      validation and regression entrypoints
├── my_paper/        abstract, images, references, generated figures
├── nn_models/       active paper checkpoints
├── data/            active paper datasets for estimator / residual experiments
├── paths/           reference path CSVs
├── verification/    shared-bundle verification helpers and notes
├── logs/            generated validation logs
├── plots/           generated benchmark plots / diag CSVs
└── archive/         optional components copied for completeness but not
                     required by the active paper path
```

## Included Scope

Included:

- active MPC runtime and Chrono launch path
- active `paper_v2_*` tire checkpoints
- active sliding-window terrain-estimator checkpoints
- terrain-estimator datasets used by the current paper scripts
- vehicle-force traces for force-residual work
- paper figure builders and the current paper figures

Not included from the source project:

- historical repo archives
- `simulation/plots/` from the working repo
- most generated runtime logs

## Environment

Use the `sim` conda environment.

```bash
conda activate sim
cd /home/kyle/Documents/chrono-HIL/chrono-HIL/project/SCM_Shared
```

## Core Commands

Closed-loop learned-estimator validation:

```bash
python test_suite/validate_closed_loop_estimator.py --terrains clay dirt sand
```

Random unseen-soil validation:

```bash
python utilities/generate_random_terrains.py --n-terrains 6
python test_suite/validate_random_terrains_closed_loop.py
```

Tire-model benchmark matrix:

```bash
python utilities/bench_tire_models.py --workers 4
```

Joint `(n, phi)` experiment:

```bash
python utilities/exp_joint_n_phi.py
```

## Verification

The verification workflow and the latest bundle-specific notes live in
`verification/`.
