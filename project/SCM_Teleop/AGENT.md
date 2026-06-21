# SCM_Teleop — agent onboarding

This file is the entry point for any AI agent (or human) picking up this
project. It's written so a cold reader can come in, run the right test
the first time, and know which files are load-bearing vs deprecated.

The paper draft is at [my_paper/abstract.tex](my_paper/abstract.tex). The
project topic is **safe shared / teleoperated control and autonomous
control of HMMWVs (military vehicles) on deformable terrain (clay, dirt,
sand)** using PyChrono's SCM (Soil Contact Model) deformable-terrain
simulator. Feel free to edit `abstract.tex` whenever the story moves.

---

## Rules (do not break)

1. Update [TRACKING.md](TRACKING.md) with the most up-to-date thoughts,
   edits, findings, bugs, results, and open questions. It is the
   running notebook for this project; AGENT.md is the static map.
2. **Never** claim anything works or is good until you have tested it
   with a real Chrono simulation run. Compile-only checks and standalone
   solver tests are necessary but not sufficient.
3. We want this code to work in real life eventually, so never rely on
   oracle data at inference. Example: during *training* of a terrain
   classifier you may extract forces from Chrono, but the classifier
   itself must run on what an IMU + driver inputs can give you.
4. Do not delete entire files. Move them to `archive/` instead.
5. No band-aids, magic numbers, or workarounds. Diagnose the root cause
   and fix canonically. Always ask: *if a paper reviewer saw this code,
   would they be surprised or offended?*
6. Don't stop until the code is paper-ready (figures + numbers
   reproducible from the repo).

---

## Environment

- **Conda env: `sim`** is the only correct env to run code in.
  Activate with:
  ```
  source /home/ksha/miniconda3/etc/profile.d/conda.sh && conda activate sim
  ```
- **`ACADOS_SOURCE_DIR` must be set** before running anything that
  imports `acados_template`. The path is `/home/ksha/Documents/sbel/acados`.
  `acados_mpc_solver.py` and `acados_mpcc_solver.py` both have a preamble
  that sets it from the env and pre-loads `libqpOASES_e.so` /
  `libacados.so` via `ctypes.CDLL(..., RTLD_GLOBAL)`. If you skip the
  env var the dynamic linker can't find the shared libs and the
  controller crashes during `AcadosOcpSolver(...)`. **Set it before
  every shell session** or you will waste minutes debugging.
- **PyChrono** lives at `/home/kyle/Documents/chrono_fork/chrono` — that
  is the source of truth for the HMMWV vehicle model and SCM terrain.
  We call it via SWIG from Python.

---

## Repository layout (current, post-2026-04-20 cleanup)

| Directory                  | What lives there                                                                 |
|----------------------------|----------------------------------------------------------------------------------|
| `simulation/`              | Runtime code: sim node, MPC controllers, ZMQ messaging, ref path, safety filter |
| `simulation/safety/`       | Predictive safety shields (MPPI / NMPC / legacy DOB-CBF)                         |
| `nn_models/`               | Trained tire surrogate checkpoints (see "NN models" below)                       |
| `nn_training/`             | Training scripts for tire-force MLPs                                             |
| `data/`                    | Datasets used to train NN surrogates and terrain estimators                      |
| `data_collection/`         | Parallel scenario collectors (e.g. `collect_closed_loop_data.py`)                |
| `utilities/`               | Data / benchmark helpers (not the canonical entrypoints)                         |
| `test_suite/`              | Validation entrypoints — the runnable benchmarks                                 |
| `new_diagnostics/`         | Per-run diagnostic scripts                                                       |
| `my_paper/`                | Paper LaTeX, figures, figure builders                                            |
| `my_paper/paper_figures/`  | Final-quality figures and the datasets that back them                            |
| `logs/`                    | Shared logs (collision_log.csv, shield logs)                                     |
| `archive/`                 | **Deprecated** code. Never delete from here; never delete *into* here without a date stamp |

---

## NN tire surrogates — which one to use

Multiple generations exist; only one is current.

| Model dir                              | Status        | Trained on                                                |
|----------------------------------------|---------------|-----------------------------------------------------------|
| `paper_v1_*`                           | **deprecated**| Early datasets, do not use                                |
| `paper_v2_static_*`                    | legacy (rig)  | `data/normal_mlp_resnet/scm_static_100k_v4.csv` — *rig*-style probes |
| `paper_v2_rate_*`                      | legacy (rig)  | `data/rate_mlp_resnet/rate_v1_100k.csv` — adds steering rate |
| **`closed_loop_v1_mlp_32_16`**         | **CURRENT**   | `data/closed_loop_v1/...` — 100k+ samples from closed-loop MPC runs across terrains/paths/speeds |

The closed-loop NN is what every MPC / MPCC / safety-shield run should
load by default (`--nn-model closed_loop_v1_mlp_32_16`). It was retrained
in 2026-05 after the rig-trained `paper_v2` series turned out to mispredict
peak Fy in closed-loop driving (rig vs closed-loop distribution gap).
On the closed-loop test split it scores R²_Fy ≈ 0.83.

**Sign convention gotcha**: the rig dataset stores Fy in tire frame; the
closed-loop dataset stores Fy in body frame. The training script flips
sign during ingest so the network output is in tire frame either way.
If you retrain on a new closed-loop dataset, replicate that sign flip
or the controller will steer the wrong direction.

---

## Controller stack

There are now **two** acados controllers wired through the same
launcher. They share the ZMQ message contract (`VehicleState` in,
`ControlCommand` out) but solve different OCPs:

### 1. Standard MPC (`acados_mpc_controller_node.py` + `acados_mpc_solver.py`)

The original reference-tracking MPC.
- Tracks `(x_ref(t), y_ref(t), ψ_ref(t), v_ref(t))` produced by
  `reference_path.py`.
- `v_ref` comes from a *curvature-derived speed profile* built at
  startup. This is the lever-arm we identified as suboptimal — it
  ignores the friction ellipse, assumes constant accel/brake authority,
  and is pre-computed open-loop.
- Heavy feature surface: DOB residual adapter, online terrain estimator
  hooks, dynamics-GP residual, predictive safety shield wrapper, tire
  CSV logger, etc. ~1900 lines.
- This is still the *baseline* for paper comparisons.

### 2. MPCC (`acados_mpcc_controller_node.py` + `acados_mpcc_solver.py`)  *— new, 2026-05*

Model Predictive Contouring Control. Augments the state with
path-progress `θ` and makes `vθ = θ̇` a control input. The optimizer
chooses its own speed, subject to a soft curvature-derived cap.

- State (NX=9): `[x, y, ψ, u, v, ω, ax, δ, θ]`
- Control (NU=3): `[δ̇, jx, vθ]`
- Params per stage (NP=11): `(x_p, y_p, sin_ψ_p, cos_ψ_p,
  Kphi, Kc, n_terrain, c, φ, k_janosi, v_max_stage)`
- Cost = contour² · w_c + lag² · w_l − w_prog · vθ + control reg
  + soft speed cap penalty
- **Friction ellipse constraint is intentionally disabled** — see comment
  at `acados_mpcc_solver.py:385`. The closed-loop NN under-predicts
  peak Fy, so the hard ellipse was chronically infeasible. The soft
  curvature cap + lag cost give us a workable approximation. Re-enable
  via `friction_ellipse=True` once a peak-Fy probing strategy exists.
- Defaults (after 2026-05-13 tuning sweep):
  `w_contour=3000, w_lag=2000, w_progress=0.5, w_speed_cap=300,
   w_delta_dot=80, vtheta_max=5.0, delta_dot_max=1.0 rad/s,
   N=20, dt=0.1`.
  The `w_delta_dot=80, δ̇_max=1.0` defaults match the standard MPC's
  steering smoothness; earlier defaults `(5, 1.5)` let the
  optimizer chatter at 83 Hz between solves even though the
  solver's per-stage δ̇ was within its bound.
- The controller is deliberately minimal (~300 lines) so it can be
  audited and head-to-head'd against the standard MPC without dragging
  in every feature flag. **It does NOT have**: dynamics-GP, online
  terrain estimator, residual adapter, rate-NN variant, safety shield.
  Those are deferred until the MPCC baseline is published.

### Selecting the controller

`launch_decoupled.py` has `--controller-mode {standard, mpcc}` (default
`standard`). MPCC mode skips all of the standard-MPC-only flags (DOB,
kappa, terrain estimator, etc.) — the launcher gates them on
`use_mpcc`. If you add a new standard-MPC flag, add it under the
`if not use_mpcc:` guard or MPCC will crash on unknown arg.

---

## How to run a test

### MPCC end-to-end smoke test (the command you most likely want)

```bash
source /home/ksha/miniconda3/etc/profile.d/conda.sh && conda activate sim
export ACADOS_SOURCE_DIR=/home/ksha/Documents/sbel/acados
cd /home/ksha/Documents/sbel/chrono_hil/chrono-HIL/project/SCM_Teleop

python -u simulation/launch_decoupled.py \
    --controller-mode mpcc \
    --nn-model closed_loop_v1_mlp_32_16 \
    --terrain clay --path sinusoidal \
    --speed 5 --time 10 --lead-in 5 --rocks 0 \
    --no-vis --no-plot --no-noise \
    --sim-port 5965 --ctrl-port 5966 \
    --mpcc-diag-csv /tmp/mpcc_diag.csv
```

Expected: closed-loop run, ~10 s sim in ~25 s wall (first build adds
~7 s for acados codegen), MPCC controller log shows `solve=~1 ms`,
diag CSV at `/tmp/mpcc_diag.csv` with ~830 rows. Drop `--no-vis` to
see the Irrlicht chase cam, and add `--live-plot
--live-plot-every 5` to open a matplotlib debug window (path,
predicted trajectory, body speed vs `v_max`, CTE, cost).

### Standard MPC equivalent (baseline)

```bash
python -u simulation/launch_decoupled.py \
    --controller-mode standard \
    --nn-model closed_loop_v1_mlp_32_16 \
    --model nn --terrain clay --path sinusoidal \
    --speed 5 --time 10 --lead-in 5 --rocks 0 \
    --no-vis --no-plot --no-noise \
    --sim-port 5965 --ctrl-port 5966
```

### Paper-grade sweeps — use the orchestrated suite

The canonical paper benchmarks now live under `paper_scripts/`. The old
`test_suite/benchmark_mpcc_vs_mpc.py` and
`test_suite/sweep_safety_shields.py` entrypoints were superseded and
moved to `archive/2026-05-16_superseded/`.

```bash
# One command runs every paper sweep + publishes figures:
python paper_scripts/run_paper_suite.py --tier pilot

# Or a single sweep:
python paper_scripts/run_paper_suite.py --tier pilot --only safety
python paper_scripts/run_paper_suite.py --tier pilot --only mpcc
```

`--tier smoke` runs a one-case-per-script health check (~15 min);
`--tier pilot` is the multi-seed matrix that backs the paper tables.
Results land in `paper_scripts/results/<sweep>_<ts>/` and the suite's
final step publishes merged figures to `my_paper/paper_figures/`. See
[paper_scripts/README.md](paper_scripts/README.md) and
[my_paper/PAPER_READINESS.md](my_paper/PAPER_READINESS.md).

---

## Decoupled sim ↔ controller architecture

`launch_decoupled.py` spawns **two** subprocesses:
1. `chrono_sim_node.py` — PyChrono HMMWV + SCM terrain.
   Publishes `VehicleState` over ZMQ pub/sub (default port 5555),
   subscribes to `ControlCommand` (default port 5556).
2. The chosen controller (`acados_mpc_controller_node.py` or
   `acados_mpcc_controller_node.py`).

Handshake: the sim waits for the controller's "ready ping"
(a neutral `ControlCommand`) before starting physics. Because ZMQ
pub/sub drops messages sent before the subscriber connects, **the
controller sends ready pings on a 0.3 s timer** until the first
`VehicleState` arrives. If you write a new controller node, replicate
this pattern (see `acados_mpcc_controller_node.py:149-170` for the
canonical form). A one-shot ready ping will silently deadlock.

**`ZMQSubscriber.recv()` returns `(topic, msg)` — a tuple, not the
message.** Unpack it. The MPCC node failed end-to-end the first time
because the handshake checked `isinstance(msg, VehicleState)` on the
tuple. This is exactly the kind of "would a reviewer be surprised"
gotcha that wastes a real day if not caught.

**Control loop runs at the state-message rate (~83 Hz) but the MPC
solver dt is 0.1 s (10 Hz).** Never apply `z_pred[1, IDELTA]`
directly — over `ctrl_dt ≈ 0.012 s` the solver's allowed per-stage
δ change (`δ̇_max · solver_dt ≈ 0.15 rad`) is ~8× the physical budget
per control step (`δ̇_max · ctrl_dt ≈ 0.018 rad`), and the optimizer
will happily flip ±MAX_STEER between consecutive 12 ms solves. The
correct pattern is: pull `u0[0]` (δ̇), integrate it over `ctrl_dt`,
then clip to `delta_applied ± δ̇_max · ctrl_dt` against the
previously-applied δ. Same for `ax`/`jx`. See
[acados_mpc_controller_node.py:1136](simulation/acados_mpc_controller_node.py#L1136)
"Post-MPC rate limiter" for the standard MPC version, and
`acados_mpcc_controller_node.py` for the MPCC version.

---

## Data & plots

- Closed-loop NN training data: `data/closed_loop_v1/`
  (~100k+ samples, generated by `data_collection/collect_closed_loop_data.py`).
- Per-run sim plots and diag CSVs: `simulation/plots/<run_tag>/`
- MPCC vs MPC benchmark output: `simulation/plots/mpcc_vs_mpc/<ts>/`
- Shield sweep output: `simulation/plots/shield_sweep/<ts>/`
- Paper-quality figures: `my_paper/paper_figures/` (commit both the
  figure and the dataset that produced it).

---

## Active paper scope (what to keep, what is parked)

Active:
- Sliding-window MLP terrain estimator (`n` only, and joint `n` / `phi`)
- Force-residual adapter on top of the NN surrogate
- Dynamics GP residual
- Closed-loop NN tire surrogate (current default)
- Predictive safety shields (MPPI primary, NMPC ablation, DOB-CBF legacy)
- **MPCC controller (new, 2026-05)** — head-to-head against standard MPC

Parked / archived:
- UKF / hybrid estimators
- Observer experiments
- GP force-residual code (separate from dynamics GP)
- `paper_v1` NN series
- Rig-trained `paper_v2` NN series (now superseded by closed-loop NN)

---

## Open knobs / known issues

- **MPCC speed cap is soft**, not the friction ellipse. The vehicle
  can briefly exceed the curvature-derived `v_max` if contour cost
  dominates the over-speed penalty. Increase `--mpcc-w-progress` lower
  or `w_speed_cap` in the solver if this becomes a problem on a new
  path.
- **MPCC currently uses the NN surrogate static-MLP only.** Rate-MLP
  variants and the GRU exist for the standard MPC but were not ported
  to keep the new controller small. If accuracy at high steering rates
  matters for a paper figure, port the rate variant.
- **Acados version warning** (`CasADi >= 3.7 required, got 3.6.7`) is
  cosmetic — the SQP_RTI / EXTERNAL cost path we use works on 3.6.7.
- **Closed-loop chrono test runs at RT 1.0×**. Real-time pacing is the
  default and disabling it (`--no-rt` on the sim node) breaks the
  decoupled MPC because the controller assumes wall-clock dt.
- **`logs/collision_log.csv` is shared** across runs. Benchmarks that
  care about collisions copy it into the per-run folder immediately
  after each run. If you add a new benchmark, follow that pattern.

---

## Recent results (2026-05-12, MPCC vs standard MPC, clay/sinusoidal, speed=5, 10 s)

| Controller   | RMS CTE | Max CTE | Mean u   | Mean solve |
|--------------|---------|---------|----------|------------|
| Standard MPC | 0.321 m | 0.756 m | 3.35 m/s | 5.1 ms     |
| MPCC (tuned) | 0.145 m | 0.456 m | 4.09 m/s | 1.0 ms     |

The tuning that made MPCC win on both axes was `vtheta_max=5,
w_contour=3000`. Before tuning, MPCC ran fast (5.3 m/s) but tracked
poorly (1.4 m RMS) — characteristic of an under-constrained progress
reward. See TRACKING.md for the full sweep once it lands.

---

## Where to look first when something breaks

1. Did you `export ACADOS_SOURCE_DIR=/home/ksha/Documents/sbel/acados`? (90% of "controller dies silently" issues.)
2. Are you in the `sim` conda env? (`which python` should be
   `/home/ksha/miniconda3/envs/sim/bin/python`.)
3. Are the sim/ctrl ZMQ ports free? `lsof -iTCP:5965 -sTCP:LISTEN`.
   Stale processes from a killed run hold the ports.
4. Is the NN model dir present at `nn_models/<name>/` with `weights.npz`
   and `metadata.json`?
5. Check `simulation/plots/<run_tag>/run.log` for the actual
   controller stderr — `launch_decoupled.py` inherits stdout, so most
   errors land in the run log.
