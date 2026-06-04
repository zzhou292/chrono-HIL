# AGENTS.md — onboarding for any agent (or human) picking up SCM_Final

You are working in the consolidated `SCM_Final` snapshot of the
**Terrain-Aware, Latency-Robust Control** project. The paper draft at
`my_paper/paper.tex` is the single source of truth for what has been
built and what has been claimed; the corresponding compiled PDF is at
`my_paper/paper.pdf`.

Several previously-explored research threads were archived in the
2026-05-23 final cleanup and are not part of the current paper:

* The **PIL** (physics-informed online identification) terrain
  estimator and matching PIL tire model — archived in
  `archive/2026-05-23_final_cleanup/pil/`.
* The **MPCC** (Model Predictive Contouring Control) variant —
  archived in `archive/2026-05-23_final_cleanup/mpcc/`. Standard NMPC
  is now the only controller.
* The **UKF** (Dallas-style augmented-state) terrain estimator —
  archived in `archive/2026-05-23_final_cleanup/ukf/`. The paper no
  longer claims the learned regressor *replaces* a UKF; it stands on
  its own closed-loop accuracy.
* The **online residual** force/dynamics learning adapters —
  archived in `archive/2026-05-23_final_cleanup/residual_learning/`.

The project topic is **safe shared and autonomous control of HMMWVs
on deformable terrain (clay, dirt, sand)** using PyChrono's SCM (Soil
Contact Model) deformable-terrain simulator, evaluated under
human-in-the-loop teleoperation with command and camera latency.

---

## Rules (do not break)

1. **The paper is the contract.** Before changing any numerical
   claim, ablation framing, or figure source, check
   `my_paper/paper.tex`. If a code change would invalidate a claim,
   fix both at once.
2. **Test in a real Chrono run.** Compile-only or standalone-solver
   checks are necessary but not sufficient. Use
   `python benchmarking/run.py --tier smoke` to exercise every
   sub-script for ~15 minutes before declaring anything works.
3. **No oracle data at inference.** During *training* of a tire or
   terrain surrogate you may consume ground-truth Chrono signals
   (forces, true terrain params). The *deployed* estimator and
   controller must only see what an IMU plus wheel encoders plus
   commanded inputs can give you.
4. **Do not delete files.** Move them to `archive/<YYYY-MM-DD_label>/`
   with a README explaining what was moved and why. Anything that has
   been in the paper or in a sweep manifest must stay recoverable.
5. **No band-aids or magic numbers.** If a sweep is flaky, fix the
   root cause (e.g. the ZMQ bind retry that landed in
   `simulation/hil_messages.py`). If a paper claim does not match the
   data, change the claim, not the metric. The standing test for any
   patch is "would a reviewer be surprised or offended by this?"
6. **Keep the current story and the archive separated.** The paper
   and presentations emphasize the framework, terrain-aware NMPC,
   swappable safety filters, latency robustness, and the HIL
   protocol. Archived ablations remain reproducible, but do not
   reintroduce them into the paper or slides without intentional
   scope change.
7. **Parallelize new sweeps.** Every Chrono-driving sweep is
   embarrassingly parallel — each run is its own `launch_decoupled.py`
   subprocess with its own ZMQ port pair. A fresh sweep script must
   use a `ProcessPoolExecutor` and accept `--workers` (default 6 on
   the 24-core workstation). The reference pattern is in
   `data_collection/collect_closed_loop_data.py` and
   `benchmarking/rig_vs_vehicle_tire_sweep.py`. See **Parallelism**
   below.
8. **No biased data collection — uniform LHS over the full input
   space only.** Training datasets for tyre and terrain surrogates
   MUST be sampled with uniform Latin-Hypercube over the documented
   input box (slip ratio, slip angle, velocity, vertical load, all
   six Bekker–Mohr soil parameters). It is **never acceptable** to
   narrow the LHS box to favour a particular terrain class, vehicle
   scenario, or operating regime — that produces models that fail
   outside the favoured region (see the closed-loop
   `vehicle_rate_64_32_lhs` checkpoint, which had its slip-angle mass
   concentrated near zero by the bootstrap controller and consequently
   under-predicts Fy on firm soil at large slip). Paper118 (Dallas
   2021 IEEE TVT) Table I documents the canonical ranges and is the
   reference for both `data/tire_rig/rate_v2_100k.csv` and
   `data/tire_rig/rate_paper118_v2_15k.csv`. If you need broader
   coverage, *widen* the LHS box, never *target* a sub-region.

---

## Parallelism

Every Chrono closed-loop sweep is embarrassingly parallel and **must** be
parallelized for any matrix beyond a smoke. A serial 288-run sweep is
~60 min at 13 s/run; the same matrix at `--workers 6` is ~12 min. The
24-core box (`nproc = 24`) comfortably runs 6–8 PyChrono SCM sims at
once.

### Required pattern (copy from `rig_vs_vehicle_tire_sweep.py`)

```python
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

@dataclass(frozen=True)
class Task:           # pickle-friendly description of one run
    idx: int
    sim_port: int     # base + 2*idx, so workers never collide
    ctrl_port: int
    ...

def _run_one(task):   # module-level so the pool can pickle it
    return launch_and_collect(..., sim_port=task.sim_port,
                              ctrl_port=task.ctrl_port, ...)

# Cache prewarm: acados + CasADi share a codegen cache keyed on NN model
# + solver options. Running cold codegen in N workers simultaneously
# races on the generated files. Always run task 0 solo first.
first = _run_one(tasks[0])
remaining = tasks[1:]

with ProcessPoolExecutor(max_workers=workers) as ex:
    futs = {ex.submit(_run_one, t): t for t in remaining}
    for fut in as_completed(futs):
        res = fut.result()
        # ... write to results.csv incrementally
```

### Rules

1. **Stride ZMQ ports by `2 * idx`** so each worker gets a unique
   sim/ctrl port pair. The retry-with-backoff in
   `simulation/hil_messages.py::ZMQPublisher.bind` covers brief TIME_WAIT
   races, but unique-by-construction ports avoid those entirely.
2. **acados build dir is SHARED per `(model, fingerprint)` — never keyed
   on PID.** The solver caches generated C + the compiled
   `.so` in `<tmpdir>/acados_mpc_<model>_<fp_hash>/` (`<tmpdir>` honours
   `$TMPDIR`). The fingerprint covers solver config + NN weights but NOT
   terrain (terrain is a runtime parameter), so every run with the same
   model + options reuses one compiled solver. `acados_mpc_solver.py`
   validates the fingerprint and, on a cache miss, takes an exclusive
   `fcntl` lock on `<build_dir>.lock` while it codegens/compiles — so
   concurrent workers in a multi-model sweep can't clobber each other's
   `.so`; late arrivals block, then reuse. **Do NOT append `os.getpid()`
   to the build dir** (the old `ACADOS_UNIQUE_BUILD_DIR=1` behaviour):
   the controller is a *fresh subprocess per run*, so a PID-keyed dir
   forced a ~60 s cold recompile on *every* run (a no-op `none`-filter
   run took 89 s, ~60 s of it `cc1`), left the 24-core box ~70 % idle,
   and spawned thousands of throwaway dirs. `ACADOS_UNIQUE_BUILD_DIR` is
   now inert. A solo prewarm (run task 0 before the pool) still helps by
   compiling the common model once up front.
2b. **Pin numpy/BLAS to 1 thread per run.** `benchmarking/common.py::
   run_process` sets `OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=
   NUMEXPR_NUM_THREADS=1` in each run's env. The MPPI shield's
   vectorized rollout is many tiny matmuls; default 24-thread OpenBLAS
   × N workers oversubscribes the box ~6× and inflated MPPI runs from
   ~100 s to ~350 s (some hit the timeout). acados (BLASFEO) and
   Chrono's OpenMP are unaffected. The MPPI shield rate-model path in
   `surrogate_dynamics.py` is also vectorized (`_rate_mlp_batch`); the
   old per-sample CasADi loop ran the shield at 0.03× real-time.
2c. **Per-run log dir.** `run_process` sets `HIL_RUN_LOG_DIR=<run_dir>`;
   the collision logger (`chrono_sim_node`), CBF filter
   (`safety/__init__`), and MPPI/NMPC shields (`predictive_shield`) all
   honour it and write per-run instead of into a shared global `logs/`.
   The old global path raced on truncation across workers
   (`FileNotFoundError`) and cross-contaminated collision counts. Env
   unset → falls back to the global `logs/` (live/manual HIL runs).
3. **An empty log CSV is the expected shape of an uneventful run, not
   an error.** When a sim run has zero collisions, `collision_log.csv`
   is written header-only; when no safety-filter activations happen,
   the shield log is zero bytes. `pd.read_csv` raises
   `EmptyDataError` / `ParserError` on those files and a naïve parser
   takes the worker down with it. `benchmarking/common.py` parsers —
   `parse_collision_csv`, `parse_shield_csv`, `parse_sim_diag_csv`,
   `parse_diag_csv` — all catch `(EmptyDataError, ParserError)` and
   return the "no events" payload. When adding a new sweep, **always
   wrap CSV reads with the same guard**; treat the empty file as a
   real signal, not a fault.
4. **Write `results.csv` incrementally** as each future completes, so
   an interrupted sweep is still usable.
5. **Default `--workers 6`; `12` is safe with the caching/BLAS fixes.**
   6 is the conservative default. With the shared acados cache (rule 2),
   single-thread BLAS (2b), and per-run logs (2c), the box runs cleanly
   at `--workers 12` (load ≈ 23/24, ~13 GB free) and the full paper
   matrix drops from ~50 h to ~10 h. `benchmarking/run.py` now forwards
   `--workers N` and `--timeout S` to every Chrono sub-sweep
   (latency_profile, 0 runs, is skipped), e.g.
   `run.py --tier paper --workers 12 --timeout 400`. MPPI runs finish in
   ~60–110 s; keep `--timeout` ≥ 300.
6. **Don't pickle `args`/`Namespace` through the pool.** Use a frozen
   dataclass per task that captures *only* the per-run inputs. Anything
   shared (output dir, sim time) can also go on the task or be imported
   in the worker.
7. **Per-process resource hygiene.** Each worker spawns child
   `launch_decoupled.py` + `acados_mpc_controller_node.py` processes
   that exit cleanly when the parent exits. If a sweep is killed
   mid-flight, sweep any strays:
   `pkill -9 -f launch_decoupled.py; pkill -9 -f acados_mpc_controller_node.py`.

### When NOT to parallelize

* **HIL rounds with a wheel.** `human_delay_compensation_rounds.py`
  reads one G29 — keep it serial.
* **Smoke tier.** `benchmarking/run.py --tier smoke` is meant to be
  fast-to-fail, not fastest possible.

---

## Environment

* Conda env: **`sim`** is the only valid env.
  `source ~/miniconda3/etc/profile.d/conda.sh && conda activate sim`
* `ACADOS_SOURCE_DIR=~/Documents/sbel/acados` **must** be exported
  before any acados import; the controller's preamble pre-loads
  `libqpOASES_e.so` / `libacados.so` from this path via
  `ctypes.CDLL(..., RTLD_GLOBAL)`. Missing the env var produces a
  silent solver crash on first `AcadosOcpSolver(...)`.
* `which python` should be `~/miniconda3/envs/sim/bin/python`.

---

## Directory map

| Path | What lives there |
| --- | --- |
| `simulation/` | Runtime: sim node, NMPC controller, ZMQ messaging, reference paths, tire surrogates (loaders + CasADi export), terrain estimator (`learned_terrain_estimator.py` — runtime only; trainer lives in `nn_training/`) |
| `simulation/safety/` | Predictive safety shields (`predictive_shield.py` MPPI + NMPC, DOB-CBF in `safety/__init__.py`) AND the modular forward collision-warning module `collision_warning.py` (terrain-aware + latency-aware TTC warning; factory `make_collision_warning_system`) |
| `benchmarking/` | The single benchmarking folder. `run.py` is the orchestrator (flagged by `--tier` and `--only`); each sub-script tests one paper claim and writes a timestamped folder under `benchmarking/results/`. `collision_warning_test.py` is the standalone sweep that exercises the warning module under terrain × latency without a controller. |
| `nn_training/` | Canonical trainers: `train_static_v3.sh` (rig static), `train_rate_v2.sh` (rig rate), `train_vehicle_lhs.sh` (whole-vehicle variants), `train_terrain_window_mlp.py` (window terrain estimator, deployed), `train_terrain_window_lstm.py` (LSTM smoke variant, not deployed), and `train_vehicle_fy_surrogate.py` (whole-vehicle Fy surrogate for the Dallas UKF, §VI). `train_variant.py` is the shared tire-NN trainer |
| `data_collection/` | Chrono SCM tire-rig binaries (`collect_static_data.cpp`, `collect_rate_data.cpp`), closed-loop tire-surrogate collector (`collect_closed_loop_data.py`), the broad multi-axis terrain-estimator collector (`collect_broad_terrain.py`), and the Dallas-UKF SCM collectors (`run_dallas_scm.py` single scripted run, `collect_lhs_training_scms.py` parallel LHS sweep) |
| `utilities/` | Closed-loop trace collection (`collect_diverse_terrains.py`, `collect_rich_excitation.py`, etc.) and offline diagnostics |
| `nn_models/` | Active runtime checkpoints: `rig_rate_64_32`, `vehicle_rate_64_32_lhs`, `terrain_window_mlp`, plus the paper118-spec rig surrogate `rig_rate_paper118_v2_64_32` used by the Dallas-style UKF reproducer in `deliverables/ukf_paper_validation.py` |
| `data/` | Four categories: `tire_rig/`, `whole_vehicle/`, `terrain_estimator/` (window-MLP traces), and `dallas_scm/` (Dallas-UKF SCM logs: `lhs_train300/` training sweep, `lhs100/`+`lhs100_cl/` benchmarks, canonical `clay/sandy_loam/sand.npz`) |
| `config/` | `latency_profiles/` (5G latency profile JSONs) and `terrain_yamls/` (LHS-sampled terrain configs) |
| `docs/` | Active design docs: `FRAMEWORK_CONTRACTS.md`, `SURROGATE_RETRAIN_FINDINGS.md` |
| `experiment_results/` | Output sink for orchestrator logs (and any timestamped snapshot a re-run cares to keep) |
| `my_paper/` | The paper draft (`paper.tex` / `paper.pdf`) and `paper_figures/`. Figures are refreshed by `publish_paper_figures.py` |

---

## How to run a test

### Smoke (every sub-script, ~15 min)
```bash
python benchmarking/run.py --tier smoke
```

### One sweep at pilot tier
```bash
python benchmarking/run.py --tier pilot --only safety
python benchmarking/run.py --tier pilot --only dob_cbf_ablation
```

### Full pilot (~6 hr)
```bash
python benchmarking/run.py --tier pilot
```

### Full paper matrix (the contract for `paper.pdf`)
```bash
python benchmarking/run.py --tier paper
```

Every invocation ends by running `publish_paper_figures.py`, which
merges any new result folders against the per-prefix history (deduped
on the run key) and overwrites `my_paper/paper_figures/` with the
regenerated figures and CSVs.

### Human-in-the-loop rounds (paper §VI-A)
```bash
# Default G29 protocol (symmetric link, camera delay = command delay)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none mppi dob_cbf nmpc \
    --delays 0.0 0.15 0.30 \
    --rounds 3 --manual-mode g29

# Asymmetric 5G-style link (camera downlink is heavier than command
# uplink). The learned 5G profile measures ~1.6x.
python benchmarking/human_delay_compensation_rounds.py \
    --filters none mppi dob_cbf \
    --delays 0.0 0.15 0.30 \
    --camera-delay-scale 1.6 \
    --rounds 3 --manual-mode g29

# Pipeline smoke (no wheel, 8 s, no driver input — verifies wiring only)
python benchmarking/human_delay_compensation_rounds.py --quick --auto-start
```

Each round delays the operator command path (`--manual-input-delay`)
AND the driver POV camera (`--camera-input-delay = camera_delay_scale
* delay`), and propagates the command delay to the active safety
filter as `--teleop-delay` so its predictive horizon is delay-aware.
Use `--manual-mode wasd` instead of `g29` for keyboard smoke without
the wheel.

Aggregated metrics (one row per (filter, delay) cell):

* **safety**: `collisions_mean/std`, `near_misses_mean`,
  `min_clearance_m_mean/std`;
* **intrusiveness**: `intervention_rate_pct_mean/std`,
  `mean_abs_dsteer_mean`, `mean_abs_dthrottle_mean`;
* **tracking**: `rms_cte_m_mean/std`, `speed_ratio_mean/std`.

The 6-panel `human_delay_compensation_summary.png` covers safety
(collisions, clearance, intervention rate) and intrusiveness
(|Δsteer|, |Δthrottle|, RMS CTE) vs.\ delay, which is the
safety-vs-intrusiveness trade the paper argues about.

---

## NN models

| Directory | Status | What it is |
| --- | --- | --- |
| `rig_rate_64_32/` | **active** | Tire-rig rate surrogate retained for rig-vs-vehicle diagnostics |
| `vehicle_rate_64_32_lhs/` | **default** (`DEFAULT_NN_MODEL`) | Whole-vehicle LHS rate surrogate used by standard NMPC and safety-filter sweeps |
| `terrain_window_mlp/` | **active** (v7) | Online closed-loop terrain estimator, `n` output only. Trained on `traces_broad_v7/` (3600 scenarios spanning 180 LHS cells × scripted + closed-loop NMPC × 3 speeds × 3 paths × 2 bumpiness levels). 26-feature MLP (planar IMU + wheel slip + vertical-dynamics block: az std/p95, pitch-rate std/p95, roll-rate std). Best val RMSE 0.20, held-out LHS RMSE 0.098, median \|err\| 0.10 across 200 random Bekker-jittered evaluation terrains. |
| `vehicle_static_32_16_lhs/` | **active** (referenced by paper §III) | Static-feature vehicle tire surrogate; restored from the archive because the force-RMSE-vs-CTE analysis (`deliverables/cte_vehicle_static_vs_rate.py`) actively cites it. |
| `rig_rate_paper118_v2_64_32/` | **active** (paper §VI baseline) | Paper118-spec rate-augmented rig surrogate trained on a fresh 15 k-sample uniform LHS sweep (slip angle ±0.6 rad, per-wheel Fz 2.5–11 kN, full Bekker–Mohr box). Kept as the paper118 reproduction baseline; the deployed NN-UKF tyre backend is now `vehicle_fy_64_32` (see below). |
| `vehicle_fy_64_32/` | **active** (paper §VI deployed NN-UKF) | Whole-vehicle lateral-force surrogate that natively predicts $(F_{y,\mathrm{total}}, M_{\mathrm{yaw,total}})$ for the bicycle UKF, with no post-hoc calibration scalar. Trained on a 300-scenario disjoint uniform-LHS Chrono SCM sweep (`lhs_train300`, seed 7, `--widened-box`) with varied excitation (steer amp/period/speed + half open-loop / half PI-cruise throttle). **Critical fix:** the soil box is *widened* (Kphi from 0.3 MPa, cohesion from 300 Pa, janosi 0.008–0.028, n 0.35–1.35) so the canonical clay/dirt/sand presets are interior, not box-corner, points — the standard `TRAINING_RANGES_V6` box puts clay at a triple-edge corner and a standard-box surrogate fails clay badly. Held-out Fy R² = 0.90, M_yaw R² = 0.88. Benchmark results (100-LHS, two excitations): **OL** Buzhardt constant-throttle (paper headline, Fig. 8, `lhs100_fair.png`): NN-UKF median |Δn|/n = 12.9 % / 64 % within ±20 %, learned MLP 17.0 %, Bekker 34.3 %. **CL** PI-cruise 5 m/s (`lhs100_cl.png`): NN-UKF 9.4 % / 79 %, MLP 15.7 %, Bekker 20.9 %. Canonical spot-check (Fig. 10, `terrain_estimator_comparison.png`, amp 0.6 CL): NN-UKF clay 18.0 %, sandy loam 3.2 % (best of three on both soft soils), dry sand 10.8 %. See its `TRAINING_METADATA.md`. **Gotcha:** `run_dallas_scm.py --open-loop-throttle` must be ≥0 for a constant throttle; a negative value (e.g. −1) routes to PI cruise. Earlier code clipped −1 → 0 (no throttle) and produced degenerate stuck-vehicle logs — the bench passes −1 correctly but a *direct* CLI call did not until fixed. |

Each active checkpoint ships with a `TRAINING_METADATA.md` describing
its dataset path, generator script, hyperparameters, sign convention,
and known limitations.

Retired snapshots from the v7 terrain-estimator development cycle
(v6, v5 held-out variants, v7 held-out, v7 low-n-weighted, v7 LSTM
smoke variant, v7 named snapshot with isotonic calibration) were moved
to `archive/2026-05-25_nn_models_cleanup/`. Earlier static,
axle-rate, and joint-estimator checkpoints are in
`archive/2026-05-23_model_checkpoint_and_root_artifact_cleanup/`. The
PIL tire model is in `archive/2026-05-23_final_cleanup/pil/`.

The v7 terrain estimator pipeline (collector + trainer + eval scripts)
is captured by `data_collection/collect_broad_terrain.py`,
`nn_training/train_terrain_window_mlp.py`, and `deliverables/eval_v7_*.py`.
Backup of the deployed v7 weights + figures lives in
`experiment_results/v7_baseline_backup/`.

### Tire-rig vs.\ closed-loop training (paper §III-C)

Two generations of tire-NN training existed in the project history:

* *Rig generation*: open-loop Chrono SCM single-tire sweeps over
  $(\kappa, \alpha, F_z, \theta_\mathrm{soil})$. Collectors:
  `data_collection/collect_static_data.cpp`,
  `collect_rate_data.cpp`. Trainer: `nn_training/train_static_v3.sh`,
  `train_rate_v2.sh` (both call `train_variant.py`). The active
  retained baseline is `rig_rate_64_32`.
* *Closed-loop generation*: runs the actual MPC stack on randomised
  scenarios and logs operating points alongside SCM ground-truth
  force. Collector: `data_collection/collect_closed_loop_data.py`.
  Trainer: `nn_training/train_vehicle_lhs.sh`. The active retained
  whole-vehicle surrogate is `vehicle_rate_64_32_lhs`.

### Dallas-style UKF reproduction (paper §VI estimator comparison)

The Dallas-2021 state-augmented UKF is kept as an **offline**
estimator under `deliverables/ukf_paper_validation.py` and is *not*
wired into the runtime registries (the deployed runtime estimator is
the sliding-window MLP). The reproducer is the source of paper
Section VI's estimator-comparison figure:

* `data_collection/run_dallas_scm.py` — single-file PyChrono runner that
  drives an HMMWV through 50 s of sinusoidal steering on a chosen
  SCM preset (clay / dirt / sand) with scripted throttle, and logs
  $[t, x, y, \psi, u, v, \omega, a_x, a_y, a_z, \mathrm{roll},
  \mathrm{pitch}, \dots, w_{fl..rr}, \delta, \mathrm{throttle}]$ at
  24 ms. Writes NPZ to `data/dallas_scm/`.
* `deliverables/ukf_paper_validation.py::main_dallas_scm()` — replays
  the SCM log through a Dallas-style state-augmented UKF with both
  Bekker and NN tire backends and a 4-wheel double-track bicycle,
  using the body-frame $a_y$ as a direct measurement of
  $\Sigma F_y / m$. Writes
  `my_paper/paper_figures/ukf_dallas_validation_scm.png`.
* `deliverables/eval_terrain_estimators.py` — single-trace head-to-
  head between Bekker-UKF, NN-UKF (whole-vehicle Fy surrogate), and
  the deployed sliding-window MLP on the three canonical Chrono SCM
  presets (clay/sandy-loam/sand). Reads
  `data/dallas_scm/{clay,sandy_loam,sand}.npz` and writes
  `my_paper/paper_figures/terrain_estimator_comparison.png` + CSV
  (paper Fig. 10). Regenerate the three preset logs first with
  `run_dallas_scm.py --terrain {clay,dirt,sand} --steer-amp-rad 0.6
  --open-loop-throttle -1 --target-speed 5.0` (dirt → sandy_loam).
* `deliverables/bench_terrain_estimators_lhs.py` — **broad 100-LHS
  benchmark** driving the three estimators across 100 uniform-LHS
  Bekker–Mohr terrains. Parallelised SCM collection + parallelised
  estimator pass per CLAUDE.md §Parallelism. Flags: `--n-min/--n-max`
  (default 0.40/1.30, matches MLP range), `--steer-amp-rad`,
  `--open-loop-throttle` (≥0 = constant OL throttle; −1 = PI cruise
  to `--target-speed`), `--log-suffix` (separate SCM-log dir so CL and
  OL runs co-exist). Paper figures: OL headline →
  `lhs100_fair.png` (`--open-loop-throttle 0.75 --out-name lhs100_fair`);
  CL → `lhs100_cl.png` (`--open-loop-throttle -1 --target-speed 5.0
  --log-suffix _cl --out-name lhs100_cl`).
* `deliverables/plot_cl_vs_ol.py` — combines `lhs100_cl.csv` and
  `lhs100_fair.csv` into the CL-vs-OL robustness panel
  `lhs100_cl_vs_ol.png` (paper Fig. 9). Re-run after both benches.
* `data_collection/collect_lhs_training_scms.py` — generates 300 SCM
  scenarios with uniform LHS over the six Bekker–Mohr soil
  parameters and three excitation axes (steer amplitude, steer
  period, throttle: half constant open-loop, half PI cruise). Pass
  `--widened-box` (recommended) to sample soils from the enlarged box
  so canonical presets are interior. Logs land in
  `data/dallas_scm/lhs_train300/`, disjoint (seed 7) from the
  seed-42 benchmark set.
* `nn_training/train_vehicle_fy_surrogate.py` — trains the
  `vehicle_fy_64_32/` whole-vehicle Fy surrogate on the
  `lhs_train300/` logs (`--hidden 128 64 --epochs 400 --decim 2
  --test-frac 0.10`), 90/10 split by scenario. Aggregates ≈ 190 k
  labelled rows and stores `(weights.pt, scaler.pkl, config.json)`.

Patches applied while reviving the reproducer:
* `project/SCM_Teleop/data_collection/collect_rate_data.cpp` — fix-up
  for the newer Chrono `ChTireTestRig` API (the removed
  `GetSlipAngle / GetLongSpeed / GetAngSpeed / GetLongitudinalSlip`
  methods are reconstructed from the local `slip_func / v_func /
  omega_func` objects) plus a widened slip-angle range (±0.6 rad,
  matching paper118 Table I) and per-wheel Fz range ([2.5, 11] kN to
  cover Chrono HMMWV's outer-wheel excursions). Build with
  `cmake --build build --target collect_rate_data` after a fresh
  `cmake .` to pick up the new target.

---

## Controller stack

Only one acados controller is shipped now:
`acados_mpc_controller_node.py` + `acados_mpc_solver.py` is the
standard reference-tracking NMPC and is the planner for every paper
sweep. (MPCC was archived in 2026-05-23; see top of this doc.)

`launch_decoupled.py` launches the sim + controller pair over ZeroMQ
pub/sub, with the controller subscribing to `VehicleState` and
publishing `ControlCommand`.

---

## Safety shield architecture

* `simulation/safety/predictive_shield.py` contains the predictive MPPI
  shield and the SLSQP NMPC comparison filter. Both consume the same
  `surrogate_dynamics.py` rollout that the planner uses. The paper
  frames the safety layer as swappable: DOB-CBF is the
  minimum-deviation, intent-preserving filter for HIL commands, while
  MPPI is the predictive learned-dynamics shield.
* The MPPI sample set is augmented with six **hand-crafted seed
  trajectories** (passthrough, full brake, coast, evade-left,
  evade-right, brake-while-turn) injected unconditionally. The ablation
  in paper §VI-C shows removing the seeds multiplies the collision
  rate by ~40×.
* The shield receives terrain updates from the controller over the
  same ZMQ socket as `ControlCommand` (terrain fields are piggy-backed
  on `ControlCommand` because `ZMQ_CONFLATE` drops any separate
  message).
* `simulation/safety/collision_warning.py` is the modular forward
  collision-warning system. It runs in parallel with the safety filter
  rather than replacing it — its job is to emit a discrete severity
  signal to the remote driver (HMI), not to override commands. Same
  swappable-factory pattern as the safety filters
  (`make_collision_warning_system(flavor='ttc')`). The default flavor
  is TTC + terrain-conditioned brake deceleration + latency-conditioned
  operator reaction time.
* **Brake-deceleration table.** Built *analytically at init time* by
  querying the deployed rig surrogate (`rig_rate_64_32`) over braking
  slip ratios κ∈[−0.4, 0] at each `n̂ ∈ [0.40, 1.30]` on a 0.05 grid;
  the table runs from ~2.3 m/s² on soft clay to ~4.7 m/s² on firm
  sand. The live `n̂` from the terrain estimator indexes the table at
  runtime. The previous hand-tuned 2.5/6.0 m/s² fallback is still
  available but the analytical table is the default (and the one
  validated in paper §VIII).
* **Validators.** Two scripts ship with the module:
  - `benchmarking/collision_warning_test.py` — forward sweep over
    terrain × latency at fixed throttle into a single rock; verifies
    that lead time grows monotonically with softer soil and longer
    delay.
  - `benchmarking/brake_test.py` — 27 actual Chrono SCM brake stops
    (3 terrains × 3 initial speeds × 3 seeds) used to validate the
    analytical `a_b(n̂)` table. Mean stopping-distance error 0.17 m
    (2.6× better than the hand-tuned fallback).

---

## ZMQ + handshake gotchas

* `ZMQSubscriber.recv()` returns `(topic, msg)` — a tuple, not the
  message. Unpack it.
* The controller sends "ready ping" `ControlCommand`s on a 0.3 s timer
  until the first `VehicleState` arrives. A one-shot ready ping
  silently deadlocks because pub/sub drops messages sent before the
  subscriber connects.
* `ZMQPublisher.bind` retries 16x with 0.5 s backoff
  (`simulation/hil_messages.py`). Without this, rapid sweep cycling
  loses ~1% of runs to `TIME_WAIT` `Address already in use` races.
* The control loop runs at the state-message rate (~83 Hz) but the
  MPC solver dt is 0.1 s (10 Hz). Never apply `z_pred[1, IDELTA]`
  directly — the optimizer flips $\pm$max-steer between consecutive
  12 ms solves. Pull `u0[0]` ($\dot\delta$), integrate over
  `ctrl_dt`, then clip to $\delta_\mathrm{applied} \pm
  \dot\delta_\mathrm{max}\,\mathrm{ctrl\_dt}$.

---

## Adding a new sweep

1. Write the sub-script in `benchmarking/` modeled on
   `mppi_seed_ablation.py` (or one of the other simple ablations).
   The script must define a `plot_figures(results_csv, out_dir)` if
   `publish_paper_figures.py` is going to re-plot on merge.
2. Add the sweep to `benchmarking/run.py`'s port allocation list and
   to the relevant tier in `build_commands(...)`.
3. Add a `PublishSpec` to `publish_paper_figures.py` so the sweep's
   figures and CSVs land in `my_paper/paper_figures/`.
4. Smoke it:
   `python benchmarking/run.py --tier smoke --only <name>`.
5. Pilot it:
   `python benchmarking/run.py --tier pilot --only <name>`.
6. Update `paper.tex` to reference the new figures/tables.

---

## Where to look first when something breaks

1. Did you `export ACADOS_SOURCE_DIR=~/Documents/sbel/acados`? (90 %
   of "controller dies silently" issues.)
2. Are you in the `sim` conda env? `which python` should be in
   `~/miniconda3/envs/sim/`.
3. Are the sim/ctrl ZMQ ports free?
   `lsof -iTCP:5965 -sTCP:LISTEN`. Stale processes from a killed run
   hold the ports.
4. Is the NN model dir present at `nn_models/<name>/`? Only three
   models are kept; deprecated names will not load.
5. Check `simulation/plots/<run_tag>/run.log` (when present) or the
   sweep's `raw/<idx>_*/run.log` for the actual controller stderr —
   `launch_decoupled.py` inherits stdout into those files.
