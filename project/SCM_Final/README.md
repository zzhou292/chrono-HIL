# SCM_Final — terrain-aware off-road autonomy and teleoperation

A consolidated checkpoint of the framework, benchmarks, datasets, and
paper draft for *Terrain-Aware, Latency-Robust Control for Off-Road
Autonomy and Teleoperation*. Every figure and table in
`my_paper/paper.pdf` is regenerable from a single command against the
code and data shipped in this directory.

## Layout

| Path | Contents |
| --- | --- |
| `simulation/` | Runtime: Chrono sim node, acados NMPC controller, ZMQ messaging, learned terrain estimator, safety shields, tire surrogates |
| `simulation/framework/` | The six-swap-point contract: `interfaces.py` declares one `Protocol` per role (CommandSource, SafetyFilter, CollisionWarning, TireModel, TerrainEstimator, LatencyProfile); `registry.py` keeps a per-role `Registry` with a `@register("flavor")` decorator; `builtins.py` wires the shipped concrete classes into those registries; `test_conformance.py` instantiates one of each shipped flavor and `isinstance`-checks it against its Protocol so the swap-ability claim is structural rather than rhetorical. |
| `simulation/safety/` | Swappable safety filters (DOB-CBF for intent-preserving HIL, MPPI predictive shield, NMPC gradient comparison) and the terrain- and latency-aware `collision_warning.py` warning module |
| `benchmarking/` | The full benchmarking suite. `benchmarking/run.py` is the single orchestrator; `publish_paper_figures.py` writes canonical figures into `my_paper/paper_figures/`. Sub-scripts test one paper claim each |
| `nn_training/` | Canonical trainers: `train_static_v3.sh` (rig static), `train_rate_v2.sh` (rig rate), `train_vehicle_lhs.sh` (whole-vehicle variants), `train_terrain_window_mlp.py` (window terrain estimator), `train_vehicle_fy_surrogate.py` (whole-vehicle Fy surrogate for the Dallas UKF). `train_variant.py` is the shared tire-NN trainer |
| `data_collection/` | Tire-rig (`collect_static_data.cpp`, `collect_rate_data.cpp`), closed-loop tire-surrogate (`collect_closed_loop_data.py`), the broad multi-axis terrain-estimator collector (`collect_broad_terrain.py`), and the Dallas-UKF SCM collectors (`run_dallas_scm.py` single-run, `collect_lhs_training_scms.py` LHS sweep) |
| `utilities/` | Closed-loop trace collection (`collect_diverse_terrains.py`, `collect_rich_excitation.py`) and diagnostic/offline utilities |
| `nn_models/` | Active trained checkpoints only: one rig tire surrogate, one whole-vehicle tire surrogate, and one n-only terrain estimator |
| `data/` | Four categories: `tire_rig/`, `whole_vehicle/`, `terrain_estimator/` (window-MLP traces), and `dallas_scm/` (Dallas-UKF SCM logs: `lhs_train300/` training sweep, `lhs100/` + `lhs100_cl/` benchmarks, canonical `clay/sandy_loam/sand.npz`) |
| `config/` | `latency_profiles/` (generated 5G traffic profiles) and `terrain_yamls/` (LHS-sampled terrain configs for `closedloop_sine_lhs_fair_v2` and `rich` excitation sets) |
| `docs/` | Active design docs: `FRAMEWORK_CONTRACTS.md`, `SURROGATE_RETRAIN_FINDINGS.md` |
| `experiment_results/` | Output sink for re-runs: orchestrator logs and any timestamped result snapshots that feed `my_paper/paper_figures/` |
| `my_paper/` | `paper.tex` (IEEEtran two-column), `paper.pdf`, `paper_figures/` (current figures + CSV backings), original ACMD abstract |
| `archive/` | Date-stamped folders of removed/superseded code and data. The most recent (`2026-05-23_final_cleanup/`) holds the PIL terrain estimator, MPCC controller, UKF baseline, online residual learning, and legacy `closed_loop_v1..v3` datasets |

## Framework contracts

The runtime is organised around six swap points. Each has one
`Protocol` in `simulation/framework/interfaces.py` and one `Registry`
in `simulation/framework/registry.py`. Adding a new flavor is one
decorator with no edits to the consumers:

```python
from simulation.framework import SAFETY_FILTERS, SafetyFilter

@SAFETY_FILTERS.register("my_filter")
class MyFilter:
    def filter(self, s, t, b, state, obs): ...
    def update_command_age(self, t): ...
    def set_teleop_delay(self, d): ...
    def get_diagnostics(self): ...

# anywhere in the runtime / benchmarks:
shield = SAFETY_FILTERS.create("my_filter", vehicle_params=..., ...)
```

| Registry | Protocol method | Shipped flavors |
| --- | --- | --- |
| `COMMAND_SOURCES`    | `next_command`     | acados NMPC, Logitech G29, WASD |
| `SAFETY_FILTERS`     | `filter`           | DOB-CBF, MPPI, NMPC, none |
| `COLLISION_WARNINGS` | `evaluate`         | ttc (terrain + latency) |
| `TIRE_MODELS`        | `predict`          | rate-MLP, axle-rate-MLP, Pacejka, TMeasy |
| `TERRAIN_ESTIMATORS` | `observe`/`estimate` | sliding-window MLP (`n`), Dallas-style UKF (offline only) |
| `LATENCY_PROFILES`   | `delay`            | constant, replay, learned 5G N-HiTS |

`python simulation/framework/test_conformance.py` instantiates one of
each shipped flavor and `isinstance`-checks it against the declared
Protocol; a flavor that drifts from its API fails the check rather
than producing silent runtime errors. See
`docs/FRAMEWORK_CONTRACTS.md` for the full contract map and current
boundary limitations.

## Environment

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sim
export ACADOS_SOURCE_DIR=~/Documents/sbel/acados
cd project/SCM_Final
```

The `sim` env carries PyChrono, acados / acados_template, CasADi 3.6,
PyTorch, NumPy/Pandas/Matplotlib, and pyzmq. acados shared libraries
are preloaded by the controller modules; setting `ACADOS_SOURCE_DIR`
every shell is mandatory.

## Reproducing the paper

```bash
# 1. Run every sub-sweep at the full paper matrix (large; multi-hour)
python benchmarking/run.py --tier paper

# 2. (or) Pilot tier (smaller matrix, ~6 hr on a 24-core box)
python benchmarking/run.py --tier pilot

# 3. (or) Run one sweep at a time
python benchmarking/run.py --tier pilot --only safety
python benchmarking/run.py --tier pilot --only dob_cbf_ablation

# 4. (or) Smoke check that every script launches (~15 min)
python benchmarking/run.py --tier smoke

# 5. Publish figures into my_paper/paper_figures/ (the orchestrator
#    above invokes this automatically at the end of each run)
python benchmarking/publish_paper_figures.py

# 6. (figures only) Regenerate EVERY figure in paper.tex into
#    my_paper/paper_figures/ with the exact filenames, from the latest
#    results -- one command, no Chrono re-run. Prints [ok]/SKIP/FAIL per
#    figure. (Run the sweeps/benches in steps 1-4 first.)
python benchmarking/make_paper_figures.py
```

### Terrain-estimator comparison (paper §VI)

Three estimators are compared on the same Chrono SCM ground truth:
the Dallas-style **state-augmented UKF** with two tire backends
(analytical **Bekker** and the whole-vehicle **NN** surrogate
`nn_models/vehicle_fy_64_32/`), and the deployed sliding-window
**MLP** regressor. The full rebuild is four steps (A → D).

The benchmark LHS box restricts `bekker_n` to `[0.40, 1.30]` so all
three estimators are evaluated inside the window-MLP's training range
and the SCM patch's physical regime (the SCM model is unsimulable
below n ≈ 0.37). The NN-UKF surrogate is trained on a *widened* soil
box (so the canonical clay/dirt/sand presets are interior, not
box-corner, points — a standard-box surrogate fails on clay).

**Step A — train the NN-UKF tire surrogate (widened box, ~25 min)**

```bash
# 300 disjoint LHS training scenarios (seed 7), widened soil box,
# half open-loop / half PI-cruise throttle
python data_collection/collect_lhs_training_scms.py --n 300 --workers 8 \
    --seed 7 --widened-box --out-dir data/dallas_scm/lhs_train300

# train the (128,64) MLP, 90/10 split-by-scenario → vehicle_fy_64_32/
python nn_training/train_vehicle_fy_surrogate.py \
    --lhs-dir data/dallas_scm/lhs_train300 \
    --hidden 128 64 --epochs 400 --decim 2 --test-frac 0.10
```

**Step B — broad 100-LHS benchmark, two excitation modes (Fig. 8 + 9)**

```bash
# Open-loop (constant throttle 0.75) — the paper-headline Fig. 8.
# This is the MLP's native training excitation. ~30 min incl. SCM.
python benchmarking/bench_terrain_estimators_lhs.py --n 100 --workers 8 \
    --n-min 0.40 --n-max 1.30 --steer-amp-rad 0.6 \
    --open-loop-throttle 0.75 --out-name lhs100_fair

# Closed-loop (PI cruise to 5 m/s) — separate SCM-log dir via suffix
python benchmarking/bench_terrain_estimators_lhs.py --n 100 --workers 8 \
    --n-min 0.40 --n-max 1.30 --steer-amp-rad 0.6 \
    --open-loop-throttle -1 --target-speed 5.0 \
    --log-suffix _cl --out-name lhs100_cl

# CL-vs-OL robustness panel (Fig. 9) from the two CSVs above
python benchmarking/plot_cl_vs_ol.py
```

Writes `lhs100_fair.{png,csv}`, `lhs100_cl.{png,csv}`,
`lhs100_cl_vs_ol.png`. (Add `--skip-collection` to re-run only the
estimator pass on existing SCM logs, ~2 min.)

**Step C — 3-preset single-trace spot check (Fig. 10)**

```bash
# Regenerate the three canonical logs at the benchmark excitation
# (amp 0.6, PI cruise). NOTE: --open-loop-throttle -1 selects PI
# cruise; a value >=0 would be a constant open-loop throttle.
for t in clay dirt sand; do
  out=$([ $t = dirt ] && echo sandy_loam || echo $t)
  python data_collection/run_dallas_scm.py --terrain $t --time 50 --lead-in 3 \
      --steer-amp-rad 0.6 --open-loop-throttle -1 --target-speed 5.0 \
      --output data/dallas_scm/${out}.npz
done
# run all three estimators → terrain_estimator_comparison.{png,csv}
python benchmarking/eval_terrain_estimators.py
```

**Step D (optional) — paper118-faithful Bekker-vs-NN UKF only**

```bash
python deliverables/ukf_paper_validation.py   # → ukf_dallas_validation_scm.png
```

**Current results** (NN-UKF = best median in every mode):

| Benchmark | Bekker-UKF | NN-UKF | Window MLP |
| --- | --- | --- | --- |
| 100-LHS open-loop (Fig. 8) median | 34.3 % | **12.9 %** | 17.0 % |
| 100-LHS closed-loop (Fig. 9) median | 20.9 % | **9.4 %** | 15.7 % |
| Canonical clay (Fig. 10) | 22.9 % | **18.0 %** | 24.8 % |
| Canonical sandy loam | 4.9 % | **3.2 %** | 8.7 % |
| Canonical dry sand | 36.2 % | 10.8 % | **7.5 %** |

### Human-in-the-loop safety-filter rounds

The HIL rounds (paper §VI-A) are **not** part of `benchmarking/run.py`
because they require a human driver. Run separately:

```bash
# Default G29 protocol, symmetric link (camera delay = command delay)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none mppi dob_cbf nmpc \
    --delays 0.0 0.15 0.30 \
    --rounds 3 --manual-mode g29 --vis-mode sensor

# 5G-style asymmetric link (heavier video downlink than command uplink)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none dob_cbf mppi \
    --delays 0.0 0.15 0.30 \
    --camera-delay-scale 1.6 \
    --rounds 3 --manual-mode g29 --vis-mode sensor

# WASD smoke test that runs in 8 s with no driver input — pipeline
# integration check only (vehicle sits still, no collisions, no
# intervention). Useful for verifying the wiring after env changes.
python benchmarking/human_delay_compensation_rounds.py --quick --auto-start
```

Each round delays both the operator command path
(`--manual-input-delay`) and the driver POV camera
(`--camera-input-delay = camera_delay_scale * delay`). The active
safety filter additionally receives `--teleop-delay` so its
predictive horizon is delay-aware. The script prompts before each
round so the driver can get set; pass `--auto-start` to skip.

Per-cell metrics aggregated by the script:
- safety: `collisions` (unique obstacles hit), `near_misses`,
  `min_clearance_m`;
- intrusiveness: `intervention_rate_pct`, `mean_abs_dsteer`,
  `mean_abs_dthrottle` (how often the shield fires and how far it
  pulls the operator's command);
- tracking: `rms_cte_m`, `speed_ratio`;
- each metric is reported per (filter, delay) cell with mean and
  std across the `--rounds` repeats.

Outputs land in
`benchmarking/results/human_delay_compensation_rounds_<ts>/`:
`results.csv` (per-round), `summary_by_filter_delay.csv`
(aggregated), `summary.md`, and `figures/` (6-panel
safety-vs-intrusiveness panel + collision heatmap). The publish step
picks them up if the timestamp suffix matches.

## NN models

| Directory | Role |
| --- | --- |
| `rig_rate_64_32/` | Retained tire-rig rate surrogate for rig-vs-vehicle diagnostics |
| `vehicle_rate_64_32_lhs/` | Default whole-vehicle rate surrogate for standard NMPC and safety-filter sweeps |
| `terrain_window_mlp/` | Retained online terrain estimator; n-only output |
| `rig_rate_paper118_v2_64_32/` | Paper118-spec rig NN (uniform LHS, widened α and Fz) used by the Dallas-style UKF reproduction; see `deliverables/ukf_paper_validation.py` |
| `vehicle_fy_64_32/` | **Whole-vehicle Fy surrogate** for the Dallas-style state-augmented UKF — trained on a 300-scenario disjoint widened-box LHS sweep (`--widened-box`, half OL / half PI-cruise throttle). Predicts $(F_{y,\mathrm{total}}, M_{\mathrm{yaw,total}})$ directly, replacing the rig NN + rig-to-vehicle calibration scalar (no post-hoc scalar). Held-out Fy R² = 0.90. Drives the 100-LHS benchmark and canonical spot-check in paper §VI; see its `TRAINING_METADATA.md`. |

Older static, axle-rate, and joint-estimator checkpoints were archived
in `archive/2026-05-23_model_checkpoint_and_root_artifact_cleanup/`.
The PIL tire model is archived in
`archive/2026-05-23_final_cleanup/pil/`. Restore from those archives
only when replaying historical ablations.

## Datasets

The active data tree has exactly three categories:

| Directory | Purpose |
| --- | --- |
| `data/tire_rig/` | Open-loop single-tire SCM rig CSVs (`scm_static_100k_v4.csv`, `rate_v2_100k.csv`, `rate_paper118_v2_15k.csv`) used by `train_static_v3.sh` / `train_rate_v2.sh` / the Dallas UKF baseline. The v1 sweeps (`rate_v1_100k`, `rate_paper118_30k`) were archived 2026-05-31 |
| `data/whole_vehicle/lhs/` | Closed-loop LHS-sampled training data used by `train_vehicle_lhs.sh` to produce `vehicle_rate_64_32_lhs` and its matched-architecture variants |
| `data/terrain_estimator/` | Sliding-window traces used to train `terrain_window_mlp` — `traces_broad_v7/` (active; 3600 scenarios spanning 180 LHS cells × scripted+closed-loop × 3 speeds × 3 paths × 2 bumpiness), `traces_vertical_v5/` (predecessor, 400 scripted traces with vertical IMU channels) |

Legacy `closed_loop_v1/v2/v3_rich/` datasets and the `5g_generated/`
profile cache were archived to
`archive/2026-05-23_final_cleanup/old_data/`.

## Paper

`my_paper/paper.tex` (IEEEtran two-column) is the self-contained full
paper. Compile with `pdflatex paper.tex` twice (or `latexmk -pdf
paper.tex`). `my_paper/abstract.tex` is the original single-page
ACMD 2026 abstract.

## Tire-rig training (preserved baseline)

The tire-surrogate pipeline went through two generations (paper
§III-C). The active root keeps one checkpoint from each generation;
retired variants are archived.

| Generation | Where | What it does |
| --- | --- | --- |
| Tire rig | `data_collection/collect_static_data.cpp`, `collect_rate_data.cpp` | Chrono SCM single-tire rig sweep over $(\kappa, \alpha, F_z, \theta_\mathrm{soil})$; logs the ground-truth tire force per query |
| Tire rig trainer | `nn_training/train_static_v3.sh`, `train_rate_v2.sh` (call `train_variant.py`) | Trains static / rate variants; the retained active baseline is `rig_rate_64_32` |
| Closed-loop | `data_collection/collect_closed_loop_data.py` | Runs the actual MPC stack on randomised scenarios and logs the live operating point with the SCM ground-truth force |
| Closed-loop trainer | `nn_training/train_vehicle_lhs.sh` | Trains the retained LHS whole-vehicle checkpoint `vehicle_rate_64_32_lhs` |

If you need to retrain the rig models from scratch, re-collect via the
Chrono rig binaries in `data_collection/` and rerun
`bash nn_training/train_static_v3.sh` / `train_rate_v2.sh`.

## Collision warning (modular HMI signal)

`simulation/safety/collision_warning.py` is a swappable forward
collision-warning module that runs in parallel with whatever safety
filter is selected (or none). It outputs a discrete severity
{GREEN, YELLOW, ORANGE, RED} signal that downstream code (e.g. an HMI
overlay) can consume. The default flavor is time-to-collision with
two extensions: braking deceleration is computed *analytically at
init time* by querying the deployed rig surrogate
(`rig_rate_64_32`) over a sweep of braking slip ratios at each
$\hat n \in [0.40, 1.30]$ on a 0.05 grid; the resulting table runs
from $a_b\!\approx\!2.3\,\mathrm{m/s^2}$ on soft clay to
$4.7\,\mathrm{m/s^2}$ on firm sand. The live $\hat n$ from the
terrain estimator indexes this table at runtime. The operator
reaction budget inflates with the EMA one-way latency plus the
standard deviation of recent jitter. Soft soil and high jitter both
fire the warning earlier.

Two validators ship with the warning module:

```bash
# 1) Forward lead time at fixed throttle into a single rock — sweeps
#    terrain x latency and verifies lead time grows monotonically with
#    softer soil and longer one-way delay.
python benchmarking/collision_warning_test.py --workers 3

# 2) Brake-decel validation against actual Chrono SCM stops — 27 full
#    brake trials (3 terrains x 3 initial speeds x 3 seeds). The
#    analytical a_b(n) prediction lands within 0.17 m mean absolute
#    error of the recorded stopping distance.
python benchmarking/brake_test.py
```

## Scope notes

The live terrain-estimation story is n-only. The controller maps the
estimated `n` along the retained clay--dirt--sand Bekker--Mohr manifold
to recover the complete soil vector used by NMPC and the safety shield.

The safety layer is presented as swappable rather than as a single
winning filter. DOB-CBF has the cleanest intent-preserving HIL story
because it solves for the closest safe command. MPPI is still
important as a predictive learned-dynamics shield, especially for
testing seeded recovery rollouts under the same scenarios.

Archived exploratory ablations remain in `archive/` and in the raw
paper-figure directory for reproducibility, but the current paper and
slides should not present them as part of the selected framework story.
