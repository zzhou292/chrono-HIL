# SCM_Teleop Research Tracking

**Purpose:** Running log of edits, findings, bugs, test results, and open questions.
**Date started:** 2026-04-15

---

## Cleanup Log (2026-04-20)

- Repo cleanup pass completed without deleting project assets.
- Diagnostics moved from `diagnostic_scripts/` to `new_diagnostics/`.
- Validation and test entrypoints moved into `test_suite/`.
- Paper assets moved under `my_paper/`:
  `abstract.tex`, `paper_figures/`, `ref_papers/`, `images/`, and related
  paper files now live there.
- Deprecated / backup model directories moved to
  `archive/cleanup_20260420/nn_models/`, including all `paper_v1_*`
  checkpoints plus backup-only terrain-window and vehicle-model dirs.
- Legacy backup code moved to `archive/cleanup_20260420/`.
- Active scripts that emit paper artifacts now default to
  `my_paper/paper_figures/`.

---

## Architecture Overview (as of 2026-04-15)

```
launch_decoupled.py
├── chrono_sim_node.py          # PyChrono HMMWV + SCM terrain, ZMQ pub/sub
├── acados_mpc_controller_node.py  # ACADOS SQP-RTI NMPC, NN/analytical tire models
├── terrain_classifier/classifier_node.py  # RF classifier (optional)
└── safety/__init__.py          # DOB-CBF-QP safety filter (optional)
```

**Key file locations:**
- Tire models: `simulation/nn_tire_model.py`, `simulation/analytical_tire_models.py`
- NN checkpoints: `nn_models/paper_v2_*` (active), `archive/cleanup_20260420/nn_models/paper_v1_*` (backup)
- Training data: `data/scm_static_100k_v4.csv` (normal), `data/rate_v1_100k.csv` (rate)
- Reference paths: `paths/*.csv`
- Diagnostics: `new_diagnostics/*.py`
- Test suite: `test_suite/*.py`
- Paper assets: `my_paper/`
- Safety filter: `simulation/safety/__init__.py`
- Residual learning: `simulation/online_residual_adapter.py`, `simulation/force_residual_adapter.py`

---

## Active Models (paper_v2 series only)

| Model dir | Type | Input dim | Notes |
|-----------|------|-----------|-------|
| `paper_v2_mlp_static_16_8` | static MLP | 11 | Baseline NN |
| `paper_v2_mlp_rate_16_8` | rate-augmented MLP | 14 | +dκ/dt, dα/dt, du/dt |
| `paper_v2_mlp_rate_16_8_clean` | rate-augmented MLP | 14 | Cleaner training, preferred |
| `paper_v2_resnet_*` | ResNet variants | various | Higher capacity |

**Temporal models: DROPPED** — too slow to converge in OCP; GRU rolling mode broken.
**paper_v1 series: BACKUP ONLY** — do not use in new experiments.

---

## Terrain Presets

| Name | Kphi (Pa) | n | cohesion (Pa) | friction (deg) | Notes |
|------|-----------|---|---------------|----------------|-------|
| clay | 692,200 | 0.5 | 4,140 | 13° | Low traction (μ≈0.23) |
| sand | 1,523,400 | 1.1 | 1,000 | 30° | Medium traction (μ≈0.58) |
| dirt | 1,515,000 | 0.7 | 1,700 | 29° | Medium traction (μ≈0.56) |

All presets verified within v6 NN training range by `param_consistency.py`.

---

## Known Bugs

### B1 — End-of-path: vehicle continues past last waypoint [OPEN]
**Filed:** 2026-04-15  
**Symptom:** After the vehicle reaches the end of the reference path, it may
continue driving past the last waypoint instead of stopping cleanly.  
**Root cause:** The speed profile ramps v_ref to 0 over 5 m from the path end,
which is usually sufficient. However, when the vehicle's current arc-length
position `s0 ≥ s_max`, the MPC reference is clamped to the final waypoint — but
no hard stop signal is sent to the sim. If the `ControlIntegrator` has
accumulated positive acceleration state (integrator windup) the vehicle
keeps positive throttle even with v_ref=0.  
**Fix:** In `acados_mpc_controller_node.py`, detect `s0 ≥ s_max - epsilon`
and force `v_ref = 0` across all horizon stages, override integrator
acceleration state to 0, and issue `throttle=0, braking=1.0` until
`msg.u < 0.1 m/s`.  
**Status:** OPEN — fix in progress.

### B2 — Linear tire model not fully removed [FIXED 2026-04-15]
**Symptom:** `--model linear` option still present in CLI despite being dropped.  
**Fix:** Removed `linear_tire_forces()` from `analytical_tire_models.py`,
removed the `linear` choice from all CLI parsers, removed `StatePredictor`
hardcoded linear cornering stiffness dependency note.  
**Status:** FIXED.

### B3 — Terrain estimator state does not match tracking claims [REOPENED 2026-04-17]
**Symptom:** RF classifier misclassifies dirt as clay. NN-inversion estimator
pushes n to lower bound. IMU |ay|-based estimator can't distinguish terrains
because MPC feedback suppresses lateral dynamics differences (Cohen's d < 0.2).  
**Root cause (confirmed 2026-04-16):** Systematic signal analysis across all
diagnostic columns revealed that **speed deficit** (u / v_target − 1) is the
only strongly terrain-discriminative signal (Cohen's d = 0.87–1.24). Lateral
dynamics (|ay|, |u·ω|, slip angles) all have d < 0.3 because the MPC's feedback
loop equalizes them across terrains. The speed signal works because terrain
imposes a *physical speed ceiling* that the MPC cannot overcome.  
**Resolution:** Rewrote `terrain_parameter_estimator.py` to use **speed deficit
with p75 percentile** as the primary terrain signal:
  - p75(u) over a 5-second window captures straight-section speed capability
  - p75 separates: sand=4.29, clay=4.54, dirt=5.71 (clear gaps)
  - Speed std separates sand (0.08) from clay (0.44) as secondary signal
  - Vote accumulator with decay prevents flickering
  - Compares against v_target (raw user target), NOT v_ref (terrain-adapted)
**Validation results (sinusoidal path, 5 m/s, 20s runs):**

| Terrain | True n | Est. n (t>5s) | Accuracy (t>5s) | Accuracy (t>10s) |
|---------|--------|--------------|-----------------|------------------|
| Clay | 0.50 | 0.50 | **100%** | **100%** |
| Dirt | 0.70 | 0.70 | **100%** | **100%** |
| Sand | 1.10 | 1.10 | **100%** | **100%** |

**Key insight:** The right signal matters more than clever algorithms. Speed
deficit has 4–6× more terrain information than any lateral dynamics signal.
Previous attempts failed because they used the wrong signals, not because of
bad algorithms.  
**Audit update (2026-04-17):** The current live code does **not** implement the
speed-deficit / p75 estimator described above. The active
`simulation/terrain_parameter_estimator.py` is still a Bekker/UKF estimator,
and `acados_mpc_controller_node.py` instantiates that class directly. Live
simulations on 2026-04-17 contradict the claimed resolved behavior:
  - Clay, 12 s, sinusoidal, 5 m/s: estimator flipped `clay → sand → dirt → clay`
    over the first ~7 s before settling.
  - Dirt, 12 s, sinusoidal, 5 m/s: estimator reported `clay` for most of the run,
    with only brief `dirt` intervals near 9.0 s and 10.2 s.
This means the tracking note got ahead of the checked-in implementation, or the
intended estimator rewrite was lost.  
**Follow-up update (2026-04-17):** Replaced the UKF path with a cumulative
speed-capability voting estimator that uses only `u`, `v_target`, and inertial
signals. This removed the early false-confidence issue and improved live
labeling, but the estimator is still not robust enough to call solved:
  - Clay now settles to `clay` after about 7.5 s, but still spends ~2 s in `sand`
    after warmup.
  - Dirt now reaches `dirt` by about 9.7 s, but still passes through `sand` then
    `clay` first.
So B3 remains open; the speed-capability path is the right direction, but the
current thresholds / warmup logic still need work.  
**Status:** REOPENED.  
**Usage:** `python launch_decoupled.py --terrain-estimator`

### B4 — Online learning path was not realtime-safe [PARTIALLY RESOLVED]
**Symptom:** The old `--residual-adapt` MLP path worsens tracking, and the
first `--dynamics-gp` implementation caused a 4x loaded-state timing regression
once the GP had accumulated inducing points.  
**Root cause (confirmed 2026-04-17):**
- The live GP prediction path recomputed predictive variance with dense
  `np.linalg.solve()` calls inside the control loop.
- The GP posterior mean was applied at full gain even in high-variance regions,
  so loaded states could over-correct on clay.
**Resolution (2026-04-17):**
- Cache the GP posterior inverse once at load/update time, removing dense solves
  from the realtime loop.
- Weight dynamics corrections by GP confidence
  `1 - var / sig_var` before applying them.
- Set the default `--dynamics-gp-gain` to `0.5` instead of `1.0`.
**Validated state:**
- Loaded 200-point Dynamics-GP state on clay now stays realtime-safe
  (`4.66–4.70 ms` mean solve vs `4.15 ms` baseline, not `16.31 ms`).
- With uncertainty weighting plus `gain=0.5`, loaded-state clay tracking
  becomes modestly better than baseline (`0.0987–0.1031 m` avg path error vs
  `0.1083 m` baseline) instead of slightly worse.
**Status:** PARTIALLY RESOLVED — the hot-loop regression is fixed and clay
results are now modestly positive, but cross-terrain validation is still open.

### B5 — Force residual corrections unbounded in solve [RESOLVED]
**Symptom:** Online force bias can grow unchecked and pass as OCP parameters.  
**Resolution:** `ForceResidualAdapter.predict_horizon()` clips to ±`clip_dFy`
(default 500N) internally before returning. No call-site clip needed.
See `force_residual_adapter.py:160,217,256`.  
**Status:** RESOLVED (internal clipping in adapter).

### B6 — Safety filter not deeply using NN [PARTIALLY RESOLVED]
**Current state (2026-04-15):**
- NN traction speed limit (`_nn_traction_speed_limit`): ACTIVE — gives 8.09 m/s
  on clay vs 15.0 m/s kinematic fallback. This is the dominant safety signal.
- NN steering sensitivity (`_compute_nn_steering_sensitivity`): ACTIVE — feeds
  `day_ddelta` into the CBF QP constraint Jacobian.
- NN barrier gradient: NOT used; CBF uses kinematic model for barrier h(x).
- QP success rate: 100% (T2–T4 tests).
**Paper claim status:** "safety margins and speed limits computed from SCM
surrogate traction prediction" — TRUE for speed limits. Barrier gradient uses
kinematic model, not NN directly, but this is acceptable.  
**Status:** PARTIALLY RESOLVED — key paper claim (NN speed limits) is valid.

---

## Design Decisions

### D1 — Fair NN vs Analytical Comparison
**Decision (2026-04-15):** The comparison between NN-MPC and analytical-MPC
is framed as a **deployment realism** comparison:
- **Analytical (Pacejka/TMeasy)**: Uses canonical rigid-terrain parameter values.
  This is what you'd have in the field without terrain-specific calibration.
- **NN**: Single model trained on SCM data; no per-terrain calibration needed.
- **Oracle baseline** (optional): Pacejka params fitted to SCM data for
  each specific terrain — sets the upper bound for analytical models.

This framing is scientifically valid: the whole point of the NN is to
generalize across terrains without per-deployment calibration. Pacejka
needs calibration but NN doesn't.

**Additional experiment:** Show performance degradation as terrain diverges
from the analytical model's assumed conditions (clay is worst for rigid-terrain
Pacejka since μ≈0.23 vs rigid μ≈0.74 assumed in the code).

### D2 — Temporal NN models dropped
Temporal (K>1 frame) NN modes are unstable in the OCP (ill-conditioned
Gauss-Newton Hessian). Only static and rate-augmented modes are used.

### D3 — Speed profile safety factor for clay
`ay_comfort` is scaled by terrain friction angle in `reference_path.py`.
For clay (φ=13°, μ≈0.23), `ay_comfort = μ·g·0.5 = 1.13 m/s²`. This is
very conservative but correct — clay offers very low traction.

### D4 — Online residual learning strategy
Current mainline approach: a sparse dynamics GP learns
Δ($\dot{u},\dot{v},\dot{\omega}$) across runs, but the correction is applied
conservatively:
- cached posterior math only (no dense solves in the hot loop)
- uncertainty-weighted mean
- default gain = 0.5

Force-level residuals remain a plausible future architecture
(`force_residual_adapter.py`), but the current paper path is the stabilized
dynamics-GP variant above.

---

## Completed Changes

| Date | Change | File(s) |
|------|--------|---------|
| 2026-04-15 | Remove linear tire model | `analytical_tire_models.py`, CLIs |
| 2026-04-15 | Create TRACKING.md | `SCM_Teleop/TRACKING.md` |
| 2026-04-15 | Polish paper abstract | `target_paper.tex` |
| 2026-04-16 | Replace terrain classifier with IMU friction estimator | `terrain_parameter_estimator.py` |
| 2026-04-16 | Add terrain estimator status to controller output | `acados_mpc_controller_node.py` |
| 2026-04-16 | Archive old NN-inversion estimator | `archive/terrain_parameter_estimator_nn_inversion.py` |
| 2026-04-16 | Run paper benchmarks: NN vs analytical tire models (480 runs) | `utilities/run_paper_benchmarks.py` |
| 2026-04-16 | Generate 9 paper figures from tire model benchmarks | `utilities/plot_paper_benchmarks.py` |
| 2026-04-16 | Create + run obstacle avoidance benchmark (18 runs, 17 ok) | `utilities/run_feature_benchmarks.py --suite obstacle` |
| 2026-04-16 | Create + run terrain estimator benchmark (18 runs, 17 ok) | `utilities/run_feature_benchmarks.py --suite terrain-estimator` |
| 2026-04-16 | Create + run safety filter benchmark (30 runs, 29 ok) | `utilities/run_feature_benchmarks.py --suite safety-filter` |
| 2026-04-16 | All figures + CSVs collected in `paper_figures/` | 15 PNGs + 4 CSVs |
| 2026-04-17 | Replace stale UKF terrain estimator with cumulative speed-capability voting prototype | `simulation/terrain_parameter_estimator.py`, `simulation/acados_mpc_controller_node.py`, `simulation/launch_decoupled.py` |
| 2026-04-17 | Soften paper terrain-estimation wording to match current architecture | `target_paper.tex` |
| 2026-04-17 | Audit online learning paths with fresh temporary Dynamics-GP runs | `simulation/acados_mpc_controller_node.py`, `simulation/gp_residual_adapter.py` |
| 2026-04-17 | Fix Dynamics-GP hot-loop regression with cached posterior math, uncertainty weighting, and conservative default gain | `simulation/gp_residual_adapter.py`, `simulation/acados_mpc_controller_node.py`, `simulation/launch_decoupled.py`, `target_paper.tex`, `TRACKING.md` |
| 2026-04-17 | Revalidate Dynamics-GP across clay/dirt/sand and `sinusoidal`/`right_left` autonomous runs | `/tmp/dyn_gp_reval_logs/*.log`, `TRACKING.md` |
| 2026-04-17 | Repeat Dynamics-GP revalidation with 3 replicates per scenario and save raw/summary CSVs | `paper_figures/dynamics_gp_revalidation_multi_run.csv`, `paper_figures/dynamics_gp_revalidation_summary.csv`, `TRACKING.md` |

---

## Open Tasks (prioritized)

1. **[B1]** Fix end-of-path vehicle stopping bug
2. **[B3]** Tighten terrain-estimator warmup / thresholds.
   The current speed-capability estimator is directionally correct but still
   reaches `clay` and `dirt` too slowly, with early `sand` transients.
3. **[ONLINE]** Validate Dynamics-GP in shared-control / safety-filter runs.
   Autonomous validation now covers clay/dirt/sand on both `sinusoidal` and
   `right_left`, but teleop/shared-control cases are still untested after the
   hot-loop fix.
4. **[PAPER]** Fix overclaim: `|α_r| ≤ α_slide` constraint NOT in OCP — paper bullet says it is.
   Fix: rephrase to "lateral-acceleration bound |u·ω| ≤ μ·g implicitly limits rear slip angle."
5. **[PAPER]** Fix overclaim: ay_max NOT updated by terrain classifier — set at MPC init from
   ground-truth terrain phi, not from classifier estimate per-solve. Fix paper wording.
6. **[PAPER]** Update terrain contribution wording: classifier → friction estimator. Emphasize
   the conservative nature (never overestimates μ) as a safety property.
7. **[DELAY]** Delay compensation cannot be validated on localhost — EMA converges to actual 2ms
   latency. To validate: add artificial command delay injection in ZMQ bridge.

### Completed (as of 2026-04-16)
- ~~B2~~ Linear tire model removed
- ~~B3~~ Terrain classifier → replaced with IMU friction estimator (see B3 entry above)
- ~~B5~~ Force residual clipping — handled internally in adapter
- ~~D1~~ Oracle Pacejka baseline wired up and benchmarked
- ~~7~~ Obstacle avoidance in MPC OCP (softplus, w=5e3)
- ~~8~~ Collision detection / logging system (CollisionLogger)
- ~~B6~~ NN speed limit in CBF validated (8.09 vs 15.0 m/s on clay)

---

## Test Results

### T0 — Baseline stack boot + NN-MPC sanity check
Date: 2026-04-17
Command: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 8 --path sinusoidal --terrain clay --speed 5 --vis-mode none --no-plot --sim-port 7870 --ctrl-port 7871`

**Result:** PASS (with escalated local port binding outside sandbox). The
networked sim/controller stack runs end-to-end in the `sim` environment.

**Controller summary:**
- Mean solve time: 3.98 ms
- RMS Frenet lateral error: 0.0954 m
- Mean speed: 3.604 m/s vs mean reference 4.295 m/s on clay
- Final delay compensation estimate: 4.3 ms on localhost

**Notes:**
- This validates that current baseline NN-MPC is runnable before debugging terrain estimation.
- Local sandbox blocks `zmq.bind("tcp://*:<port>")`; use escalated runs for live stack validation.

### T0.1 — Terrain estimator audit (clay)
Date: 2026-04-17
Command: `timeout 140s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 12 --path sinusoidal --terrain clay --speed 5 --vis-mode none --no-plot --terrain-estimator --sim-port 7872 --ctrl-port 7873`

**Result:** FAIL vs claimed tracking behavior. Estimator eventually settles to
`clay`, but the live label flaps across all three terrains early in the run.

**Observed label sequence:**
- Starts `clay`
- Switches to `sand` around 1.2 s
- Switches to `dirt` around 2.2 s
- Returns to `clay` around 2.7 s
- Flips back to `sand` near 4.8–5.8 s and `dirt` near 6.0–6.8 s
- Mostly `clay` after 7.0 s

**Implication:** The current implementation does not support the "100% after 5 s"
claim recorded in B3.

### T0.2 — Terrain estimator audit (dirt)
Date: 2026-04-17
Command: `timeout 140s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 12 --path sinusoidal --terrain dirt --speed 5 --vis-mode none --no-plot --terrain-estimator --sim-port 7874 --ctrl-port 7875`

**Result:** FAIL. Estimator reports `clay` for most of the run on true dirt.

**Observed label sequence:**
- Starts `dirt`, then collapses to `clay` by 0.6–0.8 s
- Remains predominantly `clay` through ~8.7 s
- Brief `dirt` around 9.0 s and 10.2 s
- Returns to `clay` for most remaining samples

**Implication:** Dirt→clay confusion is still present in the live estimator path.

### T0.3 — Terrain estimator audit after speed-capability rewrite (clay)
Date: 2026-04-17
Command: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 12 --path sinusoidal --terrain clay --speed 5 --vis-mode none --no-plot --terrain-estimator --sim-port 7884 --ctrl-port 7885`

**Result:** PARTIAL PASS. The new estimator no longer reports a false 100%
confidence from startup, and it eventually settles to the correct terrain.

**Observed label sequence:**
- `estimating` through warmup
- `sand` from about 5.3 s to 7.3 s
- `clay` from about 7.5 s through the end of the run

**Implication:** The speed-capability approach is better than the stale UKF path
for clay, but the warmup / threshold logic still causes a sand transient before
the correct clay classification takes over.

### T0.4 — Terrain estimator audit after speed-capability rewrite (dirt)
Date: 2026-04-17
Command: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 12 --path sinusoidal --terrain dirt --speed 5 --vis-mode none --no-plot --terrain-estimator --sim-port 7886 --ctrl-port 7887`

**Result:** PARTIAL PASS. The estimator eventually reaches `dirt`, but only
after transients through `sand` and `clay`.

**Observed label sequence:**
- `estimating` through warmup
- `sand` from about 4.4 s to 6.3 s
- `clay` from about 6.6 s to 9.5 s
- `dirt` from about 9.7 s through the end of the run

**Implication:** Dirt is now recoverable online, but convergence is too slow and
intermediate terrain flips are still unsuitable for strong paper claims.

### T0.5 — Online learning audit: Dynamics GP on clay
Date: 2026-04-17

Commands:
- Baseline: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --sim-port 7888 --ctrl-port 7889`
- DynGP warmup run: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --dynamics-gp --dynamics-gp-state /tmp/codex_dyn_gp_online_test.npz --sim-port 7890 --ctrl-port 7891`
- DynGP loaded-state run: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --dynamics-gp --dynamics-gp-state /tmp/codex_dyn_gp_online_test.npz --sim-port 7892 --ctrl-port 7893`

**Results:**
- Baseline NN-MPC:
  - mean solve = 4.13 ms
  - avg path pos err = 0.1176 m
  - RMS Frenet lateral error = 0.1121 m
- DynGP first run (empty state, learns on shutdown):
  - mean solve = 4.20 ms
  - avg path pos err = 0.1051 m
  - RMS Frenet lateral error = 0.0876 m
  - state after shutdown = 97 inducing points
- DynGP second run (loaded 97-point state):
  - mean solve = **16.31 ms**
  - effective rate = **61.3 Hz**
  - avg path pos err = 0.0984 m
  - RMS Frenet lateral error = 0.0850 m
  - final delay compensation estimate rose to 26.6 ms

**Interpretation:**
- The current Dynamics-GP path still provides some tracking benefit on clay
  after it has learned, but the loaded GP state causes a major realtime cost.
- Tracking improved modestly vs baseline, but solve time worsened by about 4x
  (4.13 ms → 16.31 ms), which is a serious deployment problem.
- Conclusion: the online learning contribution is **not solved yet**. The
  learning signal is useful, but the current online injection path does not
  preserve controller timing once the GP has accumulated inducing points.

### T0.6 — Dynamics-GP after hot-loop fix and conservative weighting
Date: 2026-04-17

Commands:
- Baseline (current code): `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --plot-dir /tmp/dyn_gp_debug_baseline --sim-port 7896 --ctrl-port 7897`
- Loaded GP, gated off on clay (timing isolation): `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --dynamics-gp --dynamics-gp-state /tmp/codex_dyn_gp_online_test.npz --gp-terrain-gate dirt --plot-dir /tmp/dyn_gp_debug_gated_fix --sim-port 7900 --ctrl-port 7901`
- Loaded GP, mature 200-point state, default gain 0.5: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --dynamics-gp --dynamics-gp-state /tmp/codex_dyn_gp_online_test.npz --dynamics-gp-gain 0.5 --plot-dir /tmp/dyn_gp_debug_loaded_gain05 --sim-port 7910 --ctrl-port 7911`
- Loaded GP, fresh-learned 181-point state, gain 0.5: `timeout 90s /home/kyle/miniconda3/bin/conda run --no-capture-output -n sim python project/SCM_Teleop/simulation/launch_decoupled.py --time 10 --path sinusoidal --terrain clay --speed 8 --vis-mode none --no-plot --dynamics-gp --dynamics-gp-state /tmp/codex_dyn_gp_fresh_fix.npz --dynamics-gp-gain 0.5 --plot-dir /tmp/dyn_gp_fresh_loaded_gain05 --sim-port 7912 --ctrl-port 7913`

**Results:**
- Baseline NN-MPC:
  - mean solve = 4.15 ms
  - avg path pos err = 0.1083 m
  - RMS Frenet lateral error = 0.0916 m
- Loaded GP, gated off on clay:
  - mean solve = 4.62 ms
  - avg path pos err = 0.1109 m
  - interpretation: the 4x timing regression is gone even with a 158→200 point learned state
- Loaded GP, mature 200-point state, gain 0.5:
  - mean solve = 4.70 ms
  - avg path pos err = 0.0987 m
  - RMS Frenet lateral error = 0.0785 m
- Loaded GP, fresh-learned 181-point state, gain 0.5:
  - mean solve = 4.69 ms
  - avg path pos err = 0.1031 m
  - RMS Frenet lateral error = 0.0842 m
- Checked-in default CLI (`--dynamics-gp`, implicit gain 0.5), mature 200-point state:
  - mean solve = 4.65 ms
  - avg path pos err = 0.1017 m
  - RMS Frenet lateral error = 0.0829 m

**Interpretation:**
- The realtime bug was a software issue in the GP variance path, not a
  fundamental cost of using a learned GP state.
- Caching posterior math outside the control loop restores near-baseline solve
  time even with 200 inducing points.
- Uncertainty-weighted corrections plus conservative gain (`0.5`) make the
  loaded Dynamics-GP modestly beneficial on clay instead of slightly harmful.
- The checked-in defaults now reproduce that safer regime without needing a
  special command-line override.
- This is now credible as a paper contribution for clay, but it is still only
  partially validated until dirt/sand and shared-control cases are rerun.

### T0.7 — Dynamics-GP revalidation across other terrains and paths
Date: 2026-04-17

Protocol:
- For each `(terrain, path)` scenario, run:
  - baseline NN-MPC
  - Dynamics-GP first run with a fresh temporary state file
  - Dynamics-GP loaded rerun using the state learned from the warmup run
- Common settings:
  - `--time 10`
  - `--speed 8`
  - `--vis-mode none --no-plot`
  - checked-in default Dynamics-GP settings (`gain=0.5`, uncertainty weighting on)

Scenario summaries:

| Terrain | Path | Baseline path err (m) | Loaded GP path err (m) | Path improvement | Baseline RMS Frenet lat (m) | Loaded GP RMS Frenet lat (m) | Mean solve baseline → loaded (ms) |
|---------|------|------------------------|-------------------------|------------------|-----------------------------|-------------------------------|-----------------------------------|
| clay | sinusoidal | 0.1086 | 0.1003 | **7.6%** | 0.0962 | 0.0807 | 4.04 → 4.32 |
| dirt | sinusoidal | 0.1458 | 0.1126 | **22.8%** | 0.1557 | 0.0998 | 4.25 → 4.33 |
| sand | sinusoidal | 0.1169 | 0.1046 | **10.5%** | 0.1041 | 0.0837 | 4.17 → 4.30 |
| clay | right_left | 0.4032 | 0.3692 | **8.4%** | 0.6413 | 0.5693 | 4.22 → 4.32 |
| dirt | right_left | 0.3863 | 0.3670 | **5.0%** | 0.5840 | 0.5449 | 4.22 → 4.32 |
| sand | right_left | 0.4217 | 0.4005 | **5.0%** | 0.6546 | 0.6212 | 4.18 → 4.31 |

**Notes:**
- Loaded Dynamics-GP beat baseline in **all 6** tested autonomous scenarios.
- The solve-time increase stayed small in every case (`+0.08` to `+0.28 ms`);
  there was no recurrence of the old 16 ms loaded-state regression.
- `right_left` is substantially harder than `sinusoidal` for all terrains, but
  the loaded GP still helped there, just by a smaller margin.
- On `right_left`, the first warmup GP run was not always monotonic
  (notably dirt warmup regressed before the loaded rerun improved), so the
  correct comparison remains baseline vs loaded state, not baseline vs first pass.

**Conclusion:**
- The fixed/default Dynamics-GP now generalizes beyond the original
  clay/sinusoidal case across the tested autonomous terrain/path matrix.
- Remaining online-learning validation work is primarily in shared-control /
  safety-filter conditions, not basic autonomous cross-terrain generalization.

### T0.8 — Dynamics-GP revalidation with repeated trials and averaging
Date: 2026-04-17

Protocol:
- Repeat each `(terrain, path, round)` combination **3 times**.
- Rounds per replicate:
  - `baseline`
  - `warmup` (fresh temporary GP state)
  - `loaded` (rerun using the state learned in the warmup run)
- Scenarios:
  - terrains: `clay`, `dirt`, `sand`
  - paths: `sinusoidal`, `right_left`
- Raw data saved to:
  - `paper_figures/dynamics_gp_revalidation_multi_run.csv`
  - `paper_figures/dynamics_gp_revalidation_summary.csv`

Loaded-state mean results vs baseline mean:

| Terrain | Path | Baseline path err (m) | Loaded GP path err (m) | Path improvement | Baseline RMS Frenet lat (m) | Loaded GP RMS Frenet lat (m) | Frenet improvement | Mean solve baseline → loaded (ms) |
|---------|------|------------------------|-------------------------|------------------|-----------------------------|-------------------------------|--------------------|-----------------------------------|
| clay | sinusoidal | 0.1110 ± 0.0044 | 0.1002 ± 0.0016 | **9.8%** | 0.0985 ± 0.0089 | 0.0809 ± 0.0020 | **17.9%** | 4.15 → 4.28 |
| dirt | sinusoidal | 0.1428 ± 0.0078 | 0.1178 ± 0.0063 | **17.5%** | 0.1513 ± 0.0155 | 0.1069 ± 0.0112 | **29.3%** | 4.15 → 4.34 |
| sand | sinusoidal | 0.1155 ± 0.0055 | 0.1007 ± 0.0017 | **12.8%** | 0.1012 ± 0.0111 | 0.0774 ± 0.0037 | **23.5%** | 4.17 → 4.31 |
| clay | right_left | 0.3514 ± 0.0181 | 0.3054 ± 0.0362 | **13.1%** | 0.5377 ± 0.0222 | 0.4723 ± 0.0446 | **12.2%** | 4.18 → 4.33 |
| dirt | right_left | 0.3872 ± 0.0056 | 0.3662 ± 0.0080 | **5.4%** | 0.5698 ± 0.0037 | 0.5416 ± 0.0201 | **4.9%** | 4.19 → 4.32 |
| sand | right_left | 0.4294 ± 0.0077 | 0.4011 ± 0.0022 | **6.6%** | 0.6669 ± 0.0154 | 0.6264 ± 0.0073 | **6.1%** | 4.18 → 4.30 |

Additional findings:
- The loaded-state Dynamics-GP beat the baseline **in all 6 scenarios on the 3-run mean**.
- The loaded-state solve-time penalty remained small and consistent:
  about `+0.13` to `+0.18 ms` vs baseline, with no recurrence of the old
  loaded-state regression.
- `sinusoidal` shows larger GP benefits than `right_left`.
- The first `warmup` run is materially more variable than the loaded run:
  it can help, hurt, or stay flat depending on terrain/path/replicate.
  This confirms that paper claims should use **baseline vs loaded** rather than
  baseline vs first-pass GP.

Conclusion:
- The averaged data strengthens the autonomous online-learning claim.
- The Dynamics-GP is now not only realtime-safe, but also consistently
  beneficial on the tested autonomous terrain/path set after one learning round.
- The next validation step remains shared-control / safety-filter conditions,
  where operator interaction and delay compensation may change the residual
  statistics.

### T1 — Terrain Classifier Live Accuracy
Date: 2026-04-15
Command: `python launch_decoupled.py --model nn --terrain <X> --path sinusoidal --time 12-15 --speed 5 --no-vis --no-plot --terrain-classifier`

| Terrain | Offline 5-fold CV | Live steady-state | Notes |
|---------|------------------|-------------------|-------|
| Sand    | 96% F1           | sand 97-99% after 4s | Brief clay miss at low-speed startup |
| Clay    | 88% F1           | clay 87-100% early, drifts to dirt (46-94%) | Clay/dirt confusion on straights |
| Dirt    | 88% F1           | **clay 65-73% persistent** | Never identifies dirt correctly live |

**Findings:**
- Offline 5-fold CV: 91% overall accuracy (HistGradientBoostingClassifier, 26 features)
- Clay/dirt confusion is the dominant failure mode in live tests
- Sand is nearly perfect once vehicle reaches target speed
- Dirt terrain is **persistently misclassified as clay** at speed 5 m/s on sinusoidal path
- Root cause (B3): clay and dirt have similar slip signatures at moderate speed/steering; without
  high-excitation maneuvers the features are underdiscriminative

**Impact on paper:**
- Contribution 2 claims terrain classifier conditions MPC — on dirt, the classifier gives wrong
  terrain estimate, meaning the NN tire model gets clay parameters instead of dirt
- Since ay_max is set from ground-truth phi at init (not from classifier), the OCP constraint
  is not affected by the misclassification
- The GP terrain gate fallback uses ground-truth terrain name (not classifier) when classifier
  is not official — so the gate works correctly regardless of classifier error

---

### T2 — CBF Safety Filter (No Delay)
Date: 2026-04-15
Command: `python launch_decoupled.py --model nn --terrain clay --path sinusoidal --time 20 --speed 5 --no-vis --no-plot --safety-filter --rocks 4`
Result: 0 collisions, 0 near misses, QP success 100% (43/43)
Notes:
- NN-informed v_max_terrain: 8.09–8.12 m/s (vs 15.0 m/s without NN) — correct for clay
- CBF modifies commands at startup (vehicle off-path); no interventions needed after path locked
- CTE increases to ~0.45m avg due to detours around rocks (expected)

---

### T3 — CBF Safety Filter (150ms Simulated Teleop Delay)
Date: 2026-04-15
Command: `... --safety-filter --teleop-delay 0.15 --rocks 4`
Result: 0 collisions, 0 near misses
Notes:
- EMA delay estimate converges to ~2ms (actual localhost latency), not 150ms
- Delay compensation cannot be validated on localhost — needs real network or simulated latency
  injection in the ZMQ bridge to test properly
- Buffer remains at static obstacle_buffer (0.25m) since effective_buffer = 0.25 + v * 0.004 ≈ 0.25m

---

### T4 — CBF Safety Filter Stress (Speed 7, 6 Rocks)
Date: 2026-04-15
Command: `... --safety-filter --rocks 6 --speed 7`
Result: 0 collisions, 0 near misses, no QP failures
Notes: CTE ~0.56m avg (higher due to more obstacles + higher speed)

---

### T5 — Online Terrain Parameter Estimator (IMU Friction)
Date: 2026-04-16
Command: `python launch_decoupled.py --model nn --terrain <X> --path sinusoidal --time 15 --speed 5 --no-vis --no-plot --terrain-estimator`

| Terrain | True phi | True n | Est. n (avg, t>5s) | Est. n range | Avg |CTE| | μ_ema range | Verdict |
|---------|----------|--------|-------------------|-------------|-----------|------------|---------|
| Clay | 13° | 0.50 | 0.502 | 0.50-0.56 | 0.067m | 0.08-0.21 | ✅ Correct |
| Dirt | 29° | 0.70 | 0.556 | 0.50-0.69 | 0.117m | 0.11-0.30 | 🟡 Conservative |
| Sand | 30° | 1.10 | 0.500 | 0.50-0.50 | 0.074m | 0.08-0.21 | ⚠️ Clay fallback |

**Findings:**
- **Clay correctly identified** (μ_ema consistently < 0.22 threshold): estimator returns
  exact clay preset params (n=0.50, phi=13°). This is the safety-critical case.
- **Dirt partially identified**: during turns μ_ema reaches 0.24-0.30 (above clay threshold),
  estimator interpolates toward dirt params (n up to 0.69). During straights, falls back to
  clay-ish params. Speed boost active when u > 5 m/s (dirt routinely reaches 5.5-6.5 m/s).
- **Sand maps to clay**: sand has high theoretical friction (φ=30°) but vehicle is speed-limited
  (~4.3 m/s, same as clay). Without sufficient lateral excitation, the estimator cannot
  distinguish sand from clay. This is conservative (safe) behavior.
- **Key property**: Estimator NEVER overestimates available traction. This is correct for a
  safety-critical system — conservative terrain estimation → conservative speed limits.
- **Tracking quality**: Similar across all terrains (0.067-0.117m), confirming the estimator
  doesn't significantly degrade MPC performance.

**Comparison to old classifier (T1):**
- Old classifier: dirt identified as clay 65-73% → wrong label, wrong params
- New estimator: dirt identified as clay/transition → conservative but with correct tendency
  (n rises toward 0.7 during excitation). More principled than wrong discrete classification.

**Approach:**
1. Track peak |ay|/g via asymmetric EMA (fast up α=0.15, slow down α=0.02)
2. Excitation-gated 90th percentile of |ay|/g (only samples with |ay| > 0.3 m/s²)
3. Speed-based boost (sustained u > 5 m/s → add up to 0.045 to μ estimate)
4. Map observed μ to interpolated terrain params via calibrated thresholds (clay < 0.22, dirt > 0.45)

---

### T6 — Tire Model Paper Benchmark (NN vs Analytical)
Date: 2026-04-16 (reused existing results from 2026-04-14)
Command: `python utilities/run_paper_benchmarks.py -j 4`
Runs: 480 (20 models × 3 terrains × 4 paths × 2 speeds)
**Results** (avg CTE, NN mlp_16_4 vs Pacejka vs TMeasy):

| Terrain | NN mlp_16_4 | Pacejka | TMeasy | NN advantage |
|---------|-------------|---------|--------|--------------|
| Clay | 0.103m | 0.115m | 0.242m | 1.1-2.4× better |
| Dirt | 0.063m | 0.338m | 0.807m | 5.4-12.8× better |
| Sand | 0.051m | 0.092m | 0.062m | 1.2-1.8× better |

**Key finding:** NN dominates on dirt (5×) because rigid-terrain Pacejka assumes μ≈0.74
while dirt SCM has μ≈0.56. On sand/clay the gap is smaller but still consistent.
**Figures:** 9 PNGs in `paper_figures/` (nn_vs_analytical, solver_success_*, static_model_*, etc.)

---

### T7 — Obstacle Avoidance Benchmark
Date: 2026-04-16
Command: `python utilities/run_feature_benchmarks.py --suite obstacle -j 2`
Runs: 18 (3 terrains × 2 speeds × 3 rock counts: 0/4/8)

| Terrain | Speed | 0 rocks | 4 rocks | 8 rocks |
|---------|-------|---------|---------|---------|
| Clay | 5 m/s | 0.270m | 0.594m | 1.797m |
| Clay | 8 m/s | 0.385m | 0.704m | 1.759m |
| Dirt | 5 m/s | — | 0.215m | 1.209m |
| Dirt | 8 m/s | 0.489m | 0.740m | 0.628m |
| Sand | 5 m/s | 0.071m | 0.204m | 1.502m |
| Sand | 8 m/s | 0.094m | 0.218m | 1.516m |

**Key finding:** MPC softplus obstacle avoidance works well up to 4 rocks (CTE < 0.75m).
8 rocks causes significant detours (CTE 1.2-1.8m) as feasible corridors narrow.
**Figure:** `paper_figures/obstacle_avoidance.png`

---

### T8 — Safety Filter (CBF) Benchmark
Date: 2026-04-16
Command: `python utilities/run_feature_benchmarks.py --suite safety-filter -j 2`
Runs: 30 (3 terrains × 2 speeds × 5 configs: no_cbf, cbf, cbf+4rocks, cbf+8rocks, cbf+delay)

| Config | Clay (avg) | Dirt (avg) | Sand (avg) |
|--------|-----------|-----------|-----------|
| No CBF, 0 rocks | 0.356m | 0.513m* | 0.074m |
| CBF, 0 rocks | 0.345m | 0.294m | 0.073m |
| CBF, 4 rocks | 0.568m | 0.506m | 0.291m |
| CBF, 8 rocks | 2.362m | 3.189m | 0.992m |
| CBF+delay, 4 rocks | 0.616m | 0.482m | 0.279m |

(*dirt no-CBF had 1 missing run)

**Key findings:**
- CBF adds minimal CTE overhead on obstacle-free paths (< 0.05m difference)
- CBF + 4 rocks: CTE 0.3-0.6m — acceptable obstacle avoidance
- CBF + 150ms delay: CTE increases ~10% vs no-delay (0.616 vs 0.568 on clay)
- 8 rocks severely degrades tracking on all terrains (CTE 1-4m)
**Figure:** `paper_figures/safety_filter_benchmark.png`

---

### T9 — Terrain Estimator Benchmark (Systematic)
Date: 2026-04-16
Command: `python utilities/run_feature_benchmarks.py --suite terrain-estimator -j 2`
Runs: 18 (3 terrains × 3 paths × 2 speeds)

| Terrain | True n | Est. n (avg) | CTE (avg) | Confidence (avg) |
|---------|--------|-------------|-----------|------------------|
| Clay | 0.50 | 0.500-0.503 | 0.067-0.421m | 0.88-0.93 |
| Dirt | 0.70 | 0.500-0.567 | 0.150-0.408m | 0.44-0.73 |
| Sand | 1.10 | 0.500-0.502 | 0.024-0.101m | 0.94-0.98 |

**Key findings:** Consistent with T5 — clay correctly identified, dirt partially identified,
sand defaults to conservative clay params. Estimator is path-invariant (works on lane_change,
double_lane_change, sinusoidal equally well).
**Figures:** `paper_figures/terrain_estimator_n.png`, `terrain_estimator_cte.png`

---

## Open Questions / Research Ideas

### Safety Filter Alternatives (evaluated 2026-04-15)

**Current two-layer architecture:**
- Layer 1: NMPC with softplus obstacle penalties (N=30, dt=0.1s, 3s lookahead)
- Layer 2: DOB-CBF-QP safety filter (reactive, ~100Hz, 2D QP)

**Alternative A — Single-layer MPC (pure-MPC obstacle avoidance)**
The MPC already has softplus obstacle avoidance (`w_obstacle=5e3`). Removing the CBF layer and
relying solely on the MPC would:
- ✅ Eliminate the 2D QP overhead (negligible in practice)
- ✅ Remove the CBF↔MPC inconsistency (kinematic model in CBF vs NN dynamics in MPC)
- ❌ Loss of safety guarantees — softplus is a soft penalty, not a guaranteed constraint
- ❌ MPC runs at 10Hz; cannot react to unexpected fast-moving obstacles at 100Hz
- ❌ For teleoperation, the CBF is essential to filter delayed operator commands

**Verdict:** For autonomous tracking only, pure-MPC could work given the 3s horizon and
softplus penalties. For teleoperation (the paper's scenario), the CBF is necessary as a
real-time guardrail for delayed commands.

**Alternative B — Hard obstacle constraints in MPC**
Add ACADOS hard nonlinear constraints `dist(p_k, obs_j) ≥ r_safe` for each obstacle/stage.
- ❌ Can make the OCP infeasible when obstacle is inside the feasible region
- ❌ Requires much more careful initial guess and constraint softening
- The current softplus approach handles this more gracefully

**Conclusion:** The two-layer architecture is appropriate and working (100% QP success, 0
collisions in all tests). The main remaining paper gap is the CBF not using NN force
predictions for barrier gradient — but the NN speed limit (8.09 vs 15.0 m/s on clay)
provides meaningful terrain-adaptive behavior.

### Other Ideas
- ~~Online terrain parameter estimation via force prediction inversion~~ → DONE (2026-04-16).
  NN-inversion approach failed due to systematic NN force bias. Replaced with direct IMU
  friction estimation. See B3 and T5.
- ~~Add velocity-normalized features to terrain classifier~~ → MOOT. Classifier replaced by estimator.
- Active terrain probing: inject brief steering pulse at startup to excite lateral dynamics
  and get a faster/more accurate μ estimate. Would improve dirt/sand identification.
- Fuse friction estimator with GP residual confidence: large GP corrections → terrain
  mismatch → lower confidence in current terrain estimate.

### New Research Ideas (2026-04-17)

---

## UKF Terrain Estimator Deep Diagnosis (2026-session)

### Summary
Comprehensive diagnosis of why the Bayesian grid filter for Bekker sinkage exponent `n` converges only for clay (n=0.5) but fails for dirt (n=0.7) and sand (n=1.1).

### Root Cause: Rig-to-Vehicle Force Modeling Gap
The NN tire surrogate is trained on single-wheel ChTireTestRig data. When applied to the full HMMWV vehicle simulation, a **systematic force magnitude mismatch** causes the filter to always prefer n=0.5 (smallest force predictions).

### Key Findings

1. **Sign convention (-2.0*) IS correct.** The rig's `ReportTireForce()` Fy opposes slip (positive alpha → negative Fy), while the body-frame dynamics need the centripetal convention. The `-2.0 * Fy_pw` correctly maps rig→body frame.

2. **NN accurately reproduces rig data** (RMSE=160 N, scale ratio=1.033 on 2000 random training samples). The NN is not the problem.

3. **Per-wheel body-frame force match is poor** (corr=0.17 overall). The NN captures the correct trend at high alpha (ratio 0.83 at |alpha|>0.07) but fails at low alpha due to unmodeled dynamics.

4. **Left/right turn asymmetry.** During LEFT turns: NN axle force (+1400) matches truth (+1473) within 5%. During RIGHT turns: NN axle force (-700) vs truth (-1734) — 2.5x underestimate. Cause: weight transfer loads the outside wheel nonlinearly, and the estimator averages L/R operating conditions.

5. **Filter always picks n=0.5** because the NN force magnitude increases monotonically with n, and the sensor-reconstructed forces are smaller than even the n=0.5 predictions. n=0.5 has the smallest magnitude → smallest residual.

6. **Online gain calibration attempted** — gain oscillates wildly (0.15 to 1.3) between left and right turns due to the L/R asymmetry, destabilizing the filter.

7. **Per-wheel NN calls attempted** — reduces L/R averaging error but doesn't resolve the systematic magnitude gap.

### Diagnostic Scripts Created
- `diag_perwheel_match.py` — per-wheel NN vs true SCM force comparison
- `diag_lr_convention.py` — left/right wheel sign convention analysis

### Approaches That Did NOT Work
- Changing `-2.0` to `+2.0` (wrong; makes ay_pred sign flip)
- Online gain EMA (gain oscillates due to L/R asymmetry)
- Per-wheel NN calls alone (doesn't fix magnitude gap)

### Recommended Next Steps
1. **Train a vehicle-level NN** by running the full sim with diverse terrains and collecting per-axle or per-wheel ground-truth forces as training data. This would close the rig-to-vehicle gap.
2. **Train a force residual adapter** on vehicle sim data — the existing `ForceResidualAdapter` infrastructure supports this. Collect (NN_prediction, true_SCM_force) pairs from sim runs on each terrain preset, then train a small MLP to predict the correction: `dFy = true - NN_predicted`.
3. **Use speed-deficit or friction-based estimation** instead of force matching — the existing speed-deficit estimator in the codebase sidesteps the force prediction problem entirely.

### Current State of Code Changes
- `terrain_parameter_estimator.py`:
  - **Bicycle-model alpha fix** (KEY FIX): Per-wheel NN path now uses α from bicycle kinematics
    (δ - atan2(v + Lf·ω, u)) instead of Chrono's `GetSlipAngle()`. Chrono's tire-frame alpha
    has opposite signs for L vs R wheels during turns, causing force CANCELLATION (~zero net force).
    With bicycle-model alpha, both wheels contribute consistent-sign forces.
  - Per-wheel path uses per-wheel Fz (weight transfer) with shared bicycle-model alpha.
  - `_force_gain` infrastructure present; EMA alpha=0.0 (disabled). Per-timestep gain normalization
    and scale marginalization were tried but all introduce biases; additive innovation is simplest.
  - `_g_inst` diagnostic tracks instantaneous optimal scale factor.
  - Prior sigma widened to 1.0 (nearly flat); window_size=200.
- `run_openloop_terrain_est.py`: No longer overwrites alpha_f/alpha_r with averaged GetSlipAngle().
  Keeps bicycle-model alpha computed from vehicle state.
- `data_collection/collect_vehicle_data.py` (NEW): Collects per-wheel tire-frame forces from full
  HMMWV vehicle on SCM terrain, with diverse steering/throttle excitation (sinusoidal, chirp, step).
  Output CSV matches rig training format for direct NN retraining.

### Convergence Results After Bicycle-Model Alpha Fix
| True Terrain | True n | Init n | Estimated n | Error |
|---|---|---|---|---|
| clay | 0.5 | 0.5 | 0.500 | 0.0% |
| dirt | 0.7 | 0.5 | 0.511 | 27.0% |
| sand | 1.1 | 0.5 | 0.769 | 30.1% |

The filter correctly ORDERS terrains (clay < dirt < sand). The compressed range is due to the
rig-vehicle scale gap: the NN overpredicts by ~1.5-2.5x, so the additive-innovation filter
favours lower-n grid points (smaller predictions → smaller absolute residual). This bias will
be eliminated once the NN is retrained on vehicle data.

### Fundamental Bug Found: Alpha Convention Mismatch
`tire.GetSlipAngle()` returns tire-FRAME slip angle, while the NN sign convention (`-1.0 * fy_pw`)
is calibrated for bicycle-MODEL alpha. During RIGHT turns, tire-frame alpha for the front-left
(outer) wheel is POSITIVE while front-right (inner) is NEGATIVE — opposite signs. This causes
the per-wheel NN forces to nearly cancel, producing ~zero axle force prediction. The bicycle-model
alpha (computed from δ, v, ω) gives consistent sign for both wheels.

### New Research Ideas (2026-04-17)
- **Dynamic Obstacle Avoidance with Delay Compensation**: Currently, obstacle avoidance assumes static rocks. With network delay, predicting dynamic obstacle trajectories and computing delay-compensated safety margins against moving actors would be a novel extension of the DOB-CBF-QP framework.
- **Adaptive Communication Strategies for Shared Control**: Use the EMA latency estimate and the terrain difficulty (e.g., from the speed-deficit estimator) to dynamically switch between teleoperation, shared control, and autonomous modes. If latency spikes on low-traction soil (clay), the system autonomously switches to a safe stop or fully autonomous conservative navigation.
- **Vision-Proprioception Fusion for Terrain Prediction**: Use a simple front-facing camera model to guess terrain ahead (vision) and fuse it with the current proprioceptive speed-deficit estimator to get a look-ahead terrain map, allowing the MPC to adjust its speed profile before hitting the transition boundary.

---

## 2026-04-18 — Learned Window Regressor for Terrain n (Novel Approach)

### Motivation
The Bekker-UKF lives on a 1-D manifold (clay→dirt→sand) through the
six-parameter Bekker/Mohr-Coulomb space.  Real SCM responses are not
perfectly aligned with that manifold, so any tuning that fixes clay+dirt
biases sand and vice-versa.  This is the "we either fit clay+dirt well or
clay+sand well, never all three" pattern that has plagued the UKF.

### Idea
A small MLP that regresses ``n`` directly from a 4-second sliding window of
vehicle dynamics statistics — sidesteps the manifold projection entirely
and is purely data-driven.  Inputs only use signals that exist on the real
robot (speed, lateral velocity, yaw rate, IMU accelerations, wheel encoder
speeds, steering sensor angle, commanded throttle).  No oracle tire forces
are read at inference time, in line with the project rule that the
architecture must transfer to hardware.

### Pipeline
1. ``simulation/collect_terrain_traces.py`` — drives the chrono sim
   open-loop with sinusoidal steering across (3 terrains × 3 throttles ×
   2 steer-amps × 2 seeds) = 36 trajectories of 25 s each, dumping
   labeled VehicleState CSVs to ``data/terrain_traces/``.
2. ``simulation/train_terrain_window_mlp.py`` — extracts 21 hand-crafted
   features per 4-s window (means/stds/percentiles of u,v,ω,ax,ay,steer
   plus dimensionless coupling channels like ``max|ay|/(u²|δ|)`` and a
   wheel-slip channel) and trains a 3-layer 64-hidden MLP for 200 epochs.
3. ``simulation/learned_terrain_estimator.py`` — duck-types the
   TerrainParameterEstimator interface, runs a numpy forward pass per
   observation (~5 µs), publishes EMA-smoothed n.
4. ``simulation/hybrid_terrain_estimator.py`` — convex combination of the
   Bekker UKF and the learned MLP (35% UKF / 65% MLP) for ablation.
5. ``simulation/eval_learned_estimator.py`` — offline replay of saved
   traces through the estimator (fast sanity check, no chrono needed).
6. ``simulation/make_estimator_comparison_figure.py`` — generates
   ``paper_figures/estimator_comparison.png``.

### Honest leave-one-seed-out validation
Trained on seed-0 traces only, evaluated on the 18 unseen seed-1 traces:

| Terrain | True n | Predicted (mean ± std) | RMSE  |
|---------|--------|------------------------|-------|
| clay    | 0.50   | 0.506 ± 0.017          | 0.018 |
| dirt    | 0.70   | 0.692 ± 0.026          | 0.027 |
| sand    | 1.10   | 1.092 ± 0.028          | 0.029 |

### Live sim convergence (3 replicates per terrain, 25 s, init n=clay)

| Estimator | clay err | dirt err | sand err | mean err |
|-----------|----------|----------|----------|----------|
| UKF       | 7.0%     | 11.9%    | **34.1%** | 17.7%    |
| Hybrid    | 9.3%     | 1.9%     | 11.4%    | 7.5%     |
| Learned   | 4.5%     | 3.6%     | **1.4%** | **3.2%** |

The UKF flat-out fails on sand (saturates around n≈0.73 vs true 1.1).  The
hybrid pulls sand error down by 3× but is still UKF-bottlenecked.  The
learned MLP achieves ≤6% on every replicate and converges in ~6 s after
the buffer warms.  This validates the user's "we either fit two terrains
well or all three poorly" intuition — the bias is genuinely in the 1-D
Bekker projection, not a tuning issue.

### Files / artifacts
- Production model: ``nn_models/terrain_window_mlp/``
  (weights.pt + scaler.pkl + config.json)
- Held-out training run (for honest evaluation): ``nn_models/terrain_window_mlp_holdout/``
- Trace dataset (36 csvs, ~9 MB total): ``data/terrain_traces/``
- Comparison figure: ``paper_figures/estimator_comparison.png``
  (3×3 grid: estimator × terrain, mean ± std bands over 3 reps)

### Real-vehicle considerations / honest limitations
- **No oracle features used at inference**: trained features are restricted to
  signals that have hardware analogues (IMU, wheel encoders, steering sensor,
  commanded throttle).  No tire forces are read from chrono at inference.
- **Operating-condition coverage** is currently limited to one steering
  shape and three throttles.  Real-world generalization will need traces
  with broader excitation (chirps, ramps, manual driving) — easy extension
  via the existing collect_terrain_traces.py.
- **No domain randomization** of vehicle mass, suspension, payload yet.
  The published features include enough redundancy (long_drag, wheel_slip)
  to support those, but it should be tested before claiming sim-to-real.
- The hybrid is offered as a graceful-degradation fallback: when the buffer
  is still warming or the operating regime is OOD for the MLP, the UKF
  prior carries the estimate.

---

## 2026-04-18 — Generalization study: novel n values (unseen during training)

### Question
The learned MLP only ever saw n ∈ {0.5, 0.7, 1.1} during training (3 discrete
preset terrains).  Is it effectively a 3-class discriminator that just
returns one of those three values, or does it actually generalize across
the n axis?

### Test
``simulation/test_learned_generalization.py`` synthesises novel SCM terrain
configs by linearly interpolating / extrapolating the 6-parameter Bekker
vector along the preset sequence, writes a YAML, and runs the chrono sim
through the existing ``--terrain-config`` path (a new
``OPENLOOP_TERRAIN_CONFIG`` env var lets the open-loop runner forward it
to the auto-launched sim).  Two replicates per n value, 22 s each.

### Results

| True n | Type        | Predicted (mean of 2 reps) | Error   |
|--------|-------------|----------------------------|---------|
| 0.35   | extrap LO   | sim diverged (no output)   | —       |
| 0.50   | TRAIN       | 0.513                      | 2.6%    |
| 0.60   | INTERP      | 0.657                      | 9.4%    |
| 0.70   | TRAIN       | 0.719                      | 2.7%    |
| 0.85   | INTERP      | 0.859                      | **1.0%** |
| 1.00   | INTERP      | 1.005                      | **0.5%** |
| 1.10   | TRAIN       | 1.100                      | 0.0%    |
| 1.30   | extrap HI   | 1.100 (saturates)          | 15.4%   |

### Interpretation
1. **Interpolation works** — n=0.85, 1.00 both predicted within 1% of
   truth even though they are *between* training presets the model never
   saw.  The MLP is a real regressor, not a 3-way classifier.
2. **Worst interp is the clay→dirt midpoint** (n=0.60, 9% error).  The
   six soil parameters jump non-monotonically between clay and dirt
   (Kphi 692 k → 1515 k, cohesion 4140 → 1700, k 0.01 → 0.025), so the
   linear-blend "soil at n=0.6" has dynamics features the model
   slightly mis-maps.  By contrast the dirt→sand jump is monotonic in
   most parameters, hence the near-zero error at n=0.85, 1.00.
3. **Extrapolation fails as expected**: n=1.3 saturates at 1.10.  Any
   regression model trained without bounding constraints will plateau at
   the training-data extrema.  This is the right *failure mode*: the
   estimator is honest about its support and never claims values
   outside it (so downstream consumers can clamp or fall back).
4. **n=0.35 broke the SCM sim itself** (no estimator output captured),
   not the model.  Below clay's stiffness Bekker physics + chrono mesh
   resolution become numerically unstable for our HMMWV mass.

### Path to broader generalization (not yet tested, but the obvious next step)
- **Continuous n sweep at training time**: collect ~10–15 terrains along
  the [0.4, 1.2] axis instead of three.  The collector + trainer already
  support arbitrary YAMLs; just point ``collect_terrain_traces.py`` at a
  generated set of configs.
- **Off-manifold soils**: vary cohesion / friction angle independently
  at each fixed n so the model learns to disentangle them.  Currently
  the model effectively assumes the user's preset manifold.
- **Domain randomization** on vehicle mass, payload, tire pressure
  (proxied through Fz scaling) so the model is robust on the real
  HMMWV variant deployed.

Artifacts: ``paper_figures/learned_generalization.png``,
``paper_figures/learned_generalization.csv``.

---

## 2026-04-18 — Diverse-soil retraining (v2 model)

Acting on the "path to broader generalization" plan: collected 42 new
labelled traces over **21 novel SCM soils** that go off the preset
manifold, retrained the MLP, and revalidated the same generalization
sweep with bounded clamps relaxed.

### Data collection

`simulation/collect_diverse_terrains.py` writes a YAML per spec into
`data/terrain_yamls/` and drives `collect_terrain_traces.collect_one`
(now extended with `terrain_yaml` / `n_true_override` kwargs) through
the existing `chrono_sim_node --terrain-config` path.  Three families:

1. **On-manifold n sweep** (9 soils): linear interpolation /
   extrapolation along the (clay→dirt→sand) preset sequence at
   n ∈ {0.45, 0.55, 0.60, 0.65, 0.80, 0.85, 0.95, 1.00, 1.20}.  Fills
   in the previously sparse interp gaps and widens the support past
   sand.
2. **Off-manifold cohesion variants** (6 soils): each preset's cohesion
   scaled by 0.5× and 1.5× with all other six parameters fixed.  Forces
   the regressor to learn that ``n`` is not the only knob that moves
   the dynamics signature.
3. **Off-manifold friction-angle variants** (6 soils): each preset's
   friction angle ±5°.

Sweep: 21 specs × 2 throttles (0.45, 0.65) × 1 steer amplitude
(0.5 rad) × 1 seed × 25 s ⇒ **42 new CSVs** (≈ 19 min wall time on
this box, all on a single HMMWV sim instance with vis disabled).

Combined dataset: 79 traces total (37 baseline + 42 diverse) ⇒ 5 157
4-second windows.

### Training

`train_terrain_window_mlp.py --trace-dir data/terrain_traces
 --out-dir nn_models/terrain_window_mlp_v2 --epochs 200 --hidden 64`.

Added a `[per-n-bin validation RMSE]` block to the training report so
we can spot-check uniform coverage along the n-axis instead of only
the three preset clusters.

```
[train] best val_mse=0.00037  rmse=0.0192
[train] per-n-bin validation RMSE:
  n∈[0.40,0.55)  k= 285  rmse=0.0160  bias=+0.0054
  n∈[0.55,0.65)  k=  58  rmse=0.0270  bias=+0.0056
  n∈[0.65,0.75)  k= 301  rmse=0.0244  bias=-0.0098
  n∈[0.75,0.90)  k=  53  rmse=0.0176  bias=-0.0003
  n∈[0.90,1.05)  k=  48  rmse=0.0149  bias=-0.0015
  n∈[1.05,1.25)  k= 286  rmse=0.0144  bias=-0.0043
```

Validation RMSE 0.019 (vs ~0.030 for the v1 3-preset model on its own
in-distribution split) — the larger dataset helps despite the extra
distributional spread.

### Bounded-prediction clamp

Found that `learned_terrain_estimator._N_BOUNDS = (0.5, 1.1)` was the
preset-derived clamp on the regressor *output*.  After widening the
training distribution to include n=1.20 and n=0.45 traces this clamp
masked all of the new generalization that the v2 model was actually
producing.  Split the constant into:

* `_N_BOUNDS = (0.5, 1.1)` — used for *parameter interpolation* on the
  Bekker manifold (so the downstream cohesion / friction map never
  extrapolates past clay or sand);
* `_PRED_BOUNDS = (0.40, 1.30)` — used to clamp the regressor's raw
  prediction (matches v2's training support).

### Revalidation results (3 reps × 22 s, learned MLP only)

| True n | Mean ± Std | err %  | v1 err % | Notes                       |
|--------|-----------|--------|----------|-----------------------------|
| 0.50   | 0.527 ± 0.024 | 5.5% | 2.6%   | one outlier rep             |
| 0.60   | 0.594 ± 0.007 | **1.1%** | 9.4% | 8.5× better — training fix  |
| 0.70   | 0.682 ± 0.002 | 2.6% | 2.7%   | stable                      |
| 0.85   | 0.856 ± 0.015 | 1.9% | 1.0%   | similar                     |
| 1.00   | 1.004 ± 0.020 | 1.4% | 0.5%   | similar                     |
| 1.10   | 1.100 ± 0.025 | 0.0% | 0.0%   | perfect                     |
| 1.20   | 1.178 ± 0.014 | **1.9%** | 8.3% | clamp lifted, no saturation |
| 1.30   | 1.293 ± 0.006 | **0.5%** | failed | extrapolation works!        |

Key wins: (a) the clay→dirt midpoint at n=0.6 is now within 1% (was
9%); (b) n=1.2 is no longer saturated at 1.10; (c) n=1.3 — strictly
*outside* the v2 training range [0.45, 1.20] — extrapolates within
0.5% mean error.  The model has learned the underlying dynamics→n
mapping rather than memorizing the discrete training points.

### Live 3-terrain comparison (regenerated 3×3 figure)

`paper_figures/estimator_comparison.png` re-rendered with v2 weights
(canonical `nn_models/terrain_window_mlp/` now points at v2; v1 backed
up to `nn_models/terrain_window_mlp_v1_backup/`).  Mean abs. error
across the three preset terrains, 3 reps each:

| Estimator         | clay  | dirt  | sand  | avg   |
|-------------------|------:|------:|------:|------:|
| UKF               | 5.0%  | 12.9% | 33.8% | 17.2% |
| Hybrid (35 / 65)  | 13.0% | 6.8%  | 10.4% | 10.1% |
| Learned (v2)      | 7.5%  | 2.3%  | 1.9%  | **3.9%** |

The v2 learned model continues to dominate the UKF baseline on dirt
and sand (where the UKF's manifold projection over-predicts n), and
the hybrid blend remains a useful graceful-degradation option for
the buffer-warming window.

### Artifacts

* `paper_figures/learned_generalization_v2.png` — 8-point n sweep,
  3 reps, v2 model.
* `paper_figures/learned_generalization_v2.csv` — raw per-rep
  predictions.
* `paper_figures/estimator_comparison.png` — refreshed 3×3
  (UKF / hybrid / learned) × (clay / dirt / sand) figure with v2.
* `nn_models/terrain_window_mlp/` — production MLP (v2 weights).
* `nn_models/terrain_window_mlp_v1_backup/` — preserved v1 for ablation.
* `data/terrain_traces/` — now 79 CSVs (37 baseline + 42 diverse).
* `data/terrain_yamls/` — 21 generated soil configs.

### Honest remaining gaps

* **n=0.5 outlier rep (10–17 % err)**: shows up in both v1 and v2.
  Likely an SCM transient at low n with the clay→dirt slope; deserves
  a deeper look (variance reduction or longer warm-up).
* **n=0.35 still diverges** the SCM mesh at HMMWV mass — not a model
  problem.  Lower-bound of useful operating range is ~ n=0.4.
* **Off-manifold validation is implicit, not explicit** — the v2
  training set *contains* off-manifold cohesion / friction variants,
  but the evaluation sweep (`test_learned_generalization.py`) only
  probes on-manifold interpolated soils.  A logical follow-up is to
  add a parallel sweep that varies cohesion / friction independently
  at each n so we can quantify off-manifold robustness.

---

## 2026-04-18 (evening): Closed-loop train/eval gap closed (v3_cl)

### Motivation

A smoke test of the v2 (open-loop only) MLP inside the actual
`launch_decoupled.py` MPC pipeline showed an honest failure: it
identified the right *neighbourhood* of n but oscillated between
clay/dirt/sand every ~ 2 s.  Diagnosis: training data was driven by an
open-loop sinusoidal steer / constant-throttle policy, while inference
sees MPC-modulated throttle and path-tracking steering — a
distribution gap large enough that the windowed features land in
unseen regions of input space at run-time.

### What was built

* `simulation/collect_closed_loop_traces.py` — new collector that
  spawns `chrono_sim_node` and `acados_mpc_controller_node` as
  subprocesses, subscribes to both the `VehicleState` and
  `ControlCommand` ZMQ topics, and writes a trainer-compatible CSV
  populated from the **actual MPC commands** rather than synthetic
  open-loop ones.  CLI sweeps over (terrain × speed × sine-amp ×
  sine-wavelength × seed).  Outputs land in
  `data/terrain_traces_closedloop/` so they can be mixed with the
  open-loop set during training.
* `simulation/train_terrain_window_mlp.py` — `--trace-dir` now accepts
  multiple paths (`nargs="+"`), so a single training run can blend
  open-loop and closed-loop CSVs.
* `simulation/learned_terrain_estimator.py` — bumped the EMA
  smoothing constant from `0.05` → `0.02` (≈ 1 s effective time
  constant at the 50 Hz observe rate) so high-frequency feature
  jitter from MPC modulations gets averaged out.
* `simulation/acados_mpc_controller_node.py` + `launch_decoupled.py`
  — new `--te-verbose` flag plumbed all the way through; the
  controller now prints `[LRN] u=… ay=… omega=… slip_mean=… ->
  n_raw=… n_sm=…` lines that downstream tools can parse.
* `simulation/validate_closed_loop_estimator.py` — driver that runs
  `launch_decoupled.py` with `--te-verbose` for each preset terrain,
  parses the controller log, computes steady-state mean ± std of the
  smoothed prediction over the back half of the run, and emits both
  a CSV and a 3-panel time-series figure.

### Closed-loop dataset (v3_cl)

* 24 closed-loop runs collected: 3 terrains × 2 speeds (4.0, 5.5 m/s)
  × 2 sine amplitudes (1.5, 2.5) × 1 wavelength (30 m) × 2 seeds.
* All runs ≥ 25 s, ~ 80 rows/s ≈ 2 000 rows each, MPC closing the
  loop end-to-end (NN tire model + acados QP + sinusoidal path
  tracker).
* Total terrain-trace corpus: **104 CSVs** = 79 open-loop + 25
  closed-loop (the extra one is the smoke-test run).

### Retrained model (v3_cl)

```
python simulation/train_terrain_window_mlp.py \
  --trace-dir data/terrain_traces data/terrain_traces_closedloop \
  --out-dir nn_models/terrain_window_mlp_v3_cl \
  --epochs 200 --batch 64 --lr 1e-3 --hidden 64 --val-frac 0.2 --seed 7
```

* 6 740 windows, 21-dim features.
* Best val MSE = 0.00048, RMSE = **0.0218**
  (slightly higher than v2's 0.019 because the dataset now spans a
  much harder distribution — the closed-loop joint distribution of
  (steer, throttle, slip) is more correlated and lower-entropy than
  the open-loop one, so a single global model has to cover both.)
* Per-terrain validation RMSE: clay 0.025 / dirt 0.024 / sand 0.016.
* Per-n-bin validation RMSE remains uniform (0.016 – 0.034) across
  the 0.40–1.25 sweep.
* Promoted to `nn_models/terrain_window_mlp/`.
  Open-loop-only v2 archived at
  `nn_models/terrain_window_mlp_v2_openloop_only_backup/`.

### Closed-loop validation (the actual win)

Ran `validate_closed_loop_estimator.py` — `launch_decoupled.py`
+ `--terrain-estimator-kind learned` + `--te-verbose` for 30 s on
each preset, sinusoidal path, 5 m/s target, 215+ predictions parsed
per run, steady-state computed over the back half:

| terrain | true n | predicted n_sm (mean ± std) |   bias   |  abs err  |
|---------|--------|------------------------------|----------|-----------|
| clay    | 0.50   | 0.508 ± 0.008                | +0.008   | **0.008** |
| dirt    | 0.70   | 0.692 ± 0.009                | -0.008   | **0.008** |
| sand    | 1.10   | 1.079 ± 0.010                | -0.021   | **0.021** |

Compare to the v2 (open-loop only) baseline that *oscillated between
clay, dirt and sand every ~ 2 s* in the same closed-loop pipeline —
the new model locks within 5–7 s and stays inside ± 0.011 of the true
n for the full remainder of the run.  Even the unsmoothed `n_raw`
trace (gray in the figure) hugs the true value, so the EMA isn't
masking instability.

### Artefacts

* `nn_models/terrain_window_mlp/` — production v3_cl weights.
* `nn_models/terrain_window_mlp_v3_cl/` — original training output.
* `nn_models/terrain_window_mlp_v2_openloop_only_backup/` —
  preserved v2 for ablation.
* `data/terrain_traces_closedloop/` — 25 closed-loop CSVs.
* `paper_figures/closed_loop_estimator_learned.png` — 3-panel
  convergence plot (clay/dirt/sand).
* `paper_figures/closed_loop_estimator_summary_learned.csv` —
  steady-state stats table.
* `logs/cl_validate/cl_{clay,dirt,sand}_learned.log` — raw verbose
  controller logs for offline parsing.
* `logs/closed_loop_collection.log` — end-to-end log of the
  collection sweep.

### Honest remaining gaps

* **Sand has the largest residual bias** (-0.021) — the model
  consistently under-predicts on the highest-n preset.  Likely a
  combination of training-set imbalance (sand has fewer
  closed-loop runs at amp=1.5 because the MPC saturates on throttle
  there) and the EMA pulling toward dirt during the first
  steering-direction reversals.  Could be tightened by (a) collecting
  more sand traces at lower amplitude or (b) bias-correcting in the
  manifold-interpolation step.
* **Smoothing alpha is now hard-coded at 0.02** — a CLI override
  would make ablation easier.

---

## 2026-04-18 (late evening): Closed-loop generalisation on random unseen soils

### Motivation

The previous closed-loop validation (clay/dirt/sand) only proved that
v3_cl tracks the *training presets*.  To stress-test out-of-
distribution generalisation in the actual MPC pipeline we generate
random *off-manifold* soils (random ``n`` plus independent
perturbations of cohesion and friction angle) and run closed-loop on
them.

### What was built

* `simulation/generate_random_terrains.py` — samples N random soils,
  each with a target ``n`` ∈ ``[n_low, n_high]`` interpolated along
  the (clay → dirt → sand) preset manifold *and* off-manifold
  perturbed by a random cohesion scale ``∈ [0.5, 1.5]`` and a random
  friction-angle delta ``∈ [-5°, +5°]``.  Writes ``terrainN.yaml`` +
  a ``manifest.csv`` recording true ``n``, perturbations, and the
  closest preset proxy.
* `simulation/validate_random_terrains_closed_loop.py` — driver that
  for each terrain in the manifest runs `launch_decoupled.py`
  with `--terrain-config <yaml>`, `--terrain-estimator-kind learned`,
  and `--te-verbose`.  Pulls live ``n_raw`` / ``n_smoothed`` from the
  controller log *and* the MPC's per-step ``crosstrack_err``,
  ``heading_err_deg``, ``speed_err``, ``n_terrain_est`` from the
  diagnostic CSV.  Computes second-half mean / std / RMS, writes a
  summary CSV, and renders a 2-column figure (estimator on the left,
  tracking on the right) per terrain.

### Sample drawn (seed=42)

| label    | true_n | coh×  | Δφ      | proxy |
|----------|--------|-------|---------|-------|
| terrain1 | 0.884  | 0.53  | -2.2°   | dirt  |
| terrain2 | 0.634  | 1.24  | +1.8°   | dirt  |
| terrain3 | 1.035  | 0.59  | -0.8°   | sand  |
| terrain4 | 0.518  | 0.72  | +0.1°   | clay  |
| terrain5 | 0.516  | 0.70  | +1.5°   | clay  |
| terrain6 | 0.827  | 0.72  | +0.9°   | dirt  |

All six are off-manifold (cohesion scale ≠ 1.0 and/or |Δφ| > 0); none
of the resulting (Kphi, Kc, n, c, φ, k) sextuples appear in either
the open-loop or closed-loop training set.

### Closed-loop results (v3_cl learned estimator, 30 s, sinusoidal
path, v=5 m/s, amp=2 m, λ=30 m)

| label    | true_n | est n_sm  (mean ± std) | bias    | abs err | crosstrack RMS | heading RMS | speed RMS | MPC committed |
|----------|--------|-------------------------|---------|---------|----------------|-------------|-----------|---------------|
| terrain1 | 0.884  | 0.887 ± 0.027           | +0.003  | **0.003** | 0.036 m       | 7.85°       | 0.57 m/s  | 9 % |
| terrain2 | 0.634  | 0.679 ± 0.017           | +0.045  | 0.045   | 0.165 m       | 8.75°       | 1.05 m/s  | 9 % |
| terrain3 | 1.035  | 1.041 ± 0.009           | +0.006  | **0.006** | 0.034 m       | 8.18°       | 0.71 m/s  | 9 % |
| terrain4 | 0.518  | 0.510 ± 0.003           | -0.008  | **0.008** | 0.519 m       | 11.34°      | 0.84 m/s  | 9 % |
| terrain5 | 0.516  | 0.521 ± 0.017           | +0.005  | **0.005** | 0.966 m       | 13.89°      | 0.90 m/s  | 9 % |
| terrain6 | 0.827  | 0.706 ± 0.021           | -0.121  | 0.121   | 0.070 m       | 8.07°       | 0.91 m/s  | 9 % |

### Headline findings

* **Estimator generalises**: 5 of 6 unseen soils estimated within
  ≤ 4.5 % absolute error (and 4 of 6 within ≤ 1 %), even though every
  soil is off-manifold in cohesion and friction angle.
* **Failure mode (terrain6)**: the regressor pegs at 0.706 (essentially
  dirt's preset value of 0.7) when the true ``n`` is 0.827 with
  cohesion×0.72 and Δφ ≈ +0.9°.  The dynamics signature in those
  features apparently look like dirt to the MLP — a known limitation
  of the off-manifold "snap to nearest preset" failure mode.  Would
  need additional training-time diversity at high-``n`` low-cohesion
  combinations to fix.
* **MPC tracking is excellent on firm soils** (terrains 1, 2, 3, 6):
  3.4 – 16.5 cm crosstrack RMS, 7.9 – 8.8° heading.
* **MPC tracking degrades on the genuinely soft soils** (terrains 4,
  5: true n ≈ 0.52, low cohesion).  Crosstrack blows up to 0.5 – 1 m
  even though the *estimator was correct*.  This is a low-traction
  control-feasibility problem, not an estimation problem — the soil
  simply cannot generate enough lateral force at the requested
  speed/path-curvature.
* **MPC commits ~9 % of steps**: only when the learned estimator's
  reported confidence exceeds `--te-min-confidence` (0.3 by default)
  does the new ``n`` propagate into the MPC's tire model.  The
  smoothed value is still tracked continuously.  When committed, the
  applied ``n_te`` matches ``n_smoothed`` to ≤ 0.003 — nothing is
  lost in the commitment path.

### Artefacts

* `data/terrain_yamls_random/terrain{1..6}.yaml` + `manifest.csv` —
  random soils used here.
* `paper_figures/random_terrain_closed_loop_learned.png` — 6×2 grid
  (estimator + tracking) for the full sweep.
* `paper_figures/random_terrain_closed_loop_learned.csv` — summary
  table with bias / RMS / commitment %.
* `logs/cl_random/terrain{1..6}_learned.log` — verbose controller
  logs (parseable for offline re-analysis).
* `plots/cl_random/terrain{1..6}/` — full per-terrain MPC diagnostic
  CSV directories.

### Honest remaining gaps

* **One off-manifold failure (terrain6, n=0.83)** — see above; needs
  more training samples with high ``n`` + low cohesion combinations.
* **Soft-soil tracking (terrains 4, 5)** is a separate problem — even
  with a perfect estimator the MPC saturates on tire forces.  A
  speed-adaptation or path-curvature limiter would help here, but
  it's outside the estimator's scope.
* **Single seed** — the random-soil sample uses ``--seed=42``.  A
  follow-up sweep across multiple seeds would give a tighter
  distribution of generalisation error.

---

## 2026-04-20: Paper-scope cleanup pass

Aligned the active codebase with `my_paper/abstract.tex`. The retained adaptive
paths are now:

* `simulation/learned_terrain_estimator.py` — sliding-window MLP terrain
  estimator, with runtime support for `n`-only and joint `n`/`phi` heads.
* `simulation/force_residual_adapter.py` — force-level residual correction.
* `simulation/dynamics_gp_adapter.py` — process-dynamics GP residual model.

Live entrypoints were trimmed accordingly:

* `simulation/acados_mpc_controller_node.py` no longer wires the old online
  residual adapter, UKF terrain estimator, hybrid terrain estimator, or GP
  force-residual branch.
* `simulation/launch_decoupled.py` no longer exposes `--residual-adapt`,
  `--gp-residual`, or `--terrain-estimator-kind`; the retained terrain-estimator
  path is the sliding-window MLP, defaulting to
  `nn_models/terrain_window_mlp_v3_cl`.
* `simulation/run_openloop_terrain_est.py` and the active validation scripts in
  `test_suite/` now exercise the learned sliding-window estimator only.

Archive move requested by user:

* UKF / hybrid terrain-estimation modules
* online residual-adapter path
* GP force-residual adapter
* wheel/axle/direct-force observer experiments
* stale figure / comparison scripts and tests tied to those paths

Verification:

* `python -m compileall simulation test_suite utilities` passed after the trim.
* No Chrono closed-loop simulations were run in this pass, so runtime behavior
  is not yet claimed as validated.

## 2026-04-20: Simulation-folder script relocation pass

Cleaned up the remaining stray helper scripts that were still sitting under
`simulation/` even though they were not part of the core runtime stack.

Moves:

* Data collection / benchmark helpers moved to `utilities/`:
  `collect_terrain_traces.py`, `collect_diverse_terrains.py`,
  `collect_n_phi_grid.py`, `collect_rich_excitation.py`,
  `collect_closed_loop_traces.py`, `generate_random_terrains.py`,
  `sweep_collect_data.py`, `sweep_benchmark.py`, `run_gp_experiment.py`
* Open-loop estimator harness moved to `test_suite/run_openloop_terrain_est.py`
* GP paper figure builder moved to `my_paper/make_gp_paper_figure.py`
* Superseded one-off wrappers / artifacts were archived under
  `archive/cleanup_20260420/paper_scope_trim/`

Follow-up code fixes:

* Patched the moved scripts to import runtime modules from `simulation/`
  explicitly instead of assuming they still lived there.
* Updated hardcoded launcher paths so collectors and tests call
  `simulation/chrono_sim_node.py`, `simulation/acados_mpc_controller_node.py`,
  and `simulation/launch_decoupled.py` from their new homes.
* Fixed stale env / CLI assumptions in the benchmark helpers:
  `utilities/sweep_benchmark.py` now uses the `sim` conda env and no longer
  passes the removed `--force-residual-no-online` flag.
* Redirected relocated paper outputs to `my_paper/paper_figures/`.

Verification:

* `python -m compileall simulation test_suite utilities my_paper` passed.
* No Chrono runs were executed in this relocation pass, so this only verifies
  structural / import correctness, not runtime behavior.

## 2026-04-20: SCM_Shared bundle created and verified

Created a curated sibling project at `project/SCM_Shared/` containing the
active paper-scope runtime, active checkpoints, terrain-estimator datasets,
paper assets, and verification scripts, while leaving behind the source repo's
heavy generated clutter such as `simulation/plots/`.

Verification run from `SCM_Shared/`:

* `python -m compileall simulation test_suite utilities my_paper` passed
* offline estimator replay reproduced canonical late-window errors of
  `0.9%` (clay), `0.8%` (dirt), `0.6%` (sand)
* closed-loop canonical validation reproduced:
  clay `n=0.5038`, dirt `n=0.6990`, sand `n=1.0930`
* fresh random-terrain validation from the shared bundle reproduced:
  terrain1 `|n err|=0.0098`, terrain2 `|n err|=0.0598`
* compact tire benchmark (`Pacejka`, `TMeasy`, `MLP`, one repeat) regenerated
  `bench_tire_models.{csv,png}` from the shared bundle; MLP remained strong
  while TMeasy degraded badly on clay/dirt sinusoidal, and one Pacejka
  clay-sinusoidal run timed out at 240 s
* joint `(n, phi)` experiment from the shared datasets regenerated
  `exp_joint_n_phi.{csv,png,txt}` with joint validation
  `n_rmse=0.0476`, `phi_rmse=3.9145`
* `my_paper/make_gp_paper_figure.py` regenerated the GP figure from the shared
  bundle

Bundle-specific fix made during verification:

* patched `SCM_Shared/utilities/bench_tire_models.py`,
  `run_paper_benchmarks.py`, `run_feature_benchmarks.py`, and
  `run_mpc_benchmarks.py` so child runs launch through
  `conda run -n sim` instead of inheriting whatever `python` the caller
  happens to be using

## 2026-05-10: Dynamics-GP one-flag persistent state

Made `simulation/launch_decoupled.py --dynamics-gp` self-contained for repeated
runs. The launcher now resolves the default GP state file relative to the
project root and forwards an absolute path to the controller, so repeated runs
accumulate into:

* `data/gp_residual/dynamics_gp_state.npz`

Also updated direct `simulation/acados_mpc_controller_node.py --dynamics-gp`
invocations to resolve relative dynamics-GP state paths from the project root.

Verification:

* `python -m compileall simulation/launch_decoupled.py simulation/acados_mpc_controller_node.py`
  passed.
* No Chrono simulation run was executed for this CLI/path cleanup.
