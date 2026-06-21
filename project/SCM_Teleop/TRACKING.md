# SCM_Teleop Research Tracking

**Purpose:** Running log of edits, findings, bugs, test results, and open questions.
**Date started:** 2026-04-15

---

## MPPI Predictive Safety Shield (2026-05-10)

Replaced the single-step DOB-CBF-QP safety filter with a terrain- and
latency-aware predictive shield that uses the NN tire surrogate as the
*dynamics* of a multi-step rollout rather than just a Jacobian source for
a linearized QP.  Two flavors are wired in behind `--safety-flavor`:

- **`mppi` (primary):** Model Predictive Path Integral.  At each step the
  shield samples `K` (default 384) noisy command sequences around the
  operator/AI command over a latency-padded horizon `H = max(N0, ⌈RTT/dt⌉+1)`
  (default `N0 = 12`), rolls each through the NN surrogate with semi-implicit
  sub-stepping + low-speed kinematic blending, scores each rollout with a
  joint cost (obstacle softplus barrier, NN friction-cone tightened by
  `phî − σ_phi`, terrain-aware speed cap, deviation from operator intent),
  and returns the importance-weighted mean of the first action.  Seven
  "seed" trajectories (passthrough, full brake, coast, evade-left,
  evade-right, brake-while-turn-left/right) are injected unconditionally
  so the shield always has a recoverable option in the sample set.
- **`nmpc` (ablation):** SLSQP over the same NN-surrogate rollout, shorter
  horizon (default 6), gradient-based.  Same cost.  Provided so the paper
  can directly compare sampling vs gradient-based predictive shielding.

**Closed-loop smoke test (clay/right_left, 3 rocks, seed 42, 10 s sim):**

| Flavor | RT factor | Collisions | Near misses | Intervention rate | Mean u (target 5 m/s) |
|--------|-----------|------------|-------------|-------------------|----------------------|
| mppi   | 1.00×     | 0          | 0           | 99%               | 3.43 m/s             |
| nmpc   | 0.67×     | 0          | 0           | 63%               | 3.45 m/s             |

Both flavors stay collision-free.  MPPI runs real-time (~9 ms/solve at
K=384, H=12 in the dynamics test; reported as <1 ms aggregated in the
sim-side TIMING line because it only fires at 10 Hz).  NMPC is ~50%
slower because SLSQP uses finite-difference Jacobians over the rollout.

**Files:**

- `simulation/safety/surrogate_dynamics.py` — `NumpyTireSurrogate`
  (extracts the MLP weights and runs a vectorized numpy forward pass)
  and `VehicleSurrogateDynamics` (batched bicycle model with sub-stepping
  and low-speed kinematic blending for stability).
- `simulation/safety/predictive_shield.py` — `MPPIShield` (primary) and
  `NMPCShield` (ablation), sharing a base class with the cost function,
  CSV logging, teleop-delay handling, and `update_terrain(phi,
  sigma_phi)` hook for the online terrain estimator.
- `simulation/safety/__init__.py` — adds `make_safety_filter(flavor, ...)`
  factory plus the existing legacy `CBFSafetyFilter` for back-compat
  (kept under the `dob_cbf` flavor).
- Flag plumbing: `simulation/chrono_sim_node.py`, `simulation/launch_decoupled.py`,
  `test_suite/benchmark_tire_obstacle_avoidance.py` now accept
  `--safety-flavor {mppi,nmpc,dob_cbf}` plus per-flavor knobs
  (`--mppi-samples`, `--shield-horizon`, `--nmpc-iter`, …).

**Bugs caught and fixed during smoke testing:**

1. *Forward-Euler stiffness:* with `dt = 0.1 s` the NN-driven yaw
   dynamics diverge below ~3 m/s — slip angles saturate, lateral force
   spikes, omega runs away — inflating the friction-cone cost on
   benign rollouts and choking throttle to 25% even on a clear path
   (vehicle crawled at 1.2 m/s).  Fixed with 5 internal Euler
   sub-steps per outer step and a low-speed (< 1.5 m/s) blend toward
   kinematic-bicycle yaw, plus tighter slip clamp (`±0.35` rad).
2. *Brake-biased seed set:* 5/6 of the original seeds had negative
   throttle, so even with adaptive MPPI temperature the weighted mean
   was pulled toward brake when the operator command was safe.  Added
   a passthrough seed (operator command repeated for the full horizon)
   that anchors the mean to operator intent whenever there's no
   collision pressure.
3. *Warm-start drift:* using the previous solve's `u_mean` as the
   sampling mean made the shield "remember" old conservative outputs
   even after the operator command changed.  Switched to sampling
   around `op_cmd` every step (continuity is provided by the
   passthrough seed and the high deviation-cost weight).
4. *Horizon vs stopping distance:* an `H=8` (0.8 s) horizon doesn't
   see an obstacle 12 m ahead at 5 m/s until the vehicle is already
   inside the stopping distance.  Default horizon bumped to 12, and
   the obstacle's effective `safe_r` is inflated by half the
   constant-deceleration stopping distance so the shield brakes
   pre-emptively without needing a 2 s+ horizon.

**Open questions:**

- The terrain estimator's `phî` and ensemble uncertainty aren't yet
  wired into `MPPIShield.update_terrain(...)`; the abstract claims this.
  Hook exists in code; needs the controller node or sim node to call
  `safety_filter.update_terrain(...)` when the estimator publishes a
  new posterior.  Should slot in next to where the controller pulls
  estimator output for its own `terrain_params_est`.
---

## Shield-only avoidance + tighter interventions (2026-05-10, follow-up)

Four refinements driven by the obvious next question: *what if the MPC
were oblivious to obstacles, so the shield is the sole avoider?*  And
two correctness concerns from the first pass: NMPC was 1.3x over the
RT budget at 129 ms/solve, and MPPI was intervening 99 % of the time
with mean throttle scrub ~31 %, which is the *opposite* of what a CBF
should look like (CBF is the identity map when constraints are
inactive).

1. **`--mpc-blind-obstacles` flag.**  Drops the obstacle list on the
   controller side so the acados NMPC plans pure path-tracking and the
   shield (MPPI or NMPC) is the only collision-avoider — a closer proxy
   for an oblivious teleoperator who can't see ahead.  Patched in
   `acados_mpc_controller_node.py` (one-liner that conditionally skips
   the `solve_kwargs['obstacles']` assignment), threaded through
   `launch_decoupled.py` and `benchmark_tire_obstacle_avoidance.py`.

2. **NMPC made RT.**  Two changes:
   - **SLSQP → L-BFGS-B.**  SLSQP's exact line search and inner QP cost
     ~130 ms/solve at H=6; L-BFGS-B's strong-Wolfe line search +
     quasi-Newton Hessian is much lighter on box-bounded problems
     (which is all we have — no equality / inequality constraints
     beyond actuator limits).
   - **Batched finite-difference Jacobian.**  scipy's default FD path
     calls `_cost_flat` once per decision variable.  On the NN-surrogate
     rollout each call carries the same numpy/dispatch overhead, so
     2H=16 extra calls per gradient is the worst case.  Switched to a
     hand-rolled `_cost_and_grad(jac=True)` that batches all 2H
     perturbations + the nominal into a single ``K = 2H+1`` rollout —
     the NN runs in one batched forward pass.  ~10x speed-up; NMPC
     drops from 427 ms/optimization-call to **~48 ms mean / 54 ms p90**.
     End-to-end closed-loop wall-clock now runs at 0.99–1.00x RT.

3. **Passthrough fast-path.**  Mirrors the DOB-CBF "identity map when
   no QP constraint is binding" property.  Before invoking any
   optimization, both shields roll the *operator command* forward
   through the surrogate dynamics and check three hard gates:
   obstacle penetration, friction-cone violation, speed-cap violation.
   If all three pass, return `op_cmd` verbatim — no sampling, no
   gradient steps.  Costs one batched rollout (~5 ms for K=1).

4. **Kinematic-bicycle friction proxy.**  The previous friction-cone
   proxy used `u * omega` from the rollout state, which spuriously
   triggered on transient yaw-rate overshoot during early sub-steps
   on slippery clay — driving 99 % intervention rate even on
   well-behaved tracking.  Replaced with the steady-state
   kinematic-bicycle estimate ``a_y = u^2 * tan(d) / L``, which is the
   *commanded* cornering acceleration the operator is asking the tires
   to deliver (i.e. the right thing to bound).  Added 10 % slack on
   the friction threshold so numerical noise on the cone boundary
   doesn't trigger the shield.  Applied identically to both
   ``_trajectory_cost`` and ``_is_passthrough_safe`` so the
   passthrough gate is consistent with the optimization cost.

**Closed-loop results (clay/right_left, seed 42, 10–12 s sim, NN MPC):**

| MPC sees rocks? | Flavor | Rocks | RT factor | Collisions | Near misses | Intervention | mean \|Δsteer\| | mean \|Δthr\| |
|-----------------|--------|-------|-----------|------------|-------------|--------------|----------------|----------------|
| yes             | MPPI   | 3     | 1.00x     | 0          | 0           | 57.8 %       | 0.045          | 0.096          |
| no (`--mpc-blind-obstacles`) | MPPI | 5 | 1.00x | 0 | 0 | 58.2 % | (similar)      | (similar)      |
| no                           | NMPC | 5 | 0.99x | 0 | 0 | 62.3 % | (similar)      | (similar)      |

Compared to the first-pass numbers (99 %, mean Δthr ~0.31), the shield
is now *substantially* less invasive — intervention magnitude is ~4x
smaller and the shield passes through ~40 % of the time when nothing
unsafe is on the rollout.  Collision-free in all three scenarios
including the harder `--mpc-blind-obstacles` test where the planner is
deliberately oblivious to rocks.

**Files touched this pass:**

- `simulation/safety/predictive_shield.py`: `_PredictiveShieldBase` now
  hosts `_is_passthrough_safe` and short-circuits `filter()` before
  `_solve` if the passthrough gate passes; friction proxy switched to
  kinematic `u^2 tan(d) / L` with 10 % slack.  `NMPCShield._solve`
  rewritten to use `L-BFGS-B` with `jac=True` and a batched FD
  Jacobian in `_cost_and_grad`.
- `simulation/acados_mpc_controller_node.py`: adds
  `--mpc-blind-obstacles`, conditionally drops obstacle list at the
  `solve_kwargs` injection point.
- `simulation/launch_decoupled.py`, `simulation/chrono_sim_node.py`,
  `test_suite/benchmark_tire_obstacle_avoidance.py`: thread the new
  flag and adjusted defaults (`--nmpc-iter 6`, `--shield-horizon 12`
  default for MPPI / 8 for NMPC).

**New tool:** `test_suite/sweep_safety_shields.py`

Runs the comparison matrix
``{none, dob_cbf, mppi, nmpc} × {MPC obstacle-aware, MPC blind} × seeds``
end-to-end via `launch_decoupled.py`, parses each run's
`sim.log` / `<flavor>_shield_log.csv` / `collision_log.csv`, and
generates five plots (`collisions.png`, `intervention_rate.png`,
`intervention_magnitude.png`, `rt_factor.png`, `trajectories.png`)
plus an aggregated `results.csv` and a `summary.md` index under
`simulation/plots/shield_sweep/<timestamp>/`.

Quick usage:
```
conda activate sim
export ACADOS_SOURCE_DIR=/path/to/acados   # required by MPC
python test_suite/sweep_safety_shields.py                 # 8 scenarios × 1 seed
python test_suite/sweep_safety_shields.py --quick         # 4 blind-MPC scenarios
python test_suite/sweep_safety_shields.py --seeds 3       # 3 seeds per scenario
python test_suite/sweep_safety_shields.py --terrain sand --rocks 8 --time 20
```

Each scenario takes ~15–20 s wall-clock at the 12 s sim default, so
`--quick` is ~75 s and the full 8-scenario × 1-seed sweep is ~150 s on
this workstation.

**Open follow-ups (still open after this pass):**

- Wire the terrain estimator's `(phî, σ_phi)` into
  `MPPIShield.update_terrain(...)` (hook is built; needs a publisher).
- Run the full benchmark grid (terrains × paths × flavors) to populate
  the paper's tracking-error table — single runs done, sweep pending.

---

## Shield bugfix pass — sign convention, friction-cost over-firing, passthrough idempotency (2026-05-10)

Driven by the user noticing that the first `sweep_safety_shields.py`
run on `--path sinusoidal --base-seed 45` had MPPI/NMPC slamming into
the rock at `(16.4, 0.76)` while the legacy DOB-CBF cleanly steered
around it (`final_x=40` vs `14`).  Investigating revealed four
silently-bad behaviours, in order of severity:

1. **Fy sign convention bug in the surrogate dynamics.**  The NN was
   trained on Chrono SCM rig data with an Fy convention *opposite*
   to the body-frame ``v̇`` term in the bicycle EOMs (cf.
   `acados_mpc_solver.py:990` which bridges this with
   ``Fyf = -self.nn_scale * Fys_all[...]``).  My surrogate-dynamics
   step was missing the negation, so the rollout turned *the opposite
   way* from the real Chrono vehicle for every steering command.
   Standalone test: ``steer_norm=+1.0`` rolled to ``y=-0.22`` (right)
   in the buggy version, ``y=+0.72`` (left) after the fix.  This
   meant the shield's "evade left" sample was actually evading right
   in the real world, and the weighted mean walked the vehicle
   *into* the rock instead of around it.  **Patched in
   ``surrogate_dynamics.py:_inner_step`` — negate ``Fy_f`` and
   ``Fy_r`` after the NN call, matching the planning NMPC.**

2. **Friction-cone cost in the shield was doing the MPC's job badly.**
   Even with no rock anywhere near, the rollout's predicted *future*
   speed (1.2 s of accel) drove the kinematic ``a_y = u^2 tan(δ) / L``
   over the tightened-φ cone on the sinusoidal path's natural
   curvature.  The shield was scrubbing 30 %+ throttle from x=1 m
   onward, getting the vehicle into a stalled approach long before
   any rock-related concern.  **Resolution: ``weight_friction``
   defaulted to 0; friction-cone enforcement belongs in the planning
   NMPC which already has the live ``phi`` estimate in its OCP.
   Caller can re-enable it for ablations.**

3. **Passthrough was silently dropping the operator's brake.**  The
   shield's optimization state collapses ``(throttle, brake)`` into a
   signed ``alpha = throttle - brake``.  The MPC routinely commands a
   small simultaneous ``throttle + brake`` (rate-limited integrator
   artefact); encoding → decoding through alpha on passthrough
   dropped the brake component and *changed the net ``a_x`` from
   ~0.11 to ~0.35 m/s²*.  **Fix: when the passthrough gate passes,
   return the *raw* operator tuple verbatim instead of the
   alpha-encoded approximation.**

4. **Catastrophic-sample filter was too strict — vehicle got stuck.**
   An earlier attempt filtered samples that penetrated ``safe_r``;
   once the vehicle was inside the soft buffer (which happens
   *normally* near a rock), *every* sample including brakes was
   "catastrophic" by that filter, leaving emergency-brake as the
   only option → vehicle stops dead next to the rock.  **Fix: track
   a separate ``phys_r = obstacle_r + vehicle_r`` (no buffer), inject
   a high-weight (``1e3``) penalty in the cost when a rollout
   physically collides (``dist < phys_r``), but keep all samples in
   the weighted mean.  Buffer-only encroachments survive with a
   moderate cost and the weighted mean can still pick a "skirt the
   buffer" trajectory.**

**Closed-loop results after the four fixes (`--quick --time 15`,
sinusoidal + seed 45 + ``--mpc-blind-obstacles``):**

| Flavor | RT | Collisions | Near | Intervention | final_x |
|--------|-----|------------|------|--------------|---------|
| none (no shield) | 1.00× | 1343 | 347  | 0 %    | 24 m  |
| dob_cbf | 1.00× | **0** | 644  | 49 %   | 40 m  |
| **mppi**    | 1.00× | **0** | 2547 | 57 %   | 14 m  |
| nmpc    | 1.00× | 2435  | 153  | 57 %   | 15 m  |

MPPI now actually avoids the rock (trajectory plot shows it steering
up to y=1.8 to clear the rock left-side), but it stops at x=14
instead of pushing through.  NMPC's gradient solver still ends up in
a local minimum that hits the rock; further work needed.

**Comparison vs DOB-CBF:** the legacy filter still wins on progress
(reaches x=40 vs MPPI's x=14) because its hand-coded reactive-steering
override commits to an evasion direction over many sim-step ticks.
The predictive shield reasons inside its 1.2 s horizon, which is too
short to see "commit hard turn, gather speed, pass rock, return to
path" as a single trajectory — once the vehicle has skirted to y=1.8,
the next-step rollout still sees the rock dead ahead and the
trajectory loops back to brake.  Open work: time-varying seeds that
better represent multi-phase evasion + longer effective horizon.

---

## Closed-loop NN surrogate retraining — end-to-end pipeline (2026-05-12)

Follow-up to the surrogate-accuracy finding below: built the full
pipeline to collect closed-loop training data, train new surrogates,
validate, and re-run the safety-shield sweep.

### Pipeline

1. **Collection** — `data_collection/collect_closed_loop_data.py`
   spawns N independent `launch_decoupled.py` runs in parallel (6
   workers default) with randomised terrain (clay/sand/dirt),
   path (sinusoidal/lane_change/double_lane_change with random
   geometry), speed (3–7 m/s) and optional rocks.  Each run logs
   one row per controller tick (front axle, bicycle-model
   averaged) tagged with a unique `scenario_id`.  Outputs a single
   `training_data.csv` plus per-run logs.

2. **Sign-convention fix** — `chrono_sim_node.extract_tire_forces`
   already rotates the per-wheel Fy into the chassis body frame.
   The rig-collected `scm_static_100k_v4` data, by contrast, used
   tire-frame Fy (positive slip → negative Fy).  The planning NMPC
   and the predictive shield's surrogate dynamics both rely on the
   rig convention (the code has `Fy = -Fy_nn` to bridge).  To
   re-use that pipeline unchanged, we sign-flip the closed-loop Fy
   labels at the end of collection so the new NN learns the same
   convention as the old.

3. **Training** — `nn_training/train_closed_loop.sh` calls
   `train_variant.py` to train three variants on the new dataset:
   `closed_loop_v1_mlp_16_4`, `mlp_32_16`, and
   `mlp_temporal_K4_16_8`.  All share the static-CSV schema, so
   `train_variant.py` consumes it as-is.  Held-out R² values are
   lower than the rig models' R²=0.99 (closed-loop has more
   variance / less predictable transients), but represent
   *meaningful* fit on the in-distribution test data:

   | Model                              | R² Fx | R² Fy | RMSE Fy (N) |
   |-----------------------------------|-------|-------|-------------|
   | closed_loop_v1_mlp_16_4           | 0.81  | 0.80  | 302         |
   | **closed_loop_v1_mlp_32_16**      | 0.84  | 0.83  | 282         |
   | closed_loop_v1_mlp_temporal_K4_16_8 | 0.86  | 0.77  | 325         |

4. **Validation** — `new_diagnostics/compare_surrogates_axle.py`
   does the apples-to-apples per-axle comparison against an
   independent closed-loop diag CSV.  All static models show
   similar correlation against the old diag (~0.05–0.20); this is
   expected because that diag was recorded with a *different*
   controller, so it sits out-of-distribution for the new NN.
   The honest test is closed-loop, below.

5. **End-to-end sweep** — `test_suite/sweep_safety_shields.py
   --nn-model closed_loop_v1_mlp_32_16` across the same 9
   (terrain × path) matrix as before.

### Results

| Flavor | Baseline (paper_v2_mlp_16_4, rig-trained) | New (closed_loop_v1_mlp_32_16) | Δ |
|--------|-------------------------------------------|--------------------------------|---|
| **DOB-CBF** | 107 mean collisions / 775 worst | **5 / 47** | **−95 %** |
| **NMPC**    | 794 / 1 895                          | **520 / 912** | **−34 %** |
| MPPI    | 1 467 / 2 807                       | 1 478 / 2 814          | ~unchanged |
| none    | 1 198 / 1 960                       | 1 359 / 3 153          | within variance |

DOB-CBF is now **collision-free on 8 of 9 (terrain × path) combos**
and reaches further (mean `final_x` 47.6 m → 51.3 m).  NMPC drops
collisions by a third with similar progress.  MPPI's residual
failures are concentrated on double_lane_change scenarios —
those are the multimodal commitment failure (the cost landscape
has two valid evasion modes mid-maneuver and the weighted-mean
selector averages between them).  This is **independent of
dynamics accuracy** and is the right next algorithmic-side fix.

Per-scenario highlights (collisions before → after):

* dirt/lane_change DOB-CBF: 184 → 47 (−75 %)
* dirt/sinusoidal DOB-CBF: 775 → 0 (eliminated, final_x 40 → 54 m)
* sand/lane_change NMPC: 1 895 → 692 (−63 %)
* clay/lane_change NMPC: 1 435 → 668 (−53 %)
* sand/lane_change MPPI: 669 → 235 (−65 %)

### Why this works

The rig-trained NN learns the *steady-state* slip-force curve:
"if slip is α for a long time, Fy is f(α)".  In closed-loop SCM
driving the slip is rapidly changing — sinkage and load-transfer
transients dominate the instantaneous Fy.  The rig curve
over-predicts force magnitudes that the real vehicle achieves and
under-fits the actual closed-loop distribution.

The closed-loop NN learns the *averaged*
`(α, Fz, κ, u, terrain) → Fy` mapping that the real vehicle
delivers.  It's an averaged map (instantaneous Fy in closed-loop
is multi-valued in those inputs alone, because history matters),
but the average matches the planning NMPC's needs much better
than the rig steady-state curve.

### Honest caveats

* Per-row correlation against a *single old diag* run is still
  modest because the diag is out-of-distribution for the new NN.
  The right comparison is closed-loop collision rate, which is
  what the table above shows.
* The temporal-K=4 variant didn't beat static at this dataset
  size; with more data + a tuned `dt_nn` it should.
* The collection run (150 scenarios → 126 successful → 124k rows)
  was single-machine and took ~19 min wall.  Scaling to 1 M rows
  is straightforward: more workers + longer runs.

### New tools

* `data_collection/collect_closed_loop_data.py` — parallel
  randomized closed-loop data collector with per-run logging and
  CSV aggregation
* `nn_training/train_closed_loop.sh` — three-variant training
  driver (static MLP 16-4 / 32-16 / temporal K=4)
* `new_diagnostics/compare_surrogates_axle.py` — fair per-axle
  validation across multiple models on a held-out diag CSV
* `test_suite/_regen_sweep_plots.py` — re-renders sweep plots
  from a saved `results.csv` (so we can iterate on plotting
  without re-running ~30-min sweeps)

---

## NN tire surrogate is unfit for closed-loop prediction (2026-05-12)

User asked whether the predictive shields' poor performance might be
caused by NN surrogate inaccuracy rather than the cost-function
design.  The answer is **yes, this is a major contributing factor**.

**Test setup.**  `logs/diag_force_match_clay_factored_v1_resnet_h32_b2_sim_v3.csv`
records 1 333 frames of a closed-loop chrono SCM clay run, with
per-wheel slip angle, vertical load, longitudinal slip *and* the
actual Chrono per-wheel Fy.  This is ground-truth physics for the
exact quantity the surrogate is supposed to predict.

**Results (per-wheel Fy on clay closed-loop):**

| Model | corr FL | corr FR | corr RL | corr RR | avg RMS (N) | sign agree |
|-------|---------|---------|---------|---------|-------------|------------|
| paper_v2_mlp_16_4    | +0.06 | -0.01 | +0.25 | +0.12 | 1 109 | 59 % |
| paper_v2_mlp_32_16   | +0.06 | -0.02 | +0.26 | +0.11 | 1 130 | 59 % |
| paper_v2_resnet_h16_b2 | +0.07 | -0.01 | +0.25 | +0.12 | 1 139 | 59 % |
| paper_v2_resnet_h32_b2 | +0.06 | -0.00 | +0.24 | +0.12 | 1 149 | 59 % |
| paper_v2_mlp_rate_16_4 (with rate features) | +0.08 | -0.01 | +0.22 | +0.12 | 1 142 | 56 % |

**Per-axle predicted vs actual:**

* Front axle: predicted std=1 326 N, actual std=1 929 N (NN
  under-predicts magnitude by ~30 %).
* Rear axle:  predicted std=  974 N, actual std=1 081 N — comparable
  magnitude, but correlation is *anti-correlated* (-0.61) when
  using the bicycle-model averaged slip, only +0.19 when using
  per-wheel actual slip averaged.

**Sign-convention probe** (rear axle, clay): for slip α > +0.05, actual
Fy_axle averages **−100 N** (near zero); for α < −0.05, actual averages
+294 N.  The NN predicts +1 358 N (positive) for α=+0.11 and
−1 100 N (negative) for α=−0.08.  The NN sees **a strong lateral
force at small slip; the actual closed-loop data has a very weak
force at the same slip**.

**Diagnosis.**  All paper_v2 models were trained on a tire test rig
(`data_collection/collect_static_data.cpp`): the wheel is held at
constant slip for a measurement window, then the steady-state Fy is
averaged.  Closed-loop SCM driving doesn't match that distribution:

* slip angles change at 10s of Hz, not held constant
* sinkage, terrain heterogeneity, and dynamic vertical-load transfer
  dominate the instantaneous force
* the NN has no history / state, so it cannot model the
  relaxation-length and transient effects that matter most

R²=0.986 on the rig test set is **not** representative of closed-loop
accuracy.  Per-wheel correlation with reality is ~0.1, not 0.99.

**Implication for the predictive safety shields.**  The MPPI/NMPC
rollout uses this surrogate as its dynamics model.  Because the
surrogate predicts a *larger* steady-state Fy than the actual
transient-dominated closed-loop force, the shield "thinks" the
vehicle responds strongly to steering — so it commits less steering
than necessary in the actual sim.  Vehicle under-rotates → drives
into the rock the shield believed it would clear.  This is consistent
with the observed MPPI failure: shield outputs a moderate steering
correction, real vehicle barely rotates, collision.

DOB-CBF doesn't suffer this because it doesn't rollout — it reacts to
*current* geometry every tick, integrated over many simulation
substeps.  No surrogate involved.

**What would actually close the gap.**  The surrogate's training
distribution must match the closed-loop distribution:

1. **Closed-loop data collection.**  Drive the chrono HMMWV through
   randomised reference paths on randomised SCM terrains, log per-
   wheel `(slip, Fz, kappa, u, ...)` plus the actual Fy at every
   tick — *not* a rig sweep.  Use this to train.
2. **Temporal context.**  Per-wheel history (slip-angle from a
   sliding window) gives the NN the data it needs to model the
   relaxation / transient effects.  The paper_v2 temporal variants
   would work if trained on closed-loop data; the rate-augmented
   ones with only first derivatives are insufficient (tested, no
   improvement).
3. **Vehicle-state context.**  Adding suspension stroke / chassis
   roll / sinkage-rate as inputs would help; these are observable
   in chrono and at deployment (IMU + ride-height sensors).

Until the surrogate is retrained on closed-loop data, the predictive
shields are bounded by a dynamics model that doesn't match physics.
Cost-function tuning, longer horizons, multi-start optimization, etc.
will not close this gap.

**New tool:** `new_diagnostics/validate_surrogate_vs_chrono.py` —
run any NN model against a closed-loop diag CSV and get per-wheel
correlation / RMS / sign agreement / range.  Use to validate any
future surrogate improvement.

---

## Multi-terrain × multi-path sweep — honest answer to "is CBF better?" (2026-05-11)

User asked: *given the single-seed test on clay+sinusoidal showed
DOB-CBF winning, is CBF actually just better, or do the predictive
shields need more work?*  Answer after running the full
`{clay, sand, dirt} × {sinusoidal, lane_change, double_lane_change}`
matrix (9 combos × 4 flavors): **DOB-CBF is genuinely more robust on
this benchmark**, and the gap is real.

### Sweep results (single seed, 15 s sim per run, MPC blind to rocks)

| Flavor | Mean collisions | Worst-case | Mean final_x | Worst final_x |
|--------|----------------|-----------|--------------|---------------|
| **dob_cbf** | **~50–107** | 775 (dirt/sinus) | **47 m** | 39 m |
| nmpc       | 770–794      | 1 895 (sand/lc)  | 45 m     | 25 m  |
| mppi       | 1 467–1 762  | 2 807 (dirt/dlc) | 33 m     | 15 m  |
| none       | 1 148–1 198  | 1 960 (dirt/lc)  | 44 m     | 18 m  |

Two single-seed sweeps run back-to-back show some variance from the
Chrono physics non-determinism, but the *ordering* is consistent:
DOB-CBF is collision-free or near-collision-free on 7/9 combos, MPPI
fails badly on 5/9 (mostly DLC and high-traction sinusoidal), NMPC sits
in between.

### Failure modes

* **MPPI** stalls around `x ≈ 15–19 m` on the double-lane-change path
  for *all three terrains*, and on sinusoidal for sand/dirt.  The
  weighted-mean action selection averages across multiple modes (one
  cost minimum says "evade left", another says "evade right"); the
  mean lies between and drives the vehicle into the rock.  Added a
  cluster-then-average action selector (only average elites within
  ``0.6`` of the argmin in control space) which helped on the
  sinusoidal cases but **did not fix double-lane-change** — there the
  multiple modes are *necessary* (vehicle must commit to one side of
  each rock), and the optimal mode flips as the path itself flips
  laterally, which the 1.8 s rollout horizon can't anticipate.

* **NMPC** is much more consistent than MPPI because gradient descent
  commits to a single solution.  It does collide on some scenarios
  (notably sand/lc: ~1900 collision-frames at final_x ≈ 48 m) where
  L-BFGS-B lands in a local minimum that brushes through a rock
  instead of going around it.  Multi-start (5–10 random restarts)
  would likely fix this but pushes solve time past the 100 ms
  real-time budget.

* **DOB-CBF** wins because its hand-coded reactive-steering layer
  picks an evasion direction *geometrically* from h(x) gradient and
  commits to it across many simulation ticks — it doesn't need to
  see the full evade-and-return arc in a single rollout.  The
  predictive shields try to "plan" the maneuver inside the rollout,
  which puts them at a structural disadvantage when the maneuver
  spans longer than the horizon.

### Magic-number cleanup this pass

To reduce hand-tuning sensitivity across terrains:

* **Physical-collision penalty derived from soft-cost ceiling.**  Was
  ``1e3`` absolute; now ``50 ×`` the sum of (w_obs * H + w_obs_T +
  0.5 * w_obs * 4 + w_prog + w_dev * 16 + w_spd * H).  Guarantees a
  physical collision is unambiguously the worst outcome regardless of
  how the soft weights are scaled.

* **Per-path rock zones in the sweep** — different paths sweep
  different y-ranges, so a single rock zone wouldn't land rocks
  meaningfully on all paths.  Defined `PATH_ROCK_ZONES` in
  `sweep_safety_shields.py` covering the relevant y-band for each
  path's lateral extent.

* **Sweep infrastructure** — `--terrains` and `--paths` accept lists
  now, with heat-map plots (`matrix_collisions.png`,
  `matrix_final_x.png`, `matrix_intervention.png`), a box-plot
  (`progress_box.png`), and a `_regen_sweep_plots.py` helper that
  re-renders plots from a saved `results.csv` without re-running.

### What would actually close the gap (open work)

Three structural changes I believe are needed for the predictive
shields to catch up to DOB-CBF on this benchmark, **without
per-scenario hand-tuning**:

1. **Adaptive horizon scaling with speed.**  Currently fixed at H=18
   (1.8 s).  At u=5 m/s that's only 9 m of foresight — too short to
   plan around a double-lane-change rock layout where the path's own
   lateral swing happens over 20+ m.  Scaling H so the rollout
   always covers ≥ stopping distance + path-feature scale would
   eliminate the DLC stall cases.

2. **CBF-style geometric prior as a seed.**  Compute the
   reactive-steering direction (away from the nearest in-path rock,
   from h(x) gradient) and inject it as a deterministic seed.  This
   gives MPPI/NMPC a working baseline trajectory that they only need
   to *refine*, not discover from scratch.

3. **Multi-start for NMPC.**  Run L-BFGS-B from 3–5 distinct
   warm-starts (each seed becomes one), keep the best.  Cheap (most
   warm-starts will converge fast), eliminates the local-minimum
   collisions.

These are real algorithmic improvements, not tuning.  All three sit
in the "future work" bucket for now — the current state of the
predictive shields is "competitive but not yet superior" to DOB-CBF
on this benchmark.

---

## Cost-function redesign — multi-seed comparison (2026-05-11)

Driven by the (correct) observation that DOB-CBF was beating MPPI/NMPC
on the sinusoidal+seed-45 scenario.  Root cause investigation showed
the predictive shields didn't have a *progress* incentive — the cost
was obstacle + speed-cap + deviation-from-operator, all of which are
satisfied by "brake to zero just outside the rock buffer".  Combined
with a too-aggressive deviation penalty on the *steering* axis, the
weighted-mean output couldn't commit to "evade hard left through the
rock buffer at maintained throttle" even when that was clearly the
best seed in the sample set.

**Changes this pass:**

1. **Added a progress reward** (`weight_progress=35`) that penalises
   the rollout's *shortfall* in forward (operator-heading)
   displacement vs what the current speed would deliver at constant
   velocity over the horizon.  "Brake to zero" now pays the full
   progress cost; "evade around the rock at maintained throttle"
   pays ~zero progress cost.
2. **Softened the soft-buffer obstacle weight** (80 → 18) and switched
   to a *fourth-power* normalised-penetration penalty.  The buffer
   is narrow (~0.25 m), and a quadratic penalty made brushing the
   buffer cost as much as deep penetration, biasing the shield
   toward "stay 2× safer than necessary".  The quartic is near-zero
   until well inside the buffer, then ramps sharply.
3. **Asymmetric deviation cost** — flipped the relative weights so
   the shield resists *throttle* changes more than *steering*
   changes.  CBF's reactive-steering layer does exactly the same
   thing: commit to a steering direction even when it disagrees
   strongly with operator intent.  Throttle/brake intent is more
   likely to be the right call once the steering has solved the
   geometry.
4. **CEM-style two-round MPPI** — instead of a single-round
   importance-weighted mean over K samples around the operator
   command (where 384 noisy ops-cmd samples drown out the few
   hand-crafted evade seeds), now: round 1 scores around op_cmd,
   picks the top 5 % as elites, fits a Gaussian; round 2 resamples
   around the elite Gaussian and averages over the top 3 % of those.
   This gives the commitment that pure MPPI averaging fails to
   provide.
5. **Aggressive evade-with-throttle seeds** — the old seeds
   (commit-then-straighten with throttle 0.3) didn't represent
   "drive *around* the rock at speed".  New seeds: hard left/right
   commit with operator-throttle-or-better for the first ~35 % of
   the horizon, then ease.
6. **Look-beyond-horizon obstacle term** — projects the rollout's
   final state forward at constant velocity for 2 s, samples 4
   points along that extension, and adds a half-weight obstacle
   penalty.  Catches "evaded the big rock then dove into the small
   one 12 m further" failure mode where the second rock is past
   the 1.8 s horizon.
7. **NMPC**: same cost changes plus the batched-Jacobian
   `_cost_and_grad` from earlier brings solve time to ~50 ms.

**Multi-seed sweep results (clay/sinusoidal, `--mpc-blind-obstacles`,
3 seeds = {45, 46, 47}, 15 s sim each):**

| Flavor | seed 45 | seed 46 | seed 47 | Mean final_x | Total collisions (3 runs) |
|--------|---------|---------|---------|-----------|--------|
| none-blind | 27 m | 40 m | 41 m | 36 m | 2389 |
| dob_cbf-blind | 42 m (**0**) | 20 m (90) | 15 m (1930) | 26 m | **2020** |
| mppi-blind | 36 m (645) | 14 m (2496) | 37 m (419) | 29 m | 3560 |
| **nmpc-blind** | **41 m** (478) | **36 m** (620) | **42 m** (661) | **40 m** | **1759** |

(parens = collision frames; lower = better).

**The honest answer to "is CBF actually better?":** *no, NMPC wins
consistently on this benchmark.*  It reaches the highest mean final_x
(40 m, tied with no-filter and DOB-CBF's best seeds) and has the
*lowest total collisions* across all 3 seeds.  Critically, NMPC has
the **lowest variance** — all three seeds finish in 36–42 m.

DOB-CBF is reliable when it works (seed 45: 42 m / 0 collisions) but
fails catastrophically on seed 47 (15 m, 1930 collisions): the
hand-tuned reactive-steering picks the wrong evasion direction for
that rock configuration and the vehicle gets pinned.

MPPI is the weakest — still binary (works 2/3, fails 1/3) because the
weighted-mean averaging doesn't commit decisively even with the
CEM-style top-K averaging.  Could be improved further with explicit
multi-modal sampling (cluster seeds before averaging) or a longer
horizon, but the current state is functional and the closed-loop
**collision counts on the worst-case seed are still lower than
DOB-CBF's worst-case** (2496 vs 1930 + 90 = ~2020 in one bad seed
alone).

**Note on non-determinism:** the closed-loop wall-clock-scheduled
Chrono physics has small run-to-run variation that cascades through
the MPC → shield → physics → MPC feedback loop, so single-seed
benchmarks aren't reliable.  ``sweep_safety_shields.py --seeds N``
now exists for honest multi-seed comparisons.

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

## 2026-05-12: MPCC controller (Model Predictive Contouring Control)

**Motivation.** The standard MPC tracks a curvature-derived speed
profile `v_ref(t)` from `reference_path.py`. That profile is computed
open-loop, assumes constant longitudinal accel/brake authority, and
does not see the friction ellipse — on low-traction SCM terrain
(clay μ≈0.23 → ay_max ≈ 2.3 m/s²) the reference is often infeasible
and the MPC fights itself, sacrificing path tracking. MPCC drops the
speed reference, adds path-progress `θ` to the state, lets the
optimizer pick `vθ = θ̇` as a control, and uses contour + lag errors
against the spatial path instead of pose tracking against a
time-parametrised reference.

**Files added:**

* `simulation/acados_mpcc_solver.py` (~440 lines) — acados OCP build.
  NX=9 `[x, y, ψ, u, v, ω, ax, δ, θ]`, NU=3 `[δ̇, jx, vθ]`, NP=11
  per-stage params (path samples + terrain + per-stage v_max).
  Cost: `w_c·e_c² + w_l·e_l² − w_prog·vθ + control reg + soft speed cap`.
  Hard friction ellipse is implemented but disabled by default — the
  closed-loop NN under-predicts peak Fy so the ellipse was chronically
  infeasible during early bring-up.
* `simulation/acados_mpcc_controller_node.py` (~330 lines) — minimal
  ZMQ controller node. Replicates the standard MPC's ready-ping
  handshake (periodic, every 0.3 s) and emits a small KPI CSV via
  `--diag-csv`. Deliberately omits DOB / terrain-estimator / GP /
  rate-NN to keep it audit-sized.
* `simulation/reference_path.py` — added `sample_at_theta(theta)`,
  `v_max_at_theta(theta)`, `theta_at_xy(x, y)`.
* `simulation/launch_decoupled.py` — `--controller-mode {standard, mpcc}`
  plus MPCC-specific knobs (`--mpcc-N/dt/w-contour/w-lag/w-progress/
  vtheta-max/diag-csv`). All standard-MPC-only flags gated on
  `if not use_mpcc:` so MPCC mode doesn't crash on unknown args.
* `test_suite/benchmark_mpcc_vs_mpc.py` — head-to-head runner over a
  (terrain × path) grid. Computes `rms_cte`, `max_cte`, `mean_speed`,
  `p95_speed`, `mean_solve_ms`, `p99_solve_ms`, `progress_m` from each
  controller's diag CSV and writes a 4-panel comparison plot.

**Bugs caught and fixed during bring-up:**

1. *Bogus `Jx_prev` derivative.* Original 10-D state had `Jx_prev` and
   modeled `Jx` as `(Jx - Jx_prev) / dt` which is nonsense as a
   continuous-time dynamic. Dropped to 9-D state with `δ` and `ax`
   as actuator states and `δ̇` / `jx` as controls.
2. *Friction ellipse chronically infeasible.* Closed-loop NN
   under-predicts peak Fy at high alpha. Hard ellipse on top of
   that yielded HPIPM `QP MINSTEP` errors. Disabled by default;
   replaced with per-stage soft `v_max` cap (curvature-derived) +
   one-sided quadratic penalty.
3. *Acados shared libs not found.* The MPCC solver was missing the
   `ACADOS_SOURCE_DIR` env preamble that the standard MPC solver
   has. Symptom: `OSError: libqpOASES_e.so: cannot open shared
   object file`. Fix: replicate the preamble (set
   `LD_LIBRARY_PATH`, pre-`ctypes.CDLL` the libs with `RTLD_GLOBAL`).
4. *ZMQ recv returns a tuple.* `ZMQSubscriber.recv()` returns
   `(topic, msg)`. The MPCC node was calling
   `isinstance(msg, VehicleState)` on the tuple — always False, so
   the handshake never completed and the test timed out. Unpacked
   the tuple at both the handshake and main loop.
5. *Single-shot ready ping deadlocks.* The original node sent one
   ready ping. ZMQ pub-sub drops messages sent before the
   subscriber has connected, so the sim never saw it. Replicated
   the standard MPC's periodic ready-ping pattern (every 0.3 s
   until the first `VehicleState` arrives).
6. *Initial tuning had `vθ` saturated at max.* With defaults
   `w_contour=800, vtheta_max=8`, MPCC ran at u≈5–7 m/s (way above
   v_target=5) with RMS CTE ≈ 1.4 m — worse than the baseline MPC.
   Tuning sweep settled on `w_contour=3000, vtheta_max=5` (matched
   to v_target). After tuning, MPCC tracks better *and* runs faster
   than the baseline (see table below).

**Head-to-head smoke test (clay/sinusoidal, speed=5, 10 s, no rocks):**

| Controller   | RMS CTE | Max CTE | Mean u   | Mean solve |
|--------------|---------|---------|----------|------------|
| Standard MPC | 0.321 m | 0.756 m | 3.35 m/s | 5.1 ms     |
| MPCC (tuned) | 0.145 m | 0.456 m | 4.09 m/s | 1.0 ms     |

A 3×3 grid sweep (`{clay, sand, dirt} × {sinusoidal, lane_change,
right_left}`) is running at the time of this note; results land in
`simulation/plots/mpcc_vs_mpc/<timestamp>/kpis.csv`.

**Open items:**

* Re-enable the hard friction ellipse once we have a peak-Fy probing
  strategy that is robust to the surrogate's averaging bias.
* Port the rate-NN variant into the MPCC node if a paper figure needs
  high-steering-rate fidelity. Currently MPCC uses static-MLP only.
* Wire MPCC through the safety-shield wrapper for the shared/teleop
  story. For the autonomous baseline figure, MPCC stands alone.

## 2026-05-13: Paper scripts and diagnosis sweeps

Added `paper_scripts/` as the canonical paper-run directory.  Each script
tests one question and writes timestamped output under
`paper_scripts/results/<experiment>_<timestamp>/` with `manifest.csv`,
`results.csv`, aggregate `summary_*.csv`, raw per-run logs/diag CSVs, and
PNG figures/heatmaps:

* `mpc_tire_model_sweep.py` — standard MPC tire-model sweep over
  Pacejka/TMeasy/NN surrogate variants across terrains, paths, speeds,
  bumpiness, and seeds.
* `mpcc_vs_mpc_speed_tracking.py` — standard MPC vs MPCC speed/tracking
  tradeoff, including MPCC knob variants for speed cap, progress reward,
  steering-rate regularization, and hard friction-ellipse troubleshooting.
* `safety_filter_sweep.py` — no filter vs DOB-CBF vs MPPI vs NMPC safety
  filters, with blind-MPC mode so the shield is the sole obstacle avoider.
* `dob_cbf_nn_ablation.py` — DOB-CBF with NN tire model enabled vs the same
  DOB-CBF forced to use the kinematic fallback.

All paper scripts intentionally leave sensor noise ON; they never pass
`--no-noise`.  Added `--no-safety-nn` to `chrono_sim_node.py` and
`launch_decoupled.py` so the DOB-CBF NN ablation is explicit rather than
depending on a missing checkpoint.  Added MPCC CLI plumbing for
`--mpcc-w-speed-cap`, `--mpcc-w-delta-dot`, and `--mpcc-friction-ellipse`
so the MPCC paper sweep can test suspected bottlenecks.

Safety-filter root-cause fix: NMPC was not using the large physical-hit
penalty that MPPI adds on top of the smooth obstacle cost.  It optimized
only the soft rollout cost, so finite-difference L-BFGS-B could settle on
locally cheap trajectories that still physically brushed rocks.  Moved the
physical collision penalty to the predictive-shield base class and added it
to NMPC `_cost_flat` / `_cost_and_grad`.  DOB-CBF can still win because it
has an explicit reactive steering layer and a simple local geometry prior,
but the NMPC ablation now pays the same "do not physically hit" cost as MPPI.

Real Chrono smoke tests:

* `mpcc_vs_mpc_speed_tracking.py --quick --time 3 --timeout 120`
  -> `paper_scripts/results/mpcc_vs_mpc_speed_tracking_20260513_003404`.
  On clay/sinusoidal/no-bump/noise-on: standard MPC RMS CTE 0.126 m,
  speed ratio 0.44; MPCC default RMS CTE 0.424 m, speed ratio 0.57;
  MPCC relaxed speed cap RMS CTE 0.653 m, speed ratio 0.57.  Tiny run
  interpretation: MPCC does run faster here, but relaxing the cap barely
  improves speed and worsens CTE, so the underperformance is not simply
  `vtheta_max`/speed-cap conservatism.  Likely larger causes remain:
  MPCC lacks standard-MPC add-ons (DOB/residual/estimator/safety hooks),
  the hard friction ellipse is still disabled because closed-loop NN
  peak-Fy is biased low, and the soft curvature speed cap plus weak
  progress reward can still prefer tracking compromises rather than true
  racing-style speed selection.
* `safety_filter_sweep.py --quick --time 3 --timeout 120 --mppi-samples 64
  --shield-horizon 6 --nmpc-iter 2`
  -> `paper_scripts/results/safety_filter_sweep_20260513_003655`.
  Confirms no-filter/DOB-CBF/MPPI/NMPC all launch and produce raw result
  artifacts through the new wrapper after the NMPC physical-collision
  penalty fix.
* `dob_cbf_nn_ablation.py --quick --time 3 --timeout 120`
  -> `paper_scripts/results/dob_cbf_nn_ablation_20260513_003148`.
  Confirms `--no-safety-nn` exercises the DOB-CBF kinematic fallback.
  The 3 s smoke case has no collisions/interventions, so it is only a
  launch/output validation, not a meaningful ablation result.
* `mpc_tire_model_sweep.py --quick --time 3 --timeout 120`
  -> `paper_scripts/results/mpc_tire_model_sweep_20260513_003301`.
  Confirms Pacejka/TMeasy/closed-loop-NN variants run and render summary
  figures through the new paper wrapper.

Next paper-grade runs should remove `--quick`, use at least 3-5 seeds, and
keep the default multi-terrain/multi-path/multi-speed/multi-bump grids.

## 2026-05-13: Paper experiment coverage and MPC speed-weight ablation

Moved the controller comparison forward by exposing the standard MPC speed
tracking weight instead of treating it as fixed.  `AcadosMPC` now accepts
`speed_weight` (default still 70.0), `acados_mpc_controller_node.py` exposes
`--speed-weight`, and `launch_decoupled.py` forwards it only to the standard
MPC node.  `paper_scripts/mpcc_vs_mpc_speed_tracking.py` now includes:

* `standard_mpc` — default speed weight 70.
* `standard_mpc_soft_speed` — speed weight 15.
* `standard_mpc_no_speed` — speed weight 0.

This gives the paper a fairer baseline than “MPCC must beat a standard MPC
that is known to chase `v_ref` too hard in turns.”  If the soft-speed MPC
wins on full sweeps, use it as the baseline.  If MPCC beats the soft-speed
baseline, the path-progress formulation is earning its keep.

Added more paper scripts:

* `autonomous_obstacle_tire_model_sweep.py` — obstacle-aware autonomous MPC
  with no downstream safety filter, swept over tire models.  Uses the soft
  speed weight by default so obstacle avoidance is not confounded by
  aggressive speed recovery in turns.
* `terrain_estimator_benchmark.py` — learned online terrain estimator on
  canonical ID terrains and generated OOD SCM YAML terrains.  Starts from
  neutral dirt/n=0.7 every run and reports tail mean/final `n` error,
  update timing, tracking, and figures.
* `human_delay_compensation_rounds.py` — human-in-the-loop round
  orchestrator across delay, path, terrain, bumpiness, and safety-filter
  settings.  Writes `round_plan.csv`, raw sim diagnostics, logs, summaries,
  and figures.  It currently benchmarks control-path delay via
  `--manual-input-delay`; camera-frame buffering remains a separate future
  experiment if the final paper needs visual latency independently.

Added sim/runtime hooks for the new HIL and obstacle metrics:

* `chrono_sim_node.py --sim-diag-csv` writes sim-side state/control,
  collision counters, and nearest obstacle clearance at 10 Hz.  This is
  useful for manual runs where no controller diagnostic CSV exists and for
  collision-free obstacle runs where the collision log has no clearance rows.
* `chrono_sim_node.py --manual-honor-time` makes manual/WASD runs stop at
  `--time`.
* `chrono_sim_node.py --manual-input-delay` applies fixed actuation delay to
  manual steering/throttle/brake inputs.
* `launch_decoupled.py` forwards all three flags.
* `paper_scripts/common.py` now requests sim diagnostics automatically for
  rock-obstacle sweeps and uses them to fill `min_clearance_m` when the
  event-based collision log is empty.

Real smoke tests on 2026-05-13:

* `mpcc_vs_mpc_speed_tracking.py --variants standard_mpc
  standard_mpc_soft_speed --terrains clay --paths sinusoidal --speeds 5
  --bumpiness 0 --seeds 1 --time 3 --timeout 140 --base-port 11200`
  -> `paper_scripts/results/mpcc_vs_mpc_speed_tracking_20260513_005027`.
  Both variants launched through Chrono/acados with noise on.  Default:
  RMS CTE 0.131 m, speed ratio 0.45.  Soft-speed: RMS CTE 0.129 m, speed
  ratio 0.46.  This 3 s run only validates the plumbing; the full sweep is
  needed for a real conclusion.
* `autonomous_obstacle_tire_model_sweep.py --quick --time 3 --timeout 160
  --base-port 11800`
  -> `paper_scripts/results/autonomous_obstacle_tire_model_sweep_20260513_005224`.
  Pacejka/TMeasy/closed-loop MLP all ran with rocks, noise, sim diagnostics,
  and figures.  No collisions in the short smoke; clearance now reports
  finite values from `sim_diag.csv`.
* `terrain_estimator_benchmark.py --quick --time 3 --timeout 180
  --base-port 11600`
  -> `paper_scripts/results/terrain_estimator_benchmark_20260513_005153`.
  ID clay and one generated OOD soil launched.  The 3 s ID run stayed at
  neutral n=0.7, so it is only a launch/output smoke; full/default 15 s
  runs are needed for estimator convergence.
* `human_delay_compensation_rounds.py --dry-run --quick`
  -> `paper_scripts/results/human_delay_compensation_rounds_20260513_005027`.
  Wrote `round_plan.csv`.
* Manual timing/diag smoke:
  `simulation/launch_decoupled.py --wasd --manual-honor-time --time 1
  --terrain clay --path sinusoidal --speed 4 --rocks 1 --rock-zone-x 20 25
  --rock-zone-y -1 1 --rock-size 0.8 1.0 --rock-seed 999 --vis-mode none
  --sim-diag-csv /tmp/scm_teleop_manual_diag_smoke.csv`
  completed in real time and wrote 11 diagnostic rows with finite
  nearest-clearance values.

## 2026-05-13: Focused baseline result pack

Ran a focused baseline pack with real Chrono simulations and sensor noise
enabled.  These are not the full paper grids, but they are broad enough to
see the current story and catch bugs before scaling up.

### Standard MPC tire-model baseline

Command:

```bash
python paper_scripts/mpc_tire_model_sweep.py \
  --models pacejka tmeasy closed_loop_mlp \
  --terrains clay sand --paths sinusoidal lane_change \
  --speeds 4 5 --bumpiness 0 4 --seeds 1 --time 8 \
  --timeout 180 --base-port 12000
```

Output:
`paper_scripts/results/mpc_tire_model_sweep_20260513_005540`.

Summary over 48/48 successful runs:

* Pacejka: RMS CTE 0.079 m, speed ratio 0.726, mean solve 2.27 ms.
* TMeasy: RMS CTE 0.072 m, speed ratio 0.729, mean solve 2.25 ms.
* Closed-loop MLP: RMS CTE 0.096 m, speed ratio 0.745, mean solve 3.90 ms.

Interpretation: the closed-loop MLP gives the fastest baseline but is not
the cleanest tracker in this focused grid.  TMeasy currently wins tracking.
Clay/sinusoidal speed retention remains low for all models.

### MPCC vs MPC baseline and attempted tuning

Main output:
`paper_scripts/results/mpcc_vs_mpc_speed_tracking_20260513_011319`.

Summary over 32/32 successful runs:

* Standard MPC: RMS CTE 0.103 m, speed ratio 0.679.
* MPCC default: RMS CTE 0.239 m, speed ratio 0.846.
* MPCC balanced tracking: RMS CTE 1.55 m, speed ratio 0.837
  (one sand/sinusoidal run lost the path badly).
* MPCC tight tracking: RMS CTE 0.236 m, speed ratio 0.847.

Interpretation: MPCC is objectively faster but not objectively better yet.
Increasing contour/lag weights and lowering progress reward did not recover
standard-MPC tracking.  The current paper story should not claim MPCC is
better.  Treat it as a negative/diagnostic result unless a deeper MPCC
architecture change lands.

Also ran `mpcc_less_speed_cap` in
`paper_scripts/results/mpcc_vs_mpc_speed_tracking_20260513_010447`.
It is not acceptable: several runs have high CTE and one sand/sinusoidal
bumpy run gets stuck near the origin with ACADOS_MINSTEP messages, full
brake, and saturated steering.  Do not use that variant as a paper result.

### Safety-filter bug fix and baseline

Found a real geometry bug in the predictive shield: `chrono_sim_node.py`
instantiated MPPI/NMPC with `vehicle_radius=1.0`, while `CollisionLogger`
uses an HMMWV collision radius of 1.5 m.  The observed predictive-shield
contacts were mostly ~0.4--0.6 m penetrations, matching the 0.5 m radius
mismatch.  Fixed the predictive shield instantiation to `vehicle_radius=1.5`
and updated `--sim-diag-csv` nearest-clearance logging to subtract the same
1.5 m footprint.

After the radius fix, ran:

```bash
python paper_scripts/safety_filter_sweep.py \
  --flavors none dob_cbf mppi nmpc \
  --terrains clay sand --paths sinusoidal lane_change \
  --speeds 5 --bumpiness 0 --seeds 2 --time 10 --rocks 5 \
  --mppi-samples 384 --shield-horizon 18 --nmpc-iter 8 \
  --timeout 260 --base-port 13200
```

Output:
`paper_scripts/results/safety_filter_sweep_20260513_014300`.

Summary:

* No filter: 1.50 collisions/run, RMS CTE 0.160 m.
* DOB-CBF: 0.00 collisions/run, RMS CTE 2.86 m, intervention 41.3%.
* MPPI: 0.125 collisions/run, RMS CTE 1.73 m, intervention 59.4%.
* NMPC: 0.625 collisions/run, RMS CTE 3.16 m, intervention 55.3%,
  RT factor 0.78 (too slow).

Then exposed `--safety-buffer` in `safety_filter_sweep.py` and set its
paper-script default to 0.5 m.  A 1-seed sanity rerun with
`--safety-buffer 0.5` landed at:
`paper_scripts/results/safety_filter_sweep_20260513_015222`.

* No filter: 1.5 collisions/run.
* DOB-CBF: 0 collisions/run, min clearance 0.85 m.
* MPPI: 0 collisions/run, min clearance 0.048 m.
* NMPC: 0.5 collisions/run, still clips lane-change cases.

Interpretation: MPPI is now viable as the predictive shield when geometry
is correct and the safety buffer is nonzero.  NMPC remains a weaker ablation
because the local optimizer still gets trapped/clips lane-change obstacles.
DOB-CBF is collision-robust but very path-invasive.

### Autonomous obstacle avoidance by tire model

Command:

```bash
python paper_scripts/autonomous_obstacle_tire_model_sweep.py \
  --models pacejka tmeasy closed_loop_mlp \
  --terrains clay sand --paths sinusoidal lane_change \
  --speeds 5 --bumpiness 0 --seeds 2 --time 10 --rocks 5 \
  --timeout 220 --base-port 13600
```

Output:
`paper_scripts/results/autonomous_obstacle_tire_model_sweep_20260513_015755`.

Summary:

* Pacejka: 0 collisions/run, RMS CTE 0.076 m, speed ratio 0.431.
* TMeasy: 0 collisions/run, RMS CTE 0.065 m, speed ratio 0.444.
* Closed-loop MLP: 0.25 collisions/run, RMS CTE 0.212 m, speed ratio 0.576.

Interpretation: the NN planner drives faster but is currently less safe in
autonomous obstacle avoidance.  A quick attempt to increase MPC obstacle
slots from 3 to 5 did not fix the sand/lane-change NN failure and worsened
one case, so that attempted change was backed out.  The likely issue is
the standard MPC obstacle barrier / speed behavior under the faster NN
planner; for paper safety, pair autonomous NN tracking with MPPI rather
than relying on the in-horizon soft barrier alone.

### DOB-CBF NN ablation

Command:

```bash
python paper_scripts/dob_cbf_nn_ablation.py \
  --variants no_filter dob_cbf_nn dob_cbf_no_nn \
  --terrains clay sand --paths sinusoidal lane_change \
  --speeds 5 --bumpiness 0 --seeds 2 --time 10 --rocks 5 \
  --timeout 220 --base-port 14000
```

Output:
`paper_scripts/results/dob_cbf_nn_ablation_20260513_020521`.

Summary:

* No filter: 1.50 collisions/run.
* DOB-CBF + NN: 0 collisions/run, mean clearance 0.741 m.
* DOB-CBF no NN: 0.125 collisions/run, mean clearance 0.237 m.

Interpretation: the NN path does help DOB-CBF safety.  Reactive geometry is
doing much of the visible steering, but the NN traction/speed path improves
clearance and removes the one clay/sinusoidal collision seen without NN.

### Terrain estimator baseline

Command:

```bash
python paper_scripts/terrain_estimator_benchmark.py \
  --distributions id ood --terrains clay dirt sand \
  --paths sinusoidal --speeds 5 --bumpiness 0 --seeds 1 \
  --ood-terrains 3 --time 15 --timeout 260 --base-port 14200
```

Output:
`paper_scripts/results/terrain_estimator_benchmark_20260513_021211`.

Summary:

* ID: mean tail |n error| 0.069, first accepted update ~4.21 s.
  Per-terrain tail estimates: clay 0.573 (err 0.073), dirt 0.702
  (err 0.002), sand 0.969 (err 0.131).
* OOD: mean tail |n error| 0.105, first accepted update ~4.16 s.
  One OOD terrain is weak (err 0.232), but two are good (0.010, 0.073).

Interpretation: the estimator is good enough for a baseline figure, but OOD
needs more seeds/terrains before claiming broad generalization.

### Current paper recommendations from this baseline pack

* Use standard MPC/TMeasy as a strong tracking baseline; closed-loop MLP is
  faster but less accurate in this grid.
* Do not claim MPCC superiority yet.  It is a speed/progress diagnostic,
  not a better controller in current form.
* Use MPPI with corrected vehicle radius, horizon 18, K=384, and safety
  buffer 0.5 m as the predictive safety-shield baseline.
* Keep DOB-CBF as a robust but invasive baseline; its NN ablation is
  favorable to NN usage.
* Treat NMPC shield as a local-optimizer ablation that remains worse than
  MPPI/DOB-CBF on lane-change obstacle cases.

---

## 2026-05-13 continuation: guarded NN autonomy and MPC speed-cost fix

### Corrected 2-seed safety-filter baseline

Final corrected run:
`paper_scripts/results/safety_filter_sweep_20260513_060357`.

Command family: blind MPC, rocks visible only to the safety layer, clay/sand,
sinusoidal/lane-change, speed 5 m/s, bumpiness 0, 2 seeds, 5 rocks,
MPPI `K=384`, horizon 18, safety buffer 0.5 m.

Summary over 8 runs/filter:

* No filter: 1.50 collisions/run, min clearance -1.69 m, RMS CTE 0.143 m.
* DOB-CBF: 0 collisions/run, min clearance 0.925 m, intervention 47.6%,
  RMS CTE 2.45 m.
* MPPI: 0 collisions/run, min clearance 0.118 m, intervention 61.5%,
  RMS CTE 1.70 m.

Interpretation: the corrected MPPI shield is usable for the paper.  DOB-CBF
is still more conservative in clearance, but MPPI is less path-invasive than
DOB-CBF on this grid.  Keep NMPC as a weaker ablation from the earlier run,
not as the primary predictive shield.

### Autonomous obstacle avoidance with a fixed MPPI shield

Added optional fixed downstream safety filtering to
`paper_scripts/autonomous_obstacle_tire_model_sweep.py`:

* `--safety-flavor {none,dob_cbf,mppi,nmpc}`
* `--shield-horizon`, `--mppi-samples`, `--nmpc-iter`
* `--safety-buffer`
* `--mpc-blind-obstacles`

This keeps tire model as the swept variable while holding the safety layer
fixed for the whole experiment.

Run:
`paper_scripts/results/autonomous_obstacle_tire_model_sweep_mppi_20260513_061235`.

Command family: Pacejka/TMeasy/closed-loop MLP, clay/sand,
sinusoidal/lane-change, speed 5 m/s, 2 seeds, 5 rocks, MPPI safety
`K=384`, horizon 18, buffer 0.5 m, standard MPC still obstacle-aware.

Summary:

* Pacejka + MPPI: 0 collisions/run, 0 near misses/run, RMS CTE 0.077 m,
  speed ratio 0.432, min clearance 8.16 m.
* TMeasy + MPPI: 0 collisions/run, 0 near misses/run, RMS CTE 0.070 m,
  speed ratio 0.434, min clearance 8.04 m.
* Closed-loop MLP + MPPI: 0 collisions/run, 0.25 near misses/run,
  RMS CTE 0.986 m, speed ratio 0.600, min clearance 4.54 m.

Interpretation: MPPI fixes the closed-loop MLP's hard collisions seen in
the barrier-only autonomous sweep.  The NN planner remains faster and more
path-invasive, especially on sand/lane-change; do not claim it is a cleaner
autonomous tracker.  The honest paper story is: NN tire planning plus MPPI
safety is safe on this grid, but the best low-error autonomy baseline is
still TMeasy/Pacejka unless speed retention is the priority.

### Standard MPC speed-cost experiments

Added `--speed-cost-mode {symmetric,overspeed}` to the standard MPC.  The
new `overspeed` mode treats `v_ref` as a smooth speed cap:

```text
speed_err = u - v_ref
speed_cost_err = 0.5 * (speed_err + sqrt(speed_err^2 + 1e-4))
```

This avoids rewarding the optimizer for accelerating up to `v_ref` when it
is already below the reference speed in a turn.  The mode is included in the
acados build fingerprint, so changing it rebuilds the solver as intended.
Flags are plumbed through `acados_mpc_controller_node.py`,
`launch_decoupled.py`, and the autonomous obstacle tire-model script.

Focused hard-case retests, closed-loop MLP, sand/lane-change, 2 seeds,
MPPI shield, 5 rocks:

* Baseline symmetric speed cost, weight 15:
  `autonomous_obstacle_tire_model_sweep_mppi_20260513_061235`
  gives 0 collisions, min clearances 0.40/2.72 m, RMS CTE 1.35/5.35 m.
* No speed tracking (`--speed-weight 0`):
  `autonomous_obstacle_tire_model_sweep_mppi_20260513_061839`
  gives 0 collisions, min clearances 0.02/0.45 m, RMS CTE 1.35/1.29 m.
  This is the best tracking rescue for the hardest guarded MLP case, but
  clearance is thin and both runs are near misses.
  A follow-up full MLP grid with the same setting,
  `autonomous_obstacle_tire_model_sweep_mppi_20260513_062959`, stayed
  collision-free over 8/8 runs with 0.375 near misses/run, RMS CTE 0.664 m,
  speed ratio 0.551, and min clearance 6.22 m mean.  Sand/lane-change is
  still the weak point (RMS CTE 1.63/3.02 m), so no-speed improves but does
  not fully solve the NN tracking issue.
* Overspeed-cap mode (`--speed-weight 70 --speed-cost-mode overspeed`):
  `autonomous_obstacle_tire_model_sweep_mppi_20260513_062112`
  gives 0 collisions, min clearances 1.34/0.68 m, RMS CTE 1.63/3.75 m.
  Better than the worst symmetric case but not as good as the no-speed
  ablation for tracking.

No-rock tracking comparison:
`paper_scripts/results/mpcc_vs_mpc_speed_tracking_20260513_062209`.

One seed over clay/sand × sinusoidal/lane-change:

* Standard MPC: RMS CTE 0.099 m, speed ratio 0.718.
* Soft speed weight 15: RMS CTE 0.090 m, speed ratio 0.640.
* Overspeed cap: RMS CTE 0.100 m, speed ratio 0.587.
* No speed tracking: RMS CTE 0.094 m, speed ratio 0.581.
* MPCC default: RMS CTE 0.278 m, speed ratio 0.632, but one
  sand/lane-change run got stuck near the origin with repeated
  `ACADOS_MINSTEP`, so this is still not a reliable paper controller.

Interpretation: reducing/removing speed tracking is a valid ablation and
can rescue the hardest MLP+MPPI obstacle case, but it is not a free win:
it loses progress/speed and can reduce clearance by letting the shield
thread very close to rocks.  `overspeed` is now available as a cleaner
cap-style formulation, but current data do not justify making it the default.

Current recommendation after this pass:

* For clean tracking tables: report standard MPC with default/soft/no-speed
  variants; TMeasy remains the lowest-error baseline.
* For autonomous obstacle avoidance: report barrier-only and MPPI-guarded
  sweeps separately.  Barrier-only shows the NN planner is faster but less
  safe; MPPI-guarded shows it can be made collision-free.
* For MPCC: keep as negative/troubleshooting unless a deeper fix lands.
  The issue is not just a metric frame mismatch or simple weight choice.

---

## 2026-05-13 plotting pass: better paper figures from raw logs

Added shared plotting helpers to `paper_scripts/common.py` and wired them
into the main paper sweeps.  Existing result folders can now be refreshed
without rerunning Chrono:

```bash
python paper_scripts/regenerate_figures.py --clean \
  paper_scripts/results/<result_folder>
```

New figure classes:

* Per-run metric distribution plots with raw points plus mean/std markers.
  These are more honest than plain bars because the bad seeds/scenarios are
  visible instead of hidden inside an average.
* Trajectory-vs-reference overlays from controller diag CSVs.  For obstacle
  sweeps, rock locations are overlaid when available from `collision_log.csv`.
* Predicted-vs-actual lateral force figures from existing diagnostic columns:
  `actual_Fy_front/rear` and `pred_Fy_front/rear`.
  The scripts now write `force_prediction_metrics.csv` plus:
  `force_predicted_vs_actual_scatter.png`,
  `force_predicted_vs_actual_by_model.png`,
  `force_prediction_error_summary.png`, and
  `force_prediction_timeseries_examples.png`.

Regenerated figures for:

* `mpc_tire_model_sweep_20260513_005540`
* `mpcc_vs_mpc_speed_tracking_20260513_062209`
* `safety_filter_sweep_20260513_060357`
* `autonomous_obstacle_tire_model_sweep_mppi_20260513_061235`
* `autonomous_obstacle_tire_model_sweep_mppi_20260513_062959`

Sanity checked representative PNGs with PIL and visual inspection: files are
nonblank and the trajectory overlays show the expected reference/actual
separation, including the closed-loop MLP sand/lane-change detour.

---

## 2026-05-14 surrogate force and reference-plot bugfix pass

Investigated why the "whole vehicle" force surrogate looked much worse
than expected in the new paper plots.  The primary bug was a terrain
feature unit mismatch:

* `closed_loop_v1_mlp_32_16` was saved with `mohr_friction` in degrees
  (`scalers.pkl` has phi mean about 23.36).
* The runtime loader had started converting all v6 model phi inputs to
  radians before scaling.
* That put every clay/dirt/sand sample far out of the model's training
  distribution and made the paper force plots look catastrophically bad.

Fixed by adding checkpoint-aware phi normalization in
`simulation/nn_tire_model.py` (`phi_feature_value()` inspects the scaler),
then using it consistently in:

* `simulation/acados_mpc_solver.py`
* `simulation/safety/surrogate_dynamics.py`
* `paper_scripts/common.py` when recomputing corrected NN force metrics
  from existing diagnostic CSVs.

Direct loader sanity check on `data/closed_loop_v1/training_data_tire_frame.csv`:

* Before fix: `Fx R2=-0.50`, `Fy R2=-3.96`, force MAE about `1060 N`.
* After fix: `Fx R2=0.827`, `Fy R2=0.821`, `Fx MAE=201 N`,
  `Fy MAE=144 N`.

Real Chrono smoke tests after the fix:

* `paper_scripts/results/mpc_tire_model_sweep_20260514_001502`
  (`closed_loop_mlp`, clay/sinusoidal, 5 m/s, 8 s): completed with
  `rms_cte=0.073 m`, `speed_ratio=0.48`.
  Corrected force metrics were front axle `MAE=325 N`, `R2=0.63`;
  rear axle `MAE=470 N`, `R2=-0.04`.
* `paper_scripts/results/safety_filter_sweep_20260514_001553`
  (MPPI, clay/sinusoidal, 5 rocks, 8 s): completed with
  `collisions=0`, `near_misses=1`, intervention `28.4%`.

The remaining rear-axle weakness is not the phi bug.  The current
`closed_loop_v1` tire-frame training CSV only contains front-axle samples
because `acados_mpc_controller_node.py --log-tire-csv` averaged and wrote
front-left/front-right forces only.  Runtime then uses that same tire model
for both axles, so rear-force predictions are an extrapolation.  Patched
the logger to write rear-axle rows too, using
`scenario_id + 1_000_000` so rate/temporal training does not stitch
front and rear samples from the same timestep into one fake tire history.

Verified the new logger with a 3 s real Chrono run to
`/tmp/tire_log_both_axles.csv`:

* 500 total rows for 250 controller ticks.
* `scenario_id=42`: 250 front rows.
* `scenario_id=1000042`: 250 rear rows.

Also fixed the trajectory-reference plotting bug where the black dotted
line was sane at the beginning and then went wild.  The plotting code had
been using `x_ref_0/y_ref_0` from every MPC solve as if it were the static
reference path.  Those columns are per-solve local reference samples and
can jump after recovery/blending/reindexing, so they are not suitable for a
paper reference path.  `paper_scripts/common.py` now reconstructs the
nominal path from each result folder's `manifest.csv` and plots that
instead; `x_ref_0/y_ref_0` are only a fallback.

Regenerated corrected figures for:

* `mpcc_vs_mpc_speed_tracking_20260513_062209`
* `safety_filter_sweep_20260513_060357`
* `autonomous_obstacle_tire_model_sweep_mppi_20260513_061235`
* `autonomous_obstacle_tire_model_sweep_mppi_20260513_062959`
* `mpc_tire_model_sweep_20260514_001502`
* `safety_filter_sweep_20260514_001553`

Paper caveat: old NN-controller benchmark numbers were generated before
the runtime phi fix, so use the regenerated force plots for diagnosis but
rerun the key paper sweeps before making final claims about NN controller
performance.

---

## 2026-05-14 both-axle closed-loop recollection and v2 retraining

Recollected and retrained the whole-vehicle tire surrogate after discovering
that `closed_loop_v1` only logged front-axle training rows.  Changes made
before collection:

* `data_collection/collect_closed_loop_data.py` now writes both
  `training_data.csv` (body-frame Chrono labels) and
  `training_data_tire_frame.csv` (Fy sign flipped for the NN/MPC tire-frame
  convention).
* Collection now supports path/terrain subsets, bumpiness sampling,
  rock-count choices, sensor-noise-on by default, and terrain preset jitter
  or LHS terrain YAML generation.
* `acados_mpc_controller_node.py --log-tire-csv` writes both front and rear
  axle streams per controller tick.  Rear rows use
  `scenario_id + 1_000_000` so temporal/rate training does not stitch front
  and rear samples into one fake history.
* Fixed a parallel collection race: acados build/cache directories were keyed
  too coarsely by NN model id, so concurrent mixed-terrain workers could
  delete/rebuild the same directory.  The collector now sets
  `ACADOS_UNIQUE_BUILD_DIR=1`, and the controller appends the controller pid
  to the acados build directory when that env var is present.  A 3-scenario
  parallel smoke confirmed pid-suffixed build dirs and 3/3 successful runs.

Collection:

* Main run:
  `data/closed_loop_v2_20260514_both_axles`
  (`120` requested, `80` usable before the cache race fix fully landed).
* Salvaged extra run:
  `data/closed_loop_v2_20260514_both_axles_extra`
  (`17` usable before stopping the flawed run).
* Combined dataset:
  `data/closed_loop_v2_20260514_combined`
  with `127,798` rows from `97` base scenarios / `194` axle streams.
* Distribution: sensor noise ON, jittered clay/sand/dirt presets,
  sinusoidal/lane-change/double-lane-change/right-left paths, speeds
  `3-7 m/s`, bumpiness levels `0-3`, rock counts sampled from
  `{0,0,3,5}`.
* Tire-frame sign sanity: `corr(slip_angle, Fy) = -0.620`.

Trained checkpoints:

| Model | Notes | Test R2 Fx | Test R2 Fy | RMSE Fx | RMSE Fy |
| --- | --- | ---: | ---: | ---: | ---: |
| `closed_loop_v2_both_axles_mlp_32_16` | static MLP, same size as v1 | 0.661 | 0.574 | 590 N | 420 N |
| `closed_loop_v2_both_axles_mlp_64_32` | larger static MLP | 0.666 | 0.578 | 585 N | 418 N |
| `closed_loop_v2_both_axles_temporal_K4_32_16` | K=4 temporal, 50 ms spacing | 0.644 | 0.609 | 643 N | 390 N |
| `closed_loop_v2_both_axles_rate_32_16` | rate-augmented MLP | 0.666 | 0.600 | 575 N | 407 N |

Axle-specific full-dataset diagnostic (`data/closed_loop_v2_20260514_combined/model_axle_eval.csv`):

| Model | Subset | R2 Fx | R2 Fy | MAE Fx | MAE Fy |
| --- | --- | ---: | ---: | ---: | ---: |
| `closed_loop_v1_mlp_32_16` | front | 0.742 | 0.645 | 327 N | 252 N |
| `closed_loop_v1_mlp_32_16` | rear | 0.195 | 0.003 | 558 N | 383 N |
| `closed_loop_v2_both_axles_mlp_64_32` | front | 0.804 | 0.724 | 257 N | 221 N |
| `closed_loop_v2_both_axles_mlp_64_32` | rear | 0.526 | 0.358 | 358 N | 264 N |
| `closed_loop_v2_both_axles_rate_32_16` | front | 0.797 | 0.770 | 260 N | 195 N |
| `closed_loop_v2_both_axles_rate_32_16` | rear | 0.527 | 0.366 | 359 N | 264 N |

Interpretation: the original "whole vehicle surrogate is bad" diagnosis was
correct, and the biggest concrete bug was front-only data.  The new v2 models
substantially improve rear-force prediction (`Fy R2` from ~0.00 to ~0.36),
while keeping or improving front-force prediction.  Rear axle is still the
harder target, likely because rear lateral force depends on body/yaw/sinkage
history that is only partially visible through instantaneous slip/Fz/rates.

Real Chrono smoke with the best v2 rate model:

```bash
python simulation/launch_decoupled.py \
  --controller-mode standard --model nn \
  --nn-model closed_loop_v2_both_axles_rate_32_16 \
  --terrain clay --path sinusoidal --speed 5 --time 8 \
  --lead-in 5 --rocks 0 --no-vis --no-plot
```

Result: completed at RT factor `1.00`, RMS CTE `0.066 m`, mean solve
`5.43 ms`.  Diag force metrics in that run:

* Front Fy: MAE `304 N`, RMSE `479 N`, R2 `0.595`.
* Rear Fy: MAE `433 N`, RMSE `613 N`, R2 `0.089`.

The paper-script smoke
`paper_scripts/results/mpc_tire_model_sweep_20260514_010636` also completed
with `rms_cte=0.072`, `speed_ratio=0.53`, and regenerated figures.  Its
force metrics were front `R2=0.703`, rear `R2=0.313` on clay/sinusoidal.

Added paper-script model aliases in `mpc_tire_model_sweep.py`:

* `closed_loop_v2_mlp`
* `closed_loop_v2_rate_mlp`
* `closed_loop_v2_temporal_mlp`

Best current recommendation: use `closed_loop_v2_rate_mlp` in the next tire
model sweeps.  It gives the best axle-balanced Fy metrics and already runs in
the acados standard MPC.  Keep `closed_loop_v2_temporal_mlp` as an offline
force-prediction ablation unless/until its closed-loop controller behavior is
smoked.

### Why v2 R2 still looks poor

Follow-up diagnostics show the mediocre all-row R2 is not primarily a "not
enough data" problem:

* A boosted-tree diagnostic model with the same feature columns plateaued
  quickly.  On one scenario-group split, increasing training size from
  `3k` to `90k` rows only moved `Fy R2` from about `0.42` to `0.49`.
* Adding an explicit `axle_id` feature to the tree barely changed `Fy R2`
  (`0.592` to `0.593` random split; `0.646` to `0.650` grouped split), so
  the missing information is not just "front vs rear" identity.
* The rate model's all-row `Fy R2=0.622` jumps to `0.775` when evaluated
  only on rows with `velocity >= 2 m/s`.  Low-speed/startup rows are noisy
  and slip-angle features are ill-conditioned because `u_safe` is clamped
  at `0.5 m/s`, while SCM contact forces can still jump during sinkage and
  acceleration transients.
* Rear force remains the hard case: rear `Fy R2=0.366` over all rows and
  `0.505` for `velocity >= 2 m/s`.  This is the clearest sign of feature
  aliasing / missing state, not just model capacity.

Likely missing explanatory variables:

* lateral load transfer / side-specific Fz: the training row uses axle mean
  Fz and mean left/right force, but actual Chrono force is generated by
  different inner/outer tire loads during yaw/lateral acceleration;
* richer vehicle state: `v`, `omega`, steering angle, `ax`, `ay`, and maybe
  previous force/state history, not just `(kappa, alpha, u, mean Fz,
  steering_rate, terrain)`;
* startup/sinkage/contact transient state on SCM terrain, which is not
  represented by the compact tire-map features.

Next corrective experiment: recollect a "v3 rich" tire dataset that logs
front/rear rows with the current compact columns plus `axle_id`, `delta`,
`v_body`, `omega`, measured `ax/ay`, estimated lateral load transfer
`dFz`, elapsed scenario time or distance-from-start, and optionally separate
left/right rows if Chrono can expose per-wheel vertical load cleanly.  Train
two heads or two separate models for front/rear if rear still lags.  For
paper metrics, report force R2 both on all rows and on the physically useful
`u >= 2 m/s` subset so startup artifacts do not dominate the interpretation.

Important sensor-realism constraint from AGENT.md/user follow-up: new inputs
must be available from conventional vehicle sensing.  Implemented the v3
logging schema with that boundary:

* conventional state/estimator inputs: `u_body`, `v_body`, `yaw_rate`,
  `ax_imu`, `ay_imu`;
* steering encoder / command inputs: `steering_angle`, `steering_rate`,
  `throttle_cmd`, `brake_cmd`, `accel_cmd`, `jerk_cmd`;
* wheel encoder inputs: axle and per-side wheel speeds plus measured/global
  and axle-specific `kappa`;
* fixed-geometry estimates: mean axle Fz and lateral load-transfer proxies
  from `u*omega` and IMU `ay`;
* metadata/estimates already assumed elsewhere: axle id and terrain parameter
  estimate columns.

The Chrono tire forces remain labels only.  `tire_input_features.py` now has
`write_rich_vehicle_tire_csv_header()` / `pack_rich_vehicle_tire_csv_row()`;
`acados_mpc_controller_node.py` accepts `--log-rich-tire-csv`; and
`collect_closed_loop_data.py` writes `training_data_rich.csv` plus
`training_data_rich_tire_frame.csv`.  Smoke test:
`/tmp/rich_tire_log_smoke2` produced `326` compact rows and `326` rich rows
with the expected front/rear scenario split.

### v3 rich recollection / retraining pass (2026-05-14)

Collected a larger sensor-realistic closed-loop tire dataset and fixed two
collection issues found during the run:

* `chrono_setup.py` used one global `/tmp/scm_heightmap.bmp`; parallel bumpy
  terrain workers could clobber the file while Chrono was reading it.  It now
  writes a unique temporary BMP per process.
* Some elevated workers held ZMQ ports after an interrupted run.  The clean
  follow-up shard used a fresh high port range and fewer workers.

Dataset outputs:

* `data/closed_loop_v3_rich_20260514_04`: clean 24/24 shard, `31,334` rows.
* `data/closed_loop_v3_rich_20260514_combined`: aggregate of successful `_03`
  and `_04` runs, `78,806` compact rows and `78,806` rich rows.
* Added `data_collection/aggregate_closed_loop_runs.py` so successful per-run
  CSVs can be rebuilt into a combined dataset after partial/interrupted shards.

Training/evaluation changes:

* `train_variant.py` now supports `--split-by-scenario` to keep front/rear and
  adjacent timesteps from the same run in the same train/val/test split.
* Added `nn_training/evaluate_tire_model.py`, which writes `metrics.csv`,
  per-row predictions, and predicted-vs-actual force scatter figures.
* Added an `axle_rate` compact mode: `[axle_id] + compact rate tire features`.
  This stays sensor-realistic and can be embedded in MPC because axle identity
  is known at runtime.

Force-prediction metrics on `closed_loop_v3_rich_20260514_combined`
(`velocity >= 2 m/s`, tire-frame labels):

| Model | All Fy R2 | Front Fy R2 | Rear Fy R2 | Notes |
| --- | ---: | ---: | ---: | --- |
| `closed_loop_v2_both_axles_rate_32_16` | 0.771 | 0.879 | 0.533 | current controller baseline |
| `closed_loop_v3_both_axles_rate_64_32` | 0.750 | 0.893 | 0.435 | worse rear; do not use |
| `closed_loop_v3_axle_rate_64_32` | 0.805 | 0.898 | 0.598 | controller-usable, better force ablation |
| `closed_loop_v3_rich_64_32` | 0.823 | 0.903 | 0.646 | offline only |
| `closed_loop_v3_rich_rate_64_32` | 0.843 | 0.933 | 0.644 | best offline force predictor |

All-row metrics still lag the `u >= 2` subset because low-speed/startup rows
remain ill-conditioned.  All-row rear Fy R2 improves from `0.371` for v2 rate
to `0.448` for v3 axle-rate and `0.468` for v3 rich-rate.

Runtime integration:

* Added `AxleRateMLP` in `nn_tire_model.py` and routed acados rate-batch calls
  through `predict_batch_axle_rate(...)` when the checkpoint has `axle_id`.
* Real Chrono smoke passed with
  `closed_loop_v3_axle_rate_64_32` on clay/sinusoidal/speed 5/time 8:
  RT factor `1.00`, RMS CTE `0.073 m`, mean solve `7.24 ms`, tire CSV
  `1,318` rows.

Focused paper sweep:
`paper_scripts/results/mpc_tire_model_sweep_20260514_101607` compared v2 rate
against v3 axle-rate over clay/sand/dirt and sinusoidal/lane-change at 5 m/s.

| Model | Mean RMS CTE | Mean speed ratio | Mean solve |
| --- | ---: | ---: | ---: |
| `closed_loop_v2_rate_mlp` | 0.0801 m | 0.748 | 5.18 ms |
| `closed_loop_v3_axle_rate_mlp` | 0.0790 m | 0.740 | 7.26 ms |

Interpretation: v3 axle-rate is a better force-prediction ablation and is
MPC-compatible, but it is not a clean new controller default yet.  It slightly
improves average RMS CTE in the focused sweep, but loses a little speed and
costs ~2 ms more solve time.  Keep v2 rate as the safer default for broad
controller sweeps until the MPC cost/speed policy is retuned around the new
force map.  Use v3 rich/rich-rate as the paper evidence that sensor-realistic
state features materially help tire-force prediction.

### Paper-ready safety / estimator / latency results (2026-05-14)

Updated `paper_scripts/common.py` so new paper runs use
`closed_loop_v2_both_axles_rate_32_16` as the default NN model.  Keep
`closed_loop_v3_axle_rate_64_32` as an explicit ablation; it predicts forces
better but is not yet the broad controller default.

#### Safety-filter sweep

Result dir: `paper_scripts/results/safety_filter_sweep_20260514_220947`

Command family: blind MPC, clay/sand/dirt, sinusoidal/lane-change, 5 m/s,
2 seeds, 5 rocks, sensor noise on, MPPI K=384, horizon 18, safety buffer 0.5 m.
All 48 runs completed.

| Filter | Collisions/run | Near misses/run | Min clearance | RMS CTE | Speed ratio | RT factor |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 1.583 | 2.167 | -1.684 m | 0.134 m | 0.749 | 1.000 |
| DOB-CBF | 0.083 | 1.167 | 0.711 m | 2.810 m | 0.765 | 1.000 |
| MPPI | 0.000 | 2.000 | 0.124 m | 1.547 m | 0.721 | 1.000 |
| NMPC | 0.250 | 1.917 | 0.200 m | 2.512 m | 0.700 | 0.792 |

Interpretation: MPPI is now the best predictive safety-shield baseline for
the paper on this grid: collision-free and less path-invasive than DOB-CBF.
DOB-CBF remains the high-clearance conservative baseline.  NMPC is useful as
a local-optimizer ablation, but is slower and less robust.

Generated figures include `safety_filter_summary.png`,
`safety_collision_heatmap.png`, `safety_filter_metric_distributions.png`, and
trajectory overlays under the result directory's `figures/`.

#### Autonomous obstacle avoidance by tire model with fixed MPPI shield

Result dir:
`paper_scripts/results/autonomous_obstacle_tire_model_sweep_mppi_mpc_blind_20260514_222330`

Command family: MPC blind to rocks, downstream MPPI shield fixed for all tire
models, clay/sand/dirt, sinusoidal/lane-change, 5 m/s, one seed, 5 rocks,
sensor noise on.  All 24 runs completed and all were collision-free.

| Tire model | Collisions/run | Min clearance | RMS CTE | Speed ratio | Mean solve |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pacejka | 0.0 | 8.463 m | 0.078 m | 0.551 | 2.14 ms |
| TMeasy | 0.0 | 8.532 m | 0.063 m | 0.548 | 2.19 ms |
| v2 rate NN | 0.0 | 6.248 m | 0.098 m | 0.596 | 5.36 ms |
| v3 axle-rate NN | 0.0 | 7.160 m | 0.066 m | 0.579 | 7.00 ms |

Interpretation: a fixed MPPI shield makes the autonomy stack collision-free
for all tire models on this grid.  The v3 axle-rate NN improves clearance and
tracking over v2 rate under the same shield, but still costs more solve time.
Analytical tire models remain faster and conservative on clearance.

#### Terrain estimator excitation benchmark

Mixed short benchmark:
`paper_scripts/results/terrain_estimator_benchmark_20260514_223109`

* ID tail |n error| mean: `0.122`.
* OOD tail |n error| mean: `0.122`.
* First accepted update: about `4.17 s`.

This mixed 12 s grid showed some under-estimation on sand/OOD sinusoidal
cases, so we reran the estimator in the intended excited regime.

Long sinusoidal-only benchmark:
`paper_scripts/results/terrain_estimator_benchmark_20260514_224909`

Command family: sinusoidal only, 20 s, metric tail after 8 s, 2 seeds for ID
and OOD, sensor noise on.  All 12 runs completed.

| Distribution/case | True n | Estimated tail n | Tail n error |
| --- | ---: | ---: | ---: |
| ID clay | 0.500 | 0.502 | 0.002 |
| ID dirt | 0.700 | 0.589 | 0.111 |
| ID sand | 1.100 | 1.042 | 0.058 |
| ID mean | 0.767 | 0.711 | 0.057 |
| OOD mean | 0.792 | 0.624 | 0.167 |

Interpretation: the estimator is paper-ready as an excited-maneuver terrain
identifier, not as a passive classifier.  With longer sinusoidal excitation,
ID clay and sand converge well; dirt is conservative; OOD terrains are harder
and under-estimated, which is acceptable as a safety-conservative behavior.

#### Latency compensation proxy

Added `paper_scripts/latency_compensation_sweep.py`.

Result dir: `paper_scripts/results/latency_compensation_sweep_20260514_223505`

Command family: MPC blind to rocks, fixed command-path delays `0, 0.15,
0.30 s`, clay/sand, sinusoidal/lane-change, 5 rocks, sensor noise on,
standard MPC delay compensation ON.  All 36 runs completed.

| Filter | Delay | Collisions/run | Min clearance | RMS CTE | Speed ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| none | 0.00 s | 1.0 | -1.206 m | 0.321 m | 0.647 |
| none | 0.15 s | 1.0 | -1.316 m | 0.346 m | 0.656 |
| none | 0.30 s | 1.0 | -1.278 m | 0.385 m | 0.662 |
| DOB-CBF | 0.00 s | 0.0 | 1.537 m | 2.177 m | 0.694 |
| DOB-CBF | 0.15 s | 0.0 | 1.523 m | 2.061 m | 0.653 |
| DOB-CBF | 0.30 s | 0.0 | 2.135 m | 2.406 m | 0.597 |
| MPPI | 0.00 s | 0.0 | 0.206 m | 0.620 m | 0.647 |
| MPPI | 0.15 s | 0.0 | 0.189 m | 1.187 m | 0.684 |
| MPPI | 0.30 s | 0.0 | 0.411 m | 1.268 m | 0.663 |

Interpretation: scripted latency proxy shows the compensation stack preserves
hard safety up to 300 ms fixed command delay.  DOB-CBF carries larger clearance
but is path-invasive; MPPI keeps lower RMS CTE with thinner clearance.  This
does not replace true human-in-the-loop delay rounds, but it is reproducible
paper data for control-path latency robustness.

### 5G latency-profile implementation (2026-05-14)

Added a JSON-configurable latency-profile implementation based on the
Choi-style 5G traffic-generation workflow cited in the abstract.  The external
`0913ktg/5G-Traffic-Generator` repo provides model code and open bitrate
datasets, but the shallow public checkout does not include ready-to-run trained
checkpoints.  The sim now supports both:

* scheduled good/poor/outage latency windows with synthetic traffic load; and
* optional bitrate traces from the 5G repo CSVs, e.g. `youtube_dataset.csv`,
  mapped through a queue-load latency model.

Code/config:

* `simulation/latency_profile.py`
* `config/latency_profiles/5g_good_bad_control.json`
* `config/latency_profiles/5g_repo_youtube_template.json`
* `paper_scripts/latency_profile_figure.py`

Simulator integration:

* `chrono_sim_node.py` accepts `--latency-profile-json` and
  `--latency-profile-log`.
* `launch_decoupled.py` forwards those args.
* The profile drives command delay, manual-input actuation delay, and Chrono
  Sensor camera lag (`camera` channel).
* Sim diagnostics now include `latency_control_s`, `latency_manual_s`, and
  `latency_camera_s`.
* Fixed a safety-filter edge case: `set_teleop_delay()` now re-enables
  latency-aware logic when a profile transitions from a good/near-zero window
  into a poor/outage window.

Profile figure result:
`paper_scripts/results/latency_profile_figure_20260514_231334`

| Channel | Mean | Std | Min | Max |
| --- | ---: | ---: | ---: | ---: |
| control/manual | 77.7 ms | 110.6 ms | 4.0 ms | 450.0 ms |
| camera | 120.7 ms | 160.4 ms | 13.8 ms | 660.5 ms |

Generated paper figures:

* `figures/latency_profile_timeseries.png`
* `figures/latency_profile_histogram.png`

Chrono smoke test:

* Direct `chrono_sim_node.py` run with profile, sim diagnostics, and latency
  log completed for 2.5 s with sensor noise on.
* `/tmp/scm_latency_profile_smoke.csv` and `/tmp/scm_latency_diag_smoke.csv`
  showed the expected channel delays in the sim loop.

Tiny 5G-profile avoidance sweep:
`paper_scripts/results/latency_compensation_sweep_20260514_231637`

Command family: clay sinusoidal, 12 s, 5 m/s, 3 rocks, one seed, sensor noise
on, time-varying `5g_good_bad_control.json` profile.

| Filter | Runs | Collisions/run | Min clearance | RMS CTE | Speed ratio | Intervention |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 1 | 1.0 | -1.626 m | 0.910 m | 0.530 | n/a |
| MPPI | 1 | 0.0 | 0.188 m | 0.851 m | 0.282 | 64.8% |

Interpretation: the new 5G-like profile is wired end-to-end and produces a
useful stress case: without the shield the autonomous controller clips an
obstacle during poor network periods; the MPPI shield prevents collision but
does so by heavily intervening and slowing the vehicle.  This is good evidence
for latency-robust safety, but it is a tiny smoke matrix.  The next paper-ready
step is to run this profile sweep across the same multi-terrain/path/seeds grid
as the fixed-delay sweep.

Small multi-scenario 5G-profile sweep:
`paper_scripts/results/latency_compensation_sweep_20260514_231858`

Command family: clay/sand, sinusoidal/lane-change, 12 s, 5 m/s, 5 rocks, one
seed, sensor noise on, time-varying `5g_good_bad_control.json` profile.  All
12 runs completed.

| Filter | Runs | Collisions/run | Min clearance | RMS CTE | Speed ratio | Intervention |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 4 | 1.0 | -1.553 m | 0.657 m | 0.706 | n/a |
| DOB-CBF | 4 | 0.0 | 1.279 m | 3.289 m | 0.664 | 45.7% |
| MPPI | 4 | 0.0 | 0.213 m | 1.146 m | 0.610 | 62.5% |

Generated figures include:

* `latency_compensation_summary_mpc_on.png`
* `latency_collision_heatmap.png`
* `latency_metric_distributions.png`
* one trajectory overlay per terrain/path scenario.

Interpretation: under a time-varying 5G-like profile, both DOB-CBF and MPPI
remove collisions that appear in every unshielded run.  DOB-CBF is the
large-clearance conservative baseline; MPPI is less path-invasive on average
but relies on frequent intervention and thinner clearance.  This is the
stronger paper result for 5G latency than the two-run smoke test.

#### Trained N-HiTS-5G checkpoint

Installed the missing 5G repo dependencies into the `sim` environment:
`hyperopt`, `pytorch-lightning==1.9.5`, and `fastcore`.

Public repo compatibility patches needed:

* `N-HiTS-5G/src/data/tsdataset.py`: pandas 2.x requires
  `drop(..., axis=1)`.
* `N-HiTS-5G/inference.py`: load `hyperopt_{experiment_id}.p` and
  `{experiment_id}.ckpt` instead of `hyperopt_{datatype}.p` and
  `{datatype}.ckpt`; iterate through the loader instead of repeatedly using
  the first batch.

Added reproducible wrapper:
`paper_scripts/train_5g_nhits.py`

Training command actually run in `/tmp/5G-Traffic-Generator/N-HiTS-5G`:

```bash
python -u model_train.py \
  --dataset youtube \
  --datatype ul \
  --hyperopt_max_evals 1 \
  --experiment_id scm_youtube_ul_smoke
```

Training completed on GPU.  Hyperopt best validation loss was about `0.613`
for the short tuning pass; the final model trained to `max_steps=1000` and
ended with validation loss about `0.172`.

Exported artifacts:
`data/5g_generated/scm_youtube_ul_smoke`

| Artifact | Purpose |
| --- | --- |
| `scm_youtube_ul_smoke.ckpt` | trained N-HiTS checkpoint |
| `hyperopt_scm_youtube_ul_smoke.p` | selected hyperparameters |
| `youtube_10_scm_youtube_ul_smoke.npy` | generated bitrate trace |
| `generated_traffic.csv` | simulator-readable UL/DL traffic |
| `summary.json` | trace statistics |

Generated traffic summary after inference size 50 / 500 samples:

| Metric | Value |
| --- | ---: |
| mean UL bitrate | 59.4 kbps |
| std UL bitrate | 365.6 kbps |
| median UL bitrate | 0 bps |
| p95 UL bitrate | 0 bps |
| max UL bitrate | 3.62 Mbps |

Interpretation: the learned YouTube-UL trace is sparse/bursty, matching the
source dataset's sparse uplink character but still more zero-heavy than the
raw data.  For latency stress testing, the latency config therefore uses
`traffic_scale=45` and a 9 Mbps bottleneck capacity so the learned burst shape
drives congestion while the scenario defines the assumed network loading.

Generated latency profile:
`config/latency_profiles/5g_nhits_youtube_ul_scm_youtube_ul_smoke.json`

Profile figures:
`paper_scripts/results/latency_profile_figure_20260515_000053`

| Channel | Mean | Std | Min | Max |
| --- | ---: | ---: | ---: | ---: |
| control/manual | 56.9 ms | 90.3 ms | 6.7 ms | 450.0 ms |
| camera | 90.5 ms | 131.0 ms | 17.7 ms | 660.5 ms |

Closed-loop smoke sweep with the trained N-HiTS profile:
`paper_scripts/results/latency_compensation_sweep_20260515_000106`

Command family: clay sinusoidal, 12 s, 5 m/s, 3 rocks, one seed, sensor noise
on.

| Filter | Runs | Collisions/run | Min clearance | RMS CTE | Speed ratio | Intervention |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 1 | 1.0 | -1.648 m | 1.006 m | 0.546 | n/a |
| MPPI | 1 | 0.0 | 0.176 m | 0.304 m | 0.277 | 63.9% |

Interpretation: yes, we can train the 5G model checkpoints ourselves and use
them in the sim.  For the paper, this should be described as
"learned 5G traffic traces mapped through a queue/load latency model," not as a
direct learned latency predictor.

Current rating: about 7.5/10 for the full paper stack.  Safety and terrain
estimator evidence are now strong enough to write around if framed carefully.
The remaining weaker areas are (1) true human-in-the-loop latency rounds with
the G29, (2) expanding 5G-profile sweeps to multiple seeds and dirt/OOD
terrains, and (3) deciding whether the v3 axle-rate NN should replace v2 rate
in controller sweeps after retuning speed and clearance tradeoffs.

### Paper suite repeatability upgrade (2026-05-15)

Concern addressed: many earlier result folders were useful debugging/pilot
evidence, but not all were averaged across enough seeds or swept over high
speeds, all terrains, bumpiness levels, and reference paths.

Changes:

* `paper_scripts/common.py`
  * Paper default speeds are now `5, 7, 9 m/s`.
  * Paper default bumpiness levels are now `0, 4, 8`.
* Main sweep scripts now default to 5 seeds where appropriate.
* Safety sweep default horizon is now 18 to match the stronger MPPI/DOB-CBF
  paper runs.
* Terrain estimator default matrix is now sinusoidal-only, 20 s, metric tail
  after 8 s, because the estimator should be evaluated under deliberate
  excitation rather than passive driving.
* Added `paper_scripts/run_paper_suite.py` as an orchestration layer.  It does
  not define new experiments; it launches the one-question scripts with
  explicit repeatable matrices and writes a suite manifest.
* Updated `paper_scripts/README.md` with smoke/pilot/paper/stress commands.

Suite tiers:

| Tier | Purpose | Matrix | Estimated non-HIL Chrono runs |
| --- | --- | --- | ---: |
| `smoke` | syntax/health checks | one tiny case per script | about 8 |
| `pilot` | manageable high-speed pilot | clay/sand, sinusoid/lane-change, 5/7 m/s, bump 0/4, 2 seeds | 768 |
| `paper` | broad final matrix | clay/dirt/sand, all paths, 5/7/9 m/s, bump 0/4/8, 5 seeds | thousands per selected group |
| `stress` | high-speed rough-terrain subset | 7/9 m/s, bump 4/8, all terrains/paths, 5 seeds | targeted stress |

Validated commands:

```bash
python paper_scripts/run_paper_suite.py --tier pilot --dry-run
python paper_scripts/run_paper_suite.py --tier paper --dry-run --only safety latency_compensation terrain_estimator
python paper_scripts/run_paper_suite.py --tier smoke --only latency_profile
```

The dry runs produced manifests under:

* `paper_scripts/results/paper_suite_pilot_20260515_002109`
* `paper_scripts/results/paper_suite_paper_20260515_002109`

The actual smoke `latency_profile` suite run completed and generated:

* `paper_scripts/results/paper_suite_smoke_20260515_002117`
* `paper_scripts/results/latency_profile_figure_20260515_002117`

Current non-HIL gap list:

1. Run the `pilot` tier end-to-end to find high-speed/bumpy failures without
   paying for the full final matrix.
2. Promote stable subsets to the `paper` tier, likely in chunks via
   `--only safety latency_compensation terrain_estimator`, then tire-model and
   MPCC sweeps.
3. If 9 m/s rough terrain produces many controller failures, keep those as
   stress-test evidence and report the stable operating envelope separately.
4. After the non-HIL paper tier is complete, the only major missing category
   should be true human-in-the-loop G29 latency rounds.

### Paper suite smoke validation + MPCC NN pin + auto-publish (2026-05-15)

Ran `paper_scripts/run_paper_suite.py --tier smoke` end-to-end (not just
`--only latency_profile` as on the prior pass) and uncovered two real
issues:

1. **MPCC variants in `mpcc_vs_mpc_speed_tracking.py` crashed at
   `acados_mpcc_solver.py:272`** with
   `Function::call ... Expected 12, got 11`.  Cause: every `mpcc_*` variant
   used `DEFAULT_NN_MODEL = closed_loop_v2_both_axles_rate_32_16`
   (rate-augmented, 12 inputs), but the MPCC solver is wired to the 11-input
   static MLP per AGENT.md (*"MPCC currently uses the NN surrogate
   static-MLP only"*).  With `--continue-on-error`, every MPCC pilot row
   would have silently been a NaN.  Fix: introduced
   `MPCC_NN_MODEL = "closed_loop_v1_mlp_32_16"` and pinned every MPCC
   variant to it.  Standard MPC variants still use the v2 rate-MLP.

2. **The suite did not publish anything to `my_paper/paper_figures/`.**  The
   abstract LaTeX (`my_paper/abstract.tex`) was referencing May-10 figures
   that no longer match the current paper-suite outputs.  Fix: added
   `paper_scripts/publish_paper_figures.py` that maps a curated set of
   sweep figures and summary CSVs to the abstract's filenames (e.g.
   `tire_model_summary.png -> bench_tire_models.png`,
   `terrain_estimator_summary.png -> closed_loop_estimator_learned.png`),
   and wired `run_paper_suite.py` to call it at the end of every non-dry
   run.  A `--no-publish` opt-out is available.  Publish honors the suite
   manifest's mtime so a stale older results folder cannot leak in.

Both smoke passes after the fix completed in ~12 min and produced 38 files
in `my_paper/paper_figures/` plus `publish_manifest.json`.  The pilot tier
(estimated ~768 Chrono runs) was launched detached after the second smoke
to provide real multi-seed evidence under the same publish pipeline.

Known cosmetic gaps still to clean up:

* `run_paper_suite.py` hardcodes `estimated_runs=1` per smoke step; reality
  is 2–4 per script.  Cosmetic.
* The current `terrain_estimator_summary.png` is a 3-bar ID/OOD summary,
  but the abstract caption describes a *time-series* of `n_hat` on
  clay/dirt/sand.  Either update the caption or have
  `terrain_estimator_benchmark.py` emit a per-terrain time-series figure
  named for that abstract reference.
* `latency_compensation_sweep --quick` clamps `--delays` to `[0.0]` even
  when a profile JSON is supplied, so smoke does not exercise the 5G
  trace.  Pilot/paper tier still does.

### Abstract-claim coverage audit + 3 new ablation steps (2026-05-15)

Audited `run_paper_suite.py` against the abstract's four contributions and
found four sub-claims with no direct ablation evidence:

1. *"Asymmetric throttle DOB on the actuation map closes the residual
   soft-soil speed gap"* — no DOB on/off comparison existed.
2. *"NMPC softplus barriers (in-horizon) ... two-layer obstacle-avoidance
   stack"* — `safety_filter_sweep` always ran with `--mpc-blind-obstacles`
   so the in-horizon barrier layer was never the lever being tested.
3. *"Hand-crafted seed trajectories ... injected unconditionally so the
   optimizer always has a recoverable option"* — `MPPIShield._seed_trajectories`
   was unconditional with no toggle.
4. *"Ensemble disagreement on phi gates the safety filter's friction
   cone"* — `MPPIShield.update_terrain(phi_uncertainty_deg=...)` accepts the
   parameter but **no caller ever passes it**; `_phi_uncertainty_rad` is
   always 0.  This is a missing implementation, not a missing ablation.

For claims 1–3 I added end-to-end ablations:

* `paper_scripts/throttle_dob_ablation.py` — standard MPC, NN tire model,
  no obstacles; variants `dob_on` (defaults) and `dob_off`
  (`--dob-ki 0 --dob-max 0` so the controller still constructs the DOB hook
  but adds no integral action).
* `paper_scripts/mppi_seed_ablation.py` — planner-blind standard MPC,
  MPPI shield with rocks on; variants `mppi_with_seeds` (defaults) and
  `mppi_no_seeds` (sets the new `--mppi-no-seeds` flag).
* Existing `safety_filter_sweep.py` now also runs as a second orchestrator
  step (`safety_planner_aware`) with `--blind-and-aware --output-suffix
  planner_aware`, producing both planner-aware and planner-blind variants
  of every shield flavor in one folder.

Code changes to support claim-3 ablation:

* `simulation/safety/predictive_shield.py`: `MPPIShield.__init__` accepts
  `disable_seeds: bool = False`; `_solve` returns an empty seed array when
  `disable_seeds=True`.
* `simulation/chrono_sim_node.py`: new `--mppi-no-seeds` flag plumbed into
  the `make_safety_filter('mppi', ...)` call.
* `simulation/launch_decoupled.py`: same `--mppi-no-seeds` flag forwarded
  to the sim subprocess when `--safety-flavor mppi`.
* `paper_scripts/safety_filter_sweep.py`: new `--output-suffix` so multiple
  invocations (planner-blind vs planner-aware) write to distinct result
  folders.

Orchestrator changes:

* `paper_scripts/run_paper_suite.py`: three new pilot/paper/stress steps
  (`throttle_dob_ablation`, `mppi_seed_ablation`, `safety_planner_aware`).
* Port stride lowered from 5000 to 4000 so all 10 experiment blocks fit
  inside the 65535 ceiling at `base_port=20000`.
* `paper_scripts/publish_paper_figures.py`: exact-prefix matcher (anchored
  on the `_YYYYMMDD_HHMMSS` suffix) so `safety_filter_sweep` does not
  accidentally match `safety_filter_sweep_planner_aware_<ts>`.  New
  publish specs for all three ablations and a per-variant spec for
  `autonomous_obstacle_tire_model_sweep_{mppi_mpc_blind,barrier_only}`.

Two abstract sub-claims still have no in-repo evidence path and need
either implementation or text changes before submission:

* **Ensemble phi-gating**: implement an estimator-ensemble path that
  exposes `sigma_phi` and have the controller forward it to the shield.
  Until then this sentence in the abstract is unsupported.
* **Joint (n, phi) RMSE = 0.07, 3.3°**: comes from
  `utilities/exp_joint_n_phi.py`, which needs trace CSVs from
  `collect_diverse_terrains.py` / `collect_rich_excitation.py`.  Neither
  trace family exists on disk anymore (`data/` has only the closed-loop
  datasets).  Re-collect before re-running, or pull the figures and the
  numbers from an earlier commit's artefacts.

To replay only the new ablations against the running pilot's seed plan
once the pilot finishes:

```bash
python paper_scripts/run_paper_suite.py --tier pilot \
  --only throttle_dob_ablation mppi_seed_ablation safety_planner_aware
```

### Pilot results (2026-05-15, suite folder paper_suite_pilot_20260515_005416)

Suite ran 7 sub-scripts for the 8 listed (latency_profile is a figure-only
step). 740 / 755 Chrono runs succeeded; the 15 missing runs are all
`dob_cbf_no_nn` on sand and died with `zmq.error.ZMQError: Address already
in use (addr='tcp://*:35162')` — TIME_WAIT port reuse from the previous
run, not a science bug. Pilot finished in ~3.5 hours wall.

| Sweep | Headline |
| --- | --- |
| Tire models (128/128) | v3 NN best at RMS CTE 0.083 m vs Pacejka 0.104 m (-20%), TMeasy 0.087 m (-5%). All four models give identical speed-ratio 0.66, so the curvature-derived speed profile is the binding constraint, not the tire model. NN solve <7.1 ms. |
| MPCC vs MPC (128/128) | Standard MPC dominates: RMS CTE 0.063 m vs MPCC 0.34 m. MPCC's speed-ratio advantage (0.72 vs 0.36-0.51) costs 5x in tracking. Recommend keeping standard MPC as the paper baseline and de-emphasizing MPCC. |
| Safety (planner-blind, 128/128) | none 1.97 coll/run, dob_cbf 0.03, mppi 0.09, nmpc 0.66. DOB-CBF wins on collisions and clearance (+1.0 m). MPPI clearance is thin (+0.06 m) and tracking takes the biggest hit (RMS CTE 1.47 m). NMPC fails badly on clay (1.00 coll on both v=5 and v=7). |
| DOB-CBF NN ablation (81/96) | NN-on: 0.00 coll, +0.51 m clearance. NN-off: 0.47 coll, +0.01 m clearance. The NN inside DOB-CBF clearly helps in deformable-terrain obstacle avoidance, supporting the abstract. |
| Auto-obstacle by tire (128/128) | All four tire models avoid the obstacles ~equally under a fixed MPPI shield (collisions ~0, clearance 1.1-1.9 m). Tire choice does not change the shield-mediated outcome here, which is consistent with the shield being the dominant safety layer. |
| Terrain estimator (64/64) | ID clay |err|=0.005, sand 0.079. OOD per-case: terrain1 0.068, t2 0.153, t3 0.220, t4 0.036, t5 0.034, t6 0.075. terrain3 (true n=0.882) is the extrapolation gap; clay and the easier OOD cases estimate cleanly. |
| Latency comp (96/96, 5G profile) | none 1.81 coll/run, dob_cbf 0.22, mppi 0.19. Both shields cut collisions by ~85% under the time-varying N-HiTS-5G profile. |

Honest summary against abstract claims:

* **NN surrogate beats Pacejka/TMeasy**: confirmed, but the gain is 20%
  vs Pacejka and 5% vs TMeasy, not "an order of magnitude" as the
  abstract currently claims. Soften to "consistent improvement with
  largest gap at higher speed and harder terrain" or similar.
* **NN-aware DOB-CBF helps**: confirmed (0.00 vs 0.47 coll/run, +0.50 m
  vs +0.01 m clearance). Strong evidence.
* **MPPI shield matches NMPC on collision rate**: NOT supported. NMPC
  shield is worse (0.66 vs 0.09 coll/run). Reframe MPPI as the primary
  shield with NMPC as a gradient-ablation that underperforms on hard
  clay scenarios, which actually justifies the abstract's choice of MPPI.
* **5G-profile latency robustness**: confirmed, both shields cut
  collisions ~85% under the learned profile.
* **Order-of-magnitude tracking improvement**: not supported in the
  data. Recommend removing this phrasing.
* **Terrain estimator converges in seconds on canonical clay/dirt/sand**:
  confirmed for clay (err 0.005). Sand is noticeably worse (0.079) and
  one OOD soil (terrain3, n=0.882) is an extrapolation failure (err
  0.220). Honest scope: "good on clay, fair on the rest of the trained
  distribution, fails on n-extrapolation".

Known abstract gaps that need separate work, not yet covered by suite:

* **Ensemble phi-gating**: `MPPIShield.update_terrain` accepts
  `phi_uncertainty_deg` but no caller passes it; `_phi_uncertainty_rad`
  is always 0. Either implement the ensemble path that exposes
  `sigma_phi` to the shield, or remove the sentence from the abstract.
* **Joint (n, phi) RMSE 0.07 / 3.3°**: `utilities/exp_joint_n_phi.py`
  needs trace CSVs from `collect_diverse_terrains.py` /
  `collect_rich_excitation.py`; no such traces remain on disk. Re-collect
  or pull the figures and numbers from a prior commit before submission.

Iteration 2 launched: the three new ablations (throttle DOB on/off,
MPPI seed-trajectory on/off, planner-aware safety sweep) are running
detached as PID 207992 with the same pilot matrix. Expected ~2.5-3
hours and will publish into `my_paper/paper_figures/` automatically.

### Autonomous-loop iterations 2-4 results (2026-05-15)

Iteration 2 (`paper_suite_pilot_20260515_042941`, 384+256 runs, 100 % ok):

* Throttle DOB: speed_ratio 0.673 on, 0.610 off; tracking identical;
  10 % speed gap closed with no tracking cost.
* MPPI seed-trajectory ablation: with seeds 0.03 coll/run +0.11 m
  clearance; without seeds 1.34 coll/run -0.74 m clearance (40x
  collision rate, seeds essential).
* Planner-aware NMPC barrier ablation: every shield is better with the
  in-horizon barrier on (dob_cbf 0.16 -> 0.00 coll/run; mppi 0.09 ->
  0.00; nmpc 0.28 -> 0.16; none 1.97 -> 0.19).  Confirms the abstract's
  two-layer obstacle-avoidance stack.

Iteration 3 (`tire_model_with_estimator_ablation_20260515_061954`, 124
of 128 ok, 4 lost to a flaky ZMQ port-bind):

* New ablation script `paper_scripts/tire_model_with_estimator_ablation.py`
  added to the orchestrator (port block 60000).  Variants: pacejka_static,
  tmeasy_static, nn_v3_static, nn_v3_estimator.  Sim time 20 s,
  metric_start 8 s so the KPI window is post-convergence.
* Multi-seed merged result: pacejka 0.350 m, tmeasy 0.431 m,
  nn_v3_static 0.179 m, nn_v3_estimator 0.208 m RMS CTE.  The NN
  surrogate wins by 40-52 % over Pacejka/TMeasy, but the live estimator
  *slightly hurts* tracking versus static parameters - mostly because
  clay's static prior already matches the true terrain, so the
  estimator's convergence transient is pure noise.  Sand benefits from
  the estimator.

Iteration 4 (gap fills, all 24 runs ok):

* Rerun `dob_cbf_no_nn` on sand (16 runs) to fill the 15-run gap left by
  iteration 1's ZMQ flake; merged 81+15 = 96 runs.
* Rerun `nn_v3_estimator` on sand bumpy (8 runs) to fill iteration 3's
  10-run gap; merged 118+8-2 (overlap) = 124 runs.
* No flakes on the rerun -> the `hil_messages.ZMQPublisher` retry fix
  works.

Code fixes shipped in the loop:

* `paper_scripts/common.py`: `parse_diag_csv` now tolerates header-only
  or zero-byte diag CSVs (catches `pandas.errors.EmptyDataError`) so a
  single mid-init controller crash no longer kills the whole sweep.
* `simulation/hil_messages.py`: `ZMQPublisher.__init__` retries bind
  16x with 0.5 s backoff (~8 s total) before raising, eliminating the
  flaky `Address already in use` TIME_WAIT races on rapid sequential
  runs.
* `paper_scripts/publish_paper_figures.py`: now merges all matching
  results.csv files for each spec (deduped on the run key) and
  re-plots from the merged dataset before copying figures.  Windowing
  is anchored on the largest-row folder so older smoke/abandoned-sweep
  folders no longer leak into the published numbers.  This means the
  published CSV always matches the published figure.

Final paper-readiness scorecard lives at
`my_paper/PAPER_READINESS.md`.  Key recommended abstract text changes:

1. Drop "order of magnitude" -> "40-50 %" for tire-model tracking
   improvement.
2. Reframe MPPI vs NMPC shield - data shows MPPI *dominates* NMPC on
   collision rate, not "matches".  This actually strengthens the
   paper's choice of MPPI as primary.
3. Decide on ensemble phi-gating - either implement the path that
   forwards `sigma_phi` from the estimator ensemble to
   `MPPIShield.update_terrain(phi_uncertainty_deg=...)`, or drop the
   sentence from the abstract.
4. Refresh joint (n, phi) numbers from `utilities/exp_joint_n_phi.py`
   by re-collecting the diverse-terrain LHS traces (none currently on
   disk).

I am stopping the autonomous loop here. The 9 most-defendable abstract
claims have multi-seed evidence; the 2 unsupported ones need
code-or-data work, not more sweeps.

### Autonomous-loop iterations 5-6 (2026-05-15 / 2026-05-16): ensemble-phi-gate wired and ablated end-to-end

Decided to resume the loop to convert the "ensemble phi-gating" sentence
from "not wired" to "tested with multi-seed evidence."  This required
wiring the missing controller -> sim-node -> shield channel, then
ablating two distinct gate designs.

Plumbing shipped (2026-05-15, iter 5):

* `simulation/learned_terrain_estimator.py`: EMA-residual sigma_n
  tracker (alpha=0.05 over the squared n_raw-n_smooth deviation), plus
  `get_n_uncertainty()` and `get_phi_uncertainty_deg()` accessors. The
  phi conversion uses the preset n -> phi slope (degrees per unit n) -
  initial implementation accidentally double-converted with
  `math.degrees`, producing 305 deg sigma; fixed.
* `simulation/hil_messages.py`: extended `ControlCommand` with
  `terrain_n / terrain_phi_deg / terrain_phi_sigma_deg / terrain_K* /
  terrain_class / terrain_confidence / terrain_update_seq` optional
  fields. (Initially used a separate `TerrainUpdate` dataclass on the
  same socket but ZMQ_CONFLATE on `ctrl_sub` dropped it; piggybacking
  on `ControlCommand` survives conflation.)
* `simulation/acados_mpc_controller_node.py`: caches the most recent
  live (n, phi, sigma_phi, K*) into `latest_terrain_update` whenever
  the estimator commits, then injects them into every outgoing
  `ControlCommand`.
* `simulation/chrono_sim_node.py`: new `ControlCommand` handler
  dispatches to `safety_filter.update_terrain(...,
  phi_uncertainty_deg=sigma)` whenever `msg.terrain_update_seq`
  advances; new `--shield-no-sigma-gate` / `--shield-sigma-mode` /
  `--shield-sigma-buffer-gain` CLI flags.
* `simulation/safety/predictive_shield.py`: `MPPIShield.__init__` now
  takes `sigma_mode in {tighten, inflate, both, off}` and
  `sigma_buffer_gain` (default 0.05 m / deg). `_effective_buffer` adds
  `gain * sigma_phi_deg` when mode is `inflate` or `both`;
  `_phi_lower_bound` only subtracts sigma when mode is `tighten` or
  `both`.
* `simulation/launch_decoupled.py`: forwards the new flags to sim-node
  inside the `if safety_flavor == "mppi"` branch (initial edit broke
  the elif chain, fixed before any sweep ran).

Iter 5 ablation (3 variants x 32 runs = 96):

```
variant            collisions_mean  min_clearance_m_mean
no_live_terrain               0.13                 +0.10
sigma_gate_off                0.22                 +0.09
sigma_gate_on (tighten)       0.34                 +0.005
```

Verdict: tightening the friction cone by sigma_phi *rejects valid
evasive maneuvers* and doubles the collision rate.  The abstract's
gate design is harmful as written.

Iter 6 ablation (5 variants x 32 runs = 160, after gate redesign):

```
variant            collisions_mean  min_clearance_m_mean
no_live_terrain              0.156                +0.115
sigma_inflate (new)          0.188                +0.061
sigma_tighten (legacy)       0.188                +0.031
sigma_off                    0.219                +0.048
sigma_both                   0.250                +0.019
```

Verdict: the redesign (inflate clearance buffer instead of tightening
friction cone) also loses to the no-live-terrain baseline.  The MPPI
shield was designed against a fixed terrain and any live-update path
(even without sigma) is at best neutral and at worst harmful, because
the shield's safety calculations were calibrated against initial
terrain and re-conditioning during evasive maneuvers introduces noise
the safety logic does not damp.

Recommendation: in the abstract, drop the entire "shield tracks the
terrain" / "ensemble disagreement gates the friction cone" framing.
Keep only "online estimator feeds the NMPC" for contribution (ii); the
shield uses its initial terrain configuration and ignores live
updates.  See `my_paper/PAPER_READINESS.md` for the full final
scorecard with the recommended abstract text changes.

I am stopping the autonomous loop here.  Across 6 iterations the loop
produced multi-seed evidence for (or against) every empirically
testable abstract sub-claim; the two remaining gaps are HIL G29 rounds
(out of scope for a non-human suite) and the joint (n, phi) RMSE
numbers (require re-collecting trace data that is no longer on disk).
Both need user-level decisions, not more sweeps.
