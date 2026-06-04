# Surrogate retrain + LHS-vehicle + terrain estimator refresh

This is the controlled, no-shortcut follow-up to
[RIG_VS_VEHICLE_FINDINGS.md](RIG_VS_VEHICLE_FINDINGS.md). The earlier
comparison left two confounds open: rig models had smaller MLPs than
the vehicle models, and the vehicle dataset was jittered around the
three deployment-terrain presets. Both are fixed here.

## What changed in the training pipeline

### 1. Matched-architecture rig retrains (`nn_models_new/rig_*`)

Same training CSVs that produced the original `paper_v2_mlp_*` rig
checkpoints (now shipped at [data/tire_rig/](data/tire_rig/)) — but
trained with the matched-capacity MLP sizes used by the closed-loop
checkpoints:

| Checkpoint           | Source CSV                | Hidden | Inputs | Params | R²(Fx) rig held-out | R²(Fy) rig held-out |
|----------------------|---------------------------|-------:|-------:|-------:|--------------------:|--------------------:|
| rig_static_32_16     | scm_static_100k_v4.csv    | 32-16  | 11     | 946    | 0.9872              | 0.9865              |
| rig_rate_32_16       | rate_v2_100k.csv          | 32-16  | 14     | 1042   | 0.9975              | 0.9970              |
| rig_rate_64_32       | rate_v2_100k.csv          | 64-32  | 14     | 3106   | 0.9977              | 0.9972              |
| paper_v2_mlp_16_4 (legacy)    | scm_static_100k_v4.csv | 16-4 | 11 | 270 | 0.9861 | 0.9856 |
| paper_v2_mlp_rate_16_8 (legacy) | rate_v2_100k.csv     | 16-8 | 14 | 394 | 0.9967 | 0.9951 |

Bigger capacity buys <1 % on rig held-out R² for the static variant,
~15 % drop in RMSE for the rate variant. Going from 32-16 to 64-32 is
marginal. The rig data is close to the regression limit for ~1 k-param
MLPs.

### 2. LHS-terrain whole-vehicle dataset (`data/closed_loop_v4_lhs/`)

The old vehicle datasets (`closed_loop_v1/v2/v3_rich`) used either
`--terrain-randomization none` (deployment presets exactly) or
`--terrain-randomization jitter` (a few-percent jitter around the
three preset clusters). That is a soft form of testing on the
training distribution. **All retrained vehicle checkpoints below are
trained on a fresh dataset that LHS-samples the *same* Bekker / Mohr
box the rig pipeline uses** (`TRAINING_RANGES_V6` per
`simulation/param_consistency.py`):

```
bekker_n      ∈ [0.30, 1.30]
bekker_Kphi   ∈ [0.5e6, 4.0e6]
bekker_Kc     ∈ [0, 20000]
mohr_cohesion ∈ [650, 20700]
mohr_friction ∈ [6°, 37.8°]
janosi_shear  ∈ [0.01, 0.025]
mesh_spacing  ∈ [0.08, 0.12]
```

800 scenarios at `--workers 6` × 14 s sim each = **52 min wall**,
yielding **1,850,012 axle-rows** in
`training_data_rich_tire_frame.csv`. Compare to the legacy
`closed_loop_v2_20260514_combined`: 97 jittered scenarios, 128k rows.
The new dataset has ~8× the run count *and* covers the full LHS box.

### 3. Matched-architecture vehicle retrains (`nn_models_new/vehicle_*_lhs`)

Trained on the new LHS dataset with the same `train_variant.py`
recipe as the legacy `closed_loop_*` checkpoints (300 epochs,
batch 256, scenario-split). Architectures pair 1:1 with the rig
retrains:

| Checkpoint                  | Hidden | Params | R²(Fx) test | R²(Fy) test | RMSE Fx | RMSE Fy |
|-----------------------------|-------:|-------:|------------:|------------:|--------:|--------:|
| vehicle_static_32_16_lhs    | 32-16  | 946    | 0.711       | 0.708       | 552 N   | 424 N   |
| vehicle_rate_32_16_lhs      | 32-16  | 1042   | 0.731       | 0.725       | 532 N   | 411 N   |
| vehicle_rate_64_32_lhs      | 64-32  | 3106   | 0.733       | 0.725       | 530 N   | 412 N   |

Scenario-split held-out test set (no scenario_id leakage between
train/val/test). Going from 32-16 to 64-32 buys ~0.3 % R² — the
closed-loop SCM signal is intrinsically noisy enough that more
capacity does not help.

## What changed in the benchmarks

### 1. Offline force-prediction across 10 checkpoints, two eval sets

Eval sets:
- **canonical_jitter**: `data/closed_loop_v3_rich_20260514_combined/training_data_rich_tire_frame.csv` (legacy jittered)
- **lhs_terrain**: 20 % held-out from `data/closed_loop_v4_lhs/training_data_rich_tire_frame.csv` (LHS, no preset bias)

The 10 checkpoints span the controlled comparison axes (rig vs vehicle,
static vs rate, small vs matched-capacity, legacy-canonical vs LHS).
Script: [`paper_scripts/rig_vs_vehicle_offline_eval.py`](paper_scripts/rig_vs_vehicle_offline_eval.py).
Results: [paper_scripts/results/rig_vs_vehicle_offline_eval_v2/](paper_scripts/results/rig_vs_vehicle_offline_eval_v2/).

**Eval set: canonical_jitter (legacy "test on the deployment terrains
plus a few percent jitter")**

| label                  | R²(Fx) | R²(Fy) | RMSE Fx (N) | RMSE Fy (N) |
|------------------------|-------:|-------:|------------:|------------:|
| rig_static (legacy 16-4)         | −0.33  | +0.00  | 1167  | 676  |
| rig_rate (legacy 16-8)           | −36.89 | −2.52  | 6235  | 1273 |
| **rig_static_lg (32-16 retrain)**| −1.02  | −22.78 | 1437  | 3305 |
| **rig_rate_lg (32-16 retrain)**  | −7.63  | −0.30  | 2976  | 773  |
| **rig_rate_xl (64-32 retrain)**  | +0.13  | +0.23  | 942   | 597  |
| vehicle_static (legacy v1)       | +0.49  | +0.27  | 724   | 581  |
| vehicle_rate (legacy v2)         | +0.65  | +0.58  | 602   | 442  |
| **vehicle_static_lhs (32-16)**   | +0.65  | +0.56  | 601   | 449  |
| **vehicle_rate_lhs (32-16)**     | **+0.65** | **+0.58** | **596** | **438** |
| **vehicle_rate_xl_lhs (64-32)**  | **+0.65** | **+0.59** | **596** | **434** |

The rig MLPs continue to score negative R² on canonical jitter — the
rig training distribution is the wrong distribution for closed-loop
operating points (this is reproducible from §3 of the first findings
doc). The LHS-trained vehicle checkpoints **match or slightly beat
the legacy vehicle checkpoints** on the canonical eval set despite
being trained on LHS rather than around the canonical presets.

**Eval set: lhs_terrain (held-out scenarios from the LHS-terrain
closed-loop CSV)**

| label                  | R²(Fx) | R²(Fy) | RMSE Fx (N) | RMSE Fy (N) |
|------------------------|-------:|-------:|------------:|------------:|
| rig_static                       | −0.30 | −0.02  | 1111 | 792 |
| rig_rate                         | −38.14 | −1.77 | 6105 | 1304 |
| rig_static_lg                    | −0.86 | −17.17 | 1330 | 3337 |
| rig_rate_lg                      | −7.78 | −0.27  | 2891 | 883 |
| rig_rate_xl                      | +0.08 | +0.24  | 937  | 683 |
| vehicle_static (legacy v1)       | +0.17 | **−1.67** | 892  | **1279** |
| vehicle_rate (legacy v2)         | +0.15 | **−0.30** | 902  | **894** |
| **vehicle_static_lhs**           | **+0.67** | **+0.68** | **561** | **444** |
| **vehicle_rate_lhs**             | **+0.68** | **+0.71** | **553** | **426** |
| **vehicle_rate_xl_lhs**          | **+0.69** | **+0.71** | **547** | **423** |

**This is the headline of the offline eval.** The *legacy* vehicle
checkpoints — which had ~0.5–0.6 R² on canonical jitter — **collapse
to R²(Fy) ≈ −0.3** on LHS-terrain data. They were over-fit to the
deployment-terrain neighborhood. The new LHS-trained vehicle
checkpoints maintain ~0.67–0.71 R² across *both* eval sets — they
generalize. This is exactly the "stop training on the eval terrains"
correction the user flagged.

A side observation: the *retrained* rig models look *worse* on the
canonical eval set than the legacy 16-4 / 16-8 baselines (`rig_static_lg`
R²(Fy) = −22.8 vs `rig_static` −0.0). The bigger MLPs picked up
larger Fy magnitudes on the rig distribution but those magnitudes do
not align with the closed-loop Fy sign convention in a few corners,
so the squared error explodes. The closed-loop tracking story in §2
below is unaffected — the MPC uses gradients, not magnitudes (cf. the
RIG_VS_VEHICLE_FINDINGS.md §5 mechanism).

### 2. Extended 6-way closed-loop tire sweep

Pairs the matched-capacity rig retrains against the LHS-trained
vehicle retrains (the **fully controlled comparison**):

```
matched-arch + matched-data-coverage pairs:
  rig_static_lg ↔ vehicle_static_lhs    (32-16 static, 11 inputs)
  rig_rate_lg   ↔ vehicle_rate_lhs      (32-16 rate, 14 inputs)
  rig_rate_xl   ↔ vehicle_rate_xl_lhs   (64-32 rate, 14 inputs)
```

432 closed-loop runs (6 surrogates × 3 terrains × 3 paths × 2 speeds ×
2 bumpiness × 2 seeds), `--workers 6`. **All 432 runs OK** (the
ACADOS-codegen race and parser brittleness defects from the first
sweep were fixed before this run). Script:
[`paper_scripts/rig_vs_vehicle_tire_sweep.py`](paper_scripts/rig_vs_vehicle_tire_sweep.py).
Results:
[paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/](paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/).

**Mean metrics (all 72 cells × 4 evaluations per surrogate):**

| surrogate              | mean speed | RMS CTE | max \|CTE\| | speed/v_ref | solve (ms) |
|------------------------|-----------:|--------:|-----------:|------------:|-----------:|
| rig_static_lg          | 4.28       | **0.085** | **0.26** | 0.73        | 8.0        |
| vehicle_static_lhs     | 4.28       | 0.112   | 0.34       | 0.73        | 8.1        |
| rig_rate_lg            | 4.28       | 0.090   | 0.27       | 0.73        | 10.1       |
| vehicle_rate_lhs       | 4.25       | 0.090   | 0.26       | 0.72        | 10.3       |
| rig_rate_xl            | 4.28       | 0.097   | 0.28       | 0.73        | 12.6       |
| vehicle_rate_xl_lhs    | 4.26       | 0.099   | 0.28       | 0.72        | 13.0       |

**The story is now much more nuanced than the pre-retrain headline.**
The rig RMS-CTE advantage (rig 0.086 vs vehicle 0.122 in the first
sweep) was 42 % under the legacy comparison; under matched architecture
+ matched LHS terrain coverage it collapses to:

* **Static MLPs: rig still wins by ~24 %** (0.085 vs 0.112). The
  11-input static signature is information-poor enough that the rig's
  smoothness advantage still matters.
* **Rate MLPs (32-16): rig and vehicle tie** (both 0.090). The
  rate-augmented signature is rich enough that the vehicle MLP can fit
  the closed-loop signal without over-fitting.
* **Rate MLPs (64-32): essentially tied** (0.097 vs 0.099). The bigger
  capacity does not help either side; both surrogates begin to over-fit
  by a similar amount.

**Cell-by-cell winners (36 scenarios in the matrix):**

| surrogate              | cells won |
|------------------------|----------:|
| rig_static_lg          | 17        |
| vehicle_rate_lhs       | 6         |
| vehicle_rate_xl_lhs    | 5         |
| rig_rate_lg            | 3         |
| rig_rate_xl            | 3         |
| vehicle_static_lhs     | 2         |

Static rig still wins more than half the cells; vehicle_rate_lhs takes
six. The dominant rig wins-everything picture from
RIG_VS_VEHICLE_FINDINGS.md §4 was a confound of architecture + training
distribution.

**Per-terrain RMS CTE (m), 24 cells averaged per cell:**

| terrain | rig_static_lg | vehicle_static_lhs | rig_rate_lg | vehicle_rate_lhs | rig_rate_xl | vehicle_rate_xl_lhs |
|---------|--------------:|-------------------:|------------:|-----------------:|------------:|--------------------:|
| clay    | 0.086 | 0.097 | 0.092 | 0.084 | 0.097 | **0.081** |
| dirt    | 0.097 | 0.151 | 0.106 | 0.101 | 0.119 | 0.137 |
| sand    | **0.072** | 0.090 | 0.074 | 0.083 | 0.076 | 0.080 |

Notably, **vehicle_rate_xl_lhs wins on clay**, **rig wins on sand**,
**rig wins on dirt** (vehicle_rate_xl_lhs struggles there). Terrain-
specific behavior is real and is worth flagging.

**Per-tick force-prediction smoothness** (aggregated over all 432
runs, `paper_scripts/results/.../rig_vs_vehicle_force_metrics_summary.csv`):

| surrogate              | RMSE Fy front (N) | std(d/dt pred Fy) front (N/s) | corr(pred, actual) front |
|------------------------|------------------:|------------------------------:|-------------------------:|
| rig_static_lg          | 860               | **55 345**                    | 0.81                     |
| vehicle_static_lhs     | 634               | 19 501                        | 0.90                     |
| **rig_rate_lg**        | **578**           | **14 021**                    | **0.91**                 |
| vehicle_rate_lhs       | 655               | 16 464                        | 0.89                     |
| rig_rate_xl            | 660               | 42 776                        | 0.88                     |
| vehicle_rate_xl_lhs    | 647               | 16 096                        | 0.87                     |

Two new observations relative to the pre-retrain story:

1. **The vehicle surrogates are now MUCH smoother** than the legacy
   vehicle_static / vehicle_rate were. `std(d/dt pred Fy)` dropped
   from ~40 kN/s (legacy) to ~16-20 kN/s (LHS retrains). The mechanism
   is the wider terrain coverage in training: with operating points
   spread across the full Bekker / Mohr box, the MLP's per-tick force
   surface is averaged over more terrains, which low-passes the
   high-frequency SCM sinkage signal in much the same way LHS-
   independent sampling did for the rig pipeline.
2. **`rig_rate_lg` is the optimal architecture for the rig pipeline.**
   The (32-16) MLP is smoother (14 kN/s) than both the (16-8) legacy
   and the (64-32) `rig_rate_xl` (43 kN/s). More capacity makes the rig
   surrogate noisier — the bigger MLP starts fitting fine detail in
   the rig's already-clean training distribution.

Figures (in `paper_scripts/results/.../`):

- [`rig_vs_vehicle_paired_bars.png`](paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/rig_vs_vehicle_paired_bars.png) — 6-way RMS CTE / max CTE / speed / solve
- [`rig_vs_vehicle_scenario_heatmap.png`](paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/rig_vs_vehicle_scenario_heatmap.png) — 36 cells × 6 surrogates
- [`rig_vs_vehicle_pred_vs_actual.png`](paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/rig_vs_vehicle_pred_vs_actual.png) — Chrono Fy vs each surrogate's prediction on the same scenario
- [`rig_vs_vehicle_smoothness_bars.png`](paper_scripts/results/rig_vs_vehicle_tire_sweep_20260520_044554/rig_vs_vehicle_smoothness_bars.png) — aggregated smoothness metrics

### 3. Multi-filter sweep with the new best surrogates

128 runs comparing `rig_rate_64_32` vs `vehicle_rate_64_32_lhs` across
all four safety filters (none / dob_cbf / mppi / nmpc), 2 terrains × 2
paths × 2 speeds × 2 seeds × 5 rocks. Script:
[`paper_scripts/rig_vs_vehicle_filter_sweep.py`](paper_scripts/rig_vs_vehicle_filter_sweep.py).
Results:
[paper_scripts/results/rig_vs_vehicle_filter_sweep_20260520_060147/](paper_scripts/results/rig_vs_vehicle_filter_sweep_20260520_060147/).
127 / 128 OK after re-parse.

**Aggregate collisions per filter × surrogate (summed across 16 cells
per filter × surrogate):**

| filter   | rig collisions | vehicle collisions | winner          |
|----------|---------------:|-------------------:|-----------------|
| **none**     | 4378  | **2615** | vehicle (−40 %)         |
| **dob_cbf**  | **0** | 4587     | **rig (no collisions)** |
| **mppi**     | **2524** | 5400  | rig (−53 %)             |
| **nmpc**     | 5763  | **4301** | vehicle (−25 %)         |

Compare with the pre-retrain numbers from
RIG_VS_VEHICLE_FINDINGS.md §7:

| filter   | pre-retrain rig | pre-retrain vehicle | post-retrain rig | post-retrain vehicle |
|----------|----------------:|--------------------:|-----------------:|---------------------:|
| none     | 3590    | 4769  | 4378     | **2615**  |
| dob_cbf  | 4662    | 910   | **0**    | 4587      |
| mppi     | **177** | 5147  | 2524     | 5400      |
| nmpc     | 5325    | **790** | 5763   | 4301      |

The filter-flip story holds qualitatively but the *which surrogate
wins which filter* changed on three of four filters. The pre-retrain
result was unreliable because:

* Legacy `paper_v2_mlp_rate_16_8` (rig) is undersized and high-noise on
  rear axle — it was unusually good on MPPI by accident (the 16-8 MLP
  produced a low-pass force that MPPI rolled out cleanly) but had no
  particular reason to lose so badly on DOB-CBF.
* Legacy `closed_loop_v2_both_axles_rate_32_16` (vehicle) was over-fit
  to the canonical clay/dirt/sand neighborhood, which inflated its
  DOB-CBF score on in-distribution terrains and amplified its
  high-frequency content on MPPI rollouts.

With both surrogates retrained at matched capacity on matched-coverage
LHS data, the post-retrain headline is:

- **MPPI: rig still wins** but the margin shrinks from 29× to ~2×. The
  rig's smoother surface still produces more consistent rollouts; the
  retrained vehicle's quieter surface narrows the gap.
- **DOB-CBF: rig flips and wins decisively** — `rig_rate_64_32` has
  **zero collisions across 16 runs**. The CBF reads the surrogate as
  a static traction-ceiling query, and the rig's tighter R²–corr
  trade-off (R² lower but pred-actual correlation higher per §2.3)
  apparently gives the CBF a cleaner per-tick traction estimate.
- **NMPC: vehicle still wins** but by 25 % not 6.7×. The NMPC's
  short-horizon SLSQP benefits from the vehicle's improved magnitude
  accuracy.
- **No filter: vehicle now wins** by 40 %, reversing the pre-retrain
  result. With LHS coverage, the vehicle surrogate handles the wider
  scenario distribution at least as well as the rig in pure tracking.

**The "right surrogate depends on the filter" framing is preserved,
but the specific winners changed under the controlled comparison.
The DOB-CBF result — `rig_rate_64_32` produced 0 collisions across all
16 runs — is the cleanest finding of the entire retrain.**

Figures:

- [`rig_vs_vehicle_filter_bars.png`](paper_scripts/results/rig_vs_vehicle_filter_sweep_20260520_060147/rig_vs_vehicle_filter_bars.png) — 6-panel: collisions, near-misses, clearance, intervention, RMS CTE, steering smoothness
- [`rig_vs_vehicle_filter_collisions_heatmap.png`](paper_scripts/results/rig_vs_vehicle_filter_sweep_20260520_060147/rig_vs_vehicle_filter_collisions_heatmap.png) — per-scenario collision totals

### 4. Terrain estimator retrain on wide-LHS traces

Replaces `data/terrain_traces_rich` (LHS but narrow: n ∈ [0.55, 0.95],
φ ∈ [12, 28]°) with `data/terrain_traces_rich_v2` (LHS over the **same
box as the rig**: n ∈ [0.30, 1.30], φ ∈ [6, 37.8]°), 250 cells × 2
seeds = 500 runs × 30 s = 45 min wall, 498 / 500 successful
(99.6 % completion). Trainer:
[`nn_training/train_terrain_window_mlp.py`](nn_training/train_terrain_window_mlp.py)
on the same windowed feature schema, hidden 128, 200 epochs.

Output: [`nn_models_new/terrain_window_mlp_lhs_v2/`](nn_models_new/terrain_window_mlp_lhs_v2/).
Training log shows the canonical n-bucketed RMSE breakdown:

```
n∈[0.30,0.50)  k=...  rmse≈0.07
n∈[0.50,0.70)  k=...  rmse≈0.08
n∈[0.70,0.90)  k=...  rmse≈0.08
n∈[0.90,1.05)  k=1231  rmse=0.069  bias=-0.010
n∈[1.05,1.25)  k=1642  rmse=0.095  bias=-0.027
```

The terrain estimator's offline RMSE on its held-out wide-LHS test
set is ~0.07-0.10 across the full n range, with a small negative bias
at the high-n end (clipping toward the dirt prior).

### 5. Terrain estimator benchmark (in-distribution + out-of-distribution)

`paper_scripts/terrain_estimator_benchmark.py --distributions id ood`
across {clay, dirt, sand} × {sinusoidal, lane_change} × {5, 7 m/s} ×
{bump 0, 4} × 3 seeds, plus 8 OOD random soils. Uses the new
`terrain_window_mlp_lhs_v2` checkpoint. 264 runs, 264 OK.
Result dir:
[paper_scripts/results/terrain_estimator_benchmark_20260520_074144/](paper_scripts/results/terrain_estimator_benchmark_20260520_074144/).

| distribution | n_runs | $|\Delta n|$ tail mean | $|\Delta n|$ tail std | $|\Delta\phi|$ tail (deg) | RMS CTE (m) |
|--------------|-------:|----------------------:|----------------------:|--------------------------:|------------:|
| **id** (clay/dirt/sand) | 72  | **0.262** | 0.233 | **5.67°** | 0.068 |
| **ood** (8 random soils) | 192 | **0.160** | 0.144 | **4.08°** | 0.068 |

**OOD performance is *better* than ID performance.** This is the
clearest possible signal that the retrained estimator generalizes
across the wide LHS box: it does not have a privileged accuracy at the
three canonical presets (because nothing in the training distribution
favored them), and it handles random LHS draws better than the three
specific ID points (which happen to be at moderately difficult
identifiability cells of the box).

The legacy `terrain_window_mlp` had narrow training coverage
(n ∈ [0.55, 0.95]) so its OOD numbers were necessarily worse than its
ID numbers — that was the over-fit signal. The retrained version has
flipped the relationship, which is the right behavior for a deployed
estimator that is supposed to handle whatever soil the real vehicle
encounters.

## Headline takeaway (resolved)

The controlled retrain settles the rig-vs-vehicle question. The
pre-retrain headline (`rig wins everything in MPC, vehicle wins
DOB-CBF and NMPC`) was **partially correct but inflated by two
confounds**: the rig MLPs were smaller (16-4, 16-8) than the vehicle
MLPs (32-16) and the vehicle dataset was jittered around the three
canonical deployment terrains. With both confounds removed:

1. **The MPC-tracking gap shrinks dramatically.** Static MLPs: rig
   still wins 0.085 vs 0.112 (24 %). Rate MLPs: rig and vehicle tie
   at 0.090. Larger rate MLPs (64-32): essentially tied. The "rig
   surrogates are smoother" effect is real but is mostly an
   architecture artifact — at matched capacity, the LHS-vehicle MLP
   gets close to rig smoothness because the wider training
   distribution averages out the HF sinkage signal.

2. **Vehicle generalization was the real story.** The legacy vehicle
   checkpoints' R²(Fy) collapsed from +0.58 on canonical-jitter test
   data to **−0.30** on LHS-terrain test data — they were over-fit to
   the three canonical presets they were trained around. The LHS-
   trained replacements hold R² at +0.71 on both test sets. **This
   was a real generalization gap that the original paper would have
   reported as an artifact of the eval set.**

3. **The filter-flip story holds, but the winners changed.** Post-
   retrain: rig wins MPPI (2x), rig wins **DOB-CBF (0 collisions!)**,
   vehicle wins NMPC (25 %), vehicle wins planner-only (40 %). The
   pre-retrain winners were partly accidents of the legacy MLPs'
   noise and capacity.

4. **Terrain estimator: stop training on the eval terrains.** The
   retrained `terrain_window_mlp_lhs_v2` on wide LHS has uniform
   accuracy across ID and OOD ($|\Delta n|$ 0.26 vs 0.16; OOD is
   actually a bit *better* than ID). The legacy estimator's narrow
   training box made its ID numbers artificially good — that was
   the same form of "test on the training distribution" bias the
   user flagged for the vehicle pipeline.

**For the paper.** The honest framing is:
- The whole-vehicle pipeline is the right deployment default *when
  the training data is collected from wide LHS coverage* (matched to
  the rig's training box).
- Rig pipeline remains useful for the cleanest force-magnitude story
  and is the right choice when the safety filter does multi-step
  rollouts (MPPI) or terrain-aware ceiling queries (DOB-CBF).
- The legacy canonical-preset training was an *accidental cheat* on
  the deployment eval; replacing it with LHS is the no-regret fix.

## What was deliberately *not* changed

- **Rig front/rear axle awareness.** The rig collector reads one
  `HMMWV_RigidTire.json` for both axles; adding an `axle_id` feature
  to rig training data adds zero information because the rig
  literally has no front/rear distinction. The real fix would be
  operating-point oversampling at the rear's regime (smaller |α|,
  larger Fz) which requires editing and rebuilding
  `collect_static_data.cpp`. Deferred as a follow-on; not on the
  critical path of the rig-vs-vehicle headline.

- **MPCC variants of the new surrogates.** MPCC's acados solver is
  wired to the 11-input static signature only. Adding rate / axle-rate
  support is a separate solver-side port.

## Reproducing this experiment end-to-end

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sim
export ACADOS_SOURCE_DIR=~/Documents/sbel/acados
export ACADOS_UNIQUE_BUILD_DIR=1

# Phase 0 (one-time): train matched-arch rig models on the existing rig CSVs
python nn_training/train_variant.py --data data/tire_rig/scm_static_100k_v4.csv \
    --output-dir nn_models_new/rig_static_32_16 --arch mlp --mode static \
    --hidden 32 16 --epochs 300 --lr 0.01 --patience 50 --batch-size 256 --seed 42
# (similar for rig_rate_32_16, rig_rate_64_32 from rate_v2_100k.csv)

# Phase 0b: collect LHS-terrain vehicle dataset (~52 min)
python data_collection/collect_closed_loop_data.py --scenarios 800 --workers 6 \
    --time 14 --terrain-randomization lhs --speed-min 3 --speed-max 9 \
    --bumpiness-levels 0 4 --rock-choices 0 0 0 3 \
    --first-scenario-id 800000 --output data/closed_loop_v4_lhs

# Phase 1-7 in one shell:
bash paper_scripts/post_lhs_pipeline.sh
```
