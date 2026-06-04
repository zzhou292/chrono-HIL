# vehicle_fy_64_32 — training metadata

| | |
| --- | --- |
| Deployment status | **active** (tire-force backend of the Dallas-style state-augmented UKF in `deliverables/ukf_paper_validation.py::_bicycle_step_vehicle_fy`; rebuilds `my_paper/paper_figures/{lhs100_fair,lhs100_cl,estimator_overall,terrain_estimator_comparison}.png`). |
| Architecture | 2-hidden-layer MLP, hidden = [128, 64], ReLU, ≈ 12 k params |
| Inputs (10) | `u, v, omega, delta, Kphi, Kc, n, cohesion, friction_angle, janosi_shear` |
| Outputs (2) | `(Fy_total, M_yaw_total)` — body-frame total lateral tyre force and yaw moment of the whole HMMWV |
| Held-out metrics | Fy R² = 0.90, RMSE = 1156 N; M_yaw R² = 0.88, RMSE = 773 N·m (split by scenario; 30 of 300 LHS scenarios held out) |

## Why this checkpoint exists

The paper118-spec **rig** surrogate (`rig_rate_paper118_v2_64_32`) is
trained on a single-tyre Chrono SCM test rig and therefore
under-predicts the *vehicle*'s body-frame Fy by ≈15 %, varying
non-trivially with soil parameters. Previous iterations of the
Dallas-style UKF wrapped the rig NN in a hand-tuned rig-to-vehicle
gain scalar (1.15×) and then in a small MLP that learned a
soil-dependent SCALE; neither generalised to a broad LHS sweep,
median error stayed ≥ 13 % on the 100-LHS benchmark.

`vehicle_fy_64_32` removes the calibration step entirely. It is
trained on **whole-HMMWV** Chrono SCM logs to predict the same total
body-frame lateral force the bicycle UKF needs, with the rig-vs-
vehicle gap absorbed into the weights. The UKF picks it up with no
post-hoc scalar.

## Training data

- **Source**: 300 Chrono SCM HMMWV runs with scripted sinusoidal
  steering; half use constant open-loop throttle, half a PI
  cruise-speed loop.
- **Generator**: [`data_collection/collect_lhs_training_scms.py`](../../data_collection/collect_lhs_training_scms.py)
  `--widened-box` — runs [`data_collection/run_dallas_scm.py`](../../data_collection/run_dallas_scm.py)
  in a `ProcessPoolExecutor` per CLAUDE.md §Parallelism.
- **Logs**: [`data/dallas_scm/lhs_train300/`](../../data/dallas_scm/lhs_train300/)
  (300 NPZ + JSON files).
- **Sampling**: 9-axis **uniform LHS** with `seed=7` (disjoint from
  the benchmark's `seed=42`), over a *widened* soil box so the
  canonical clay/dirt/sand presets are interior (not box-corner)
  points — they sit on the EDGES of the deployed
  `TRAINING_RANGES_V6` box (clay: Kphi at 5 %, janosi at the 0 %
  floor), which is why the prior standard-box surrogate failed on
  clay. Widening (never narrowing — CLAUDE.md §8) makes them
  interpolation targets:

  | Axis | Range (widened) | vs deployed box |
  | --- | --- | --- |
  | bekker_Kphi    | 0.3e6 … 4.0e6 N/m^(n+2) | low end 0.5e6→0.3e6 |
  | bekker_Kc      | 0 … 20,000 N/m^(n+1) | unchanged |
  | bekker_n       | 0.35 … 1.35 | 0.30→0.35 / 1.30→1.35 |
  | mohr_cohesion  | 300 … 20,700 Pa | low end 650→300 |
  | mohr_friction  | 6° … 37.8° | unchanged |
  | janosi_shear   | 0.008 … 0.028 m | 0.010→0.008 / 0.025→0.028 |
  | steer amplitude| 0.20 … 0.65 rad | brackets paper118 0.50 & Buzhardt 0.60 |
  | steer period   | 2.0 … 5.0 s | — |
  | target speed   | 3.5 … 7.0 m/s | (PI-cruise runs only) |

  This satisfies CLAUDE.md §8 — uniform LHS over the (widened) soil
  and excitation boxes, no narrowing toward a canonical preset or
  controller-conditioned operating point.

- **Aggregated rows**: ≈ 190 000 (every 2nd sample at 24 ms over the
  active steering window of each run).

## Targets

- `Fy_total = m · ay_log` — body-frame total lateral tire force,
  derived from Chrono's body-frame `ay` measurement.
- `M_yaw_total = Iz · dω/dt` — yaw moment, derived by 5-point central
  difference on the log's yaw rate.

## Trainer

- Script: [`nn_training/train_vehicle_fy_surrogate.py`](../../nn_training/train_vehicle_fy_surrogate.py).
- Hyperparameters: Adam, lr 3e-3, batch 512, ≤ 400 epochs with
  patience 60, scaler-normalised inputs and outputs.
- Split: 90 / 10 by *scenario* (180 train / 20 test), so the held-out
  metrics measure generalisation to genuinely unseen soils and
  manoeuvres.

## Sign / frame convention

- `Fy_total` is the body-frame *total* — already accounts for
  steering-induced front-axle rotation, lateral load transfer and
  suspension geometry. The UKF uses it directly:
  `vdot = Fy_total / m - u · ω`.
- `M_yaw_total` is the body-frame total yaw moment with the same
  sign convention; `ωdot = M_yaw_total / Iz`.

## Known limitations

- Trained on a single vehicle (Chrono `HMMWV_Full`). Different
  vehicle masses or suspensions need retraining; the surrogate is
  not vehicle-mass-invariant.
- Operating points are limited to the LHS box above. The UKF's
  sigma-point spread normally stays inside that box, but a future
  iteration could widen velocity / steer ranges if needed.
- The held-out Fy R² is 0.90 — the residual ~10 % of variance
  (largely suspension transients the bicycle state can't see) is what
  bounds the UKF's median |Δn|/n_true at ≈ 9–13 % depending on
  excitation mode.

## Benchmark on disjoint LHS

100 LHS terrains (seed=42, n in [0.40, 1.30], disjoint from the
seed=7 training set), single Dallas sinusoidal manoeuvre per terrain:

| Estimator | mean | median | p90 | within 10 % | within 20 % |
| --- | --- | --- | --- | --- | --- |
| Bekker UKF      | 26.4 % | 20.9 % | 53.8 % | 28 % | 49 % |
| Learned MLP     | 18.9 % | 15.7 % | 38.6 % | 31 % | 59 % |
| **NN UKF (this)** | **13.5 %** | **9.4 %** | **31.6 %** | **53 %** | **79 %** |

The benchmark uses closed-loop PI-cruise excitation (5 m/s target,
0.6 rad sinusoidal steering at 3 s period) — the deployment-
realistic operating mode. An open-loop variant (constant throttle
0.75, same steering) is reported in `lhs100_cl_vs_ol.png`; the NN-UKF
is essentially mode-invariant while the Bekker-UKF degrades under
open-loop (its bicycle's analytical Fx integral assumes a fixed slip
ratio that constant-throttle excursions violate).

Canonical-preset spot check (amp 0.6, clean logs): clay NN-UKF
18.0 % (best of 3), sandy loam 3.2 % (best of 3), dry sand 10.8 %.
The earlier >100 % clay error was an artefact of a degenerate
zero-throttle ground-truth log plus box-corner extrapolation, both
fixed here.

Figure: `my_paper/paper_figures/lhs100_cl.png`; CSV:
`my_paper/paper_figures/lhs100_cl.csv`. Reproduce with::

    python data_collection/collect_lhs_training_scms.py --n 300 --seed 7 \
        --widened-box --out-dir data/dallas_scm/lhs_train300
    python nn_training/train_vehicle_fy_surrogate.py \
        --lhs-dir data/dallas_scm/lhs_train300 \
        --hidden 128 64 --epochs 400 --decim 2 --test-frac 0.10
    python deliverables/bench_terrain_estimators_lhs.py --n 100 \
        --workers 8 --n-min 0.40 --n-max 1.30 --steer-amp-rad 0.6 \
        --open-loop-throttle -1 --target-speed 5.0 \
        --log-suffix _cl --out-name lhs100_cl
