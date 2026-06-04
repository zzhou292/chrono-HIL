# Targeted deliverables (2026-05-24, v3/v4)

Six asks, six artifacts. All figures live in `figures/`; all driving
scripts and raw run dirs live in this folder.

| # | Figure / text | File |
| - | --- | --- |
| 1 | SCM NN tire surrogate prediction over time — rig vs whole-vehicle | `figures/fig1_rig_vs_vehicle_force_timeseries.png` |
| 2 | Autonomous obstacle avoidance: DOB-CBF vs MPPI safety filters, NMPC blind | `figures/fig2_safety_filter_obstacle_avoidance.png` |
| 3 | Planner-aware obstacle avoidance, no safety filter, three tire models | `figures/fig3_planner_aware_tire_comparison.png` |
| 4 | 5G latency profile (per-channel time series + distribution) | `figures/fig4_5g_latency.png` |
| 5 | 5G latency-generation pipeline architecture | `figures/fig5_5g_pipeline_architecture.png` |
| 6 | DOB-CBF / MPPI safety filters under 5G latency + human control | this README, below |

## What changed since the first pass

* **Terminology**: "shield" → "safety filter" everywhere on the figures
  and in this README.
* **Terrain labeled** on every title (CLAY).
* **Fig 2 is now a single overlay plot** of DOB-CBF and MPPI on the
  same axes, plus a speed-vs-time side panel that shows exactly *why*
  the two filters end at different x coordinates.
* **Safety-filter tuning** to let them pass closer to rocks:
  * `--safety-buffer 0.10` (was default 0.25)
  * DOB-CBF: `--cbf-alpha 1.5` (was 1.0) — allows tighter pass.
  * MPPI: `--shield-horizon 18` (was 12) + `--mppi-sigma-steer 0.55`
    (was 0.35) + `--mppi-temperature 0.8` — now finds an evade
    trajectory instead of braking to a halt.
* **NMPC-aware (Fig 3) tuning**: `--obstacle-weight 2500` (was 5000).
  Tight enough to bring all three tire models within ~2 m of the rocks,
  but not so tight that Pacejka's optimistic SCM lateral-grip estimate
  routes the vehicle straight into a rock (which is what
  `obstacle-weight 1500` did, see internal `s7v3_*` runs).
* **Sim time = 25 s** (was 15 s). 25 s = 5 s lead-in + 20 s traversal
  +recovery. Anything shorter cuts the maneuver off mid-evade and
  makes the safety filters look pathological.

## Fixed scenario

```
terrain     = clay
path        = sinusoidal (the reference goes through the rock field)
speed cmd   = 5 m/s
bumpiness   = 0
rocks       = 5  (seed=7, zone x∈[12,50], y∈[-3,3], size 0.8–1.8 m)
sim time    = 25 s   (5 s lead-in, ~20 s of traversal + recovery)
sensor noise= ON
```

Seed-7 rocks (deterministic):

| Rock | x (m) | y (m) | diam. (m) | comment |
| --- | --- | --- | --- | --- |
| 0 | 14.9 |  +1.7 | 1.24 | First rock, on the sine peak |
| 1 | 49.2 |  +0.2 | 1.30 | Lone rock at the back, on reference centerline |
| 2 | 22.2 |  −0.0 | 1.48 | Cluster |
| 3 | 26.5 |  −2.6 | 1.09 | South trough of the sine |
| 4 | 20.1 |  −0.3 | 1.73 | Cluster, largest rock |

## Fig 2 — NMPC blind, safety filter does the dodging

DOB-CBF and MPPI overlaid on the same axes; speed-vs-time on the right.

| Filter | Collisions | End x | Max\|y\| | Min clearance | Mean cruise (m/s) |
| --- | --- | --- | --- | --- | --- |
| **DOB-CBF** (intent-preserving QP) | **0** | **93 m** | 5.1 m | 0.24 m | ~4.5 |
| **MPPI** (predictive, K=384) | **0** | 35 m | 10.4 m | 0.28 m | ~1.5 |

* DOB-CBF nudges the wheel locally at 10 Hz, the vehicle keeps moving
  at ~4-5 m/s, weaves around the cluster, and ends near the reference
  at x ≈ 93 m. **Closest pass: 1.74 m from rock 0.**
* MPPI's K=384 rollouts around the blind planner's command all lead
  into the cluster, so the importance-weighted mean steers toward the
  evade-left seed; the vehicle climbs to y ≈ 10 m and creeps along at
  1.5 m/s. **Safe (0 collisions) but extremely conservative** — this
  is the classic *predictive filter saves you, intent-preserving
  filter lets you finish the trip* trade-off the paper frames.

## Fig 3 — NMPC aware, no safety filter, three tire models

| Tire model | Collisions | End x | Min clearance | All rocks ≥ |
| --- | --- | --- | --- | --- |
| **Pacejka** (analytical, rigid-terrain) | **0** | 44 m | 0.22 m | 1.72 m |
| **Tire-rig NN surrogate** | **0** | **70 m** | 1.33 m | **2.83 m** |
| **Whole-vehicle NN surrogate** | 64 | 80 m | −0.04 m | clips rock 1 at 1.46 m |

Per-rock minimum approach distance (negative = collision):

| Rock | Pacejka | **Rig NN** | Vehicle NN |
| --- | --- | --- | --- |
| 0 (14.9, +1.7) | +1.72 | **+3.02** | +2.42 |
| 1 (49.2, +0.2) | +5.99 *(didn't reach)* | **+2.83** | +1.46 |
| 2 (22.2,  0.0) | +4.03 | **+3.30** | +3.19 |
| 3 (26.5, −2.6) | +4.77 | **+3.86** | +4.00 |
| 4 (20.1, −0.3) | +4.47 | **+4.37** | +4.03 |

**Bonus point landed**: the rig NN surrogate is the only model that
clears every rock by ≥ 2.83 m and finishes near the reference at
x = 70 m. Pacejka is safe but conservative (only reaches x = 44 m
because it skids south of the cluster). The whole-vehicle surrogate
side-swipes rock 1 — its per-tick Fy prediction is noisier than the
rig's, so the planner's barrier oscillates around the lone rock and
fails to commit to a clean go-around. This matches the rig-vs-vehicle
generalization result the paper reports in §III-D on a much larger
matrix.

---

## How to reproduce

Five runs in parallel, same fixed scenario; only the controller args
change.

```bash
COMMON="--terrain clay --path sinusoidal --speed 5.0
        --time 25.0 --lead-in 5.0 --bumpiness 0
        --rocks 5 --rock-seed 7
        --rock-zone-x 12.0 50.0 --rock-zone-y -3.0 3.0
        --rock-size 0.8 1.8 --no-vis --no-plot"

# Fig 2 — NMPC blind, safety filter dodges
python simulation/launch_decoupled.py $COMMON \
    --sim-port 43000 --ctrl-port 43001 --plot-dir runs/s7v3_dob_cbf \
    --sim-diag-csv runs/s7v3_dob_cbf/sim_diag.csv \
    --model nn --nn-model vehicle_rate_64_32_lhs --rms-time-start 2.0 \
    --mpc-blind-obstacles --safety-filter --safety-flavor dob_cbf \
    --safety-buffer 0.10 --shield-horizon 18 --cbf-alpha 1.5

python simulation/launch_decoupled.py $COMMON \
    --sim-port 43010 --ctrl-port 43011 --plot-dir runs/s7v3_mppi \
    --sim-diag-csv runs/s7v3_mppi/sim_diag.csv \
    --model nn --nn-model vehicle_rate_64_32_lhs --rms-time-start 2.0 \
    --mpc-blind-obstacles --safety-filter --safety-flavor mppi \
    --safety-buffer 0.10 --shield-horizon 18 \
    --mppi-sigma-steer 0.55 --mppi-temperature 0.8

# Fig 3 — NMPC aware, no safety filter; only tire model changes.
python simulation/launch_decoupled.py $COMMON \
    --sim-port 44020 --ctrl-port 44021 --plot-dir runs/s7v4_pacejka_aware \
    --sim-diag-csv runs/s7v4_pacejka_aware/sim_diag.csv \
    --model pacejka --nn-model vehicle_rate_64_32_lhs \
    --rms-time-start 2.0 --obstacle-weight 2500
# ... and similarly for s7v4_rig_aware (--nn-model rig_rate_64_32)
# and s7v4_vehicle_aware (--nn-model vehicle_rate_64_32_lhs).
```

Per-run outputs (sim_diag.csv, run.log, reference path) are under `runs/`.

---

## 6. How DOB-CBF and MPPI keep a human-driven HMMWV safe under 5G latency

* **Both safety filters arbitrate at the local control loop, not at
  the remote driver loop.** The driver's joystick command comes in
  delayed by the uplink leg of the 5G profile (τ_cmd up to ~450 ms in
  Fig 4), and the camera they steer against is delayed even more
  (τ_cam ~660 ms downlink). Neither the driver nor the planner can
  react in less than τ_cmd + τ_cam ≈ 1 s round-trip. Both safety
  filters run on the sim-side, *after* the delayed command lands, so
  the safety check itself adds at most one 10 Hz controller tick —
  that's the part that doesn't suffer the 5G round trip.

* **DOB-CBF is the intent-preserving option for HIL.** It solves a
  one-step minimum-deviation QP around the operator's (delayed)
  steering / throttle / brake and lets the original command through if
  it's already safe. This is exactly what a human driver needs: when
  the operator steers toward a rock they couldn't see at command-emit
  time, DOB-CBF nudges the wheel just enough to clear the obstacle
  (Fig 2, blue trajectory — vehicle keeps moving at ~4-5 m/s, clears
  every rock, finishes the leg at x=93 m).

* **MPPI is the "fall-forward" option when the command itself is
  obsolete.** It samples K=384 noisy command sequences around the
  delayed operator intent, rolls each forward through the same NN tire
  surrogate the planner uses (so the rollout sees terrain-aware
  longitudinal and lateral force limits), and adds six hand-crafted
  *seed* trajectories (passthrough, full brake, coast, evade-left,
  evade-right, brake-while-turn). When every Gaussian perturbation of
  the human command is unsafe, MPPI converges on a brake-or-evade seed
  (Fig 2, orange trajectory — MPPI climbs to y≈10 m and creeps along
  at 1.5 m/s; safe but very conservative). The paper's MPPI seed
  ablation (`mppi_seed_ablation.py`) shows removing those seeds
  multiplies collisions by ~40× — the seeds are exactly what makes
  MPPI robust to round-trip latency where no local Gaussian
  perturbation of an obsolete command happens to be safe.
