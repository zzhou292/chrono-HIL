# NN-UKF closed-loop diagnosis (why firm-soil estimation failed, and what it takes to reproduce the paper)

## TL;DR
The NN-UKF is **not** fundamentally bad. The poor firm-soil closed-loop
number (sand |Δn| ≈ 0.47) was the sum of two *independent* problems:

1. **Runtime cadence bug (FIXED).** The online port stepped the UKF only at
   the MPC's terrain-pull cadence (`te_update_interval`, every 8th control
   tick ≈ 96 ms), integrating the nonlinear bicycle over too long a horizon.
   The weak firm-soil n-channel never converged — even on strong-excitation
   logs. The offline reproducer (`deliverables/ukf_paper_validation.py`)
   steps every log sample (~24 ms) and was always fine.
2. **Closed-loop observability limit (fundamental).** On firm soil, n is only
   observable under sustained large slip. A path-tracking NMPC keeps slip
   small (that's good tracking) *and* actively rejects an injected steering
   dither as a disturbance, so the n-channel stays excitation-starved.

## Evidence
Same fixed estimator + Fy surrogate, three feeds:

| feed | clay |Δn| | sand |Δn| |
|---|---|---|
| offline replay of `sand.npz`/`clay.npz` (paper-style 0.6-rad sinusoidal **steer**) | 0.039 | **0.124** |
| online port, UKF stepped every 8 ticks (the bug) | 0.014 | 0.494 |
| online port, UKF stepped **every tick** (the fix), offline replay | 0.039 | **0.124** |
| closed-loop NMPC tracking, fixed estimator | 0.039 | 0.416 |
| closed-loop + steering probe up to 0.5 rad | 0.040 | 0.373 |

- Per-tick stepping reproduces the reference-paper result on the paper's own
  excitation (sand 0.124 ≈ the published ~10 %). `upd=2` already degrades
  sand to 0.205; `upd=4` to 0.447 — the filter genuinely needs per-tick
  measurement updates.
- In closed loop, `conv_n` ≈ 0.68 on sand (true 1.10): n barely leaves the
  dirt prior (0.70). Excitation, not the filter, is the bottleneck. A steering
  dither doesn't fix it because the tracker rejects it.
- The numpy force forward was verified to match the torch forward to < 1e-3 N
  for in-range n (the apparent "795 N mismatch" was a clip-range artifact:
  online clamps n to [0.5, 1.1], offline to [0.2, 1.4]).

## What it takes to reproduce the paper's NN-UKF results
- **In the paper's setting (the published claim): done.** With the per-tick
  fix, the UKF reproduces ~10 % sand / ~4 % clay on paper-style sustained
  sinusoidal-steer excitation. This is what `ukf_paper_validation.py` and the
  §VI figures already use.
- **In closed-loop deployment:** firm-soil n needs a *dedicated excitation
  maneuver* (open-loop sinusoidal steer, like the paper), not passive
  path-tracking. Options: (a) run a brief system-ID burst before/around
  tracking; (b) the deployed window-MLP (no force model, IMU features only)
  is the better firm-soil closed-loop estimator; (c) the regime-aware
  **fused** estimator (`fused_terrain_estimator.py`) uses the NN-UKF on soft
  soil and the MLP on firm, and beats both overall closed-loop.

## Closed-loop comparison (post-fix, re-tuned fusion, 3 terrains × 2 speeds × 3 seeds)
| backend | clay | dirt | sand | ALL |
|---|---|---|---|---|
| MLP (deployed) | 0.071 | 0.075 | 0.042 | 0.063 |
| NN-UKF (per-tick fix) | 0.044 | 0.057 | 0.429 | 0.177 |
| Fused (regime, n0=0.85) | 0.044 | 0.044 | 0.037 | **0.042** |

The per-tick fix moved the NN-UKF/MLP crossover firmer (the NN-UKF now wins
clay AND dirt), so the fusion blend center was re-tuned 0.65 → 0.85; the fused
estimator is now best on every terrain class and 33 % better than the deployed
MLP overall.

Repro: `benchmarking/closed_loop_estimator_compare_fused.py` (backends).
The probe-amplitude sweep that confirmed active probing does NOT rescue
closed-loop sand was archived (negative result) to
`archive/2026-06-08_innovation_a_active_probing/nnukf_probe_sweep.py`.
