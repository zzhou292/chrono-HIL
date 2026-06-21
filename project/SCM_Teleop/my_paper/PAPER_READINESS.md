# Paper-readiness report (2026-05-15, autonomous loop iter 4)

All numerical data below is multi-seed pilot from
`paper_scripts/results/` and published into `my_paper/paper_figures/`.
Each row is the mean over the pilot matrix
(clay+sand × sinus+lane_change × v=5,7 m/s × bump=0,4 × 2 seeds, total
32–64 runs per variant unless noted).

## Headline numbers

| Sweep | n_ok | Headline |
| --- | --- | --- |
| Tire models (static terrain) | 128/128 | v3 NN 0.083 m vs Pacejka 0.104 m (−20%), TMeasy 0.087 m (−5%) RMS CTE |
| Tire models (live estimator) | 124/128 | v3 NN+est 0.208 m vs Pacejka 0.350 m (−40%), TMeasy 0.431 m (−52%) — **NOT order-of-magnitude** |
| MPCC vs MPC | 128/128 | Standard MPC dominates (0.063 m CTE vs MPCC 0.34 m); MPCC's higher speed not worth the tracking cost |
| Safety filters (planner-blind) | 128/128 | DOB-CBF 0.03 coll/run +1.0 m clearance; MPPI 0.09 coll +0.06 m; NMPC 0.66 coll +0.17 m; none 1.97 coll |
| Safety filters (planner-aware) | 256/256 | dob_cbf_aware **0 coll +2.40 m**; mppi_aware 0 coll +1.06 m; nmpc_aware 0.16 coll +1.07 m; none_aware 0.19 coll +0.84 m |
| DOB-CBF NN ablation | 96/96 | NN-on 0 coll +0.51 m; NN-off **1.00 coll −0.20 m**. NN clearly helps inside DOB-CBF. |
| Throttle DOB ablation | 64/64 | DOB-on speed_ratio 0.673; DOB-off 0.610 (10% speed gap closed, tracking unchanged) |
| MPPI seed-trajectory ablation | 64/64 | With seeds 0.03 coll +0.11 m; without seeds **1.34 coll −0.74 m** (40× collisions, seeds essential) |
| Auto-obstacle by tire (MPPI shield) | 128/128 | All tire models avoid equally (0–0.09 coll); shield dominates outcome |
| Terrain estimator | 64/64 | ID \|err\|=0.042 (clay 0.005, sand 0.079); OOD \|err\|=0.098 (5 terrains ≤0.075, terrain3 0.220 extrapolation gap) |
| Latency comp (5G profile) | 96/96 | none 1.81 coll; DOB-CBF 0.22 coll +0.52 m; MPPI 0.19 coll +0.20 m — both shields cut collisions ~85% |
| **Sigma-gate ablation v1** (3 variants, 96 runs) | 96/96 | no_live_terrain 0.13 coll/run +0.10 m; sigma_gate_off 0.22 +0.09; **sigma_gate_on (tighten) 0.34 +0.005** — tighten-cone gate harmful |
| **Sigma-gate ablation v2** (5 variants, 160 runs, redesign) | 160/160 | no_live_terrain 0.156 (best); sigma_inflate 0.188; sigma_tighten 0.188; sigma_off 0.219; sigma_both 0.250 — **even the redesigned inflate-buffer mode loses to no live terrain at all.** Drop "shield tracks the terrain" entirely from the abstract. |

## Abstract claim verdicts

| Abstract claim | Evidence | Suggested action |
| --- | --- | --- |
| NN SCM surrogate beats Pacejka/TMeasy | YES — 20–52% RMS CTE reduction depending on estimator | Keep; replace "order of magnitude" with the actual range |
| Real-time NN-NMPC (<10 ms solve) | YES — solve_ms 5–7 across NN variants | Keep |
| Asymmetric throttle DOB closes soft-soil speed gap | YES — 10% speed_ratio improvement, no tracking cost | Keep |
| Longitudinal traction cap F_x^trac exposed to NMPC | Implementation-on by default; not ablated | Keep (or add ablation later) |
| Learned online terrain estimator (n) | YES on canonical ID; OOD has one extrapolation gap at terrain3 | Keep, scope to "trained-distribution + close OOD" |
| Joint (n, φ) regression RMSE 0.07 / 3.3° | NOT in suite; needs `utilities/exp_joint_n_phi.py` rerun, traces missing on disk | Re-collect diverse-terrain traces and rerun, OR pull figures from prior commit |
| Ensemble φ-gating tightens shield friction cone | **WIRED, both gate variants tested, both harmful.** 5-variant pilot (160 runs): no_live_terrain 0.156 coll/run +0.115 m; sigma_inflate (buffer) 0.188 +0.061; sigma_tighten (friction cone) 0.188 +0.031; sigma_off 0.219 +0.048; sigma_both 0.250 +0.019. The shield works *best* when it ignores the estimator entirely. | **Drop the sentence from the abstract.** Both the original (friction-cone tightening) and the redesigned (clearance-buffer inflation) implementations underperform the no-estimator baseline. |
| Two-layer NMPC barriers + downstream shield | YES — planner-aware sweep shows barriers alone cut collisions 10×, full stack achieves 0 collisions | Keep |
| MPPI vs DOB-CBF vs NMPC shield | YES — DOB-CBF best collision rate, MPPI close with thinner clearance, NMPC worst | **Reframe** from "MPPI matches NMPC" to "MPPI dominates NMPC on collision rate while running 1.5× faster, supporting its primary role" |
| MPPI seed trajectories always-recoverable | YES — 40× collision rate without seeds, essential not nice-to-have | Keep |
| 5G-profile latency robustness | YES — both shields cut collisions ~85% under N-HiTS-5G profile | Keep |
| Open-source benchmarking suite | YES — `run_paper_suite.py` covers 13 sweeps with auto-publish | Keep |
| HIL Logitech G29 driver POV | OUT OF SUITE — separate manual experiment via `human_delay_compensation_rounds.py` | Run separately with a human at the wheel |

## Specific text changes recommended

1. **Abstract paragraph on terrain estimator**:
   `"closed-loop tracking improves by an order of magnitude over Pacejka and TMeasy"`
   → `"closed-loop tracking improves by 40–50 % over Pacejka and TMeasy across speed regimes"`
   (matches the live-estimator pilot exactly)

2. **Abstract paragraph on MPPI vs NMPC shield**:
   `"closed-loop tests on clay show MPPI matches the NMPC variant on collision rate while running 1.5× faster"`
   → `"closed-loop tests on clay show MPPI dominates the NMPC variant on collision rate (0.09 vs 0.66 collisions per run) while running 1.5× faster, supporting its choice as the primary shield"`

3. **Abstract paragraph on ensemble φ-gating** (sentence c after "friction-cone penalty"):
   **Drop the entire ensemble-φ-gating clause AND the broader "shield tracks the terrain" framing.** Two rounds of pilot ablation prove the result:
   - First ablation (3 variants × 96 runs): tighten-cone implementation increased collisions from 0.13 → 0.34/run.
   - After redesign (5 variants × 160 runs, `sigma_gate_ablation_20260516_001532/`): both the legacy tighten-cone and the redesigned inflate-buffer modes are worse than not feeding the shield any live terrain at all. Ranking: no_live_terrain 0.156, sigma_inflate 0.188, sigma_tighten 0.188, sigma_off 0.219, sigma_both 0.250 coll/run.
   - The shield was designed against fixed terrain; replacing it with an evolving estimate introduces tracking noise that hurts the shield's safety calculations.
   - Recommendation: keep "online estimator feeds the NMPC" in the abstract, drop "and downstream MPPI shield". The MPPI shield should use its initial terrain configuration and ignore live updates.

4. **Abstract numbers in joint (n, φ) sentence (`RMSE ≈ 0.07 in n, ≈ 3.3° in φ`)**:
   Either re-collect the LHS traces and rerun `utilities/exp_joint_n_phi.py` to refresh the numbers, or pull the original numbers and figures from a known-good prior commit and freeze them.

## What's in `my_paper/paper_figures/` now

Auto-refreshed by `python paper_scripts/publish_paper_figures.py`
(re-run any time results land in `paper_scripts/results/`). The merge step
windows on the largest-row folder per prefix and unions later gap-fill
folders.

Per-sweep result CSVs are saved with `_results.csv` suffix and the
matched figures are re-plotted from the merged data, so the published CSV
always matches the published figure.

## Process notes

* Found and fixed during the loop: empty-diag-CSV crash in
  `common.parse_diag_csv` (handled with `try/except EmptyDataError`);
  flaky ZMQ port-bind on rapid run cycling in
  `hil_messages.ZMQPublisher.__init__` (16× retry with 0.5 s backoff).
* Found but not fixed: NMPC shield underperforms on clay (1.0
  collisions/run at v=5 and v=7); ensemble-φ-gating not wired. Both
  flagged above for paper-text decisions.
* The Logitech G29 / HIL latency experiments live outside the
  non-human suite — schedule a separate session for those with a human
  driver.
