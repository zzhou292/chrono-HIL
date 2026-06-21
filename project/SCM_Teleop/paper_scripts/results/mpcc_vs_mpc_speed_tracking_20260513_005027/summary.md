# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; standard_mpc_soft_speed: Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,1,1,0.130834158802118,,0.44524072289156635,,2.2262036144578317,,4.1434,,3.460420100205046,
standard_mpc_soft_speed,1,1,0.1287610179326344,,0.4605662650602409,,2.3028313253012045,,4.19568,,3.4704097609005418,
```