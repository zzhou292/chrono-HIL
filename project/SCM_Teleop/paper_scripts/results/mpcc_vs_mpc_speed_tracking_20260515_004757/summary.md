# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; standard_mpc_soft_speed: Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.; mpcc_default: Current MPCC defaults with the static-MLP NN surrogate.; mpcc_less_speed_cap: Tests whether MPCC is being boxed in by vtheta_max and the soft curvature speed cap.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,1,1,0.07659095130082925,,0.52211056,,2.6105528,,5.1089955022488756,,17.009306664708227,
standard_mpc_soft_speed,1,1,0.061614610161389485,,0.38917544,,1.9458772,,5.106011994002999,,12.879111708555925,
mpcc_default,1,1,0.2846073100255859,,0.85891176,,4.2945588,,1.0237882882882885,,27.4552,
mpcc_less_speed_cap,1,1,0.601480944519442,,1.08430984,,5.4215492,,1.0697012012012013,,34.6723,
```