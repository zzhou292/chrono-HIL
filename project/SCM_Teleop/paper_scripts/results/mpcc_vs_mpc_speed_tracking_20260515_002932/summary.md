# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; standard_mpc_soft_speed: Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.; mpcc_default: Current MPCC defaults.; mpcc_less_speed_cap: Tests whether MPCC is being boxed in by vtheta_max and the soft curvature speed cap.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,1,1,0.07716120208862741,,0.5631548000000001,,2.815774,,5.154647676161918,,18.367902465885003,
standard_mpc_soft_speed,1,1,0.06961357415112372,,0.37714156,,1.8857078,,5.204917043740573,,11.827499921047886,
mpcc_default,1,0,,,,,,,,,,
mpcc_less_speed_cap,1,0,,,,,,,,,,
```