# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; standard_mpc_soft_speed: Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.; mpcc_default: Current MPCC defaults.; mpcc_less_speed_cap: Tests whether MPCC is being boxed in by vtheta_max and the soft curvature speed cap.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,8,8,0.10567982982569077,0.024072946048679156,0.68224497,0.15657907630036508,3.41122485,0.7828953815018252,4.157877336784097,0.978887501823573,22.16632669889709,5.024811970312936
standard_mpc_soft_speed,8,8,0.10266565839152686,0.02761805825172012,0.5964500791967872,0.16784293249786986,2.9822503959839355,0.839214662489349,4.1103321890850575,0.9918890265957816,19.546439544027628,5.5530301614712565
mpcc_default,8,8,0.23503722456355597,0.08436530339260545,0.847952485,0.011322670369563907,4.239762425,0.05661335184781984,0.7440964714714715,0.09570199284075734,27.4258125,0.3426909829882137
mpcc_less_speed_cap,8,8,0.35443073315057066,0.2607554616129263,0.865591625,0.375769577621579,4.327958125,1.8788478881078952,0.8519189189189189,0.1347081173255661,27.60645,11.9582434421974
```