# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; mpcc_default: Current MPCC defaults.; mpcc_less_speed_cap: Tests whether MPCC is being boxed in by vtheta_max and the soft curvature speed cap.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,1,1,0.1262712106890542,,0.4433253012048192,,2.216626506024096,,5.37484,,3.4076911332714577,
mpcc_default,1,1,0.42357653054544425,,0.5721024096385542,,2.860512048192771,,0.7289759036144577,,4.25,
mpcc_less_speed_cap,1,1,0.6530301911875205,,0.5665293975903614,,2.832646987951807,,1.0770803212851405,,4.25,
```