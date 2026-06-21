# DOB-CBF NN Ablation

Noise policy: sensor noise enabled in every run.
Default planner policy: MPC is blind to rocks, so the filter is the sole obstacle avoider. Pass `--aware` to benchmark the easier combined planner+filter stack.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std
no_filter,1,1,1.0,,1.0,,-1.0163999999999997,,,,,,,,0.12910327494400753,,0.47723372,
dob_cbf_nn,1,1,0.0,,1.0,,0.3924000000000003,,35.8,,,,,,0.8669507732570932,,0.51171792,
dob_cbf_no_nn,1,1,0.0,,1.0,,0.4075000000000002,,32.1,,,,,,0.3702179794506798,,0.4563893200000001,
```