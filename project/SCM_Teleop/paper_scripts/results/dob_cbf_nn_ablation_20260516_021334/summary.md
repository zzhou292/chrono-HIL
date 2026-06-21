# DOB-CBF NN Ablation

Noise policy: sensor noise enabled in every run.
Default planner policy: MPC is blind to rocks, so the filter is the sole obstacle avoider. Pass `--aware` to benchmark the easier combined planner+filter stack.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std
no_filter,1,1,1.0,,1.0,,-0.9708999999999999,,,,,,,,0.11265597191111532,,0.4756810000000001,
dob_cbf_nn,1,1,0.0,,1.0,,0.9383000000000004,,30.9,,,,,,0.3644414625196178,,0.43147316,
dob_cbf_no_nn,1,1,0.0,,1.0,,0.05480000000000018,,39.5,,,,,,0.551509883313273,,0.48921788,
```