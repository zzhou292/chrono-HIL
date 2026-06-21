# DOB-CBF NN Ablation

Noise policy: sensor noise enabled in every run.
Default planner policy: MPC is blind to rocks, so the filter is the sole obstacle avoider. Pass `--aware` to benchmark the easier combined planner+filter stack.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std
dob_cbf_no_nn,16,16,1.625,0.49999999999999994,2.5,0.8944271909999159,-0.438675,0.23550030290143295,49.8,8.863783992554572,,,,,1.818108186133567,1.1495006355745743,0.6317850107185732,0.1289369931301435
```