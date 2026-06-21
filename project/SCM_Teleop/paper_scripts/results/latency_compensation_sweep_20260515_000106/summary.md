# Latency Compensation Sweep

Noise policy: sensor noise enabled in every run.
`--teleop-delay` applies fixed command-path delay in the simulator and passes the same delay estimate to safety filters for horizon/buffer inflation.
`--latency-profile-json` applies channel-specific time-varying control/manual/camera latency when provided.
This is an autonomous proxy for latency robustness; true driver behavior still belongs in `human_delay_compensation_rounds.py`.

```csv
filter,delay_s,mpc_delay_comp,n_runs,n_ok,collisions_mean,min_clearance_m_mean,rms_cte_m_mean,speed_ratio_mean,intervention_rate_pct_mean
none,0.0,on,1,1,1.0,-1.6475,1.0062704393083972,0.5455228091236495,
mppi,0.0,on,1,1,0.0,0.1760999999999999,0.3043298576145468,0.2768787915407854,63.9
```