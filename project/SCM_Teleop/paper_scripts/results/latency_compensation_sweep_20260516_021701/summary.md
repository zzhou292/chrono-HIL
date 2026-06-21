# Latency Compensation Sweep

Noise policy: sensor noise enabled in every run.
`--teleop-delay` applies fixed command-path delay in the simulator and passes the same delay estimate to safety filters for horizon/buffer inflation.
`--latency-profile-json` applies channel-specific time-varying control/manual/camera latency when provided.
This is an autonomous proxy for latency robustness; true driver behavior still belongs in `human_delay_compensation_rounds.py`.

```csv
filter,delay_s,mpc_delay_comp,n_runs,n_ok,collisions_mean,min_clearance_m_mean,rms_cte_m_mean,speed_ratio_mean,intervention_rate_pct_mean
none,0.0,on,1,1,1.0,-0.0181,0.1371940987016715,0.46777536,
mppi,0.0,on,1,1,0.0,0.9345,0.2186646519704954,0.4197160829493087,40.7
```