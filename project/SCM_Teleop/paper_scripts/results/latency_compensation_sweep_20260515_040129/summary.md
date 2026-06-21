# Latency Compensation Sweep

Noise policy: sensor noise enabled in every run.
`--teleop-delay` applies fixed command-path delay in the simulator and passes the same delay estimate to safety filters for horizon/buffer inflation.
`--latency-profile-json` applies channel-specific time-varying control/manual/camera latency when provided.
This is an autonomous proxy for latency robustness; true driver behavior still belongs in `human_delay_compensation_rounds.py`.

```csv
filter,delay_s,mpc_delay_comp,n_runs,n_ok,collisions_mean,min_clearance_m_mean,rms_cte_m_mean,speed_ratio_mean,intervention_rate_pct_mean
none,0.0,on,32,32,1.8125,-1.5259843750000002,1.01807023790231,0.5680168557065683,
dob_cbf,0.0,on,32,32,0.21875,0.51823509375,1.817480523813358,0.49086918027856746,54.13125
mppi,0.0,on,32,32,0.1875,0.20220928124999998,1.2494422266305945,0.5029887727834836,67.6
```