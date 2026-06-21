# Latency Compensation Sweep

Noise policy: sensor noise enabled in every run.
`--teleop-delay` applies fixed command-path delay in the simulator and passes the same delay estimate to safety filters for horizon/buffer inflation.
`--latency-profile-json` applies channel-specific time-varying control/manual/camera latency when provided.
This is an autonomous proxy for latency robustness; true driver behavior still belongs in `human_delay_compensation_rounds.py`.

```csv
filter,delay_s,mpc_delay_comp,n_runs,n_ok,collisions_mean,min_clearance_m_mean,rms_cte_m_mean,speed_ratio_mean,intervention_rate_pct_mean
none,0.0,on,4,4,1.0,-1.5533000000000001,0.6571764052218602,0.7055628391356543,
dob_cbf,0.0,on,4,4,0.0,1.2786170000000001,3.2890996069885174,0.6643093877551021,45.699999999999996
mppi,0.0,on,4,4,0.0,0.21344999999999986,1.1458492525241386,0.609738950059965,62.5
```