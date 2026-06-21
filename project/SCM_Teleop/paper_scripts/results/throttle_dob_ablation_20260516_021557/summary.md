# Asymmetric Throttle DOB Ablation

Noise policy: sensor noise enabled in every run.
DOB-off uses --dob-ki 0 --dob-max 0 so the controller still constructs the DOB hook but adds no integral action.
Variant notes: dob_on: Standard MPC with default asymmetric throttle DOB (ki=0.15, max=0.35).; dob_off: Standard MPC with DOB gains zeroed; open-loop u_dot = a_x only.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,p95_speed_mps_mean,p95_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std
dob_on,1,1,0.07757004597395828,,0.5334653599999999,,2.6673267999999997,,3.8892249999999993,,4.987151424287855,
dob_off,1,1,0.09716089132881604,,0.24768783999999994,,1.2384391999999997,,2.577359999999999,,4.907241379310345,
```