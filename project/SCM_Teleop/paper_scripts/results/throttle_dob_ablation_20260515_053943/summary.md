# Asymmetric Throttle DOB Ablation

Noise policy: sensor noise enabled in every run.
DOB-off uses --dob-ki 0 --dob-max 0 so the controller still constructs the DOB hook but adds no integral action.
Variant notes: dob_on: Standard MPC with default asymmetric throttle DOB (ki=0.15, max=0.35).; dob_off: Standard MPC with DOB gains zeroed; open-loop u_dot = a_x only.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,p95_speed_mps_mean,p95_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std
dob_on,32,32,0.08661035930037728,0.024516362205505755,0.6725506880252101,0.13236351878764407,3.9351252588535415,0.5104853191812354,4.7818306250000004,0.76517169591658,5.252957515678481,0.4479326203806362
dob_off,32,32,0.09057667494932665,0.026526775908228765,0.6096639261061567,0.1515447681337932,3.5671806535114046,0.7234412864623533,4.662101875,0.781367384101939,5.400445406969469,0.6448313422198793
```