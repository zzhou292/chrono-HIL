# Terrain Estimator Benchmark

Noise policy: sensor noise enabled in every run.
The estimator starts from neutral dirt/n=0.7 in every run; OOD terrains are generated into this result directory.
Estimator tail metrics are averaged after t=4s, or over all finite estimates if a quick run is shorter.

```csv
distribution,n_runs,n_ok,n_abs_err_tail_mean,n_abs_err_tail_std,rms_cte_m_mean,first_update_time_s_mean
id,3,3,0.06874046215429408,0.06484522260385626,0.19831674651895215,4.212
ood,3,3,0.10502458624454143,0.11445466913249246,0.3548115173415716,4.164
```