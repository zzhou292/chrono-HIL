# Terrain Estimator Benchmark

Noise policy: sensor noise enabled in every run.
The estimator starts from neutral dirt/n=0.7 in every run; OOD terrains are generated into this result directory.
Estimator tail metrics are averaged after t=8s, or over all finite estimates if a quick run is shorter.

```csv
distribution,n_runs,n_ok,n_abs_err_tail_mean,n_abs_err_tail_std,rms_cte_m_mean,first_update_time_s_mean
id,1,1,0.22412224434389139,,0.07564128284972657,4.212
ood,1,1,0.027979171471927078,,0.09093312917084298,4.26
```