# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
No downstream safety filter is used; the standard MPC's obstacle barrier is the only autonomous obstacle-avoidance mechanism.
Standard-MPC speed-weight is 0 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC obstacle-weight is 10000.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,2,2,1.0,0.0,1.0,0.0,-1.4347500000000002,0.30667221100060565,0.22185454446918257,0.0007330327706472393,0.724258935532234,0.14838070753852808,3.7549460431654675,1.611821928669436,31.595452788859216,5.569185917728158
```