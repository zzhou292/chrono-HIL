# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
No downstream safety filter is used; the standard MPC's obstacle barrier is the only autonomous obstacle-avoidance mechanism.
Standard-MPC speed-weight is 15 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC speed-cost-mode is symmetric.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
pacejka,1,1,0.0,,0.0,,19.814444,,0.05838586085202478,,0.35320648,,2.464112291350531,,11.04874046864497,
tmeasy,1,1,0.0,,0.0,,19.859939,,0.059526952072972796,,0.3513032,,2.114506828528073,,10.996298234450663,
closed_loop_mlp,1,1,0.0,,0.0,,16.831073,,0.061042652215676216,,0.42625443999999996,,4.383854324734446,,14.051039049055944,
```