# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
Fixed downstream safety filter: mppi; tire model is the swept variable.
Standard-MPC speed-weight is 15 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
pacejka,8,8,0.0,0.0,0.0,0.0,8.155860625,3.4030223425354005,0.07650202674253757,0.019158989527774475,0.43175927419026855,0.08153569183273768,3.110700662220675,0.6635637922733318,18.607214687628723,3.4670105354593677
tmeasy,8,8,0.0,0.0,0.0,0.0,8.039720375,3.3932823336353395,0.07006897552440906,0.012874880334098298,0.43447187690672917,0.07664703414382142,3.1134824070141276,0.5580890627658543,18.697841691489472,3.320860409640817
closed_loop_mlp,8,8,0.0,0.0,0.25,0.4629100498862757,4.54187825,3.7696914407035553,0.9857106807849693,1.816565532612075,0.5998925388656322,0.16737910380636112,5.268018141998961,0.9534988752451838,26.11115683245076,7.1565883982245415
```