# terrain_window_mlp_speedcond — EXPERIMENTAL (not deployed)

Window-MLP retrained with feature_version=v2 (the five vertical-dynamics
features divided by mean speed) to test the firm-soil-at-speed aliasing seen
in the spatial-transition open-loop sweep. Stationary held-out RMSE ≈ v1
(val_mse 0.039). On the transition sweeps it cut open-loop into-sand error
~25% but did not fix it (still 0/5 reach sand); net-neutral closed-loop.
Deployed estimator remains nn_models/terrain_window_mlp (feature_version=v1).
