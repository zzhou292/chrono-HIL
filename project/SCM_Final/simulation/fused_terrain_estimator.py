"""Regime-aware fusion of the window-MLP and the NN-UKF terrain estimators.

Closed-loop evaluation showed the two learned estimators are *complementary*:
with the per-tick UKF fix the NN-UKF wins clay (|Δn|≈0.039) AND dirt (≈0.050)
but still fails firm sand (≈0.42, an observability limit), while the deployed
window-MLP is balanced and strong on firm soil (sand ≈0.04). This estimator
runs BOTH live and blends their n by soil regime: trust the NN-UKF where it
excels (soft/mid soil) and the MLP on firm soil. The MLP estimate (reliable
across the whole range) is used as the regime selector, so the blend doesn't
depend on the NN-UKF's unreliable firm-soil read.

    w_ukf(n_mlp) = sigmoid((n0 - n_mlp) / k)        # ->1 soft, ->0 firm
    n_fused      = w_ukf * n_ukf + (1 - w_ukf) * n_mlp

n0=0.85, k=0.06: w_ukf ≈ 1 at clay/dirt (n≲0.7), ≈ 0 approaching sand (n≳1.0).
(The previous n0=0.65 was tuned against the broken NN-UKF, before the per-tick
fix moved the NN-UKF/MLP crossover firmer; see NNUKF_CLOSED_LOOP_FINDINGS.md.)

Runtime-interface compatible with LearnedTerrainEstimator (so it is selectable
as --terrain-estimator-backend fused).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

_SIM_DIR = Path(__file__).resolve().parent
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from learned_terrain_estimator import LearnedTerrainEstimator, _terrain_params_for_n  # noqa: E402
from dallas_ukf_terrain_estimator import DallasUKFTerrainEstimator  # noqa: E402


class FusedTerrainEstimator:
    def __init__(self, model_dir: Optional[str] = None,
                 initial_terrain: Optional[Dict[str, float]] = None,
                 *, update_interval: int = 10, verbose: bool = False,
                 fuse_n0: float = 0.85, fuse_k: float = 0.06,
                 mlp_model_dir: Optional[str] = None,
                 fy_model_dir: Optional[str] = None,
                 window_size: int = 50, min_excitation: float = 0.0,
                 q_n: float = 0.01, **_ignored):
        mlp_dir = mlp_model_dir or str(_SIM_DIR.parent / "nn_models" / "terrain_window_mlp")
        self._mlp = LearnedTerrainEstimator(
            model_dir=mlp_dir, initial_terrain=initial_terrain,
            update_interval=update_interval, verbose=False,
            window_size=window_size, min_excitation=min_excitation)
        self._ukf = DallasUKFTerrainEstimator(
            model_dir=fy_model_dir, initial_terrain=initial_terrain,
            update_interval=update_interval, verbose=False, q_n=q_n)
        self._n0 = float(fuse_n0)
        self._k = float(fuse_k)
        self._verbose = verbose
        self.output_names = ("n",)
        self._n_fused = float((initial_terrain or {}).get("n", 0.7))
        self._estimated_params = _terrain_params_for_n(self._n_fused)
        self._confidence = 0.0
        self._terrain_name = "estimated"

    def estimate_omega_dot(self, omega: float, t: float):
        self._ukf.estimate_omega_dot(omega, t)
        return self._mlp.estimate_omega_dot(omega, t)

    def observe(self, *args, **kwargs) -> bool:
        self._ukf.observe(*args, **kwargs)
        return self._mlp.observe(*args, **kwargs)

    def should_update(self) -> bool:
        return self._mlp.should_update()

    def estimate(self) -> Tuple[Dict[str, float], float]:
        p_mlp, c_mlp = self._mlp.estimate()
        # advance the UKF too (its estimate() runs the live UKF step)
        try:
            p_ukf, c_ukf = self._ukf.estimate()
            n_ukf = float(self._ukf.get_bekker_n())
        except Exception:
            n_ukf, c_ukf = float(self._mlp.get_bekker_n()), 0.0
        n_mlp = float(self._mlp.get_bekker_n())
        # regime weight from the (reliable) MLP estimate
        w = 1.0 / (1.0 + math.exp((n_mlp - self._n0) / self._k))
        self._n_fused = w * n_ukf + (1.0 - w) * n_mlp
        self._estimated_params = _terrain_params_for_n(self._n_fused)
        self._confidence = float(w * c_ukf + (1.0 - w) * c_mlp)
        if self._verbose:
            print(f"  [fused] n_mlp={n_mlp:.3f} n_ukf={n_ukf:.3f} w_ukf={w:.2f} -> {self._n_fused:.3f}")
        return dict(self._estimated_params), self._confidence

    def get_bekker_n(self) -> float:
        return float(self._n_fused)

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_friction_angle_deg(self) -> float:
        return float(self._estimated_params["phi"])

    @property
    def mu_estimate(self) -> float:
        return float(math.tan(math.radians(float(self._estimated_params["phi"]))))

    def get_phi_uncertainty_deg(self) -> float:
        # take the more conservative (larger) of the two backends' sigma
        return max(self._mlp.get_phi_uncertainty_deg(), self._ukf.get_phi_uncertainty_deg())
