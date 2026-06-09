"""Online Bekker-UKF terrain estimator (runtime backend).

The analytical-tyre sibling of ``dallas_ukf_terrain_estimator.py``: a
state-augmented UKF whose lateral-force / yaw-moment model is the analytical
Bekker--Wong 4-wheel double-track tyre (Newton-Raphson sinkage + contact-patch
stress integration) rather than the learned vehicle_fy surrogate. Wired so the
NMPC can run it live (``--terrain-estimator-backend bekker_ukf``) for a
closed-loop, apples-to-apples comparison against the NN-UKF, the deployed MLP,
and the regime fusion --- replacing the old offline-replay-only Bekker-UKF.

State z = [x, y, psi, u, v, omega, n]; measurement y = [x, y, psi, u, v, omega,
ay] (ay = Fy_total/m). One predict+update UKF step per observe() (every control
tick), reconstructing the steering angle from the slip angle + state. Per
CLAUDE.md the soil is not known online, so the five non-n Bekker--Mohr params
are taken from the canonical clay->dirt->sand manifold evaluated at the current
n estimate (matching the deployed MLP and the online NN-UKF); only n is the
free UKF-augmented state. The contact integration is coarsened
(``BEKKER_N_INTEGRATION``) for real-time per-tick stepping.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

_SIM_DIR = Path(__file__).resolve().parent
_DELIV_DIR = _SIM_DIR.parent / "deliverables"
for _d in (str(_SIM_DIR), str(_DELIV_DIR)):
    if _d not in sys.path:
        sys.path.insert(0, _d)

from learned_terrain_estimator import _terrain_params_for_n, _N_BOUNDS  # noqa: E402

# Coarser contact-patch integration than the offline reproducer (24) so the
# 4-wheel force can be evaluated at ~30 UKF sigma points every control tick.
# n=6 keeps per-tick cost ~15 ms with no meaningful accuracy loss vs 24
# (canonical sand |dn| 0.35 vs 0.34; clay 0.065 vs 0.071).
BEKKER_N_INTEGRATION = 6


class BekkerUKFTerrainEstimator:
    """Online state-augmented UKF with the analytical Bekker tyre (n only)."""

    def __init__(self, model_dir: Optional[str] = None,
                 initial_terrain: Optional[Dict[str, float]] = None,
                 *, update_interval: int = 10, verbose: bool = False,
                 q_n: float = 0.01, smoothing_alpha: float = 0.1,
                 n_integration: int = BEKKER_N_INTEGRATION,
                 # API-compat (ignored)
                 window_size: int = 50, min_excitation: float = 0.0,
                 **_ignored):
        from ukf_paper_validation import (  # noqa: E402
            StateAugmentedUKF, SoilParams, Vehicle,
            manifold_soil_from_n, _bekker_wheel_forces, _H_CG, WHEEL,
        )
        self._SoilParams = SoilParams
        self._soil_from_n = manifold_soil_from_n
        self._bekker_wheel_forces = _bekker_wheel_forces
        self._H_CG = _H_CG
        self._WHEEL = WHEEL
        self._veh = Vehicle()
        self._n_int = int(n_integration)
        self._verbose = verbose
        self._update_interval = max(1, int(update_interval))
        self._n_lo, self._n_hi = _N_BOUNDS
        self._dphi_dn_deg = 7.0

        n0 = float((initial_terrain or {}).get("n", 0.7))
        n0 = min(max(n0, self._n_lo), self._n_hi)
        z0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n0])
        P0 = np.diag([0.5**2, 0.5**2, 0.01**2, 0.3**2, 0.3**2, 0.01**2, 0.12**2])
        Q = np.diag([0.04**2, 0.04**2, 0.002**2, 0.04**2, 0.04**2, 0.004**2, q_n**2])
        R = np.diag(np.array([0.05, 0.05, 0.005, 0.05, 0.05, 0.005, 0.3]) ** 2)
        self._ukf = StateAugmentedUKF(z0=z0, P0=P0, Q=Q, R=R, alpha=0.35, kappa=0.0)
        self._soil_template = manifold_soil_from_n(n0)

        self.output_names = ("n",)
        self._n_smooth = n0
        self._alpha_smooth = float(smoothing_alpha)
        self._estimated_params = _terrain_params_for_n(n0)
        self._terrain_name = "estimated"
        self._confidence = 0.0
        self._obs_count = 0
        self._initialized = False
        self._last_t = None
        self._latest = None
        self._omega_hist: list[float] = []
        self._omega_time: list[float] = []

    # ---- analytical 4-wheel Bekker lateral force / yaw moment ----
    def _force(self, z, delta: float, ax_in: float) -> Tuple[float, float]:
        veh = self._veh
        u, v, omega = float(z[3]), float(z[4]), float(z[5])
        n_val = float(np.clip(z[6], self._n_lo, self._n_hi))
        st = self._soil_template
        soil = self._SoilParams(kc=st.kc, kphi=st.kphi, n=n_val, c=st.c,
                                phi=st.phi, kx=st.kx, ky=st.ky)
        L = veh.Lf + veh.Lr
        T = veh.track
        Fz_f = veh.m * veh.g * veh.Lr / (2.0 * L)
        Fz_r = veh.m * veh.g * veh.Lf / (2.0 * L)
        u_L = u - 0.5 * T * omega
        u_R = u + 0.5 * T * omega
        v_f = v + veh.Lf * omega
        v_r = v - veh.Lr * omega
        cd, sd = math.cos(delta), math.sin(delta)
        wheels = [  # (Fz, vl, vc, x_offset_for_yaw, is_front)
            (Fz_f,  u_L * cd + v_f * sd, -u_L * sd + v_f * cd,  veh.Lf, -0.5 * T),
            (Fz_f,  u_R * cd + v_f * sd, -u_R * sd + v_f * cd,  veh.Lf,  0.5 * T),
            (Fz_r,  u_L,                  v_r,                 -veh.Lr, -0.5 * T),
            (Fz_r,  u_R,                  v_r,                 -veh.Lr,  0.5 * T),
        ]
        s_t = 0.05
        Fy_total = 0.0
        Mz = 0.0
        for (Fz, vl, vc, lx, ty) in wheels:
            Fx_w, Fy_w, _, _ = self._bekker_wheel_forces(
                Fz_required=Fz, vl=vl, vc=vc,
                omega=max(vl, 0.5) / (self._WHEEL.r * (1.0 - s_t)),
                soil=soil, n_integration=self._n_int)
            # Front wheels: rotate tyre-frame force into body frame.
            is_front = lx > 0.0
            if is_front:
                Fx_b = Fx_w * cd - Fy_w * sd
                Fy_b = Fx_w * sd + Fy_w * cd
            else:
                Fx_b, Fy_b = Fx_w, Fy_w
            Fy_total += Fy_b
            Mz += lx * Fy_b - ty * Fx_b
        return float(Fy_total), float(Mz)

    def _bstep(self, z, delta, ax_in, dt):
        x, y, psi, u, v, omega, n_val = z
        Fy, Mz = self._force(z, delta, ax_in)
        cp, sp = math.cos(psi), math.sin(psi)
        return np.array([
            x + dt * (u * cp - v * sp),
            y + dt * (u * sp + v * cp),
            ((psi + dt * omega + math.pi) % (2 * math.pi)) - math.pi,
            u + dt * ax_in,
            v + dt * (Fy / self._veh.m - u * omega),
            omega + dt * (Mz / self._veh.Iz),
            float(np.clip(n_val, self._n_lo, self._n_hi)),
        ], dtype=float)

    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        self._omega_hist.append(float(omega)); self._omega_time.append(float(t))
        if len(self._omega_hist) > 7:
            self._omega_hist.pop(0); self._omega_time.pop(0)
        if len(self._omega_hist) < 5:
            return 0.0
        dt = self._omega_time[-1] - self._omega_time[0]
        return 0.0 if dt <= 1e-6 else (self._omega_hist[-1] - self._omega_hist[0]) / dt

    def observe(self, kappa: float, alpha_f: float, alpha_r: float, u: float,
                Fz_f: float, Fz_r: float, sr: float, ay_imu: float,
                omega_dot: float, *, omega: float = 0.0, v_lateral: float = 0.0,
                x_pos: float = 0.0, y_pos: float = 0.0, psi: float = 0.0,
                ax_cmd: float = 0.0, sim_time: float = 0.0, ax_imu: float = 0.0,
                **_ignored) -> bool:
        u_safe = max(abs(float(u)), 0.5)
        delta = float(alpha_f) + math.atan2(float(v_lateral) + self._veh.Lf * float(omega), u_safe)
        self._latest = dict(x=float(x_pos), y=float(y_pos), psi=float(psi),
                            u=float(u), v=float(v_lateral), omega=float(omega),
                            ay=float(ay_imu), ax=float(ax_imu), delta=delta, t=float(sim_time))
        if not self._initialized:
            self._ukf.z[0:6] = [self._latest["x"], self._latest["y"], self._latest["psi"],
                                self._latest["u"], self._latest["v"], self._latest["omega"]]
            self._last_t = self._latest["t"]
            self._initialized = True
        self._obs_count += 1
        self._ukf_step()
        return True

    def _ukf_step(self) -> None:
        m = self._latest
        if m is None:
            return
        dt = m["t"] - (self._last_t if self._last_t is not None else m["t"])
        self._last_t = m["t"]
        if dt <= 1e-4 or m["u"] < 0.8:
            return
        # Refresh the (non-n) soil params from the manifold at the current n.
        self._soil_template = self._soil_from_n(float(self._n_smooth))
        delta, ax_in = m["delta"], m["ax"]

        def f_dyn(z):
            return self._bstep(z, delta, ax_in, dt)

        def h_meas(z):
            Fy_total, _ = self._force(z, delta, ax_in)
            out = np.empty(7)
            out[:6] = z[:6]
            out[6] = Fy_total / self._veh.m
            return out

        y_k = np.array([m["x"], m["y"], m["psi"], m["u"], m["v"], m["omega"], m["ay"]])
        try:
            self._ukf.step(y_k, f_dyn, h_meas)
        except Exception as exc:  # pragma: no cover
            if self._verbose:
                print(f"  [bekker_ukf] step failed: {exc!r}")
            return
        n_raw = float(np.clip(self._ukf.z[6], self._n_lo, self._n_hi))
        self._ukf.z[6] = n_raw
        self._n_smooth += self._alpha_smooth * (n_raw - self._n_smooth)

    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    def estimate(self) -> Tuple[Dict[str, float], float]:
        self._obs_count = 0
        return self._sync_outputs()

    def _sync_outputs(self) -> Tuple[Dict[str, float], float]:
        self._estimated_params = _terrain_params_for_n(self._n_smooth)
        p_nn = float(self._ukf.P[6, 6])
        self._confidence = float(1.0 / (1.0 + 50.0 * p_nn))
        return dict(self._estimated_params), self._confidence

    def get_bekker_n(self) -> float:
        return float(self._n_smooth)

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_n_uncertainty(self) -> float:
        return float(math.sqrt(max(self._ukf.P[6, 6], 0.0)))

    def get_phi_uncertainty_deg(self) -> float:
        return float(self.get_n_uncertainty() * self._dphi_dn_deg)

    def get_friction_angle_deg(self) -> float:
        return float(self._estimated_params["phi"])

    @property
    def mu_estimate(self) -> float:
        return float(math.tan(math.radians(float(self._estimated_params["phi"]))))
