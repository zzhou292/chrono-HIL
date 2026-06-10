"""Online Dallas-style state-augmented UKF terrain estimator (runtime backend).

Wraps the offline UKF math from ``deliverables/ukf_paper_validation.py`` into the
same runtime interface as ``LearnedTerrainEstimator`` so the acados NMPC can run
it live (selectable via ``--terrain-estimator-backend nn_ukf``). This is what
makes the NN-UKF closed-loop-testable instead of offline-replay-only.

State z = [x, y, psi, u, v, omega, n]; measurement y = [x, y, psi, u, v, omega,
ay] (ay = Fy_total/m). One predict+update UKF step is run per observe() call
(i.e. every control tick), reconstructing the steering angle from the slip
angle + state; estimate() simply returns the current smoothed n to the MPC at
its throttled pull cadence. Stepping the UKF per tick (not per MPC pull) is
required to identify n on firm soil. Soil is mapped from the estimated n
along the canonical clay->dirt->sand manifold (matching the deployed estimator
and giving the Fy surrogate a self-consistent soil vector).
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


class DallasUKFTerrainEstimator:
    """Online state-augmented UKF estimator (n only), runtime-interface compatible."""

    def __init__(self, model_dir: Optional[str] = None,
                 initial_terrain: Optional[Dict[str, float]] = None,
                 *, update_interval: int = 10, verbose: bool = False,
                 q_n: float = 0.01, smoothing_alpha: float = 0.1,
                 r_ay: float = 0.3,
                 mlp_meas: bool = False, mlp_meas_sigma: float = 0.12,
                 mlp_model_dir: Optional[str] = None,
                 # API-compat (ignored)
                 window_size: int = 50, min_excitation: float = 0.0,
                 **_ignored):
        # Lazy import of the UKF machinery + the Fy surrogate loader.
        from ukf_paper_validation import (  # noqa: E402
            StateAugmentedUKF, SoilParams, Vehicle,
            manifold_soil_from_n, _load_vehicle_fy_model,
        )
        self._SoilParams = SoilParams
        self._soil_from_n = manifold_soil_from_n
        self._veh = Vehicle()
        # Precompute the vehicle_fy surrogate as NUMPY weights so the per-
        # sigma-point force evals are fast (a torch forward per sigma point
        # stalls the real-time control loop). MLP: 10 -> h1 -> h2 -> 2, ReLU.
        if model_dir:
            import json as _json, pickle as _pickle, torch as _torch
            md = Path(model_dir)
            sd = _torch.load(md / "weights.pt", map_location="cpu")
            W = {k: v.detach().numpy().astype(np.float64) for k, v in sd.items()}
            with open(md / "scaler.pkl", "rb") as _f:
                sc = _pickle.load(_f)
            self._fy_dir_name = md.name
        else:
            net, sc = _load_vehicle_fy_model()
            if net is None:
                raise RuntimeError("vehicle_fy_64_32 surrogate not found (needed by nn_ukf backend)")
            W = {k: v.detach().numpy().astype(np.float64) for k, v in net.named_parameters()}
            self._fy_dir_name = "vehicle_fy_64_32"
        self._W0, self._b0 = W["net.0.weight"], W["net.0.bias"]
        self._W2, self._b2 = W["net.2.weight"], W["net.2.bias"]
        self._W4, self._b4 = W["net.4.weight"], W["net.4.bias"]
        self._xm = np.asarray(sc["x_mean"], dtype=np.float64)
        self._xs = np.asarray(sc["x_std"], dtype=np.float64)
        self._ym = np.asarray(sc["y_mean"], dtype=np.float64)
        self._ys = np.asarray(sc["y_std"], dtype=np.float64)
        self._verbose = verbose
        self._update_interval = max(1, int(update_interval))
        self._n_lo, self._n_hi = _N_BOUNDS
        self._dphi_dn_deg = 7.0  # preset phi-vs-n slope (deg per unit n), matches MLP estimator

        n0 = float((initial_terrain or {}).get("n", 0.7))
        n0 = min(max(n0, self._n_lo), self._n_hi)
        # State/cov/noise (from the offline UKF; q_n widened for online tracking).
        z0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n0])
        P0 = np.diag([0.5**2, 0.5**2, 0.01**2, 0.3**2, 0.3**2, 0.01**2, 0.12**2])
        Q = np.diag([0.04**2, 0.04**2, 0.002**2, 0.04**2, 0.04**2, 0.004**2, q_n**2])
        _Rdiag = [0.05, 0.05, 0.005, 0.05, 0.05, 0.005, float(r_ay)]
        # Optional proprioceptive (vertical-dynamics) pseudo-measurement of n,
        # realised by the deployed window-MLP. The lateral-force channel cannot
        # observe n on firm soil at small closed-loop slip (dFy/dn -> 0, signal
        # << meas noise); the MLP reads vertical/vibration features that DO
        # distinguish firm from soft soil, so we add y[7]=n_MLP with h(z)[7]=n.
        # On firm sand the force-channel gain collapses and this term carries n.
        self._mlp_meas = bool(mlp_meas)
        self._mlp = None
        self._n_mlp = n0
        self._mlp_sigma = float(mlp_meas_sigma)
        self._mlp_het = False
        if self._mlp_meas:
            from learned_terrain_estimator import LearnedTerrainEstimator  # noqa: E402
            # Prefer the heteroscedastic window-MLP (reports a calibrated
            # per-sample n-uncertainty) so the proprioceptive measurement noise
            # R_n is data-driven, not a hand-set constant. Fall back to the
            # deployed MLP (constant sigma) if the het checkpoint is absent.
            _het_dir = _SIM_DIR.parent / "nn_models" / "terrain_window_mlp_het"
            _mlp_dir = mlp_model_dir or (str(_het_dir) if _het_dir.exists()
                                         else str(_SIM_DIR.parent / "nn_models" / "terrain_window_mlp"))
            self._mlp = LearnedTerrainEstimator(
                model_dir=_mlp_dir, initial_terrain=initial_terrain,
                update_interval=1, verbose=False)
            self._mlp_het = bool(getattr(self._mlp, "_het", False))
            _Rdiag = _Rdiag + [float(mlp_meas_sigma)]   # placeholder; set per-step
        R = np.diag(np.array(_Rdiag) ** 2)
        self._ukf = StateAugmentedUKF(z0=z0, P0=P0, Q=Q, R=R, alpha=0.35, kappa=0.0)
        # dummy soil_template; manifold mapping supplies the real soil from n.
        self._soil_template = SoilParams(kc=13200.0, kphi=692200.0, n=n0,
                                         c=4140.0, phi=math.radians(13.0),
                                         kx=0.01, ky=0.01)

        self.output_names = ("n",)
        self._n_smooth = n0
        self._alpha_smooth = float(smoothing_alpha)
        self._estimated_params = _terrain_params_for_n(n0)
        self._terrain_name = "estimated"
        self._confidence = 0.0
        self._obs_count = 0
        self._initialized = False
        self._last_t = None
        self._latest = None  # most recent observation bundle
        # yaw-rate history for omega_dot
        self._omega_hist: list[float] = []
        self._omega_time: list[float] = []

    def _fy_np(self, z_aug, delta: float):
        """Numpy forward of the vehicle_fy surrogate -> (Fy_total, M_yaw).
        Input order matches ukf_paper_validation._vehicle_fy_total:
        [u, v, omega, delta, Kphi, Kc, n, c, phi_deg, kx]."""
        u, v, omega, n_val = z_aug[3], z_aug[4], z_aug[5], float(np.clip(z_aug[6], self._n_lo, self._n_hi))
        soil = self._soil_from_n(n_val)
        feats = np.array([u, v, omega, delta, soil.kphi, soil.kc, n_val, soil.c,
                          math.degrees(soil.phi), soil.kx], dtype=np.float64)
        x = (feats - self._xm) / self._xs
        h = np.maximum(self._W0 @ x + self._b0, 0.0)
        h = np.maximum(self._W2 @ h + self._b2, 0.0)
        out = (self._W4 @ h + self._b4) * self._ys + self._ym
        return float(out[0]), float(out[1])

    def _bstep(self, z, delta, ax_in, dt):
        x, y, psi, u, v, omega, n_val = z
        Fy, Mz = self._fy_np(z, delta)
        cp, sp = math.cos(psi), math.sin(psi)
        out = np.array([
            x + dt * (u * cp - v * sp),
            y + dt * (u * sp + v * cp),
            ((psi + dt * omega + math.pi) % (2 * math.pi)) - math.pi,
            u + dt * ax_in,
            v + dt * (Fy / self._veh.m - u * omega),
            omega + dt * (Mz / self._veh.Iz),
            float(np.clip(n_val, self._n_lo, self._n_hi)),
        ], dtype=float)
        return out

    # ---- interface: omega_dot (called before observe each tick) ----
    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        self._omega_hist.append(float(omega)); self._omega_time.append(float(t))
        if len(self._omega_hist) > 7:
            self._omega_hist.pop(0); self._omega_time.pop(0)
        if len(self._omega_hist) < 5:
            return 0.0
        dt = self._omega_time[-1] - self._omega_time[0]
        if dt <= 1e-6:
            return 0.0
        return (self._omega_hist[-1] - self._omega_hist[0]) / dt

    # ---- interface: observe (store live signals AND step the UKF) ----
    def observe(self, kappa: float, alpha_f: float, alpha_r: float, u: float,
                Fz_f: float, Fz_r: float, sr: float, ay_imu: float,
                omega_dot: float, *, omega: float = 0.0, v_lateral: float = 0.0,
                x_pos: float = 0.0, y_pos: float = 0.0, psi: float = 0.0,
                ax_cmd: float = 0.0, sim_time: float = 0.0, ax_imu: float = 0.0,
                **_ignored) -> bool:
        u_safe = max(abs(float(u)), 0.5)
        # Reconstruct steering angle: alpha_f = delta - atan2(v + Lf*omega, u).
        delta = float(alpha_f) + math.atan2(float(v_lateral) + self._veh.Lf * float(omega), u_safe)
        self._latest = dict(x=float(x_pos), y=float(y_pos), psi=float(psi),
                            u=float(u), v=float(v_lateral), omega=float(omega),
                            ay=float(ay_imu), ax=float(ax_imu), delta=delta, t=float(sim_time))
        if not self._initialized:
            # seed the kinematic states from the first measurement
            self._ukf.z[0:6] = [self._latest["x"], self._latest["y"], self._latest["psi"],
                                self._latest["u"], self._latest["v"], self._latest["omega"]]
            self._last_t = self._latest["t"]
            self._initialized = True
        # Feed the proprioceptive MLP (vertical-dynamics) channel, if enabled.
        if self._mlp is not None:
            self._mlp.observe(kappa=kappa, alpha_f=alpha_f, alpha_r=alpha_r, u=u,
                              Fz_f=Fz_f, Fz_r=Fz_r, sr=sr, ay_imu=ay_imu,
                              omega_dot=omega_dot, omega=omega, v_lateral=v_lateral,
                              x_pos=x_pos, y_pos=y_pos, psi=psi, ax_cmd=ax_cmd,
                              sim_time=sim_time, ax_imu=ax_imu, **_ignored)
            if self._mlp.should_update():
                self._mlp.estimate()
            self._n_mlp = float(self._mlp.get_bekker_n())
            if self._mlp_het:
                self._mlp_sigma = float(self._mlp.get_n_sigma())
        self._obs_count += 1
        # Step the UKF on EVERY observation (~per control tick). The state-
        # augmented UKF needs fine-grained measurement updates to identify n on
        # firm soil; stepping it only at the MPC pull cadence (e.g. every 8th
        # tick, ~96 ms) integrates the nonlinear bicycle over too long a horizon
        # and the weak firm-soil n-channel never converges (sand |dn| 0.49 vs
        # 0.12 at per-tick stepping). The numpy force forward is cheap enough to
        # run every tick. The MPC still pulls params only at should_update().
        self._ukf_step()
        return True

    def _ukf_step(self) -> None:
        """Run one UKF predict+update using the latest observation and the real
        per-tick dt. Updates the smoothed n estimate in place."""
        m = self._latest
        if m is None:
            return
        dt = m["t"] - (self._last_t if self._last_t is not None else m["t"])
        self._last_t = m["t"]
        if dt <= 1e-4 or m["u"] < 0.8:
            # not enough motion / time advanced; hold estimate
            return
        delta, ax_in = m["delta"], m["ax"]

        def f_dyn(z):
            return self._bstep(z, delta, ax_in, dt)

        if self._mlp_meas:
            def h_meas(z):
                Fy_total, _ = self._fy_np(z, delta)
                out = np.empty(8)
                out[:6] = z[:6]
                out[6] = Fy_total / self._veh.m
                out[7] = z[6]          # direct (proprioceptive) observation of n
                return out
            y_k = np.array([m["x"], m["y"], m["psi"], m["u"], m["v"], m["omega"],
                            m["ay"], self._n_mlp])
            # Time-varying proprioceptive measurement noise: the het MLP's own
            # per-sample sigma. Small on firm sand (MLP confident, force channel
            # dead) -> n follows the MLP; large on soils the MLP is unsure about
            # -> the force channel keeps priority. No hand-set blend.
            self._ukf.R[7, 7] = float(np.clip(self._mlp_sigma, 0.02, 0.5)) ** 2
        else:
            def h_meas(z):
                Fy_total, _ = self._fy_np(z, delta)
                out = np.empty(7)
                out[:6] = z[:6]
                out[6] = Fy_total / self._veh.m
                return out
            y_k = np.array([m["x"], m["y"], m["psi"], m["u"], m["v"], m["omega"], m["ay"]])
        try:
            self._ukf.step(y_k, f_dyn, h_meas)
        except Exception as exc:  # pragma: no cover - defensive
            if self._verbose:
                print(f"  [nn_ukf] step failed: {exc!r}")
            return
        n_raw = float(np.clip(self._ukf.z[6], self._n_lo, self._n_hi))
        self._ukf.z[6] = n_raw
        self._n_smooth += self._alpha_smooth * (n_raw - self._n_smooth)

    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    # ---- interface: estimate (return current smoothed output to the MPC) ----
    def estimate(self) -> Tuple[Dict[str, float], float]:
        # The UKF already steps every observe(); here we just hand the MPC the
        # current smoothed estimate at its (throttled) pull cadence.
        self._obs_count = 0
        return self._sync_outputs()

    def _sync_outputs(self) -> Tuple[Dict[str, float], float]:
        self._estimated_params = _terrain_params_for_n(self._n_smooth)
        # confidence from the n-covariance: tighter P[6,6] -> higher confidence
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
        return float(self._estimated_params["phi"])  # _terrain_params_for_n phi is in degrees

    @property
    def mu_estimate(self) -> float:
        return float(math.tan(math.radians(float(self._estimated_params["phi"]))))
