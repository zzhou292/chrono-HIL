"""
Batched NN-surrogate vehicle dynamics for the predictive safety shield
======================================================================

Replaces the single-step CBF linearization with a vectorized rollout that
propagates K candidate command sequences forward through the same NN tire
surrogate used by the planning NMPC.  The rollout is the foundation for both
the MPPI (sampling) and NMPC (gradient) shields.

Why numpy, not CasADi?
----------------------
For a 512-trajectory shield with a 10-step horizon, CasADi function calls
have non-trivial Python overhead even with the fixed-size batched evaluator
in ``nn_tire_model.py``.  The MLP-16-4 / MLP-32-16 models used in the paper
are tiny (~hundreds of params), so a pure numpy forward pass on a
``(input_dim, K)`` matrix runs in a few hundred microseconds and matches
the 10 Hz safety-loop budget with plenty of head-room.

Implemented for ``StaticMLP`` (the headline ``paper_v2_mlp_*`` checkpoints).
ResNet / temporal / rate variants fall back to the scalar CasADi function;
that is slower but correct, and is wired so the same shield code paths
work regardless of architecture.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


# ----------------------------------------------------------------------------
# Numpy-vectorized MLP forward pass over a loaded NNTireModel
# ----------------------------------------------------------------------------

class NumpyTireSurrogate:
    """Fast batched evaluator wrapping an ``NNTireModel`` instance.

    Exposes ``predict_batch(alpha, Fz, u, kappa, terrain) -> (Fx, Fy)`` where
    each input is a 1-D numpy array of length ``K`` (or a scalar broadcastable
    to K).  Output arrays have the same length.

    For the static MLP / static ResNet / static DenseNet architectures we run
    a pure-numpy forward pass over the raw weights.  For temporal/rate/GRU
    architectures we fall back to a per-sample loop on the CasADi scalar
    function — accurate but slower.

    Args:
        nn_model: A loaded ``NNTireModel`` (any subclass) from
            ``nn_tire_model.load_nn_tire_model``.
    """

    def __init__(self, nn_model):
        self._nn = nn_model
        self._fast = nn_model.model_type in ('static_mlp', 'static_resnet',
                                              'static_densenet')
        self._X_mean = np.asarray(nn_model._X_mean, dtype=np.float64).reshape(-1)
        self._X_scale = np.asarray(nn_model._X_scale, dtype=np.float64).reshape(-1)
        self._y_mean = np.asarray(nn_model._y_mean, dtype=np.float64).reshape(-1)
        self._y_scale = np.asarray(nn_model._y_scale, dtype=np.float64).reshape(-1)
        self._w = nn_model._weights

        # Pre-discover layer indices / weight blobs for the fast paths
        if nn_model.model_type == 'static_mlp':
            self._layer_idx = sorted(set(
                int(k.split('.')[1]) for k in self._w
                if k.startswith('layers.') and 'weight' in k
            ))
        elif nn_model.model_type == 'static_resnet':
            ckpt = nn_model._checkpoint
            self._n_blocks = ckpt.get('n_blocks', 2) if isinstance(ckpt, dict) else 2
        elif nn_model.model_type == 'static_densenet':
            ckpt = nn_model._checkpoint
            self._n_dense_layers = (ckpt.get('n_dense_layers', 4)
                                    if isinstance(ckpt, dict) else 4)

    # -- public API -----------------------------------------------------------

    def predict_batch(self, alpha, Fz, u, kappa, terrain_params):
        """Evaluate the surrogate for ``K`` samples in parallel.

        Args:
            alpha, Fz, u, kappa: arrays of length K (or scalars).
            terrain_params: dict with keys Kphi, Kc, n, c, phi, k.  ``phi`` may
                be degrees or radians; it is converted to the units expected by
                the loaded checkpoint.
                Scalar values broadcast across all samples.

        Returns:
            (Fx, Fy): two arrays of length K (per-wheel tire forces, N).
        """
        K = self._broadcast_size(alpha, Fz, u, kappa)
        alpha = np.broadcast_to(np.asarray(alpha, dtype=np.float64), (K,))
        Fz = np.broadcast_to(np.asarray(Fz, dtype=np.float64), (K,))
        u = np.broadcast_to(np.asarray(u, dtype=np.float64), (K,))
        kappa = np.broadcast_to(np.asarray(kappa, dtype=np.float64), (K,))

        sr = np.zeros(K)  # steering rate: 0 for static models
        Kphi = float(terrain_params['Kphi'])
        Kc = float(terrain_params['Kc'])
        n_t = float(terrain_params['n'])
        c = float(terrain_params['c'])
        if hasattr(self._nn, 'phi_feature_value'):
            phi = self._nn.phi_feature_value(float(terrain_params['phi']))
        else:
            phi_raw = float(terrain_params['phi'])
            phi = float(np.radians(phi_raw) if abs(phi_raw) > 2.0 * np.pi else phi_raw)
        k = float(terrain_params['k'])

        if self._fast and self._nn.model_type == 'static_mlp':
            return self._mlp_static_batch(alpha, Fz, u, kappa, sr, n_t,
                                          Kphi, Kc, c, phi, k)
        if self._fast and self._nn.model_type == 'static_resnet':
            return self._resnet_static_batch(alpha, Fz, u, kappa, sr, n_t,
                                             Kphi, Kc, c, phi, k)
        if self._fast and self._nn.model_type == 'static_densenet':
            return self._densenet_static_batch(alpha, Fz, u, kappa, sr, n_t,
                                               Kphi, Kc, c, phi, k)

        # Slow fallback: per-sample CasADi scalar evaluation.  We hold phi
        # in radians, which matches what the CasADi function expects for the
        # paper_v2 checkpoints (v6 format).
        Fxs = np.empty(K)
        Fys = np.empty(K)
        for i in range(K):
            fx, fy = self._nn.predict_numeric(
                float(alpha[i]), float(Fz[i]), float(u[i]),
                kappa=float(kappa[i]),
                n_terrain=n_t,
                terrain_params={'Kphi': Kphi, 'Kc': Kc,
                                'c': c, 'phi': np.degrees(phi), 'k': k})
            Fxs[i] = fx
            Fys[i] = fy
        return Fxs, Fys

    # -- internals ------------------------------------------------------------

    @staticmethod
    def _broadcast_size(*arrs):
        K = 1
        for a in arrs:
            arr = np.atleast_1d(np.asarray(a))
            K = max(K, arr.size)
        return K

    def _stack_features_static(self, alpha, Fz, u, kappa, sr, n_t,
                                Kphi, Kc, c, phi, k):
        """Build (11, K) feature matrix in the StaticMLP/ResNet input order.

        Order matches the symbolic function in ``nn_tire_model.py``:
            (kappa, alpha, u, Fz, sr, Kphi, Kc, n_terrain, c, phi, k)
        """
        K = alpha.size
        const_row = lambda v: np.full(K, v)
        X = np.vstack([
            kappa, alpha, u, Fz, sr,
            const_row(Kphi), const_row(Kc), np.full(K, n_t),
            const_row(c), const_row(phi), const_row(k),
        ])  # (11, K)
        return X

    def _mlp_static_batch(self, alpha, Fz, u, kappa, sr, n_t,
                          Kphi, Kc, c, phi, k):
        X = self._stack_features_static(alpha, Fz, u, kappa, sr, n_t,
                                         Kphi, Kc, c, phi, k)
        H = (X - self._X_mean[:, None]) / self._X_scale[:, None]
        for i in self._layer_idx:
            W = self._w[f'layers.{i}.weight']
            b = self._w[f'layers.{i}.bias'].reshape(-1, 1)
            H = W @ H + b
            if i < self._layer_idx[-1]:
                H = np.tanh(H)
        Y = H * self._y_scale[:, None] + self._y_mean[:, None]
        return Y[0], Y[1]

    def _resnet_static_batch(self, alpha, Fz, u, kappa, sr, n_t,
                              Kphi, Kc, c, phi, k):
        X = self._stack_features_static(alpha, Fz, u, kappa, sr, n_t,
                                         Kphi, Kc, c, phi, k)
        H = (X - self._X_mean[:, None]) / self._X_scale[:, None]
        Wi = self._w['input_proj.weight']
        bi = self._w['input_proj.bias'].reshape(-1, 1)
        H = np.tanh(Wi @ H + bi)
        for blk in range(self._n_blocks):
            res = H
            W1 = self._w[f'blocks.{blk}.fc1.weight']
            b1 = self._w[f'blocks.{blk}.fc1.bias'].reshape(-1, 1)
            W2 = self._w[f'blocks.{blk}.fc2.weight']
            b2 = self._w[f'blocks.{blk}.fc2.bias'].reshape(-1, 1)
            H1 = np.tanh(W1 @ H + b1)
            H = W2 @ H1 + b2
            H = np.tanh(H + res)
        Wo = self._w['output_proj.weight']
        bo = self._w['output_proj.bias'].reshape(-1, 1)
        Hy = Wo @ H + bo
        Y = Hy * self._y_scale[:, None] + self._y_mean[:, None]
        return Y[0], Y[1]

    def _densenet_static_batch(self, alpha, Fz, u, kappa, sr, n_t,
                                Kphi, Kc, c, phi, k):
        X = self._stack_features_static(alpha, Fz, u, kappa, sr, n_t,
                                         Kphi, Kc, c, phi, k)
        H = (X - self._X_mean[:, None]) / self._X_scale[:, None]
        Wi = self._w['input_proj.weight']
        bi = self._w['input_proj.bias'].reshape(-1, 1)
        H = np.tanh(Wi @ H + bi)
        feats = [H]
        for i in range(self._n_dense_layers):
            cat = np.vstack(feats)
            W = self._w[f'dense_layers.{i}.weight']
            b = self._w[f'dense_layers.{i}.bias'].reshape(-1, 1)
            H = np.tanh(W @ cat + b)
            feats.append(H)
        cat = np.vstack(feats)
        Wo = self._w['output_proj.weight']
        bo = self._w['output_proj.bias'].reshape(-1, 1)
        Hy = Wo @ cat + bo
        Y = Hy * self._y_scale[:, None] + self._y_mean[:, None]
        return Y[0], Y[1]


# ----------------------------------------------------------------------------
# Vehicle dynamics (bicycle model + NN lateral force, batched)
# ----------------------------------------------------------------------------

@dataclass
class SurrogateRolloutParams:
    """Parameters of the batched bicycle-model rollout."""
    M: float
    Lf: float
    Lr: float
    Izz: float
    max_steer: float = 0.528          # rad, HMMWV road-wheel limit
    max_accel: float = 3.0            # m/s² throttle authority
    max_decel: float = -6.0           # m/s² brake authority (negative)
    tau_steer: float = 0.18           # steering first-order lag (s)
    alpha_slip_clip: float = 0.35     # NN training range cutoff (rad, ~20°)
    Fz_f_per_wheel: float = 0.0       # set from vehicle params at construction
    Fz_r_per_wheel: float = 0.0
    g: float = 9.81
    # Sub-stepping: split each outer dt into ``substeps`` internal Euler
    # steps to keep the integration stable at low speeds where the slip
    # / yaw dynamics are stiff.  Pure cost is one extra batched NN call
    # per sub-step — typical 4–5 substeps stays well under 5 ms even
    # for K=512.
    substeps: int = 5
    low_speed_threshold: float = 1.5  # m/s — below this we add kinematic
                                       # yaw blending so atan2(...) noise
                                       # doesn't drive omega.


class VehicleSurrogateDynamics:
    """Batched single-track vehicle dynamics with NN-surrogate lateral forces.

    State (per sample, shape ``(K, 7)``):
        ``[x, y, psi, u, v, omega, delta]``  — world position, heading,
        longitudinal & lateral body-frame velocity, yaw rate, road-wheel angle.

    Control (per sample, shape ``(K, 2)``):
        ``[steer_norm, alpha]`` — both in ``[-1, 1]``.
        ``steer_norm`` is the commanded normalised steering, mapped to the
        target road-wheel angle by ``max_steer``.
        ``alpha`` is the unified longitudinal command: positive = throttle
        fraction (mapped via ``max_accel``), negative = brake fraction
        (mapped via ``|max_decel|``).

    Integration is forward Euler with the lateral force from the NN
    surrogate and a kinematic-bicycle yaw / position update.  Longitudinal
    acceleration is the sum of the throttle/brake command, NN-predicted
    tire drag at zero longitudinal slip (captures sinkage / rolling
    resistance on soft soils), and the centripetal coupling ``v*omega``.
    """

    STATE_DIM = 7
    CONTROL_DIM = 2

    def __init__(self, surrogate: NumpyTireSurrogate,
                 params: SurrogateRolloutParams,
                 terrain_params: dict):
        """
        Args:
            surrogate: NumpyTireSurrogate wrapping a loaded NN tire model.
            params: SurrogateRolloutParams populated for the vehicle.
            terrain_params: terrain dict with keys ``Kphi, Kc, n, c, phi, k``.
                ``phi`` MUST be in radians here (the shield converts upstream).
        """
        self.nn = surrogate
        self.p = params
        self.terrain = dict(terrain_params)
        # Pre-stash the steering smoothing coefficient
        self._smooth_cache = {}

    def update_terrain(self, terrain_params: dict):
        """Hot-swap terrain parameters (phi in radians)."""
        self.terrain = dict(terrain_params)

    def _steer_smooth(self, dt: float) -> float:
        c = self._smooth_cache.get(dt)
        if c is None:
            c = 1.0 - np.exp(-dt / max(self.p.tau_steer, 1e-3))
            self._smooth_cache[dt] = c
        return c

    def step(self, state: np.ndarray, control: np.ndarray, dt: float
             ) -> np.ndarray:
        """Advance ``K`` samples by one outer step, sub-stepping internally.

        The bicycle/NN model is stiff at low longitudinal speed: even
        small slip angles map to large lateral forces, and a 0.1 s
        forward-Euler step lets omega run away (cf. the unit test that
        drove omega to 1.3 rad/s in <1 s on the same control input that
        kinematic bicycle predicts < 0.2 rad/s for).  We split each
        outer step into ``p.substeps`` semi-implicit Euler sub-steps to
        keep the integration well-behaved.  This also matches the way
        the planning NMPC handles low-speed acados solves.
        """
        substeps = max(int(self.p.substeps), 1)
        inner_dt = dt / substeps
        s = state
        for _ in range(substeps):
            s = self._inner_step(s, control, inner_dt)
        return s

    def _inner_step(self, state: np.ndarray, control: np.ndarray,
                     dt: float) -> np.ndarray:
        """One semi-implicit Euler step at the inner sub-stepping resolution."""
        p = self.p
        x, y, psi, u, v, omega, delta = (state[:, i] for i in range(7))
        steer_norm, alpha = control[:, 0], control[:, 1]

        # First-order steering tracking
        delta_tgt = np.clip(steer_norm, -1.0, 1.0) * p.max_steer
        c_sm = self._steer_smooth(dt)
        delta_next = delta + (delta_tgt - delta) * c_sm

        # Slip angles (clamp u to avoid /0 in atan2)
        u_safe = np.maximum(u, 0.5)
        alpha_f = delta_next - np.arctan2(v + p.Lf * omega, u_safe)
        alpha_r = -np.arctan2(v - p.Lr * omega, u_safe)
        np.clip(alpha_f, -p.alpha_slip_clip, p.alpha_slip_clip, out=alpha_f)
        np.clip(alpha_r, -p.alpha_slip_clip, p.alpha_slip_clip, out=alpha_r)

        Fz_f = np.full_like(u_safe, p.Fz_f_per_wheel)
        Fz_r = np.full_like(u_safe, p.Fz_r_per_wheel)

        # Per-wheel tire forces from the NN surrogate.
        # NOTE: kappa is held at 0 (no longitudinal slip in the rollout) —
        # this means Fx_nn captures rolling resistance / sinkage drag, not
        # propulsion.  Propulsion is added explicitly from the throttle
        # command via max_accel, matching how the planning NMPC structures
        # its actuation map.
        #
        # IMPORTANT: the NN was trained on Chrono SCM rig data which uses
        # an Fy sign convention opposite to the body-frame ``v̇`` term in
        # the bicycle EOMs that follow.  The planning NMPC bridges this
        # by negating the NN's Fy output (acados_mpc_solver.py line ~990:
        # ``Fyf = -self.nn_scale * ...``).  We match that here, otherwise
        # the rollout turns the *opposite* direction from the real Chrono
        # vehicle and the shield's predictions are mirror-images — which
        # was a silent bug that caused the MPPI/NMPC shield to slam into
        # rocks while DOB-CBF (single-step QP with its own dFy/ddelta
        # estimate) avoided them.
        Fx_f, Fy_f_nn = self.nn.predict_batch(alpha_f, Fz_f, u_safe,
                                               np.zeros_like(u_safe), self.terrain)
        Fx_r, Fy_r_nn = self.nn.predict_batch(alpha_r, Fz_r, u_safe,
                                               np.zeros_like(u_safe), self.terrain)
        Fy_f = -Fy_f_nn
        Fy_r = -Fy_r_nn

        # Longitudinal: throttle (alpha>=0) -> max_accel*alpha,
        # brake (alpha<0)  -> max_decel*|alpha| = -|max_decel|*|alpha|.
        # NN supplies drag (both axles, both sides — multiply by 2 for left/right).
        ax_drive = np.where(alpha >= 0.0,
                            p.max_accel * alpha,
                            -p.max_decel * alpha)   # alpha<0, max_decel<0
        ax_drag = 2.0 * (Fx_f + Fx_r) / p.M
        cos_d = np.cos(delta_next)
        ay_tire = 2.0 * (Fy_f * cos_d + Fy_r) / p.M
        Mz_tire = 2.0 * (p.Lf * Fy_f * cos_d - p.Lr * Fy_r)

        # Below the low-speed threshold the slip-angle gain is huge and
        # the tire model is essentially in stick mode.  Blend toward the
        # kinematic-bicycle yaw rate so noise in atan2(v, u) doesn't
        # bootstrap a divergent yaw rate over the rollout.
        if p.low_speed_threshold > 0:
            kin_omega = u / (p.Lf + p.Lr) * np.tan(np.clip(delta_next, -0.5, 0.5))
            blend = np.clip(u / p.low_speed_threshold, 0.0, 1.0)
            domega_dyn = Mz_tire / p.Izz
            # Pull omega toward the kinematic value at low speed.
            target_omega = blend * (omega + domega_dyn * dt) + (1.0 - blend) * kin_omega
        else:
            domega_dyn = Mz_tire / p.Izz
            target_omega = omega + domega_dyn * dt

        # Semi-implicit Euler: update velocity-level states first, then
        # advance position-level states with the *new* velocities.
        nu = np.maximum(u + (ax_drive + ax_drag + v * omega) * dt, 0.0)
        nv = v + (ay_tire - u * omega) * dt
        nomega = target_omega
        # Cap omega magnitude — keeps degenerate samples from polluting
        # the importance weighting via a chain of huge centripetal accels.
        np.clip(nomega, -3.0, 3.0, out=nomega)

        npsi = psi + nomega * dt
        nx = x + (nu * np.cos(npsi) - nv * np.sin(npsi)) * dt
        ny = y + (nu * np.sin(npsi) + nv * np.cos(npsi)) * dt

        out = np.empty_like(state)
        out[:, 0] = nx
        out[:, 1] = ny
        out[:, 2] = npsi
        out[:, 3] = nu
        out[:, 4] = nv
        out[:, 5] = nomega
        out[:, 6] = delta_next
        return out

    def rollout(self, initial_state: np.ndarray, controls: np.ndarray,
                dt: float) -> np.ndarray:
        """Roll ``K`` samples forward for ``H`` steps.

        Args:
            initial_state: ``(K, 7)`` — same start state for every sample if
                you broadcast outside; this method does not broadcast.
            controls: ``(K, H, 2)``.
            dt: step size (s).

        Returns:
            trajectory: ``(K, H + 1, 7)`` including the initial state.
        """
        K, H, C = controls.shape
        assert C == self.CONTROL_DIM
        assert initial_state.shape == (K, self.STATE_DIM)
        traj = np.empty((K, H + 1, self.STATE_DIM))
        traj[:, 0] = initial_state
        for h in range(H):
            traj[:, h + 1] = self.step(traj[:, h], controls[:, h], dt)
        return traj


# ----------------------------------------------------------------------------
# Convenience builder
# ----------------------------------------------------------------------------

def build_rollout_params(vehicle_params: dict,
                          max_accel: float = 3.0,
                          max_decel: float = -6.0) -> SurrogateRolloutParams:
    """Populate ``SurrogateRolloutParams`` from a vehicle-params dict."""
    M = float(vehicle_params['M'])
    Lf = float(vehicle_params['Lf'])
    Lr = float(vehicle_params['Lr'])
    L = Lf + Lr
    Izz = float(vehicle_params['Izz'])
    g = 9.81
    Fz_f = M * g * Lr / L / 2.0
    Fz_r = M * g * Lf / L / 2.0
    return SurrogateRolloutParams(
        M=M, Lf=Lf, Lr=Lr, Izz=Izz,
        max_accel=max_accel, max_decel=max_decel,
        Fz_f_per_wheel=Fz_f, Fz_r_per_wheel=Fz_r,
    )
