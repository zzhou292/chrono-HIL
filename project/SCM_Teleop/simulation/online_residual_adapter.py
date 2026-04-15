#!/usr/bin/env python3
"""
Online residual adapter for l4acados-style model mismatch correction.

This module keeps a lightweight residual model:
  y = [du, dv, domega]
from features:
  x = [z_k(9), u_k(2)] where z_k is MPC state and u_k = [delta, Jx].

It supports:
  - loading an offline checkpoint,
  - online fine-tuning from streaming transitions,
  - prediction of state-bias corrections for controller-side compensation.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np
import torch
import torch.nn as nn

# Mixed ACADOS/CasADi + PyTorch in one process can be unstable with threaded BLAS/OpenMP.
# Pin Torch to single-threaded execution for robust real-time controller operation.
try:
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
except Exception:
    pass


class ResidualMLP(nn.Module):
    def __init__(self, input_dim: int = 11, output_dim: int = 3, hidden_sizes=(64, 64)):
        super().__init__()
        layers = []
        prev = int(input_dim)
        for h in hidden_sizes:
            layers.append(nn.Linear(prev, int(h)))
            layers.append(nn.Tanh())
            prev = int(h)
        layers.append(nn.Linear(prev, int(output_dim)))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class ScaledResidualMLP(nn.Module):
    """Inference/training module with internal input/output normalization."""

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_sizes,
        x_mean: np.ndarray,
        x_std: np.ndarray,
        y_mean: np.ndarray,
        y_std: np.ndarray,
    ):
        super().__init__()
        self.core = ResidualMLP(input_dim=input_dim, output_dim=output_dim, hidden_sizes=hidden_sizes)
        self.register_buffer("x_mean", torch.tensor(x_mean, dtype=torch.float32))
        self.register_buffer("x_std", torch.tensor(x_std, dtype=torch.float32))
        self.register_buffer("y_mean", torch.tensor(y_mean, dtype=torch.float32))
        self.register_buffer("y_std", torch.tensor(y_std, dtype=torch.float32))

    def forward(self, y):
        z = (y - self.x_mean) / self.x_std
        w_norm = self.core(z)
        return w_norm * self.y_std + self.y_mean


@dataclass
class ResidualAdapterConfig:
    checkpoint: Path
    correction_gain: float = 1.0
    clip_u: float = 0.25
    clip_v: float = 0.25
    clip_omega: float = 0.08
    online_enabled: bool = True
    online_lr: float = 2e-4
    online_epochs: int = 4
    online_batch_size: int = 256
    online_update_interval: int = 5
    online_buffer_size: int = 4096
    online_warmup_samples: int = 128


class OnlineResidualAdapter:
    """Online residual model + updater for controller-side state compensation."""

    def __init__(self, mpc, cfg: ResidualAdapterConfig):
        self.mpc = mpc
        self.cfg = cfg
        payload = self._load_checkpoint_payload(cfg.checkpoint)
        self._x_mean = np.asarray(payload["x_mean"], dtype=float).reshape(1, -1)
        self._x_std = np.asarray(payload["x_std"], dtype=float).reshape(1, -1)
        self._y_mean = np.asarray(payload["y_mean"], dtype=float).reshape(1, -1)
        self._y_std = np.asarray(payload["y_std"], dtype=float).reshape(1, -1)
        self._layers = self._extract_layers(payload["state_dict"])
        self._online_bias = np.zeros(3, dtype=float)

        self.online_enabled = bool(cfg.online_enabled)
        self.buffer = deque(maxlen=int(cfg.online_buffer_size))
        self.sample_count = 0
        self.update_count = 0
        self.last_update_loss = float("nan")
        self.last_update_time_s = 0.0
        self._step_counter = 0

        # f_expl for one-step nominal residual labels
        ac_model = mpc._build_acados_model()
        self._f_expl_fun = ca.Function(
            "f_expl_online_residual_adapter",
            [ac_model.x, ac_model.u, ac_model.p],
            [ac_model.f_expl_expr],
        )

    @staticmethod
    def _load_checkpoint_payload(ckpt_path: Path) -> dict:
        if not ckpt_path.exists():
            raise FileNotFoundError(f"Residual checkpoint not found: {ckpt_path}")
        # Torch >=2.6 defaults to weights_only=True; this checkpoint stores numpy stats.
        try:
            payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
        except TypeError:
            payload = torch.load(ckpt_path, map_location="cpu")
        return payload

    @staticmethod
    def _tensor_to_numpy(value) -> np.ndarray:
        if isinstance(value, torch.Tensor):
            return value.detach().cpu().numpy()
        return np.asarray(value)

    @classmethod
    def _extract_layers(cls, state_dict: dict) -> list[tuple[np.ndarray, np.ndarray]]:
        # Accept both:
        #   core.net.{idx}.weight / core.net.{idx}.bias
        #   net.{idx}.weight / net.{idx}.bias
        if any(k.startswith("core.net.") for k in state_dict.keys()):
            prefix = "core.net"
            idx_pos = 2
        elif any(k.startswith("net.") for k in state_dict.keys()):
            prefix = "net"
            idx_pos = 1
        else:
            raise KeyError("Residual checkpoint missing net.*.weight or core.net.*.weight keys")

        layer_indices = sorted(
            {
                int(key.split(".")[idx_pos])
                for key in state_dict.keys()
                if key.startswith(f"{prefix}.") and key.endswith(".weight")
            }
        )
        if not layer_indices:
            raise KeyError("Residual checkpoint contains no MLP weight layers")

        layers = []
        for idx in layer_indices:
            w_key = f"{prefix}.{idx}.weight"
            b_key = f"{prefix}.{idx}.bias"
            if w_key not in state_dict or b_key not in state_dict:
                raise KeyError(f"Residual checkpoint missing {w_key} or {b_key}")
            W = cls._tensor_to_numpy(state_dict[w_key]).astype(float, copy=True)
            b = cls._tensor_to_numpy(state_dict[b_key]).astype(float, copy=True)
            layers.append((W, b))
        return layers

    def _predict_base_batch(self, X: np.ndarray) -> np.ndarray:
        A = np.asarray(X, dtype=float)
        if A.ndim == 1:
            A = A.reshape(1, -1)
        A = (A - self._x_mean) / self._x_std
        for li, (W, b) in enumerate(self._layers):
            Z = A @ W.T + b.reshape(1, -1)
            if li < len(self._layers) - 1:
                A = np.tanh(Z)
            else:
                A = Z
        return A * self._y_std + self._y_mean

    def _make_stage_params(self, terrain_params: dict, n_terrain: float, sr_meas: float) -> np.ndarray:
        p = np.zeros(self.mpc._np_per_stage, dtype=float)
        nn = self.mpc.nn_tire_model
        model_fmt = nn.model_format if nn is not None else "v6"
        if model_fmt in ("v6", "v6_temporal", "v8_rate"):
            phi_val = np.radians(float(terrain_params["phi"]))
        else:
            phi_val = float(terrain_params["phi"])

        p[self.mpc._tp_off + 0] = float(terrain_params["Kphi"])
        p[self.mpc._tp_off + 1] = float(terrain_params["Kc"])
        p[self.mpc._tp_off + 2] = float(n_terrain)
        p[self.mpc._tp_off + 3] = float(terrain_params["c"])
        p[self.mpc._tp_off + 4] = float(phi_val)
        p[self.mpc._tp_off + 5] = float(terrain_params["k"])

        p[self.mpc._sr_off] = float(sr_meas)
        p[self.mpc._kappa_ref_front_off] = 0.15
        p[self.mpc._kappa_ref_rear_off] = 0.15
        return p

    def predict_residual(self, z_state: np.ndarray, u_ctrl: np.ndarray) -> np.ndarray:
        feat = np.concatenate([z_state, u_ctrl], axis=0).astype(float)
        out = self._predict_base_batch(feat).reshape(-1)
        if self.online_enabled:
            out = out + self._online_bias
        return out.astype(float)

    def corrected_state(self, z_state: np.ndarray, u_ctrl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pred = self.predict_residual(z_state, u_ctrl)
        pred = np.array(
            [
                np.clip(pred[0], -self.cfg.clip_u, self.cfg.clip_u),
                np.clip(pred[1], -self.cfg.clip_v, self.cfg.clip_v),
                np.clip(pred[2], -self.cfg.clip_omega, self.cfg.clip_omega),
            ],
            dtype=float,
        )
        z_corr = np.array(z_state, dtype=float, copy=True)
        z_corr[3] = np.clip(
            z_corr[3] + self.cfg.correction_gain * pred[0],
            self.mpc.u_min,
            self.mpc.u_max,
        )
        z_corr[4] = np.clip(z_corr[4] + self.cfg.correction_gain * pred[1], -10.0, 10.0)
        z_corr[5] = np.clip(z_corr[5] + self.cfg.correction_gain * pred[2], -5.0, 5.0)
        return z_corr, pred

    def observe_transition(
        self,
        z_prev: np.ndarray,
        u_prev: np.ndarray,
        z_curr: np.ndarray,
        dt: float,
        terrain_params: dict,
        n_terrain: float,
        sr_prev: float,
    ) -> None:
        if not np.isfinite(dt) or dt <= 1e-5:
            return
        dt_eff = float(np.clip(dt, 1e-3, 0.2))

        p_stage = self._make_stage_params(terrain_params, n_terrain=n_terrain, sr_meas=sr_prev)
        dx_nom = np.array(self._f_expl_fun(z_prev, u_prev, p_stage)).reshape(-1)
        z_nom_next = z_prev + dt_eff * dx_nom
        resid = z_curr[[3, 4, 5]] - z_nom_next[[3, 4, 5]]
        if (not np.isfinite(resid).all()) or (not np.isfinite(z_prev).all()) or (not np.isfinite(u_prev).all()):
            return

        feat = np.concatenate([z_prev, u_prev], axis=0).astype(np.float32)
        self.buffer.append((feat, resid.astype(np.float32)))
        self.sample_count += 1
        self._step_counter += 1

        if self.online_enabled:
            self._maybe_update()

    def _maybe_update(self) -> None:
        if len(self.buffer) < int(self.cfg.online_warmup_samples):
            return
        if (self._step_counter % int(self.cfg.online_update_interval)) != 0:
            return

        t0 = time.time()
        batch_size = min(int(self.cfg.online_batch_size), len(self.buffer))
        idx = np.random.choice(len(self.buffer), size=batch_size, replace=False)
        X = np.stack([self.buffer[i][0] for i in idx], axis=0).astype(float)
        Y = np.stack([self.buffer[i][1] for i in idx], axis=0).astype(float)

        lr = float(self.cfg.online_lr)
        for _ in range(max(1, int(self.cfg.online_epochs))):
            pred = self._predict_base_batch(X) + self._online_bias.reshape(1, -1)
            err = Y - pred
            self._online_bias = self._online_bias + lr * np.mean(err, axis=0)
            self._online_bias = np.clip(
                self._online_bias,
                np.array([-self.cfg.clip_u, -self.cfg.clip_v, -self.cfg.clip_omega], dtype=float),
                np.array([self.cfg.clip_u, self.cfg.clip_v, self.cfg.clip_omega], dtype=float),
            )

        final_pred = self._predict_base_batch(X) + self._online_bias.reshape(1, -1)
        self.last_update_loss = float(np.mean((Y - final_pred) ** 2))
        self.last_update_time_s = float(time.time() - t0)
        self.update_count += 1

    def summary(self) -> dict:
        return {
            "checkpoint": str(self.cfg.checkpoint),
            "online_enabled": bool(self.online_enabled),
            "sample_count": int(self.sample_count),
            "update_count": int(self.update_count),
            "last_update_loss": float(self.last_update_loss),
            "last_update_time_ms": float(1000.0 * self.last_update_time_s),
            "buffer_size": int(len(self.buffer)),
            "online_bias_du": float(self._online_bias[0]),
            "online_bias_dv": float(self._online_bias[1]),
            "online_bias_domega": float(self._online_bias[2]),
        }
