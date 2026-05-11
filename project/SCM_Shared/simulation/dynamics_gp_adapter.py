#!/usr/bin/env python3
"""Persistent sparse-GP dynamics residual adapter.

Paper-scope note
----------------
This module intentionally keeps only the process-dynamics GP path used to
learn velocity-state residuals ``[Δu̇, Δv̇, Δω̇]``. The older GP force-residual
adapter lives in the archive cleanup snapshot.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


DYN_INPUT_DIM = 11
DYN_OUTPUT_DIM = 3


class SparseGP:
    """Simple sparse GP regression with fixed inducing points and RBF kernel."""

    def __init__(self, ls: float, sig_var: float, noise_var: float,
                 max_inducing: int):
        self.ls = float(ls)
        self.sig_var = float(sig_var)
        self.noise_var = float(noise_var)
        self.max_inducing = int(max_inducing)
        self.X_ind: Optional[np.ndarray] = None
        self.Y_ind: Optional[np.ndarray] = None
        self._L: Optional[np.ndarray] = None
        self._alpha: Optional[np.ndarray] = None
        self._K_inv: Optional[np.ndarray] = None

    @property
    def n_inducing(self) -> int:
        return 0 if self.X_ind is None else self.X_ind.shape[0]

    def _rbf(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        sq1 = np.sum(X1 ** 2, axis=1, keepdims=True)
        sq2 = np.sum(X2 ** 2, axis=1, keepdims=True)
        sq_dist = sq1 + sq2.T - 2.0 * (X1 @ X2.T)
        np.maximum(sq_dist, 0.0, out=sq_dist)
        return self.sig_var * np.exp(-0.5 * sq_dist / (self.ls ** 2))

    def _recompute_posterior(self) -> None:
        if self.X_ind is None or len(self.X_ind) == 0:
            self._L = None
            self._alpha = None
            self._K_inv = None
            return
        M = self.X_ind.shape[0]
        K_mm = self._rbf(self.X_ind, self.X_ind)
        K_mm += self.noise_var * np.eye(M) + 1e-6 * np.eye(M)
        try:
            self._L = np.linalg.cholesky(K_mm)
            tmp = np.linalg.solve(self._L, self.Y_ind)
            self._alpha = np.linalg.solve(self._L.T, tmp)
            L_inv = np.linalg.solve(self._L, np.eye(M))
            self._K_inv = L_inv.T @ L_inv
        except np.linalg.LinAlgError:
            K_inv = np.linalg.pinv(K_mm)
            self._alpha = K_inv @ self.Y_ind
            self._L = None
            self._K_inv = K_inv

    def predict(self, X: np.ndarray, compute_var: bool = False
                ) -> tuple[np.ndarray, np.ndarray]:
        if self._alpha is None or self.X_ind is None:
            return (
                np.zeros((X.shape[0], DYN_OUTPUT_DIM)),
                np.ones(X.shape[0]) * self.sig_var,
            )

        K_star = self._rbf(X, self.X_ind)
        mean = K_star @ self._alpha
        if compute_var and self._K_inv is not None:
            tmp = K_star @ self._K_inv
            var = self.sig_var - np.sum(tmp * K_star, axis=1)
            var = np.maximum(var, 1e-6)
        else:
            var = np.zeros(X.shape[0])
        return mean, var

    def update(self, X_new: np.ndarray, Y_new: np.ndarray) -> int:
        if len(X_new) == 0:
            return self.n_inducing

        if self.X_ind is None:
            self.X_ind = X_new.copy()
            self.Y_ind = Y_new.copy()
        else:
            self.X_ind = np.vstack([self.X_ind, X_new])
            self.Y_ind = np.vstack([self.Y_ind, Y_new])

        if self.X_ind.shape[0] > self.max_inducing:
            self._prune_inducing()

        self._recompute_posterior()
        return self.n_inducing

    def _prune_inducing(self) -> None:
        M = self.X_ind.shape[0]
        K = self.max_inducing
        if M <= K:
            return
        selected = [0]
        min_dist = np.full(M, np.inf)
        for _ in range(K - 1):
            last = selected[-1]
            d = np.sum((self.X_ind - self.X_ind[last]) ** 2, axis=1)
            min_dist = np.minimum(min_dist, d)
            min_dist[selected] = -1.0
            selected.append(int(np.argmax(min_dist)))
        idx = np.array(selected)
        self.X_ind = self.X_ind[idx]
        self.Y_ind = self.Y_ind[idx]

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "ls": self.ls,
            "sig_var": self.sig_var,
            "noise_var": self.noise_var,
            "max_inducing": self.max_inducing,
        }
        if self.X_ind is not None:
            data["X_ind"] = self.X_ind
            data["Y_ind"] = self.Y_ind
        np.savez(str(path), **data)

    def load(self, path: Path) -> bool:
        if not path.exists():
            return False
        try:
            d = np.load(str(path), allow_pickle=False)
            self.ls = float(d["ls"])
            self.sig_var = float(d["sig_var"])
            self.noise_var = float(d["noise_var"])
            self.max_inducing = int(d["max_inducing"])
            if "X_ind" in d:
                self.X_ind = d["X_ind"].astype(np.float64)
                self.Y_ind = d["Y_ind"].astype(np.float64)
                self._recompute_posterior()
            return True
        except Exception as exc:
            print(f"  [DynGP] Warning: failed to load state from {path}: {exc}")
            return False


@dataclass
class DynamicsGPConfig:
    state_path: Path = Path("data/gp_residual/dynamics_gp_state.npz")
    max_inducing: int = 100
    kernel_lengthscale: float = 1.0
    kernel_variance: float = 1.0
    noise_variance: float = 0.1
    clip_dxdot: float = 2.0
    gain: float = 1.0
    bin_resolution: int = 50
    min_bin_count: int = 3
    update_on_shutdown: bool = True
    warmup_samples: int = 64


class DynamicsGPAdapter:
    """Sparse GP dynamics residual adapter for ``[Δu̇, Δv̇, Δω̇]``."""

    def __init__(self, cfg: DynamicsGPConfig):
        self.cfg = cfg
        self._x_sum = np.zeros(DYN_INPUT_DIM, dtype=np.float64)
        self._x_sq_sum = np.zeros(DYN_INPUT_DIM, dtype=np.float64)
        self._y_sum = np.zeros(DYN_OUTPUT_DIM, dtype=np.float64)
        self._y_sq_sum = np.zeros(DYN_OUTPUT_DIM, dtype=np.float64)
        self._n_obs = 0
        self._x_mean = np.zeros(DYN_INPUT_DIM, dtype=np.float64)
        self._x_std = np.ones(DYN_INPUT_DIM, dtype=np.float64)
        self._y_std = np.ones(DYN_OUTPUT_DIM, dtype=np.float64)
        self._stats_frozen = False
        self._obs_feats: list[np.ndarray] = []
        self._obs_targets: list[np.ndarray] = []
        self.gp = SparseGP(
            ls=cfg.kernel_lengthscale,
            sig_var=cfg.kernel_variance,
            noise_var=cfg.noise_variance,
            max_inducing=cfg.max_inducing,
        )
        self._loaded = self._load_state()
        if self._loaded:
            print(
                f"  [DynGP] Loaded persistent state: {self.gp.n_inducing} "
                f"inducing points from {cfg.state_path}"
            )
        else:
            print(f"  [DynGP] No prior state at {cfg.state_path} — starting fresh")

        self.sample_count = 0
        self.update_count = 0

    def _load_state(self) -> bool:
        path = self.cfg.state_path
        if not path.exists():
            return False
        loaded = self.gp.load(path)
        if loaded:
            d = np.load(str(path), allow_pickle=False)
            if "x_mean" in d:
                self._x_mean = d["x_mean"].astype(np.float64)
                self._x_std = d["x_std"].astype(np.float64)
                self._y_std = d["y_std"].astype(np.float64)
                self._n_obs = int(d.get("n_obs", 0))
                self._x_sum = self._x_mean * self._n_obs
                self._x_sq_sum = (
                    self._x_std ** 2 + self._x_mean ** 2
                ) * self._n_obs
                self._y_sq_sum = (self._y_std ** 2) * self._n_obs
                self._stats_frozen = True
        return loaded

    def _save_state(self) -> None:
        path = self.cfg.state_path
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "ls": self.gp.ls,
            "sig_var": self.gp.sig_var,
            "noise_var": self.gp.noise_var,
            "max_inducing": self.gp.max_inducing,
            "x_mean": self._x_mean,
            "x_std": self._x_std,
            "y_std": self._y_std,
            "n_obs": self._n_obs,
        }
        if self.gp.X_ind is not None:
            data["X_ind"] = self.gp.X_ind
            data["Y_ind"] = self.gp.Y_ind
        np.savez(str(path), **data)

    def _normalise_x(self, X: np.ndarray) -> np.ndarray:
        return (X - self._x_mean) / self._x_std

    def _normalise_y(self, Y: np.ndarray) -> np.ndarray:
        return Y / self._y_std

    def _denormalise_y(self, Y_norm: np.ndarray) -> np.ndarray:
        return Y_norm * self._y_std

    def _update_stats(self, X: np.ndarray, Y: np.ndarray) -> None:
        if self._stats_frozen:
            return
        n_new = X.shape[0]
        self._x_sum += X.sum(axis=0)
        self._x_sq_sum += (X ** 2).sum(axis=0)
        self._y_sum += Y.sum(axis=0)
        self._y_sq_sum += (Y ** 2).sum(axis=0)
        self._n_obs += n_new
        if self._n_obs > 1:
            self._x_mean = self._x_sum / self._n_obs
            var_x = self._x_sq_sum / self._n_obs - self._x_mean ** 2
            self._x_std = np.sqrt(np.maximum(var_x, 1e-8))
            self._x_std = np.maximum(self._x_std, 1e-4)
            var_y = self._y_sq_sum / self._n_obs
            self._y_std = np.sqrt(np.maximum(var_y, 1e-8))
            self._y_std = np.maximum(self._y_std, 1e-4)

    def predict_horizon(self, Z_prev: np.ndarray, U_prev: np.ndarray,
                        terrain_vec: np.ndarray, N: int) -> np.ndarray:
        if self.gp.n_inducing == 0 or self._n_obs < self.cfg.warmup_samples:
            return np.zeros((N + 1, DYN_OUTPUT_DIM), dtype=np.float64)
        features = self._build_features(Z_prev, U_prev, terrain_vec, N)
        X_norm = self._normalise_x(features)
        mean_norm, _ = self.gp.predict(X_norm, compute_var=False)
        mean = self._denormalise_y(mean_norm)
        mean *= self.cfg.gain
        return np.clip(mean, -self.cfg.clip_dxdot, self.cfg.clip_dxdot)

    def predict_horizon_with_uncertainty(
        self, Z_prev, U_prev, terrain_vec, N
    ) -> tuple[np.ndarray, np.ndarray]:
        if self.gp.n_inducing == 0 or self._n_obs < self.cfg.warmup_samples:
            return (
                np.zeros((N + 1, DYN_OUTPUT_DIM)),
                np.ones(N + 1) * self.gp.sig_var,
            )
        features = self._build_features(Z_prev, U_prev, terrain_vec, N)
        X_norm = self._normalise_x(features)
        mean_norm, var = self.gp.predict(X_norm, compute_var=True)
        mean = self._denormalise_y(mean_norm)
        conf = np.clip(1.0 - var / max(self.gp.sig_var, 1e-8), 0.0, 1.0)
        mean *= conf[:, None]
        mean *= self.cfg.gain
        return np.clip(mean, -self.cfg.clip_dxdot, self.cfg.clip_dxdot), var

    def observe(self, u: float, v: float, omega: float, ax: float,
                delta: float, terrain_vec: np.ndarray,
                du_err: float, dv_err: float, domega_err: float,
                dt: float, **kwargs) -> None:
        if not all(np.isfinite(vv) for vv in [du_err, dv_err, domega_err, dt]):
            return
        if dt < 1e-4:
            return
        feat = np.array([u, v, omega, ax, delta, *terrain_vec[:6]],
                        dtype=np.float64)
        target = np.array([du_err / dt, dv_err / dt, domega_err / dt],
                          dtype=np.float64)
        self._obs_feats.append(feat)
        self._obs_targets.append(target)
        self.sample_count += 1

    def flush_and_save(self) -> int:
        if len(self._obs_feats) < self.cfg.min_bin_count:
            print(f"  [DynGP] Too few observations ({len(self._obs_feats)}) — skipping")
            return 0

        X_raw = np.array(self._obs_feats, dtype=np.float64)
        Y_raw = np.array(self._obs_targets, dtype=np.float64)
        self._update_stats(X_raw, Y_raw)
        self._stats_frozen = True

        X_binned, Y_binned = self._bin_observations(X_raw, Y_raw)
        n_bins = len(X_binned) if len(X_binned) > 0 else 0
        print(f"  [DynGP] Binned {len(X_raw)} observations → {n_bins} bins")
        if n_bins == 0:
            self._obs_feats.clear()
            self._obs_targets.clear()
            self._save_state()
            return 0

        X_norm = self._normalise_x(X_binned)
        Y_norm = self._normalise_y(Y_binned)
        n_before = self.gp.n_inducing
        self.gp.update(X_norm, Y_norm)
        n_after = self.gp.n_inducing
        self.update_count += 1
        print(
            f"  [DynGP] Updated: {n_before} → {n_after} inducing points "
            f"(+{n_bins} bins, {n_after - n_before:+d} net)"
        )
        self._save_state()
        print(f"  [DynGP] State saved to {self.cfg.state_path}")
        self._obs_feats.clear()
        self._obs_targets.clear()
        return n_bins

    def shutdown(self) -> None:
        if self.cfg.update_on_shutdown and len(self._obs_feats) > 0:
            self.flush_and_save()

    def _build_features(self, Z_prev, U_prev, terrain_vec, N) -> np.ndarray:
        features = np.zeros((N + 1, DYN_INPUT_DIM), dtype=np.float64)
        for k in range(N + 1):
            z_k = Z_prev[:, k]
            delta_k = U_prev[0, min(k, N - 1)] if U_prev is not None else z_k[7]
            features[k] = [z_k[3], z_k[4], z_k[5], z_k[6], delta_k,
                           *terrain_vec[:6]]
        return features

    def _bin_observations(self, X: np.ndarray, Y: np.ndarray
                          ) -> tuple[np.ndarray, np.ndarray]:
        if len(X) == 0:
            return np.empty((0, DYN_INPUT_DIM)), np.empty((0, DYN_OUTPUT_DIM))
        key_dims = X[:, :3]
        mins = key_dims.min(axis=0)
        maxs = key_dims.max(axis=0)
        ranges = np.maximum(maxs - mins, 1e-6)
        n_bins = self.cfg.bin_resolution
        bin_idx = np.floor((key_dims - mins) / ranges * (n_bins - 1)).astype(int)
        bin_idx = np.clip(bin_idx, 0, n_bins - 1)
        keys = bin_idx[:, 0] * n_bins * n_bins + bin_idx[:, 1] * n_bins + bin_idx[:, 2]
        unique_keys = np.unique(keys)
        X_out, Y_out = [], []
        for key in unique_keys:
            mask = keys == key
            if mask.sum() >= self.cfg.min_bin_count:
                X_out.append(X[mask].mean(axis=0))
                Y_out.append(Y[mask].mean(axis=0))
        if len(X_out) == 0:
            return np.empty((0, DYN_INPUT_DIM)), np.empty((0, DYN_OUTPUT_DIM))
        return np.array(X_out), np.array(Y_out)
