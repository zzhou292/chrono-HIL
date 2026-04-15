#!/usr/bin/env python3
"""
Force-level residual adapter for MPC tire force correction.

Predicts ΔFy_front, ΔFy_rear corrections from tire operating conditions
and terrain parameters. Unlike the velocity-level OnlineResidualAdapter,
this corrects forces at every stage of the MPC prediction horizon — where
the 70%+ tire model mismatch actually lies.

Features:  [α_f, α_r, u, v, omega, ay, Fz_f, Fz_r, Kphi, Kc, n, c, phi, k] (14)
Targets:   [ΔFy_f, ΔFy_r]  where ΔFy = actual_Fy − predicted_Fy
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import torch.nn as nn

try:
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
except Exception:
    pass


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

INPUT_DIM = 11   # [α_f, α_r, u, Fz_f, Fz_r, Kphi, Kc, n, c, phi, k]
OUTPUT_DIM = 2   # [ΔFy_f, ΔFy_r]


class ForceResidualMLP(nn.Module):
    def __init__(self, input_dim=INPUT_DIM, output_dim=OUTPUT_DIM, hidden_sizes=(64, 64)):
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


class ScaledForceResidualMLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_sizes,
                 x_mean, x_std, y_mean, y_std):
        super().__init__()
        self.core = ForceResidualMLP(input_dim, output_dim, hidden_sizes)
        self.register_buffer("x_mean", torch.tensor(x_mean, dtype=torch.float32))
        self.register_buffer("x_std", torch.tensor(x_std, dtype=torch.float32))
        self.register_buffer("y_mean", torch.tensor(y_mean, dtype=torch.float32))
        self.register_buffer("y_std", torch.tensor(y_std, dtype=torch.float32))

    def forward(self, x):
        z = (x - self.x_mean) / self.x_std
        return self.core(z) * self.y_std + self.y_mean


# ---------------------------------------------------------------------------
# Runtime adapter — pure NumPy inference + online EMA bias
# ---------------------------------------------------------------------------

@dataclass
class ForceResidualConfig:
    checkpoint: Path
    clip_dFy: float = 500.0            # symmetric clip on each ΔFy channel (N)
    gain: float = 1.0                  # conservative output scaling (<1 = safer)
    online_enabled: bool = True
    online_lr: float = 5e-4
    online_epochs: int = 4
    online_batch_size: int = 256
    online_update_interval: int = 5
    online_buffer_size: int = 4096
    online_warmup_samples: int = 64


class ForceResidualAdapter:
    """Predict per-stage ΔFy corrections for the MPC horizon."""

    def __init__(self, cfg: ForceResidualConfig):
        self.cfg = cfg
        payload = self._load_checkpoint(cfg.checkpoint)
        self._input_dim = int(payload.get("input_dim", INPUT_DIM))
        self._x_mean = np.asarray(payload["x_mean"], dtype=np.float64).reshape(1, -1)
        self._x_std = np.asarray(payload["x_std"], dtype=np.float64).reshape(1, -1)
        self._y_mean = np.asarray(payload["y_mean"], dtype=np.float64).reshape(1, -1)
        self._y_std = np.asarray(payload["y_std"], dtype=np.float64).reshape(1, -1)
        self._layers = self._extract_layers(payload["state_dict"])
        self._online_bias = np.zeros(OUTPUT_DIM, dtype=np.float64)

        self.online_enabled = bool(cfg.online_enabled)
        self.buffer: deque = deque(maxlen=int(cfg.online_buffer_size))
        self.sample_count = 0
        self.update_count = 0
        self.last_update_loss = float("nan")
        self.last_update_time_s = 0.0
        self._step_counter = 0

    # ---- checkpoint loading ----

    @staticmethod
    def _load_checkpoint(ckpt_path: Path) -> dict:
        if not ckpt_path.exists():
            raise FileNotFoundError(f"Force residual checkpoint not found: {ckpt_path}")
        try:
            return torch.load(ckpt_path, map_location="cpu", weights_only=False)
        except TypeError:
            return torch.load(ckpt_path, map_location="cpu")

    @staticmethod
    def _extract_layers(state_dict: dict) -> list:
        prefix = "core.net" if any(k.startswith("core.net.") for k in state_dict) else "net"
        idx_pos = 2 if prefix == "core.net" else 1
        layer_indices = sorted({
            int(k.split(".")[idx_pos]) for k in state_dict
            if k.startswith(f"{prefix}.") and k.endswith(".weight")
        })
        layers = []
        for idx in layer_indices:
            W = state_dict[f"{prefix}.{idx}.weight"].numpy().astype(np.float64)
            b = state_dict[f"{prefix}.{idx}.bias"].numpy().astype(np.float64)
            layers.append((W, b))
        return layers

    # ---- NumPy forward pass ----

    def _predict_base(self, X: np.ndarray) -> np.ndarray:
        """Normalized forward pass. X: (B, input_dim) → (B, output_dim)."""
        A = np.atleast_2d(X).astype(np.float64)
        A = (A - self._x_mean) / self._x_std
        for i, (W, b) in enumerate(self._layers):
            Z = A @ W.T + b.reshape(1, -1)
            A = np.tanh(Z) if i < len(self._layers) - 1 else Z
        return A * self._y_std + self._y_mean

    # ---- public API ----

    def predict_single(self, alpha_f: float, alpha_r: float, u: float,
                       Fz_f: float, Fz_r: float,
                       terrain_vec: np.ndarray,
                       v: float = 0.0, omega: float = 0.0) -> np.ndarray:
        """Return [ΔFy_f, ΔFy_r] for a single operating point."""
        feat = np.array([alpha_f, alpha_r, u, Fz_f, Fz_r,
                         *terrain_vec[:6]], dtype=np.float64).reshape(1, -1)
        pred = self._predict_base(feat).reshape(-1) + self._online_bias
        pred *= self.cfg.gain
        return np.clip(pred, -self.cfg.clip_dFy, self.cfg.clip_dFy)

    def predict_horizon(self, Z_prev: np.ndarray, U_prev: np.ndarray,
                        terrain_vec: np.ndarray,
                        Lf: float, Lr: float, M: float,
                        N: int) -> np.ndarray:
        """Compute per-stage [ΔFy_f, ΔFy_r] for stages 0..N.

        Uses previous solve's predicted trajectory to compute α, Fz at
        each stage.  Fz values are per-WHEEL (matching training data from
        diagnostic CSVs where Fz_f_mean / Fz_r_mean are per-wheel).

        Args:
            Z_prev: (nx, N+1) predicted state trajectory
            U_prev: (nu, N) predicted controls
            terrain_vec: [Kphi, Kc, n, c, phi_rad, k] (6,)
            Lf, Lr, M: vehicle geometry
            N: horizon length

        Returns:
            force_residuals: (N+1, 2) array of [ΔFy_f, ΔFy_r]
        """
        g = 9.81
        L = Lf + Lr
        # Per-WHEEL static normal force (training data convention)
        Fz_static_f_pw = M * g * Lr / L / 2.0
        Fz_static_r_pw = M * g * Lf / L / 2.0

        features = np.zeros((N + 1, INPUT_DIM), dtype=np.float64)
        for k in range(N + 1):
            z_k = Z_prev[:, k]
            u_k = z_k[3]  # longitudinal speed
            v_k = z_k[4]  # lateral speed
            omega_k = z_k[5]
            ax_k = z_k[6]

            u_safe = max(abs(u_k), 0.5)
            delta_k = U_prev[0, min(k, N - 1)] if U_prev is not None else z_k[7]

            # Slip angles (bicycle model)
            alpha_f = float(delta_k - np.arctan2(v_k + Lf * omega_k, u_safe))
            alpha_r = float(-np.arctan2(v_k - Lr * omega_k, u_safe))
            # Clamp slip angles to training-data range
            _alpha_max = 0.55
            alpha_f = float(max(-_alpha_max, min(_alpha_max, alpha_f)))
            alpha_r = float(max(-_alpha_max, min(_alpha_max, alpha_r)))

            # Per-wheel normal forces with longitudinal load transfer
            h_cg = 0.5  # approx CG height (m)
            dFz_pw = M * ax_k * h_cg / L / 2.0  # per-wheel load transfer
            Fz_f_pw = max(Fz_static_f_pw - dFz_pw, 250.0)
            Fz_r_pw = max(Fz_static_r_pw + dFz_pw, 250.0)

            features[k] = [alpha_f, alpha_r, u_safe,
                           Fz_f_pw, Fz_r_pw, *terrain_vec[:6]]

        preds = self._predict_base(features) + self._online_bias.reshape(1, -1)
        preds *= self.cfg.gain
        return np.clip(preds, -self.cfg.clip_dFy, self.cfg.clip_dFy)

    # ---- online learning ----

    def observe(self, alpha_f: float, alpha_r: float, u: float,
                Fz_f: float, Fz_r: float, terrain_vec: np.ndarray,
                actual_Fy_f: float, actual_Fy_r: float,
                pred_Fy_f: float, pred_Fy_r: float,
                v: float = 0.0, omega: float = 0.0) -> None:
        """Record one closed-loop force observation for online adaptation."""
        if not (np.isfinite(actual_Fy_f) and np.isfinite(actual_Fy_r)
                and np.isfinite(pred_Fy_f) and np.isfinite(pred_Fy_r)):
            return
        feat = np.array([alpha_f, alpha_r, u, Fz_f, Fz_r,
                         *terrain_vec[:6]], dtype=np.float32)
        target = np.array([actual_Fy_f - pred_Fy_f,
                           actual_Fy_r - pred_Fy_r], dtype=np.float32)
        self.buffer.append((feat, target))
        self.sample_count += 1
        self._step_counter += 1
        if self.online_enabled:
            self._maybe_update()

    def _maybe_update(self) -> None:
        if len(self.buffer) < self.cfg.online_warmup_samples:
            return
        if (self._step_counter % self.cfg.online_update_interval) != 0:
            return
        t0 = time.time()
        bs = min(self.cfg.online_batch_size, len(self.buffer))
        idx = np.random.choice(len(self.buffer), size=bs, replace=False)
        X = np.stack([self.buffer[i][0] for i in idx]).astype(np.float64)
        Y = np.stack([self.buffer[i][1] for i in idx]).astype(np.float64)
        lr = self.cfg.online_lr
        for _ in range(max(1, self.cfg.online_epochs)):
            pred = self._predict_base(X) + self._online_bias.reshape(1, -1)
            err = Y - pred
            self._online_bias += lr * np.mean(err, axis=0)
            self._online_bias = np.clip(self._online_bias,
                                        -self.cfg.clip_dFy, self.cfg.clip_dFy)
        final = self._predict_base(X) + self._online_bias.reshape(1, -1)
        self.last_update_loss = float(np.mean((Y - final) ** 2))
        self.last_update_time_s = time.time() - t0
        self.update_count += 1

    def summary(self) -> dict:
        return {
            "checkpoint": str(self.cfg.checkpoint),
            "online_enabled": self.online_enabled,
            "sample_count": self.sample_count,
            "update_count": self.update_count,
            "last_update_loss": float(self.last_update_loss),
            "last_update_time_ms": 1000.0 * self.last_update_time_s,
            "buffer_size": len(self.buffer),
            "online_bias_dFy_f": float(self._online_bias[0]),
            "online_bias_dFy_r": float(self._online_bias[1]),
        }


# ---------------------------------------------------------------------------
# Training from diagnostic CSVs
# ---------------------------------------------------------------------------

def train_force_residual(
    diag_csv_paths: list[str],
    output_path: str,
    terrain_presets: dict,
    hidden_sizes: tuple = (64, 64),
    epochs: int = 200,
    batch_size: int = 512,
    lr: float = 1e-3,
    weight_decay: float = 0.0,
    val_fraction: float = 0.15,
    min_time: float = 2.0,
    smooth_window: int = 1,
    loss_type: str = "mse",
    output_l1_penalty: float = 0.0,
    scenario_split: bool = False,
) -> dict:
    """Train a force residual model from diagnostic CSVs.

    Each CSV must contain: alpha_f, alpha_r, u_meas, Fz_f_mean, Fz_r_mean,
    actual_Fy_front, pred_Fy_front, actual_Fy_rear, pred_Fy_rear,
    terrain_class_est.
    """
    import pandas as pd

    # Terrain class → 6-vector mapping
    def _terrain_vec(cls_name: str, phi_radians: bool = True) -> np.ndarray:
        p = terrain_presets[cls_name]
        # Convert preset keys to internal
        phi = np.radians(p["friction_angle"]) if phi_radians else p["friction_angle"]
        return np.array([p["Kphi"], p["Kc"], p["n"],
                         p["cohesion"], phi, p["janosi_shear"]], dtype=np.float32)

    all_X, all_Y = [], []
    for csv_path in diag_csv_paths:
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        required = ["alpha_f", "alpha_r", "u_meas", "v_meas", "omega_meas",
                     "Fz_f_mean", "Fz_r_mean",
                     "actual_Fy_front", "pred_Fy_front", "actual_Fy_rear",
                     "pred_Fy_rear", "terrain_class_est", "sim_time"]
        if not all(c in df.columns for c in required):
            continue
        mask = df["sim_time"] >= min_time
        d = df[mask]
        if len(d) < 10:
            continue
        terrain_cls = d["terrain_class_est"].iloc[0]
        if terrain_cls not in terrain_presets:
            continue
        tvec = _terrain_vec(terrain_cls)
        n = len(d)
        u_vals = d["u_meas"].values
        X = np.column_stack([
            d["alpha_f"].values,
            d["alpha_r"].values,
            u_vals,
            d["Fz_f_mean"].values,
            d["Fz_r_mean"].values,
            np.tile(tvec, (n, 1)),
        ]).astype(np.float32)
        Y = np.column_stack([
            d["actual_Fy_front"].values - d["pred_Fy_front"].values,
            d["actual_Fy_rear"].values - d["pred_Fy_rear"].values,
        ]).astype(np.float32)
        # Smooth targets with a causal rolling window to reduce
        # per-timestep noise from transients and soil dynamics
        if smooth_window > 1 and len(Y) >= smooth_window:
            kernel = np.ones(smooth_window, dtype=np.float32) / smooth_window
            for ch in range(Y.shape[1]):
                Y[:, ch] = np.convolve(Y[:, ch], kernel, mode='same')
        valid = np.isfinite(X).all(axis=1) & np.isfinite(Y).all(axis=1)
        all_X.append(X[valid])
        all_Y.append(Y[valid])

    X_all = np.concatenate(all_X)
    Y_all = np.concatenate(all_Y)
    print(f"Training data: {len(X_all)} samples from {len(all_X)} CSVs")
    print(f"  ΔFy_f: mean={Y_all[:, 0].mean():.1f} N, std={Y_all[:, 0].std():.1f} N")
    print(f"  ΔFy_r: mean={Y_all[:, 1].mean():.1f} N, std={Y_all[:, 1].std():.1f} N")

    # Train/val split
    n = len(X_all)
    if scenario_split and len(all_X) > 3:
        # Hold out entire CSVs so val tests cross-scenario generalization
        n_csv = len(all_X)
        csv_perm = np.random.permutation(n_csv)
        n_val_csv = max(1, int(n_csv * val_fraction))
        val_csvs = set(csv_perm[:n_val_csv].tolist())
        # Build cumulative indices
        cum = np.cumsum([len(x) for x in all_X])
        starts = np.concatenate([[0], cum[:-1]])
        train_idx = np.concatenate([np.arange(starts[i], cum[i])
                                    for i in range(n_csv) if i not in val_csvs])
        val_idx = np.concatenate([np.arange(starts[i], cum[i])
                                  for i in range(n_csv) if i in val_csvs])
        print(f"  Scenario-aware split: {n_csv - n_val_csv} train CSVs, {n_val_csv} val CSVs")
    else:
        idx = np.random.permutation(n)
        n_val = max(1, int(n * val_fraction))
        val_idx, train_idx = idx[:n_val], idx[n_val:]

    x_mean = X_all[train_idx].mean(axis=0)
    x_std = np.clip(X_all[train_idx].std(axis=0), 1e-6, None)
    y_mean = Y_all[train_idx].mean(axis=0)
    y_std = np.clip(Y_all[train_idx].std(axis=0), 1e-6, None)

    model = ScaledForceResidualMLP(
        INPUT_DIM, OUTPUT_DIM, hidden_sizes,
        x_mean, x_std, y_mean, y_std,
    )
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=20, factor=0.5, min_lr=1e-6)

    X_train_t = torch.tensor(X_all[train_idx], dtype=torch.float32)
    Y_train_t = torch.tensor(Y_all[train_idx], dtype=torch.float32)
    X_val_t = torch.tensor(X_all[val_idx], dtype=torch.float32)
    Y_val_t = torch.tensor(Y_all[val_idx], dtype=torch.float32)

    # Loss function
    if loss_type == "huber":
        loss_fn = nn.SmoothL1Loss()
        print("  Using Huber (SmoothL1) loss")
    else:
        loss_fn = nn.MSELoss()

    if output_l1_penalty > 0:
        print(f"  Output L1 penalty: {output_l1_penalty}")

    best_val_loss = float("inf")
    best_state = None
    patience_counter = 0

    for epoch in range(epochs):
        model.train()
        perm = torch.randperm(len(X_train_t))
        epoch_loss = 0.0
        n_batches = 0
        for i in range(0, len(X_train_t), batch_size):
            batch_idx = perm[i:i + batch_size]
            xb, yb = X_train_t[batch_idx], Y_train_t[batch_idx]
            pred = model(xb)
            loss = loss_fn(pred, yb)
            if output_l1_penalty > 0:
                loss = loss + output_l1_penalty * pred.abs().mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            n_batches += 1

        model.eval()
        with torch.no_grad():
            val_pred = model(X_val_t)
            val_loss = loss_fn(val_pred, Y_val_t).item()
        scheduler.step(val_loss)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
            patience_counter = 0
        else:
            patience_counter += 1

        if (epoch + 1) % 20 == 0 or epoch == 0:
            print(f"  Epoch {epoch+1:3d}: train_loss={epoch_loss/n_batches:.2f}, "
                  f"val_loss={val_loss:.2f}, best={best_val_loss:.2f}")

        if patience_counter > 60:
            print(f"  Early stop at epoch {epoch+1}")
            break

    model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        val_pred = model(X_val_t).numpy()
    val_rmse_f = float(np.sqrt(np.mean((val_pred[:, 0] - Y_val_t.numpy()[:, 0]) ** 2)))
    val_rmse_r = float(np.sqrt(np.mean((val_pred[:, 1] - Y_val_t.numpy()[:, 1]) ** 2)))

    print(f"\nBest val loss: {best_val_loss:.2f}")
    print(f"  Val RMSE ΔFy_f: {val_rmse_f:.1f} N")
    print(f"  Val RMSE ΔFy_r: {val_rmse_r:.1f} N")

    # Relative improvement (how much of the force error we capture)
    baseline_rmse_f = float(np.sqrt(np.mean(Y_val_t.numpy()[:, 0] ** 2)))
    baseline_rmse_r = float(np.sqrt(np.mean(Y_val_t.numpy()[:, 1] ** 2)))
    print(f"  Baseline RMS ΔFy_f (no correction): {baseline_rmse_f:.1f} N")
    print(f"  Baseline RMS ΔFy_r (no correction): {baseline_rmse_r:.1f} N")
    print(f"  Residual captures {100*(1 - val_rmse_f/baseline_rmse_f):.1f}% of front error")
    print(f"  Residual captures {100*(1 - val_rmse_r/baseline_rmse_r):.1f}% of rear error")

    ckpt = {
        "state_dict": best_state,
        "x_mean": x_mean, "x_std": x_std,
        "y_mean": y_mean, "y_std": y_std,
        "hidden_sizes": list(hidden_sizes),
        "input_dim": INPUT_DIM, "output_dim": OUTPUT_DIM,
        "val_rmse_dFy_f": val_rmse_f,
        "val_rmse_dFy_r": val_rmse_r,
        "n_train": len(train_idx),
        "n_val": len(val_idx),
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    torch.save(ckpt, output_path)
    print(f"\nSaved to {output_path}")

    return {
        "val_rmse_dFy_f": val_rmse_f,
        "val_rmse_dFy_r": val_rmse_r,
        "baseline_rmse_f": baseline_rmse_f,
        "baseline_rmse_r": baseline_rmse_r,
        "n_train": len(train_idx),
        "n_val": len(val_idx),
        "n_csvs": len(all_X),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import glob
    import sys

    sys.path.insert(0, str(Path(__file__).parent))
    from param_consistency import TERRAIN_PRESETS

    parser = argparse.ArgumentParser(description="Train force-level residual model")
    parser.add_argument("--plots-dir", default="plots",
                        help="Directory containing diagnostic CSV subdirs")
    parser.add_argument("--output", default="logs/force_residual/force_residual_model.pt")
    parser.add_argument("--hidden", default="64,64",
                        help="Hidden layer sizes (comma-separated)")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=0.0)
    parser.add_argument("--smooth-window", type=int, default=1,
                        help="Smooth target ΔFy with causal rolling window (1=no smoothing)")
    parser.add_argument("--loss", choices=["mse", "huber"], default="mse",
                        help="Loss function (mse or huber)")
    parser.add_argument("--output-l1", type=float, default=0.0,
                        help="L1 penalty on model outputs (encourages small predictions)")
    parser.add_argument("--scenario-split", action="store_true",
                        help="Hold out entire CSVs for validation (tests generalization)")
    args = parser.parse_args()

    hidden = tuple(int(x) for x in args.hidden.split(","))
    csv_paths = sorted(glob.glob(f"{args.plots_dir}/*/diag_*.csv"))
    print(f"Found {len(csv_paths)} diagnostic CSVs")

    train_force_residual(
        csv_paths, args.output, TERRAIN_PRESETS,
        hidden_sizes=hidden, epochs=args.epochs,
        batch_size=args.batch_size, lr=args.lr,
        weight_decay=args.weight_decay,
        smooth_window=args.smooth_window,
        loss_type=args.loss,
        output_l1_penalty=args.output_l1,
        scenario_split=args.scenario_split,
    )
