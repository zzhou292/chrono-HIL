#!/usr/bin/env python3
"""
l4acados residual-learning scaffold for SCM_Teleop.

This script does two things:
1) Train a compact residual model from controller diagnostic CSV logs.
2) Instantiate an l4acados ResidualLearningMPC on top of the current
   AcadosMPC nominal OCP.

The residual model predicts discrete one-step residuals for [u, v, omega]
using MPC stage input y_k = [x_k, u_k].

Usage examples
--------------
Train from one run:
  python utilities/l4acados_residual_experiment.py train \
    --diag-csv simulation/plots/<run>/diag_*.csv \
    --out-dir simulation/logs/l4acados_residual

Build l4acados solver from trained residual:
  python utilities/l4acados_residual_experiment.py build \
    --checkpoint simulation/logs/l4acados_residual/residual_model.pt \
    --codegen-dir /tmp/l4acados_scm_codegen
"""

from __future__ import annotations

import argparse
import glob
import json
import random
import sys
from pathlib import Path

import casadi as ca
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parent.parent
SIM_DIR = PROJECT_ROOT / "simulation"

sys.path.insert(0, str(SIM_DIR))

from acados_mpc_solver import AcadosMPC, DEFAULT_MPC_DT, DEFAULT_MPC_HORIZON_STEPS
from nn_tire_model import load_nn_tire_model
from param_consistency import get_terrain_preset, terrain_preset_to_internal


STATE_COLS = [
    "x_fa_meas",
    "y_fa_meas",
    "psi_meas",
    "u_meas",
    "v_meas",
    "omega_meas",
    "ax_state",
    "delta_prev_state",
    "Jx",
]
CTRL_COLS = ["steering_angle", "Jx"]


def _set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _resolve_terrain(terrain_name: str) -> dict:
    return terrain_preset_to_internal(get_terrain_preset(terrain_name))


def _maybe_load_nn(model_name: str, terrain_params: dict):
    model_dir = PROJECT_ROOT / "nn_models" / model_name
    if not model_dir.exists():
        raise FileNotFoundError(f"NN model directory not found: {model_dir}")
    return load_nn_tire_model(str(model_dir), terrain_params)


def _build_nominal_mpc(
    tire_model: str,
    nn_model: str,
    terrain_name: str,
    mpc_dt: float,
    mpc_n: int,
    no_lat_transfer: bool,
) -> tuple[AcadosMPC, dict]:
    terrain_params = _resolve_terrain(terrain_name)
    nn_tire = None
    if tire_model == "nn":
        nn_tire = _maybe_load_nn(nn_model, terrain_params)

    mpc = AcadosMPC(
        nn_tire_model=nn_tire,
        dt=float(mpc_dt),
        N=int(mpc_n),
        lateral_load_transfer=not bool(no_lat_transfer),
        kappa_mode="approx",
        tire_model=tire_model,
        build_solver=False,
    )
    return mpc, terrain_params


def _make_stage_params(mpc: AcadosMPC, terrain_params: dict) -> np.ndarray:
    p = np.zeros(mpc._np_per_stage, dtype=float)
    phi_rad = np.radians(float(terrain_params["phi"]))

    p[mpc._tp_off + 0] = float(terrain_params["Kphi"])
    p[mpc._tp_off + 1] = float(terrain_params["Kc"])
    p[mpc._tp_off + 2] = float(terrain_params.get("n", 1.1))
    p[mpc._tp_off + 3] = float(terrain_params["c"])
    p[mpc._tp_off + 4] = phi_rad
    p[mpc._tp_off + 5] = float(terrain_params["k"])

    p[mpc._sr_off] = 0.0
    p[mpc._kappa_ref_front_off] = 0.15
    p[mpc._kappa_ref_rear_off] = 0.15
    return p


def _make_f_expl_function(mpc: AcadosMPC) -> ca.Function:
    model = mpc._build_acados_model()
    return ca.Function("f_expl_only", [model.x, model.u, model.p], [model.f_expl_expr])


def _row_to_state(row: pd.Series) -> np.ndarray:
    return np.array([float(row[c]) for c in STATE_COLS], dtype=float)


def _row_to_control(row: pd.Series) -> np.ndarray:
    return np.array([float(row[c]) for c in CTRL_COLS], dtype=float)


def _build_training_arrays(
    diag_csv: Path,
    f_expl_fun: ca.Function,
    p_stage: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(diag_csv)
    required = ["sim_time"] + STATE_COLS + CTRL_COLS
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Diagnostic CSV is missing columns: {missing}")

    df = df.sort_values("sim_time").reset_index(drop=True)
    X_rows = []
    Y_rows = []

    for i in range(len(df) - 1):
        r0 = df.iloc[i]
        r1 = df.iloc[i + 1]

        dt = float(r1["sim_time"] - r0["sim_time"])
        if not np.isfinite(dt) or dt <= 1e-5:
            continue
        dt = float(np.clip(dt, 1e-3, 0.2))

        xk = _row_to_state(r0)
        uk = _row_to_control(r0)
        xkp1_meas = _row_to_state(r1)

        if (not np.isfinite(xk).all()) or (not np.isfinite(uk).all()) or (not np.isfinite(xkp1_meas).all()):
            continue

        dx_nom = np.array(f_expl_fun(xk, uk, p_stage)).reshape(-1)
        xkp1_nom = xk + dt * dx_nom

        # Residual channels: longitudinal/lateral/yaw-rate dynamics
        resid = xkp1_meas[[3, 4, 5]] - xkp1_nom[[3, 4, 5]]

        if not np.isfinite(resid).all():
            continue

        X_rows.append(np.concatenate([xk, uk], axis=0))
        Y_rows.append(resid)

    if not X_rows:
        raise RuntimeError("No valid training samples extracted from diagnostic CSV.")

    return np.asarray(X_rows, dtype=np.float32), np.asarray(Y_rows, dtype=np.float32)


def _expand_diag_csv_inputs(diag_csv_args: list[str]) -> list[Path]:
    files: list[Path] = []
    for token in diag_csv_args:
        p = Path(token).expanduser()
        if any(ch in token for ch in ["*", "?", "["]):
            matches = sorted(glob.glob(str(p)))
            files.extend(Path(m).resolve() for m in matches if Path(m).is_file())
        elif p.is_file():
            files.append(p.resolve())
        else:
            maybe = Path(token).expanduser().resolve()
            if maybe.is_file():
                files.append(maybe)
    uniq = []
    seen = set()
    for f in files:
        if f not in seen:
            seen.add(f)
            uniq.append(f)
    if not uniq:
        raise FileNotFoundError(
            f"No diagnostic CSV files found for inputs: {diag_csv_args}"
        )
    return uniq


class ResidualMLP(nn.Module):
    def __init__(self, input_dim: int = 11, output_dim: int = 3, hidden_sizes=(64, 64)):
        super().__init__()
        hs = list(hidden_sizes)
        layers = []
        prev = input_dim
        for h in hs:
            layers.append(nn.Linear(prev, h))
            layers.append(nn.Tanh())
            prev = h
        layers.append(nn.Linear(prev, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class ScaledResidualMLP(nn.Module):
    """Inference module that applies input/output scaling internally."""

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


def _train_residual_model(
    X: np.ndarray,
    Y: np.ndarray,
    epochs: int,
    batch_size: int,
    lr: float,
    seed: int,
    hidden_sizes: tuple[int, ...],
) -> tuple[dict, dict]:
    _set_seed(seed)

    n = X.shape[0]
    n_val = max(1, int(0.1 * n))
    perm = np.random.permutation(n)
    idx_val = perm[:n_val]
    idx_train = perm[n_val:]

    X_train = X[idx_train]
    Y_train = Y[idx_train]
    X_val = X[idx_val]
    Y_val = Y[idx_val]

    x_mean = X_train.mean(axis=0)
    x_std = X_train.std(axis=0)
    x_std = np.where(x_std < 1e-6, 1.0, x_std)
    y_mean = Y_train.mean(axis=0)
    y_std = Y_train.std(axis=0)
    y_std = np.where(y_std < 1e-6, 1.0, y_std)

    X_train_s = (X_train - x_mean) / x_std
    X_val_s = (X_val - x_mean) / x_std
    Y_train_s = (Y_train - y_mean) / y_std
    Y_val_s = (Y_val - y_mean) / y_std

    model = ResidualMLP(input_dim=X.shape[1], output_dim=Y.shape[1], hidden_sizes=hidden_sizes)
    opt = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    loss_fn = nn.MSELoss()

    ds = TensorDataset(
        torch.tensor(X_train_s, dtype=torch.float32),
        torch.tensor(Y_train_s, dtype=torch.float32),
    )
    dl = DataLoader(ds, batch_size=batch_size, shuffle=True)

    best_val = float("inf")
    best_state = None

    for ep in range(epochs):
        model.train()
        for xb, yb in dl:
            opt.zero_grad()
            pred = model(xb)
            loss = loss_fn(pred, yb)
            loss.backward()
            opt.step()

        model.eval()
        with torch.no_grad():
            val_pred = model(torch.tensor(X_val_s, dtype=torch.float32))
            val_loss = float(loss_fn(val_pred, torch.tensor(Y_val_s, dtype=torch.float32)).item())

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}

        if ep == 0 or (ep + 1) % 25 == 0:
            print(f"  epoch {ep+1:4d}/{epochs}  val_mse={val_loss:.6f}  best={best_val:.6f}")

    if best_state is None:
        raise RuntimeError("Residual model training did not produce a checkpoint.")
    model.load_state_dict(best_state)

    with torch.no_grad():
        train_pred_s = model(torch.tensor(X_train_s, dtype=torch.float32)).numpy()
        val_pred_s = model(torch.tensor(X_val_s, dtype=torch.float32)).numpy()
    train_pred = train_pred_s * y_std + y_mean
    val_pred = val_pred_s * y_std + y_mean

    train_rmse = np.sqrt(np.mean((train_pred - Y_train) ** 2, axis=0))
    val_rmse = np.sqrt(np.mean((val_pred - Y_val) ** 2, axis=0))

    ckpt = {
        "input_dim": int(X.shape[1]),
        "output_dim": int(Y.shape[1]),
        "hidden_sizes": list(hidden_sizes),
        "state_dict": model.state_dict(),
        "x_mean": x_mean.astype(np.float32),
        "x_std": x_std.astype(np.float32),
        "y_mean": y_mean.astype(np.float32),
        "y_std": y_std.astype(np.float32),
    }
    metrics = {
        "n_samples": int(n),
        "n_train": int(len(idx_train)),
        "n_val": int(len(idx_val)),
        "best_val_mse_norm": float(best_val),
        "train_rmse_du_dv_domega": train_rmse.tolist(),
        "val_rmse_du_dv_domega": val_rmse.tolist(),
    }
    return ckpt, metrics


def _load_scaled_model(ckpt_path: Path) -> ScaledResidualMLP:
    # Torch >=2.6 defaults to weights_only=True, which rejects checkpoints
    # containing numpy arrays (our scaler stats). Keep backward compatibility.
    try:
        payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    except TypeError:
        payload = torch.load(ckpt_path, map_location="cpu")
    model = ScaledResidualMLP(
        input_dim=int(payload["input_dim"]),
        output_dim=int(payload["output_dim"]),
        hidden_sizes=tuple(payload["hidden_sizes"]),
        x_mean=np.asarray(payload["x_mean"], dtype=np.float32),
        x_std=np.asarray(payload["x_std"], dtype=np.float32),
        y_mean=np.asarray(payload["y_mean"], dtype=np.float32),
        y_std=np.asarray(payload["y_std"], dtype=np.float32),
    )
    model.core.load_state_dict(payload["state_dict"])
    model.eval()
    return model


def _cmd_train(args):
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    diag_csvs = _expand_diag_csv_inputs(args.diag_csv)

    print(f"Training residual model from {len(diag_csvs)} diagnostic CSV(s)")
    mpc, terrain_params = _build_nominal_mpc(
        tire_model=args.model,
        nn_model=args.nn_model,
        terrain_name=args.terrain,
        mpc_dt=args.mpc_dt,
        mpc_n=args.mpc_n,
        no_lat_transfer=args.no_lat_transfer,
    )
    f_fun = _make_f_expl_function(mpc)
    p_stage = _make_stage_params(mpc, terrain_params)

    X_all = []
    Y_all = []
    for i, diag_csv in enumerate(diag_csvs, start=1):
        X_i, Y_i = _build_training_arrays(diag_csv, f_fun, p_stage)
        X_all.append(X_i)
        Y_all.append(Y_i)
        print(f"  [{i}/{len(diag_csvs)}] {diag_csv}  -> X={X_i.shape}, Y={Y_i.shape}")

    X = np.concatenate(X_all, axis=0)
    Y = np.concatenate(Y_all, axis=0)
    print(f"  dataset: X={X.shape}, Y={Y.shape}")

    hidden = tuple(int(h) for h in args.hidden)
    ckpt, metrics = _train_residual_model(
        X=X,
        Y=Y,
        epochs=int(args.epochs),
        batch_size=int(args.batch_size),
        lr=float(args.lr),
        seed=int(args.seed),
        hidden_sizes=hidden,
    )

    ckpt_path = out_dir / "residual_model.pt"
    torch.save(ckpt, ckpt_path)
    meta = {
        "diag_csv_list": [str(p) for p in diag_csvs],
        "terrain": args.terrain,
        "nominal_tire_model": args.model,
        "nominal_nn_model": args.nn_model if args.model == "nn" else None,
        "mpc_dt": float(args.mpc_dt),
        "mpc_n": int(args.mpc_n),
        "hidden": list(hidden),
        **metrics,
    }
    (out_dir / "residual_metrics.json").write_text(json.dumps(meta, indent=2))
    print(f"  wrote {ckpt_path}")
    print(f"  wrote {out_dir / 'residual_metrics.json'}")


def _cmd_build(args):
    # Import from local l4acados copy in simulation/l4acados/
    try:
        from l4acados.controllers.residual_learning_mpc import ResidualLearningMPC
        from l4acados.models.pytorch_models.pytorch_residual_model import PyTorchResidualModel
    except ImportError as exc:
        raise RuntimeError(
            "Could not import l4acados. Ensure simulation/l4acados/ exists."
        ) from exc

    ckpt_path = Path(args.checkpoint).expanduser().resolve()
    if not ckpt_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    scaled_model = _load_scaled_model(ckpt_path)

    mpc, _terrain_params = _build_nominal_mpc(
        tire_model=args.model,
        nn_model=args.nn_model,
        terrain_name=args.terrain,
        mpc_dt=args.mpc_dt,
        mpc_n=args.mpc_n,
        no_lat_transfer=args.no_lat_transfer,
    )

    codegen_dir = Path(args.codegen_dir).expanduser().resolve()
    codegen_dir.mkdir(parents=True, exist_ok=True)
    ocp = mpc.build_nominal_ocp(code_export_directory=str(codegen_dir / "c_generated_code"))

    # Residual channels map to [u, v, omega] state components.
    B = np.zeros((mpc.nx, 3), dtype=float)
    B[3, 0] = 1.0
    B[4, 1] = 1.0
    B[5, 2] = 1.0

    residual_model = PyTorchResidualModel(scaled_model)
    rlmpc = ResidualLearningMPC(
        ocp=ocp,
        B=B,
        residual_model=residual_model,
        build_c_code=not args.no_build,
        use_cython=not args.no_cython,
        path_json_ocp=str(codegen_dir / "residual_lbmpc_ocp_solver_config.json"),
        path_json_sim=str(codegen_dir / "residual_lbmpc_sim_solver_config.json"),
    )

    summary = {
        "checkpoint": str(ckpt_path),
        "codegen_dir": str(codegen_dir),
        "nx": int(rlmpc.nx),
        "nu": int(rlmpc.nu),
        "N": int(rlmpc.N),
        "residual_dim": int(B.shape[1]),
        "built_c_code": bool(not args.no_build),
        "use_cython": bool(not args.no_cython),
    }
    print("l4acados residual MPC initialized:")
    print(json.dumps(summary, indent=2))


def _build_parser():
    p = argparse.ArgumentParser(description="l4acados residual-learning scaffold for SCM_Teleop")
    sub = p.add_subparsers(dest="cmd", required=True)

    tr = sub.add_parser("train", help="Train residual model from one or more diagnostic CSV files")
    tr.add_argument(
        "--diag-csv",
        required=True,
        nargs="+",
        help="One or more diag_*.csv paths (globs supported)",
    )
    tr.add_argument("--out-dir", required=True, help="Output directory for residual model artifacts")
    tr.add_argument("--terrain", default="clay", choices=["sand", "clay", "dirt"])
    tr.add_argument("--model", default="pacejka", choices=["nn", "pacejka", "tmeasy", "linear"])
    tr.add_argument("--nn-model", default="paper_v2_mlp_16_4")
    tr.add_argument("--mpc-dt", type=float, default=DEFAULT_MPC_DT)
    tr.add_argument("--mpc-n", type=int, default=DEFAULT_MPC_HORIZON_STEPS)
    tr.add_argument("--no-lat-transfer", action="store_true")
    tr.add_argument("--epochs", type=int, default=200)
    tr.add_argument("--batch-size", type=int, default=256)
    tr.add_argument("--lr", type=float, default=1e-3)
    tr.add_argument("--seed", type=int, default=7)
    tr.add_argument("--hidden", nargs="+", type=int, default=[64, 64])
    tr.set_defaults(func=_cmd_train)

    bd = sub.add_parser("build", help="Build l4acados residual MPC from trained residual model")
    bd.add_argument("--checkpoint", required=True, help="Path to residual_model.pt")
    bd.add_argument("--codegen-dir", required=True, help="Directory for acados/l4acados codegen")
    bd.add_argument("--terrain", default="clay", choices=["sand", "clay", "dirt"])
    bd.add_argument("--model", default="pacejka", choices=["nn", "pacejka", "tmeasy", "linear"])
    bd.add_argument("--nn-model", default="paper_v2_mlp_16_4")
    bd.add_argument("--mpc-dt", type=float, default=DEFAULT_MPC_DT)
    bd.add_argument("--mpc-n", type=int, default=DEFAULT_MPC_HORIZON_STEPS)
    bd.add_argument("--no-lat-transfer", action="store_true")
    bd.add_argument("--no-build", action="store_true", help="Do not build C code (expects existing json/artifacts)")
    bd.add_argument("--no-cython", action="store_true", help="Use ctypes solver interface instead of cython")
    bd.set_defaults(func=_cmd_build)

    return p


def main():
    parser = _build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
