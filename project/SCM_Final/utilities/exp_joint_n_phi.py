#!/usr/bin/env python3
"""Joint (n, phi) terrain-parameter estimation experiment.

Compares three sliding-window MLPs trained on the same vehicle-dynamics
windows but with different regression targets:

* ``n``-only        — current production estimator (Bekker sinkage exponent)
* ``phi``-only      — Mohr-Coulomb friction angle, treated as a single target
* joint ``(n, phi)`` — multi-task head with normalised MSE per target

Goal: figure out whether the proprioceptive features that discriminate clay /
dirt / sand carry enough information to disentangle ``n`` from ``phi``, or
whether joint training collapses because the two targets are too correlated
on the canonical manifold and the off-manifold variants in the diverse
dataset are too few to break the correlation.

Inputs
------
* ``--trace-dir``  CSV traces from ``utilities/collect_diverse_terrains.py`` /
                   ``utilities/collect_terrain_traces.py``
* ``--yaml-dir``   directory of per-terrain YAMLs (used to look up phi for
                   off-manifold variants whose name doesn't match a preset)

Outputs
-------
* ``paper_figures/exp_joint_n_phi.csv``   per-(model, target, split, terrain) RMSE
* ``paper_figures/exp_joint_n_phi.png``   scatter + bar comparison figure
* ``paper_figures/exp_joint_n_phi.txt``   pretty-printed summary table

Use ``--out-prefix`` to write alternate generalization checks without
overwriting the paper's default joint-identification figure.
"""

from __future__ import annotations

import argparse
import json
import pickle
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import yaml

ROOT = Path(__file__).resolve().parent.parent  # project root
sys.path.insert(0, str(ROOT / "nn_training"))  # train_terrain_window_mlp lives here
sys.path.insert(0, str(ROOT / "simulation"))   # param_consistency lives here

from train_terrain_window_mlp import (  # noqa: E402
    FEATURE_NAMES,
    N_FEATURES,
    build_windows,
    compute_window_features,
    load_trace,
)
from param_consistency import TERRAIN_PRESETS  # noqa: E402

DATA_DIR = ROOT / "data"
DEFAULT_TRACE_DIR = DATA_DIR / "terrain_traces"
DEFAULT_YAML_DIR = DATA_DIR / "terrain_yamls"
OUT_DIR = ROOT / "my_paper" / "paper_figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────
# phi label recovery
# ─────────────────────────────────────────────────────────────────────
# Each suffix component is independently optional.  Older rig-style traces
# carry ``_thrXX_ampXX_seedX``, rich-excitation traces carry ``_seedX``, and
# the closed-loop sinusoidal LHS traces carry
# ``cl_<label>_v050_amp20_wl30_seedX``.  The terrain YAMLs use just
# ``<label>.yaml``, so keep stripping known run-condition tokens from the
# right edge until the stable terrain label remains.
_TRACE_SUFFIX_TOKEN_RE = re.compile(r"^(?:thr|amp|wl|v|seed)\d+$")


def trace_label_from_filename(stem: str) -> str:
    """Recover the terrain label used for the YAML / preset lookup."""
    if stem.startswith("cl_"):
        stem = stem[3:]
    parts = stem.split("_")
    while parts and _TRACE_SUFFIX_TOKEN_RE.match(parts[-1]):
        parts.pop()
    return "_".join(parts)


def lookup_phi_yaml(label: str, yaml_dir: Path) -> Optional[float]:
    """Try to read ``friction_angle`` from ``<yaml_dir>/<label>.yaml``."""
    yp = yaml_dir / f"{label}.yaml"
    if yp.exists():
        try:
            cfg = yaml.safe_load(yp.read_text())
            if cfg and "friction_angle" in cfg:
                return float(cfg["friction_angle"])
        except Exception:
            pass
    return None


def lookup_phi_preset(terrain_in_csv: str) -> Optional[float]:
    if terrain_in_csv in TERRAIN_PRESETS:
        return float(TERRAIN_PRESETS[terrain_in_csv]["friction_angle"])
    return None


# ─────────────────────────────────────────────────────────────────────
# Dataset construction
# ─────────────────────────────────────────────────────────────────────
def collect_traces(trace_dirs, yaml_dirs
                   ) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray,
                                   float, float, str, str]]:
    """Returns list of (t, dyn, thr, n_true, phi_true, terrain, label).

    ``trace_dirs`` and ``yaml_dirs`` are lists; phi labels are resolved by
    searching every ``yaml_dirs[i]`` in order until one matches.
    """
    out = []
    if isinstance(trace_dirs, (str, Path)):
        trace_dirs = [trace_dirs]
    if isinstance(yaml_dirs, (str, Path)):
        yaml_dirs = [yaml_dirs]
    yaml_dirs = [Path(d) for d in yaml_dirs]
    for trace_dir in trace_dirs:
        trace_dir = Path(trace_dir)
        for csv in sorted(trace_dir.glob("*.csv")):
            if csv.name == "manifest.csv":
                continue
            try:
                t, dyn, thr, n_true, terrain = load_trace(csv)
            except Exception as e:
                print(f"  skip {csv.name}: {e}")
                continue
            label = trace_label_from_filename(csv.stem)
            phi = None
            for yd in yaml_dirs:
                phi = lookup_phi_yaml(label, yd)
                if phi is not None:
                    break
            if phi is None:
                phi = lookup_phi_preset(terrain)
            if phi is None:
                print(f"  skip {csv.name}: no phi label for label='{label}'")
                continue
            out.append((t, dyn, thr, float(n_true),
                        float(phi), terrain, label))
    return out


def build_xy(traces, *, win_seconds: float, stride_seconds: float,
             warmup_seconds: float):
    """Adapter around build_windows that also returns phi labels and the
    *trace label* (not just the high-level terrain category)."""
    X_all = []
    y_n   = []
    y_phi = []
    terrs = []
    labels = []
    for (t, dyn, thr, n_true, phi_true, terrain, label) in traces:
        # Reuse the canonical builder one trace at a time so we get the
        # same window/stride/warmup as production.
        try:
            Xi, yi, terr_i, _ = build_windows(
                [(t, dyn, thr, n_true, terrain)],
                win_seconds=win_seconds,
                stride_seconds=stride_seconds,
                warmup_seconds=warmup_seconds,
            )
        except ValueError:
            # Some expanded-envelope traces can stop before a full valid
            # feature window is available. They are not useful for training
            # and should not abort the whole split experiment.
            continue
        if Xi.shape[0] == 0:
            continue
        X_all.append(Xi)
        y_n.append(yi)
        y_phi.append(np.full(Xi.shape[0], phi_true, dtype=np.float64))
        terrs.extend(terr_i)
        labels.extend([label] * Xi.shape[0])
    if not X_all:
        raise RuntimeError("no windows built — check trace dir")
    X = np.concatenate(X_all, axis=0)
    yn = np.concatenate(y_n, axis=0)
    yp = np.concatenate(y_phi, axis=0)
    return X, yn, yp, terrs, labels


# ─────────────────────────────────────────────────────────────────────
# Models
# ─────────────────────────────────────────────────────────────────────
class HeadMLP(nn.Module):
    """Tiny MLP with configurable output dim (1 or 2)."""

    def __init__(self, n_in: int, n_out: int, hidden: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_in, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, n_out),
        )

    def forward(self, x):
        return self.net(x)


def fit(model, X_tr, Y_tr, X_val, Y_val, *,
        epochs: int, batch: int, lr: float, device: str) -> Dict:
    """Generic single/multi-target regression fit with early best-by-val MSE."""
    model = model.to(device)
    opt = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    Xt = torch.tensor(X_tr, dtype=torch.float32, device=device)
    Yt = torch.tensor(Y_tr, dtype=torch.float32, device=device)
    Xv = torch.tensor(X_val, dtype=torch.float32, device=device)
    Yv = torch.tensor(Y_val, dtype=torch.float32, device=device)
    best = float("inf")
    best_state = None
    n = Xt.shape[0]
    hist = []
    for ep in range(1, epochs + 1):
        model.train()
        idx = torch.randperm(n, device=device)
        losses = []
        for s in range(0, n, batch):
            sel = idx[s:s + batch]
            yp = model(Xt[sel])
            loss = ((yp - Yt[sel]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(loss.item())
        model.eval()
        with torch.no_grad():
            yp_v = model(Xv)
            v = ((yp_v - Yv) ** 2).mean().item()
        if v < best:
            best = v
            best_state = {k: t.detach().cpu().clone()
                          for k, t in model.state_dict().items()}
        hist.append((ep, float(np.mean(losses)), float(v)))
        if ep == 1 or ep % 20 == 0 or ep == epochs:
            print(f"    ep={ep:3d} train={np.mean(losses):.5f} val={v:.5f}")
    if best_state is not None:
        model.load_state_dict(best_state)
    return {"best_val_loss": float(best), "hist": hist}


def evaluate(model, X, Y, terrains, labels, *, target_cols, device,
             y_mean: Optional[np.ndarray] = None,
             y_std: Optional[np.ndarray] = None) -> Dict:
    """Compute RMSE in original units, overall, per-terrain, and per-label."""
    model.eval()
    with torch.no_grad():
        Xt = torch.tensor(X, dtype=torch.float32, device=device)
        Yp = model(Xt).cpu().numpy()
    if y_mean is not None and y_std is not None:
        Yp = Yp * y_std + y_mean
    Y = np.asarray(Y)
    if Yp.ndim == 1:
        Yp = Yp[:, None]
    if Y.ndim == 1:
        Y = Y[:, None]
    out = {"per_target": {}, "per_terrain": {}, "per_label": {},
           "predictions": Yp, "targets": Y}
    for j, name in enumerate(target_cols):
        rmse = float(np.sqrt(np.mean((Yp[:, j] - Y[:, j]) ** 2)))
        bias = float(np.mean(Yp[:, j] - Y[:, j]))
        out["per_target"][name] = {"rmse": rmse, "bias": bias}

    def _agg_by(group_keys):
        groups: Dict[str, list] = {}
        for i, k in enumerate(group_keys):
            groups.setdefault(k, []).append(i)
        result = {}
        for k, idxs in groups.items():
            rec = {"n_windows": len(idxs),
                   "true_means": [float(np.mean(Y[idxs, j]))
                                  for j in range(Y.shape[1])]}
            for j, name in enumerate(target_cols):
                true_b = Y[idxs, j]
                pred_b = Yp[idxs, j]
                rec[f"{name}_rmse"] = float(np.sqrt(
                    np.mean((pred_b - true_b) ** 2)))
                rec[f"{name}_bias"] = float(np.mean(pred_b - true_b))
                rec[f"{name}_pred_mean"] = float(np.mean(pred_b))
            result[k] = rec
        return result

    out["per_terrain"] = _agg_by(terrains)
    out["per_label"] = _agg_by(labels)
    return out


# ─────────────────────────────────────────────────────────────────────
# Main experiment
# ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir", type=Path, nargs="+",
                    default=[DEFAULT_TRACE_DIR])
    ap.add_argument("--yaml-dir", type=Path, nargs="+",
                    default=[DEFAULT_YAML_DIR])
    ap.add_argument("--eval-trace-dir", type=Path, nargs="+", default=None,
                    help="Optional external trace dirs used only for the "
                         "holdout split. These traces are not included in "
                         "training, so this is useful for evaluating on a "
                         "separate LHS sweep.")
    ap.add_argument("--eval-yaml-dir", type=Path, nargs="+", default=None,
                    help="YAML dirs for --eval-trace-dir. Defaults to "
                         "--yaml-dir when omitted.")
    ap.add_argument("--win-seconds", type=float, default=4.0)
    ap.add_argument("--stride-seconds", type=float, default=0.3)
    ap.add_argument("--warmup-seconds", type=float, default=1.5)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--holdout-labels", nargs="*", default=None,
                    help="Trace labels to hold out for honest evaluation. "
                         "Use 'mphi' to hold out all phi-perturbed traces, "
                         "for example.")
    ap.add_argument("--holdout-n-range", type=float, nargs=2, default=None,
                    help="Hold out traces with n_true in [lo, hi] "
                         "(combined with phi-range as AND).")
    ap.add_argument("--holdout-phi-range", type=float, nargs=2, default=None,
                    help="Hold out traces with phi_true in [lo, hi] "
                         "(combined with n-range as AND).")
    ap.add_argument("--holdout-outside-box", action="store_true",
                    help="With --holdout-n-range and --holdout-phi-range, "
                         "hold out traces outside the box instead of inside "
                         "it. This creates a near-extrapolation split: train "
                         "on the central box and evaluate on edge/corner "
                         "soils.")
    ap.add_argument("--out-prefix", default="exp_joint_n_phi",
                    help="Filename prefix for the .txt/.csv/.png outputs.")
    ap.add_argument("--save-joint-model-dir", type=Path, default=None,
                    help="Optional output dir for saving the trained joint "
                         "(n, phi) checkpoint in LearnedTerrainEstimator format")
    args = ap.parse_args()

    print(f"[exp] loading traces from {args.trace_dir}")
    traces = collect_traces(args.trace_dir, args.yaml_dir)
    print(f"[exp] loaded {len(traces)} traces with phi labels")

    eval_traces = []
    if args.eval_trace_dir:
        eval_yaml_dirs = args.eval_yaml_dir if args.eval_yaml_dir else args.yaml_dir
        print(f"[exp] loading external eval traces from {args.eval_trace_dir}")
        eval_traces = collect_traces(args.eval_trace_dir, eval_yaml_dirs)
        print(f"[exp] loaded {len(eval_traces)} external eval traces")

    held_substrs = list(args.holdout_labels) if args.holdout_labels else []
    n_lo, n_hi = (args.holdout_n_range if args.holdout_n_range
                  else (None, None))
    p_lo, p_hi = (args.holdout_phi_range if args.holdout_phi_range
                  else (None, None))

    def _in_requested_box(tr) -> bool:
        if n_lo is not None and not (n_lo <= tr[3] <= n_hi):
            return False
        if p_lo is not None and not (p_lo <= tr[4] <= p_hi):
            return False
        return n_lo is not None or p_lo is not None

    def _is_held(tr) -> bool:
        if held_substrs and any(s in tr[6] for s in held_substrs):
            return True
        if args.holdout_outside_box:
            if n_lo is None or p_lo is None:
                raise ValueError("--holdout-outside-box requires both "
                                 "--holdout-n-range and --holdout-phi-range")
            return not _in_requested_box(tr)
        if _in_requested_box(tr):
            return True
        return False

    if eval_traces:
        held = eval_traces
        train = traces
        print(f"[exp] external hold-out: {len(held)} traces, "
              f"train: {len(train)}")
    elif held_substrs or args.holdout_n_range or args.holdout_phi_range:
        held = [tr for tr in traces if _is_held(tr)]
        train = [tr for tr in traces if not _is_held(tr)]
        print(f"[exp] hold-out: {len(held)} traces "
              f"(substrs={held_substrs}, "
              f"n_range={args.holdout_n_range}, "
              f"phi_range={args.holdout_phi_range}, "
              f"outside_box={args.holdout_outside_box}), "
              f"train: {len(train)}")
    else:
        held = []
        train = traces

    print("[exp] building windows ...")
    X, yn, yp, terrs, labels = build_xy(
        train, win_seconds=args.win_seconds,
        stride_seconds=args.stride_seconds,
        warmup_seconds=args.warmup_seconds,
    )
    print(f"[exp]   train windows: {X.shape}  n∈[{yn.min():.2f},{yn.max():.2f}]"
          f"  phi∈[{yp.min():.1f},{yp.max():.1f}]")

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(X.shape[0])
    X = X[perm]
    yn = yn[perm]
    yp = yp[perm]
    terrs = [terrs[i] for i in perm]
    labels = [labels[i] for i in perm]

    n_val = int(round(args.val_frac * X.shape[0]))
    X_val, X_tr = X[:n_val], X[n_val:]
    yn_val, yn_tr = yn[:n_val], yn[n_val:]
    yp_val, yp_tr = yp[:n_val], yp[n_val:]
    terr_val = terrs[:n_val]

    if held:
        Xh, ynh, yph, terrh, lblh = build_xy(
            held, win_seconds=args.win_seconds,
            stride_seconds=args.stride_seconds,
            warmup_seconds=args.warmup_seconds,
        )
    else:
        Xh = ynh = yph = None
        terrh = lblh = []

    x_mean = X_tr.mean(axis=0)
    x_std = X_tr.std(axis=0) + 1e-6
    Xs_tr = (X_tr - x_mean) / x_std
    Xs_val = (X_val - x_mean) / x_std
    Xs_h = (Xh - x_mean) / x_std if Xh is not None else None

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[exp] device={device}")
    torch.manual_seed(args.seed)

    # Per-target normalisation (so the joint loss isn't dominated by the
    # 0–35° phi axis vs. the 0.4–1.3 n axis).
    n_mean, n_std = float(yn_tr.mean()), float(yn_tr.std() + 1e-6)
    p_mean, p_std = float(yp_tr.mean()), float(yp_tr.std() + 1e-6)

    yn_tr_s = (yn_tr - n_mean) / n_std
    yp_tr_s = (yp_tr - p_mean) / p_std
    yn_val_s = (yn_val - n_mean) / n_std
    yp_val_s = (yp_val - p_mean) / p_std

    results = {}

    # ── (a) n-only ────────────────────────────────────────────────────
    print("\n[exp] === training n-only ===")
    m_n = HeadMLP(N_FEATURES, 1, hidden=args.hidden)
    fit(m_n, Xs_tr, yn_tr_s[:, None], Xs_val, yn_val_s[:, None],
        epochs=args.epochs, batch=args.batch, lr=args.lr, device=device)
    results["n_only"] = {
        "val": evaluate(m_n, Xs_val, yn_val[:, None], terr_val,
                        labels[:n_val], target_cols=["n"], device=device,
                        y_mean=np.array([n_mean]), y_std=np.array([n_std])),
        "holdout": (evaluate(m_n, Xs_h, ynh[:, None], terrh, lblh,
                             target_cols=["n"], device=device,
                             y_mean=np.array([n_mean]),
                             y_std=np.array([n_std])) if Xh is not None
                    else None),
    }

    # ── (b) phi-only ──────────────────────────────────────────────────
    print("\n[exp] === training phi-only ===")
    m_p = HeadMLP(N_FEATURES, 1, hidden=args.hidden)
    fit(m_p, Xs_tr, yp_tr_s[:, None], Xs_val, yp_val_s[:, None],
        epochs=args.epochs, batch=args.batch, lr=args.lr, device=device)
    results["phi_only"] = {
        "val": evaluate(m_p, Xs_val, yp_val[:, None], terr_val,
                        labels[:n_val], target_cols=["phi"], device=device,
                        y_mean=np.array([p_mean]), y_std=np.array([p_std])),
        "holdout": (evaluate(m_p, Xs_h, yph[:, None], terrh, lblh,
                             target_cols=["phi"], device=device,
                             y_mean=np.array([p_mean]),
                             y_std=np.array([p_std])) if Xh is not None
                    else None),
    }

    # ── (c) joint (n, phi) ────────────────────────────────────────────
    print("\n[exp] === training joint (n, phi) ===")
    Y_tr_j = np.stack([yn_tr_s, yp_tr_s], axis=1)
    Y_val_j = np.stack([yn_val_s, yp_val_s], axis=1)
    m_j = HeadMLP(N_FEATURES, 2, hidden=args.hidden)
    fit(m_j, Xs_tr, Y_tr_j, Xs_val, Y_val_j,
        epochs=args.epochs, batch=args.batch, lr=args.lr, device=device)
    results["joint"] = {
        "val": evaluate(m_j, Xs_val,
                        np.stack([yn_val, yp_val], axis=1),
                        terr_val, labels[:n_val],
                        target_cols=["n", "phi"], device=device,
                        y_mean=np.array([n_mean, p_mean]),
                        y_std=np.array([n_std, p_std])),
        "holdout": (evaluate(m_j, Xs_h,
                             np.stack([ynh, yph], axis=1),
                             terrh, lblh,
                             target_cols=["n", "phi"], device=device,
                             y_mean=np.array([n_mean, p_mean]),
                             y_std=np.array([n_std, p_std]))
                    if Xh is not None else None),
    }

    if args.save_joint_model_dir is not None:
        save_dir = Path(args.save_joint_model_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        torch.save(m_j.state_dict(), save_dir / "weights.pt")
        with open(save_dir / "scaler.pkl", "wb") as f:
            pickle.dump({
                "x_mean": x_mean,
                "x_std": x_std,
                "y_mean": np.array([n_mean, p_mean], dtype=np.float64),
                "y_std": np.array([n_std, p_std], dtype=np.float64),
                "feature_names": FEATURE_NAMES,
                "win_seconds": args.win_seconds,
                "hidden": args.hidden,
            }, f)
        with open(save_dir / "config.json", "w") as f:
            json.dump({
                "n_features": int(N_FEATURES),
                "output_names": ["n", "phi"],
                "feature_names": FEATURE_NAMES,
                "hidden": int(args.hidden),
                "win_seconds": float(args.win_seconds),
                "stride_seconds": float(args.stride_seconds),
                "warmup_seconds": float(args.warmup_seconds),
                "output_bounds": {
                    "n": [float(np.min(yn)), float(np.max(yn))],
                    "phi": [float(np.min(yp)), float(np.max(yp))],
                },
                "best_val_mse": float(results["joint"]["val"]["per_target"]["n"]["rmse"] ** 2),
            }, f, indent=2)
        print(f"[exp] saved joint checkpoint -> {save_dir}")

    # ── pretty summary ────────────────────────────────────────────────
    lines = []

    def _fmt(rec):
        parts = []
        for k, v in rec["per_target"].items():
            parts.append(f"{k}_rmse={v['rmse']:.4f} bias={v['bias']:+.4f}")
        return "  |  ".join(parts)

    lines.append("=" * 78)
    lines.append("[exp] joint (n, phi) estimation summary")
    lines.append(f"      windows: train={X_tr.shape[0]} val={X_val.shape[0]}"
                 f" held={Xh.shape[0] if Xh is not None else 0}")
    lines.append("=" * 78)

    for name, R in results.items():
        lines.append(f"\n--- model: {name} ---")
        lines.append(f"  val  : {_fmt(R['val'])}")
        if R["holdout"]:
            lines.append(f"  hold : {_fmt(R['holdout'])}")
        for terr, rec in sorted(R["val"]["per_terrain"].items()):
            row = [f"k={rec['n_windows']:4d}"]
            for tgt in R["val"]["per_target"]:
                row.append(f"{tgt}_rmse={rec[tgt + '_rmse']:.4f}")
                row.append(f"{tgt}_bias={rec[tgt + '_bias']:+.4f}")
            lines.append(f"      VAL  {terr:>14s}  " + "  ".join(row))
        if R["holdout"]:
            for terr, rec in sorted(R["holdout"]["per_terrain"].items()):
                row = [f"k={rec['n_windows']:4d}"]
                for tgt in R["holdout"]["per_target"]:
                    row.append(f"{tgt}_rmse={rec[tgt + '_rmse']:.4f}")
                    row.append(f"{tgt}_bias={rec[tgt + '_bias']:+.4f}")
                lines.append(f"      HELD {terr:>14s}  " + "  ".join(row))
            lines.append("      ---- per held-out trace label ----")
            for lbl, rec in sorted(R["holdout"]["per_label"].items()):
                row = [f"k={rec['n_windows']:4d}",
                       f"true=({','.join(f'{m:.2f}' for m in rec['true_means'])})"]
                for tgt in R["holdout"]["per_target"]:
                    row.append(f"{tgt}_pred={rec[tgt + '_pred_mean']:.2f}")
                    row.append(f"{tgt}_rmse={rec[tgt + '_rmse']:.3f}")
                lines.append(f"      HELD {lbl:>22s}  " + "  ".join(row))

    summary = "\n".join(lines)
    print("\n" + summary)
    txt_path = OUT_DIR / f"{args.out_prefix}.txt"
    csv_path = OUT_DIR / f"{args.out_prefix}.csv"
    fig_path = OUT_DIR / f"{args.out_prefix}.png"
    txt_path.write_text(summary + "\n")

    # ── compact CSV ───────────────────────────────────────────────────
    import csv
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["model", "split", "target", "rmse", "bias", "n_windows"])
        for name, R in results.items():
            for split in ("val", "holdout"):
                Rs = R[split]
                if Rs is None:
                    continue
                for tgt, v in Rs["per_target"].items():
                    n_win = sum(rec["n_windows"]
                                for rec in Rs["per_terrain"].values())
                    w.writerow([name, split, tgt,
                                f"{v['rmse']:.5f}", f"{v['bias']:+.5f}",
                                n_win])
    print(f"[exp] CSV → {csv_path}")

    # ── scatter & bar comparison figure ───────────────────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    have_held = results["n_only"]["holdout"] is not None
    n_cols = 4 if have_held else 2
    fig, axes = plt.subplots(2, n_cols, figsize=(3.0 * n_cols + 0.5, 6.5))
    if axes.ndim == 1:
        axes = axes[:, None]

    def _scatter(ax, true, pred, label, ax_title):
        true = np.asarray(true).ravel()
        pred = np.asarray(pred).ravel()
        lo = float(min(true.min(), pred.min()))
        hi = float(max(true.max(), pred.max()))
        pad = 0.05 * (hi - lo + 1e-6)
        ax.scatter(true, pred, s=6, alpha=0.35, color="steelblue",
                   linewidths=0)
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], "k--", lw=0.8)
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_ylim(lo - pad, hi + pad)
        ax.set_xlabel(f"true {label}")
        ax.set_ylabel(f"pred {label}")
        ax.set_title(ax_title, fontsize=9)
        ax.grid(alpha=0.25)

    rN = results["n_only"]["val"]
    rP = results["phi_only"]["val"]
    rJ = results["joint"]["val"]

    _scatter(axes[0, 0], rN["targets"][:, 0], rN["predictions"][:, 0],
             "n", f"n-only (val)\nrmse={rN['per_target']['n']['rmse']:.3f}")
    _scatter(axes[0, 1], rJ["targets"][:, 0], rJ["predictions"][:, 0],
             "n", f"joint n (val)\nrmse={rJ['per_target']['n']['rmse']:.3f}")
    _scatter(axes[1, 0], rP["targets"][:, 0], rP["predictions"][:, 0],
             r"$\phi$ (deg)",
             f"phi-only (val)\nrmse={rP['per_target']['phi']['rmse']:.2f}°")
    _scatter(axes[1, 1], rJ["targets"][:, 1], rJ["predictions"][:, 1],
             r"$\phi$ (deg)",
             f"joint phi (val)\nrmse={rJ['per_target']['phi']['rmse']:.2f}°")

    if have_held:
        rNH = results["n_only"]["holdout"]
        rPH = results["phi_only"]["holdout"]
        rJH = results["joint"]["holdout"]
        _scatter(axes[0, 2], rNH["targets"][:, 0], rNH["predictions"][:, 0],
                 "n", f"n-only (held)\nrmse={rNH['per_target']['n']['rmse']:.3f}")
        _scatter(axes[0, 3], rJH["targets"][:, 0], rJH["predictions"][:, 0],
                 "n", f"joint n (held)\nrmse={rJH['per_target']['n']['rmse']:.3f}")
        _scatter(axes[1, 2], rPH["targets"][:, 0], rPH["predictions"][:, 0],
                 r"$\phi$ (deg)",
                 f"phi-only (held)\nrmse={rPH['per_target']['phi']['rmse']:.2f}°")
        _scatter(axes[1, 3], rJH["targets"][:, 1], rJH["predictions"][:, 1],
                 r"$\phi$ (deg)",
                 f"joint phi (held)\nrmse={rJH['per_target']['phi']['rmse']:.2f}°")

    fig.suptitle("Joint (n, phi) terrain estimation — predicted vs true",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(fig_path, dpi=180, bbox_inches="tight")
    print(f"[exp] figure → {fig_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
