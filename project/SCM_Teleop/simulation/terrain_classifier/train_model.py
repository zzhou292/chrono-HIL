#!/usr/bin/env python3
"""
Train Terrain Classifier
==========================

Trains a Random Forest classifier on the collected feature data and saves the
model as a pickle file for use by the online classifier node.

Also evaluates with stratified cross-validation and prints a confusion matrix.

Usage:
    python -m terrain_classifier.train_model \\
        --data terrain_classifier/data/training_data.csv \\
        --output terrain_classifier/models/terrain_rf.pkl

Inputs:
    CSV with columns = FeatureVector.feature_names() + ["terrain_label"]
    (produced by collect_data.py)

Outputs:
    - terrain_rf.pkl  — serialized (model, label_encoder, feature_names, scaler)
    - confusion_matrix.png
    - feature_importance.png
"""

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier, HistGradientBoostingClassifier
from sklearn.metrics import (
    classification_report, confusion_matrix, ConfusionMatrixDisplay,
)
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.preprocessing import LabelEncoder, StandardScaler

# Ensure parent is on the path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from terrain_classifier.feature_extractor import FeatureVector


def train(args):
    print("=" * 60)
    print("Terrain Classifier — Model Training")
    print("=" * 60)

    # ---- Load data ----
    df = pd.read_csv(args.data)
    feature_names = FeatureVector.feature_names()

    # Backward-compat: drop legacy columns not used as ML features.
    # speed_mean excluded to avoid confound with sand's speed ceiling;
    # speed_std was never an ML feature.
    _legacy_cols = ["speed_std", "speed_mean"]
    for sc in _legacy_cols:
        if sc in df.columns and sc not in feature_names:
            df = df.drop(columns=[sc])

    # Validate columns
    missing = [c for c in feature_names + ["terrain_label"] if c not in df.columns]
    if missing:
        print(f"ERROR: Missing columns in CSV: {missing}")
        sys.exit(1)

    X = df[feature_names].values.astype(np.float64)
    y_raw = df["terrain_label"].values

    # Handle NaN/inf
    mask = np.all(np.isfinite(X), axis=1)
    if (~mask).sum() > 0:
        print(f"  Dropping {(~mask).sum()} rows with NaN/inf values")
    X = X[mask]
    y_raw = y_raw[mask]

    # ---- Derived features (computed from base features) ----
    # Base column indices (13 base features):
    #   0=slip_front_mean, 1=slip_front_std, 2=slip_front_max,
    #   3=slip_rear_mean,  4=slip_rear_std,  5=slip_rear_max,
    #   6=yaw_accel_std,   7=az_std,
    #   8=sideslip_ratio_mean, 9=yaw_rate_mean,
    #  10=ax_std, 11=ay_std,
    #  12=steering_std
    # NOTE: speed_mean excluded — it's a confound (sand can't exceed ~5 m/s).
    eps = 1e-6

    # Log-transform highly skewed slip features
    log_slip_f = np.log1p(X[:, 0:1])
    log_slip_r = np.log1p(X[:, 3:4])
    log_slip_f_max = np.log1p(X[:, 2:3])
    log_slip_r_max = np.log1p(X[:, 5:6])

    # Front/rear slip ratio (terrain-dependent traction distribution)
    slip_fr_ratio = X[:, 0:1] / np.maximum(X[:, 3:4], eps)

    # Coefficient of variation of slip (normalized variability)
    slip_f_cv = X[:, 1:2] / np.maximum(X[:, 0:1], eps)
    slip_r_cv = X[:, 4:5] / np.maximum(X[:, 3:4], eps)

    # Yaw-to-sideslip ratio (understeer/oversteer indicator)
    yaw_slip_ratio = X[:, 9:10] / np.maximum(X[:, 8:9], eps)

    # ---- Steering-normalized features (driving-intensity invariant) ----
    # For the same steering input, terrain determines slip/sideslip/yaw response.
    steer = np.maximum(X[:, 12:13], 0.01)  # steering_std with floor
    slip_per_steer = X[:, 3:4] / steer       # slip_rear_mean / steering_std
    ay_per_steer = X[:, 11:12] / steer        # ay_std / steering_std
    yaw_rate_per_steer = X[:, 9:10] / steer   # yaw_rate_mean / steering_std
    yaw_accel_per_steer = X[:, 6:7] / steer   # yaw_accel_std / steering_std
    sideslip_per_steer = X[:, 8:9] / steer    # sideslip_ratio_mean / steering_std

    derived = np.hstack([
        log_slip_f, log_slip_r, log_slip_f_max, log_slip_r_max,
        slip_fr_ratio, slip_f_cv, slip_r_cv, yaw_slip_ratio,
        slip_per_steer, ay_per_steer, yaw_rate_per_steer,
        yaw_accel_per_steer, sideslip_per_steer,
    ])
    derived_names = [
        "log_slip_front_mean", "log_slip_rear_mean",
        "log_slip_front_max", "log_slip_rear_max",
        "slip_front_rear_ratio", "slip_front_cv", "slip_rear_cv",
        "yaw_sideslip_ratio",
        "slip_rear_per_steer", "ay_per_steer", "yaw_rate_per_steer",
        "yaw_accel_per_steer", "sideslip_per_steer",
    ]

    X = np.hstack([X, derived])
    feature_names = feature_names + derived_names

    # Clean up any NaN/inf introduced by derived features
    mask2 = np.all(np.isfinite(X), axis=1)
    if (~mask2).sum() > 0:
        print(f"  Dropping {(~mask2).sum()} derived-feature NaN/inf rows")
        X = X[mask2]
        y_raw = y_raw[mask2]

    le = LabelEncoder()
    y = le.fit_transform(y_raw)

    print(f"  Samples: {len(X)}")
    print(f"  Features: {len(feature_names)}")
    print(f"  Classes: {list(le.classes_)} ({np.bincount(y)})")

    if len(X) < 30:
        print("ERROR: Not enough data to train. Need at least 30 samples.")
        sys.exit(1)

    # ---- Standardize features ----
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    # ---- Train classifier ----
    if args.classifier == "gb":
        clf = HistGradientBoostingClassifier(
            max_iter=args.n_trees,
            max_depth=args.max_depth or 6,
            min_samples_leaf=args.min_leaf,
            learning_rate=0.1,
            random_state=42,
        )
        clf_name = "HistGradientBoosting"
    else:
        clf = RandomForestClassifier(
            n_estimators=args.n_trees,
            max_depth=args.max_depth,
            min_samples_leaf=args.min_leaf,
            class_weight="balanced",
            random_state=42,
            n_jobs=-1,
        )
        clf_name = "RandomForest"

    print(f"  Classifier: {clf_name}")

    # Stratified K-fold cross-validation
    n_splits = min(5, min(np.bincount(y)))
    if n_splits < 2:
        print("  WARNING: Too few samples per class for cross-validation. "
              "Training on full dataset without CV.")
        clf.fit(X_scaled, y)
        y_pred = clf.predict(X_scaled)
    else:
        cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=42)
        y_pred = cross_val_predict(clf, X_scaled, y, cv=cv)
        # Re-train on full dataset for the final model
        clf.fit(X_scaled, y)

    # ---- Evaluation ----
    print(f"\n  Classification Report ({n_splits}-fold CV):")
    print(classification_report(y, y_pred, target_names=le.classes_, digits=3))

    cm = confusion_matrix(y, y_pred)
    print("  Confusion Matrix:")
    for i, cls in enumerate(le.classes_):
        print(f"    {cls:>6s}: {cm[i]}")

    # ---- Feature importance ----
    if hasattr(clf, 'feature_importances_'):
        importances = clf.feature_importances_
    else:
        # HistGradientBoosting uses permutation importance by default
        from sklearn.inspection import permutation_importance
        perm_result = permutation_importance(clf, X_scaled, y, n_repeats=5,
                                             random_state=42, n_jobs=-1)
        importances = perm_result.importances_mean
    idx_sorted = np.argsort(importances)[::-1]
    print("\n  Feature Importance (top 10):")
    for rank, idx in enumerate(idx_sorted[:10]):
        print(f"    {rank+1:2d}. {feature_names[idx]:25s}  {importances[idx]:.4f}")

    # ---- Save model ----
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model_bundle = {
        "model": clf,
        "label_encoder": le,
        "scaler": scaler,
        "feature_names": feature_names,
        "derived_feature_names": derived_names,
        "base_feature_names": FeatureVector.feature_names(),
        "classes": list(le.classes_),
        "n_samples": len(X),
    }
    with open(out_path, "wb") as f:
        pickle.dump(model_bundle, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"\n  Model saved: {out_path}")

    # ---- Plots (optional) ----
    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            plot_dir = out_path.parent
            # Confusion matrix
            fig, ax = plt.subplots(figsize=(6, 5))
            ConfusionMatrixDisplay.from_predictions(
                le.inverse_transform(y), le.inverse_transform(y_pred),
                ax=ax, cmap="Blues")
            ax.set_title("Terrain Classification Confusion Matrix")
            fig.tight_layout()
            cm_path = plot_dir / "confusion_matrix.png"
            fig.savefig(cm_path, dpi=150)
            plt.close(fig)
            print(f"  Plot: {cm_path}")

            # Feature importance bar chart
            fig, ax = plt.subplots(figsize=(8, 5))
            top_n = min(16, len(feature_names))
            top_idx = idx_sorted[:top_n]
            ax.barh(range(top_n), importances[top_idx], align="center")
            ax.set_yticks(range(top_n))
            ax.set_yticklabels([feature_names[i] for i in top_idx])
            ax.invert_yaxis()
            ax.set_xlabel("Feature Importance")
            ax.set_title("Terrain Classifier — Feature Importance")
            fig.tight_layout()
            fi_path = plot_dir / "feature_importance.png"
            fig.savefig(fi_path, dpi=150)
            plt.close(fig)
            print(f"  Plot: {fi_path}")

        except ImportError:
            print("  matplotlib not available — skipping plots")

    # ---- Accuracy summary ----
    acc = np.mean(y_pred == y) * 100
    print(f"\n  Overall accuracy: {acc:.1f}%")
    print("Done!")


def main():
    p = argparse.ArgumentParser(description="Train terrain classifier from collected data")
    p.add_argument("--data", "-d", default="terrain_classifier/data/training_data.csv",
                   help="Input CSV with labeled features")
    p.add_argument("--output", "-o", default="terrain_classifier/models/terrain_rf.pkl",
                   help="Output path for pickled model bundle")
    p.add_argument("--n-trees", type=int, default=200, help="Number of RF trees")
    p.add_argument("--max-depth", type=int, default=None, help="Max tree depth (None=unlimited)")
    p.add_argument("--min-leaf", type=int, default=3, help="Min samples per leaf")
    p.add_argument("--classifier", choices=["rf", "gb"], default="gb",
                   help="Classifier type: rf=RandomForest, gb=GradientBoosting")
    p.add_argument("--no-plot", action="store_true", help="Skip generating plots")
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
