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
from sklearn.ensemble import RandomForestClassifier
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

    # ---- Train Random Forest ----
    rf = RandomForestClassifier(
        n_estimators=args.n_trees,
        max_depth=args.max_depth,
        min_samples_leaf=args.min_leaf,
        class_weight="balanced",   # handles class imbalance
        random_state=42,
        n_jobs=-1,
    )

    # Stratified K-fold cross-validation
    n_splits = min(5, min(np.bincount(y)))
    if n_splits < 2:
        print("  WARNING: Too few samples per class for cross-validation. "
              "Training on full dataset without CV.")
        rf.fit(X_scaled, y)
        y_pred = rf.predict(X_scaled)
    else:
        cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=42)
        y_pred = cross_val_predict(rf, X_scaled, y, cv=cv)
        # Re-train on full dataset for the final model
        rf.fit(X_scaled, y)

    # ---- Evaluation ----
    print(f"\n  Classification Report ({n_splits}-fold CV):")
    print(classification_report(y, y_pred, target_names=le.classes_, digits=3))

    cm = confusion_matrix(y, y_pred)
    print("  Confusion Matrix:")
    for i, cls in enumerate(le.classes_):
        print(f"    {cls:>6s}: {cm[i]}")

    # ---- Feature importance ----
    importances = rf.feature_importances_
    idx_sorted = np.argsort(importances)[::-1]
    print("\n  Feature Importance (top 10):")
    for rank, idx in enumerate(idx_sorted[:10]):
        print(f"    {rank+1:2d}. {feature_names[idx]:25s}  {importances[idx]:.4f}")

    # ---- Save model ----
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model_bundle = {
        "model": rf,
        "label_encoder": le,
        "scaler": scaler,
        "feature_names": feature_names,
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
    p.add_argument("--no-plot", action="store_true", help="Skip generating plots")
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
