#!/usr/bin/env python3
"""
Temporal Neural Network Training for SCM Terrain Force Prediction

Trains an NN that takes a sliding window of K timesteps of per-tire operating
conditions (slip_ratio, slip_angle, velocity, vertical_load, steering_rate)
plus constant terrain parameters, and predicts tire forces (Fx, Fy).

This captures transient tire dynamics (relaxation, soil deformation history)
that a single-timestep model cannot represent.

Input:  K*5 operating params + 6 terrain params = K*5 + 6 features
Output: Fx, Fy (longitudinal and lateral tire force)

The architecture uses the same TerrainNN class (variable-depth tanh MLP)
from train_terrain_nn.py, just with a larger input dimension.
"""

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import argparse
import logging
import pickle
import json
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class TerrainTemporalNN(nn.Module):
    """
    Temporal NN for predicting tire forces on deformable terrain.
    Same architecture as TerrainNN (variable-depth tanh MLP),
    but designed for temporal window input (K*5 + 6 features).

    Saves temporal_K in the checkpoint for inference-time detection.
    """

    def __init__(self, input_size, output_size=2, hidden_sizes=None, temporal_K=3):
        super().__init__()
        if hidden_sizes is None:
            hidden_sizes = [8, 3]
        self.hidden_sizes = hidden_sizes
        self.temporal_K = temporal_K

        layers = []
        prev = input_size
        for h in hidden_sizes:
            layers.append(nn.Linear(prev, h))
            prev = h
        layers.append(nn.Linear(prev, output_size))
        self.layers = nn.ModuleList(layers)

        for layer in self.layers:
            nn.init.xavier_normal_(layer.weight)

    def forward(self, x):
        for layer in self.layers[:-1]:
            x = torch.tanh(layer(x))
        return self.layers[-1](x)


def build_temporal_windows(df, K=3, dt_nn=0.1, record_dt=0.005):
    """
    Build sliding-window samples from time-series data.

    Args:
        df: DataFrame with columns including scenario_id, timestep,
            operating params, terrain params, and forces.
        K: Window size (number of timesteps).
        dt_nn: Desired temporal spacing between window entries (s).
        record_dt: Recording interval in the data (s).

    Returns:
        X: ndarray [N_windows, K*5 + 6]
        y: ndarray [N_windows, 2]
    """
    # Operating feature columns (per timestep, in this ORDER for the NN)
    op_cols = ['slip_ratio', 'slip_angle', 'velocity', 'vertical_load', 'steering_rate']
    # Terrain feature columns (constant per scenario)
    terrain_cols = ['bekker_Kphi', 'bekker_Kc', 'bekker_n',
                    'mohr_cohesion', 'mohr_friction', 'janosi_shear']
    output_cols = ['Fx', 'Fy']

    # Subsampling stride: how many recorded timesteps per NN dt
    stride = max(1, int(round(dt_nn / record_dt)))
    logger.info(f"Window K={K}, dt_nn={dt_nn}s, record_dt={record_dt}s, stride={stride}")

    X_list = []
    y_list = []

    for sid, group in df.groupby('scenario_id'):
        group = group.sort_values('timestep').reset_index(drop=True)
        n_rows = len(group)

        # Subsample indices
        indices = list(range(0, n_rows, stride))
        if len(indices) < K:
            continue

        ops = group[op_cols].values    # [n_rows, 5]
        terrain = group[terrain_cols].iloc[0].values  # [6,] — constant
        forces = group[output_cols].values  # [n_rows, 2]

        # Create windows
        for w in range(K - 1, len(indices)):
            # Gather K timesteps: most recent first (t, t-1, ..., t-K+1)
            window_ops = []
            for j in range(K):
                idx = indices[w - j]  # t, t-1, t-2, ...
                window_ops.append(ops[idx])

            # Flatten: [ops_t(5), ops_{t-1}(5), ..., ops_{t-K+1}(5), terrain(6)]
            x_sample = np.concatenate(window_ops + [terrain])
            y_sample = forces[indices[w]]

            X_list.append(x_sample)
            y_list.append(y_sample)

    X = np.array(X_list, dtype=np.float32)
    y = np.array(y_list, dtype=np.float32)
    logger.info(f"Built {len(X)} windows from {df['scenario_id'].nunique()} scenarios")
    return X, y


class TemporalNNTrainer:
    """Training pipeline for temporal tire force NN."""

    def __init__(self, output_dir='nn_models', device='cpu'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.device = torch.device(device)
        self.scaler_X = StandardScaler()
        self.scaler_y = StandardScaler()

    def load_and_build_windows(self, data_file, K=3, dt_nn=0.1, record_dt=0.005):
        """Load CSV and build temporal windows."""
        logger.info(f"Loading data from {data_file}...")
        df = pd.read_csv(data_file)
        logger.info(f"Loaded {len(df)} rows, {df['scenario_id'].nunique()} scenarios")

        X, y = build_temporal_windows(df, K=K, dt_nn=dt_nn, record_dt=record_dt)

        # Remove NaN/inf
        mask = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
        X, y = X[mask], y[mask]
        logger.info(f"Valid windows: {len(X)}, input dim: {X.shape[1]}, output dim: {y.shape[1]}")

        # Split 70/15/15
        X_train, X_temp, y_train, y_temp = train_test_split(X, y, test_size=0.3, random_state=42)
        X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.5, random_state=42)

        logger.info(f"Train: {len(X_train)}, Val: {len(X_val)}, Test: {len(X_test)}")

        # Normalize
        X_train = self.scaler_X.fit_transform(X_train)
        X_val = self.scaler_X.transform(X_val)
        X_test = self.scaler_X.transform(X_test)
        y_train = self.scaler_y.fit_transform(y_train)
        y_val = self.scaler_y.transform(y_val)
        y_test = self.scaler_y.transform(y_test)

        # Save scalers
        scaler_path = self.output_dir / 'scalers.pkl'
        with open(scaler_path, 'wb') as f:
            pickle.dump({'X': self.scaler_X, 'y': self.scaler_y}, f)
        logger.info(f"Scalers saved to {scaler_path}")

        # Tensors
        self.X_train = torch.FloatTensor(X_train).to(self.device)
        self.y_train = torch.FloatTensor(y_train).to(self.device)
        self.X_val   = torch.FloatTensor(X_val).to(self.device)
        self.y_val   = torch.FloatTensor(y_val).to(self.device)
        self.X_test  = torch.FloatTensor(X_test).to(self.device)
        self.y_test  = torch.FloatTensor(y_test).to(self.device)

        self.input_size = X_train.shape[1]
        return X_train, y_train

    def train_model(self, temporal_K=3, hidden_sizes=None,
                    n_epochs=1000, batch_size=256, learning_rate=0.01, patience=50):
        """Train the temporal NN."""
        if hidden_sizes is None:
            hidden_sizes = [8, 3]

        model = TerrainTemporalNN(
            input_size=self.input_size, output_size=2,
            hidden_sizes=hidden_sizes, temporal_K=temporal_K
        ).to(self.device)

        n_params = sum(p.numel() for p in model.parameters())
        logger.info(f"Architecture: {self.input_size} -> {' -> '.join(map(str, hidden_sizes))} -> 2")
        logger.info(f"Total parameters: {n_params}")
        logger.info(f"Temporal K: {temporal_K}")

        train_loader = DataLoader(
            TensorDataset(self.X_train, self.y_train),
            batch_size=batch_size, shuffle=True
        )

        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=learning_rate, weight_decay=1e-4)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=0.5, patience=20)

        history = {'train_loss': [], 'val_loss': [], 'learning_rate': []}
        best_val_loss = float('inf')
        patience_counter = 0
        best_state = None

        for epoch in range(n_epochs):
            model.train()
            losses = []
            for bx, by in train_loader:
                optimizer.zero_grad()
                loss = criterion(model(bx), by)
                loss.backward()
                optimizer.step()
                losses.append(loss.item())

            avg_train = np.mean(losses)

            model.eval()
            with torch.no_grad():
                val_loss = criterion(model(self.X_val), self.y_val).item()

            scheduler.step(val_loss)
            lr = optimizer.param_groups[0]['lr']

            history['train_loss'].append(avg_train)
            history['val_loss'].append(val_loss)
            history['learning_rate'].append(lr)

            if val_loss < best_val_loss:
                best_val_loss = val_loss
                patience_counter = 0
                best_state = model.state_dict().copy()
                torch.save({
                    'epoch': epoch,
                    'model_state_dict': model.state_dict(),
                    'train_loss': avg_train,
                    'val_loss': val_loss,
                    'hidden_sizes': hidden_sizes,
                    'temporal_K': temporal_K,
                }, self.output_dir / 'best_terrain_nn.pt')
            else:
                patience_counter += 1

            if (epoch + 1) % 10 == 0 or epoch == 0:
                logger.info(f"Epoch [{epoch+1}/{n_epochs}] "
                           f"Train: {avg_train:.6f}, Val: {val_loss:.6f}, "
                           f"LR: {lr:.6f}, Best: {best_val_loss:.6f}")

            if patience_counter >= patience:
                logger.info(f"Early stopping at epoch {epoch+1}")
                break

        model.load_state_dict(best_state)

        with open(self.output_dir / 'training_history.json', 'w') as f:
            json.dump(history, f, indent=2)

        return model, history

    def evaluate_model(self, model):
        """Evaluate on test set."""
        model.eval()
        with torch.no_grad():
            y_pred_train = self.scaler_y.inverse_transform(model(self.X_train).cpu().numpy())
            y_pred_val   = self.scaler_y.inverse_transform(model(self.X_val).cpu().numpy())
            y_pred_test  = self.scaler_y.inverse_transform(model(self.X_test).cpu().numpy())
            y_train_orig = self.scaler_y.inverse_transform(self.y_train.cpu().numpy())
            y_val_orig   = self.scaler_y.inverse_transform(self.y_val.cpu().numpy())
            y_test_orig  = self.scaler_y.inverse_transform(self.y_test.cpu().numpy())

        def metrics(yt, yp):
            mse = np.mean((yt - yp)**2, axis=0)
            rmse = np.sqrt(mse)
            ss_res = np.sum((yt - yp)**2, axis=0)
            ss_tot = np.sum((yt - yt.mean(axis=0))**2, axis=0)
            r2 = 1 - ss_res / ss_tot
            return {'rmse': rmse, 'r2': r2}

        train_m = metrics(y_train_orig, y_pred_train)
        val_m   = metrics(y_val_orig, y_pred_val)
        test_m  = metrics(y_test_orig, y_pred_test)

        for label, m in [('Train', train_m), ('Val', val_m), ('Test', test_m)]:
            logger.info(f"{label}: Fx RMSE={m['rmse'][0]:.1f}N R²={m['r2'][0]:.4f}, "
                       f"Fy RMSE={m['rmse'][1]:.1f}N R²={m['r2'][1]:.4f}")

        # Save metrics
        out = {}
        for label, m in [('train', train_m), ('val', val_m), ('test', test_m)]:
            out[label] = {k: v.tolist() for k, v in m.items()}
        with open(self.output_dir / 'test_metrics.json', 'w') as f:
            json.dump(out, f, indent=2)

        return test_m, y_test_orig, y_pred_test

    def plot_results(self, history, y_test, y_pred_test):
        """Generate training/evaluation plots."""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        ax = axes[0, 0]
        ax.plot(history['train_loss'], label='Train', alpha=0.8)
        ax.plot(history['val_loss'], label='Val', alpha=0.8)
        ax.set_xlabel('Epoch'); ax.set_ylabel('MSE Loss')
        ax.set_title('Training History'); ax.legend(); ax.grid(alpha=0.3); ax.set_yscale('log')

        ax = axes[0, 1]
        ax.plot(history['learning_rate'], color='green')
        ax.set_xlabel('Epoch'); ax.set_ylabel('LR')
        ax.set_title('Learning Rate'); ax.grid(alpha=0.3); ax.set_yscale('log')

        for idx, (name, ax) in enumerate(zip(['Fx', 'Fy'], [axes[1, 0], axes[1, 1]])):
            ax.scatter(y_test[:, idx], y_pred_test[:, idx], alpha=0.3, s=5)
            lims = [min(y_test[:, idx].min(), y_pred_test[:, idx].min()),
                    max(y_test[:, idx].max(), y_pred_test[:, idx].max())]
            ax.plot(lims, lims, 'r--', lw=2, label='Perfect')
            ax.set_xlabel(f'True {name} (N)'); ax.set_ylabel(f'Predicted {name} (N)')
            ax.set_title(f'{name} Prediction'); ax.legend(); ax.grid(alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'training_results.png', dpi=300, bbox_inches='tight')
        logger.info(f"Plots saved to {self.output_dir / 'training_results.png'}")
        plt.close()


def main():
    p = argparse.ArgumentParser(description='Train temporal tire force NN')
    p.add_argument('--data', required=True, help='Path to temporal CSV data')
    p.add_argument('--output_dir', default='../nn_models/temporal_v1',
                   help='Output directory')
    p.add_argument('--K', type=int, default=3,
                   help='Temporal window size (timesteps, default: 3)')
    p.add_argument('--dt-nn', type=float, default=0.1,
                   help='Temporal spacing between window entries (s, default: 0.1)')
    p.add_argument('--record-dt', type=float, default=0.005,
                   help='Recording interval in data (s, default: 0.005)')
    p.add_argument('--hidden', type=int, nargs='+', default=[8, 3],
                   help='Hidden layer sizes (default: 8 3)')
    p.add_argument('--epochs', type=int, default=1000)
    p.add_argument('--batch_size', type=int, default=256)
    p.add_argument('--lr', type=float, default=0.01)
    p.add_argument('--patience', type=int, default=50)
    p.add_argument('--device', default='cpu')

    args = p.parse_args()

    logger.info("=" * 80)
    logger.info("Temporal Terrain NN Training")
    logger.info(f"Data: {args.data}")
    logger.info(f"K={args.K}, dt_nn={args.dt_nn}s, hidden={args.hidden}")
    input_dim = args.K * 5 + 6
    logger.info(f"Input dimension: {input_dim} = {args.K}*5 + 6")
    logger.info("=" * 80)

    trainer = TemporalNNTrainer(args.output_dir, args.device)
    trainer.load_and_build_windows(args.data, K=args.K, dt_nn=args.dt_nn,
                                   record_dt=args.record_dt)
    model, history = trainer.train_model(
        temporal_K=args.K, hidden_sizes=args.hidden,
        n_epochs=args.epochs, batch_size=args.batch_size,
        learning_rate=args.lr, patience=args.patience
    )
    test_m, y_test, y_pred = trainer.evaluate_model(model)
    trainer.plot_results(history, y_test, y_pred)

    logger.info("=" * 80)
    logger.info("Training complete!")
    logger.info(f"Model: {args.output_dir}/best_terrain_nn.pt")
    logger.info("=" * 80)


if __name__ == '__main__':
    main()
