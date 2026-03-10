#!/usr/bin/env python3
"""
Train small terrain neural network matching Dallas et al. paper architecture.

Dallas et al. "Terrain Adaptive Trajectory Planning and Tracking on Deformable Terrains"
Architecture: 2 hidden layers [12, 2] with tanh activation for twice-continuous differentiability.

This small network is designed for real-time inference performance.
"""

import os
import sys
import json
import pickle
import logging
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class TerrainNN(nn.Module):
    """
    Neural network for terrain force prediction.
    
    Dallas et al. architecture: 11 inputs -> 12 neurons -> 2 neurons -> 2 outputs
    Uses tanh activation for twice-continuous differentiability (required for ILC).
    """
    
    def __init__(self, input_size=11, output_size=2, hidden_sizes=None):
        super(TerrainNN, self).__init__()
        
        # Dallas paper: 2 hidden layers with 12 and 2 neurons
        if hidden_sizes is None:
            hidden_sizes = [12, 2]
        
        self.hidden_sizes = hidden_sizes
        sizes = hidden_sizes
        
        layers = []
        prev = input_size
        for h in sizes:
            layers.append(nn.Linear(prev, h))
            prev = h
        layers.append(nn.Linear(prev, output_size))
        self.layers = nn.ModuleList(layers)
        
        for layer in self.layers:
            nn.init.xavier_normal_(layer.weight)
    
    def forward(self, x):
        for layer in self.layers[:-1]:
            x = torch.tanh(layer(x))
        x = self.layers[-1](x)
        return x


class TerrainNNTrainer:
    """Handles training, validation, and evaluation of terrain neural network"""
    
    def __init__(self, data_file, output_dir='nn_models_v6_small', device='cpu'):
        self.data_file = data_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.device = torch.device(device)
        
        self.scaler_X = StandardScaler()
        self.scaler_y = StandardScaler()
        
        # Dallas paper architecture: [12, 2]
        self.hidden_sizes = [12, 2]
        
        logger.info(f"Using device: {self.device}")
        logger.info(f"Architecture: Dallas et al. [12, 2] for real-time inference")
    
    def load_and_preprocess_data(self):
        """Load data from CSV and split into train/val/test sets"""
        logger.info(f"Loading data from {self.data_file}...")
        df = pd.read_csv(self.data_file)
        
        logger.info(f"Loaded {len(df)} samples")
        logger.info(f"Columns: {df.columns.tolist()}")
        
        # Input features (11 inputs for SCM tire model) - matches Dallas et al. Table I
        # Operating params (5): slip_ratio, slip_angle, velocity, load, steering_rate
        # Terrain params (6): bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear
        
        # Handle different column naming conventions
        slip_ratio_col = 'slip_ratio' if 'slip_ratio' in df.columns else 'longitudinal_slip'
        load_col = 'vertical_load' if 'vertical_load' in df.columns else 'Fz'
        
        # Check for steering_rate (new Dallas format) vs camber_angle (old format)
        if 'steering_rate' in df.columns:
            # Dallas et al. format with steering_rate
            input_features = [
                slip_ratio_col, 'slip_angle', 'velocity', load_col, 'steering_rate',
                'bekker_Kphi', 'bekker_Kc', 'bekker_n',
                'mohr_cohesion', 'mohr_friction', 'janosi_shear'
            ]
            logger.info("Using Dallas et al. format with steering_rate")
        else:
            # Legacy format with camber_angle (for backwards compatibility)
            input_features = [
                load_col, 'slip_angle', slip_ratio_col, 'camber_angle', 'velocity',
                'bekker_Kphi', 'bekker_Kc', 'bekker_n',
                'mohr_cohesion', 'mohr_friction', 'janosi_shear'
            ]
            logger.info("Using legacy format with camber_angle")
        
        # Output targets (lateral and longitudinal forces)
        output_features = ['Fx', 'Fy']
        
        X = df[input_features].values
        y = df[output_features].values
        
        # Remove any NaN or inf values
        valid_mask = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
        X = X[valid_mask]
        y = y[valid_mask]
        
        logger.info(f"Valid samples after filtering: {len(X)}")
        logger.info(f"Input shape: {X.shape}, Output shape: {y.shape}")
        
        # Split data: 70% train, 15% val, 15% test
        X_train, X_temp, y_train, y_temp = train_test_split(
            X, y, test_size=0.3, random_state=42
        )
        X_val, X_test, y_val, y_test = train_test_split(
            X_temp, y_temp, test_size=0.5, random_state=42
        )
        
        logger.info(f"Train samples: {len(X_train)}")
        logger.info(f"Val samples: {len(X_val)}")
        logger.info(f"Test samples: {len(X_test)}")
        
        # Normalize data
        X_train = self.scaler_X.fit_transform(X_train)
        X_val = self.scaler_X.transform(X_val)
        X_test = self.scaler_X.transform(X_test)
        
        y_train = self.scaler_y.fit_transform(y_train)
        y_val = self.scaler_y.transform(y_val)
        y_test = self.scaler_y.transform(y_test)
        
        # Convert to PyTorch tensors
        self.X_train = torch.FloatTensor(X_train).to(self.device)
        self.y_train = torch.FloatTensor(y_train).to(self.device)
        self.X_val = torch.FloatTensor(X_val).to(self.device)
        self.y_val = torch.FloatTensor(y_val).to(self.device)
        self.X_test = torch.FloatTensor(X_test).to(self.device)
        self.y_test = torch.FloatTensor(y_test).to(self.device)
        
        # Save scalers
        scaler_path = self.output_dir / 'scalers.pkl'
        with open(scaler_path, 'wb') as f:
            pickle.dump({'X': self.scaler_X, 'y': self.scaler_y}, f)
        logger.info(f"Scalers saved to {scaler_path}")
        
        return X_train, y_train, X_val, y_val, X_test, y_test
    
    def train_model(self, n_epochs=1000, batch_size=128, learning_rate=0.01, patience=50):
        """Train neural network with early stopping"""
        logger.info("="*80)
        logger.info("Training Small Neural Network (Dallas et al. Architecture)")
        logger.info(f"Epochs: {n_epochs}, Batch size: {batch_size}, LR: {learning_rate}")
        logger.info("="*80)
        
        # Create data loaders
        train_dataset = TensorDataset(self.X_train, self.y_train)
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        
        input_size = self.X_train.shape[1]
        output_size = self.y_train.shape[1]
        model = TerrainNN(input_size=input_size, output_size=output_size,
                          hidden_sizes=self.hidden_sizes).to(self.device)
        
        total_params = sum(p.numel() for p in model.parameters())
        logger.info(f"Architecture: {input_size} -> {' -> '.join(map(str, self.hidden_sizes))} -> {output_size}")
        logger.info(f"Total parameters: {total_params} (optimized for real-time inference)")
        
        # Loss function and optimizer
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=learning_rate, weight_decay=1e-4)
        
        # Learning rate scheduler
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, mode='min', factor=0.5, patience=20
        )
        
        # Training history
        history = {
            'train_loss': [],
            'val_loss': [],
            'learning_rate': []
        }
        
        best_val_loss = float('inf')
        patience_counter = 0
        best_model_state = None
        
        for epoch in range(n_epochs):
            # Training phase
            model.train()
            train_losses = []
            
            for batch_X, batch_y in train_loader:
                optimizer.zero_grad()
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                loss.backward()
                optimizer.step()
                train_losses.append(loss.item())
            
            avg_train_loss = np.mean(train_losses)
            
            # Validation phase
            model.eval()
            with torch.no_grad():
                val_outputs = model(self.X_val)
                val_loss = criterion(val_outputs, self.y_val).item()
            
            # Update learning rate
            scheduler.step(val_loss)
            current_lr = optimizer.param_groups[0]['lr']
            
            # Save history
            history['train_loss'].append(avg_train_loss)
            history['val_loss'].append(val_loss)
            history['learning_rate'].append(current_lr)
            
            # Early stopping check
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                patience_counter = 0
                best_model_state = model.state_dict().copy()
                
                model_path = self.output_dir / 'best_terrain_nn.pt'
                torch.save({
                    'epoch': epoch,
                    'model_state_dict': model.state_dict(),
                    'optimizer_state_dict': optimizer.state_dict(),
                    'train_loss': avg_train_loss,
                    'val_loss': val_loss,
                    'hidden_sizes': self.hidden_sizes,
                }, model_path)
            else:
                patience_counter += 1
            
            # Logging
            if (epoch + 1) % 50 == 0 or epoch == 0:
                logger.info(f"Epoch [{epoch+1}/{n_epochs}] "
                           f"Train Loss: {avg_train_loss:.6f}, "
                           f"Val Loss: {val_loss:.6f}, "
                           f"LR: {current_lr:.6f}")
            
            # Early stopping
            if patience_counter >= patience:
                logger.info(f"Early stopping triggered at epoch {epoch+1}")
                break
        
        # Load best model
        model.load_state_dict(best_model_state)
        logger.info(f"Best model achieved at validation loss: {best_val_loss:.6f}")
        
        return model, history
    
    def evaluate_model(self, model):
        """Evaluate model on test set"""
        logger.info("="*80)
        logger.info("Evaluating Model on Test Set")
        logger.info("="*80)
        
        model.eval()
        with torch.no_grad():
            y_pred_train = model(self.X_train).cpu().numpy()
            y_pred_val = model(self.X_val).cpu().numpy()
            y_pred_test = model(self.X_test).cpu().numpy()
        
        # Convert back to original scale
        y_train_orig = self.scaler_y.inverse_transform(self.y_train.cpu().numpy())
        y_val_orig = self.scaler_y.inverse_transform(self.y_val.cpu().numpy())
        y_test_orig = self.scaler_y.inverse_transform(self.y_test.cpu().numpy())
        
        y_pred_train = self.scaler_y.inverse_transform(y_pred_train)
        y_pred_val = self.scaler_y.inverse_transform(y_pred_val)
        y_pred_test = self.scaler_y.inverse_transform(y_pred_test)
        
        # Calculate metrics
        def calc_metrics(y_true, y_pred):
            mse = np.mean((y_true - y_pred)**2, axis=0)
            rmse = np.sqrt(mse)
            mae = np.mean(np.abs(y_true - y_pred), axis=0)
            
            # R² score
            ss_res = np.sum((y_true - y_pred)**2, axis=0)
            ss_tot = np.sum((y_true - np.mean(y_true, axis=0))**2, axis=0)
            r2 = 1 - (ss_res / ss_tot)
            
            return {'mse': mse, 'rmse': rmse, 'mae': mae, 'r2': r2}
        
        train_metrics = calc_metrics(y_train_orig, y_pred_train)
        val_metrics = calc_metrics(y_val_orig, y_pred_val)
        test_metrics = calc_metrics(y_test_orig, y_pred_test)
        
        # Log results
        force_names = ['Fx', 'Fy']
        
        logger.info("\nTrain Set Metrics:")
        for i, name in enumerate(force_names):
            logger.info(f"  {name}: RMSE={train_metrics['rmse'][i]:.2f}N, "
                       f"MAE={train_metrics['mae'][i]:.2f}N, "
                       f"R²={train_metrics['r2'][i]:.4f}")
        
        logger.info("\nValidation Set Metrics:")
        for i, name in enumerate(force_names):
            logger.info(f"  {name}: RMSE={val_metrics['rmse'][i]:.2f}N, "
                       f"MAE={val_metrics['mae'][i]:.2f}N, "
                       f"R²={val_metrics['r2'][i]:.4f}")
        
        logger.info("\nTest Set Metrics:")
        for i, name in enumerate(force_names):
            logger.info(f"  {name}: RMSE={test_metrics['rmse'][i]:.2f}N, "
                       f"MAE={test_metrics['mae'][i]:.2f}N, "
                       f"R²={test_metrics['r2'][i]:.4f}")
        
        # Save metrics
        metrics = {
            'train': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                     for k, v in train_metrics.items()},
            'val': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                   for k, v in val_metrics.items()},
            'test': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                    for k, v in test_metrics.items()},
            'architecture': {
                'hidden_sizes': self.hidden_sizes,
                'total_params': sum(p.numel() for p in TerrainNN(hidden_sizes=self.hidden_sizes).parameters())
            }
        }
        
        metrics_path = self.output_dir / 'test_metrics.json'
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=2)
        logger.info(f"\nMetrics saved to {metrics_path}")
        
        return test_metrics, y_test_orig, y_pred_test
    
    def plot_results(self, history, y_test, y_pred_test):
        """Generate visualization plots"""
        logger.info("Generating plots...")
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Plot 1: Training history
        ax = axes[0, 0]
        ax.plot(history['train_loss'], label='Train Loss', alpha=0.8)
        ax.plot(history['val_loss'], label='Val Loss', alpha=0.8)
        ax.set_xlabel('Epoch')
        ax.set_ylabel('MSE Loss')
        ax.set_title('Training History (Small Network)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_yscale('log')
        
        # Plot 2: Learning rate
        ax = axes[0, 1]
        ax.plot(history['learning_rate'], color='green')
        ax.set_xlabel('Epoch')
        ax.set_ylabel('Learning Rate')
        ax.set_title('Learning Rate Schedule')
        ax.grid(True, alpha=0.3)
        ax.set_yscale('log')
        
        # Plot 3: Fx prediction vs true
        ax = axes[1, 0]
        ax.scatter(y_test[:, 0], y_pred_test[:, 0], alpha=0.5, s=10)
        ax.plot([y_test[:, 0].min(), y_test[:, 0].max()],
                [y_test[:, 0].min(), y_test[:, 0].max()],
                'r--', lw=2, label='Perfect Prediction')
        ax.set_xlabel('True Fx (N)')
        ax.set_ylabel('Predicted Fx (N)')
        ax.set_title('Longitudinal Force Prediction')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Plot 4: Fy prediction vs true
        ax = axes[1, 1]
        ax.scatter(y_test[:, 1], y_pred_test[:, 1], alpha=0.5, s=10)
        ax.plot([y_test[:, 1].min(), y_test[:, 1].max()],
                [y_test[:, 1].min(), y_test[:, 1].max()],
                'r--', lw=2, label='Perfect Prediction')
        ax.set_xlabel('True Fy (N)')
        ax.set_ylabel('Predicted Fy (N)')
        ax.set_title('Lateral Force Prediction')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        fig.suptitle(f'Dallas et al. Architecture: 11 → {" → ".join(map(str, self.hidden_sizes))} → 2', 
                     y=1.02, fontsize=12)
        plot_path = self.output_dir / 'training_results.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        logger.info(f"Plots saved to {plot_path}")
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Train small terrain neural network (Dallas et al. architecture [12,2])'
    )
    parser.add_argument('--data', type=str, required=True, help='Path to training data CSV')
    parser.add_argument('--output_dir', type=str, default='../nn_models_v6_small', 
                       help='Output directory (default: ../nn_models_v6_small)')
    parser.add_argument('--epochs', type=int, default=1000, help='Number of epochs')
    parser.add_argument('--batch_size', type=int, default=128, help='Batch size')
    parser.add_argument('--lr', type=float, default=0.01, help='Learning rate')
    parser.add_argument('--patience', type=int, default=50, help='Early stopping patience')
    parser.add_argument('--device', type=str, default='cpu', 
                       help='Device to use (cpu or cuda)')
    
    args = parser.parse_args()
    
    logger.info("="*80)
    logger.info("Small Terrain Neural Network Training")
    logger.info("Dallas et al. Architecture: [12, 2] hidden layers")
    logger.info(f"Data file: {args.data}")
    logger.info(f"Output directory: {args.output_dir}")
    logger.info("="*80)
    
    trainer = TerrainNNTrainer(args.data, args.output_dir, args.device)
    
    # Load and preprocess data
    trainer.load_and_preprocess_data()
    
    # Train model
    model, history = trainer.train_model(
        n_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        patience=args.patience
    )
    
    # Evaluate model
    test_metrics, y_test, y_pred_test = trainer.evaluate_model(model)
    
    # Generate plots
    trainer.plot_results(history, y_test, y_pred_test)
    
    logger.info("="*80)
    logger.info("Training Complete!")
    logger.info(f"Model saved to: {args.output_dir}/best_terrain_nn.pt")
    logger.info("Small network optimized for real-time inference")
    logger.info("="*80)


if __name__ == '__main__':
    main()
