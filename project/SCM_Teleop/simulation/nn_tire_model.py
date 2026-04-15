#!/usr/bin/env python3
"""
Unified Neural Network Tire Model Interface
=============================================

Provides a single entry point for loading any NN tire model variant and
building a CasADi symbolic forward pass for MPC embedding.  Model type is
auto-detected from the checkpoint metadata **and** from a standardised
directory naming convention:

Naming convention
-----------------
    {prefix}_{arch}_{spec}

    arch   := mlp | resnet
    spec   := {hidden_desc}                           (static)
            | temporal_K{K}_{hidden_desc}             (temporal)
            | rate_{hidden_desc}                      (rate-augmented)

    hidden_desc (MLP)   : e.g. 16_4 meaning [16, 4]
    hidden_desc (ResNet) : e.g. h16_b2 meaning hidden_dim=16, n_blocks=2

Examples
--------
    v6_mlp_16_4                    → static MLP  [16, 4]
    v6_mlp_temporal_K3_16_8        → temporal MLP K=3 [16, 8]
    v6_mlp_rate_16_8               → rate-augmented MLP [16, 8]
    v6_resnet_h16_b2               → static ResNet  h=16, blocks=2
    v6_resnet_temporal_K5_h32_b2   → temporal ResNet K=5 h=32
    sweep_mlp_16_4                 → (also accepted; prefix is free-form)

The loader will always cross-check the name against the checkpoint metadata
and warn on mismatch, but the **checkpoint is authoritative**.

Public API
----------
    model = load_nn_tire_model(model_dir, terrain_params)

    # For single-sample symbolic evaluation:
    Fx, Fy = model.predict(alpha, Fz, u, ...)

    # For batched symbolic evaluation inside MPC:
    Fxs, Fys = model.predict_batch(...)

    # For numeric (non-symbolic) evaluation:
    Fx, Fy = model.predict_numeric(alpha, Fz, u, ...)

    # Metadata:
    model.model_type      # 'static_mlp' | 'temporal_mlp' | 'rate_mlp'
                          # | 'static_resnet' | 'temporal_resnet'
    model.temporal_K      # 1 for static/rate, >1 for temporal
    model.rate_augmented  # True/False
    model.input_dim       # Total NN input features
    model.n_params        # Trainable parameter count
    model.model_format    # 'v6' | 'v6_temporal' | 'v8_rate'
"""

from __future__ import annotations

import re
import pickle
from pathlib import Path
from abc import ABC, abstractmethod

import numpy as np
import casadi as ca
import torch


# ============================================================================
# Naming convention parser
# ============================================================================

def parse_model_name(name: str) -> dict:
    """Parse a standardised model directory name into metadata.

    Returns dict with keys: arch, hidden_sizes | (hidden_dim, n_blocks),
    temporal_K, rate_augmented.  Values may be None if not parseable
    (the checkpoint metadata is still authoritative).
    """
    info: dict = {
        'arch': None,
        'hidden_sizes': None,
        'hidden_dim': None,
        'n_blocks': None,
        'temporal_K': 1,
        'rate_augmented': False,
    }

    # --- Detect architecture ---
    if '_resnet_' in name or name.startswith('resnet_'):
        info['arch'] = 'resnet'
    elif '_densenet_' in name or name.startswith('densenet_'):
        info['arch'] = 'densenet'
    elif '_mlp_' in name or name.startswith('mlp_'):
        info['arch'] = 'mlp'

    # --- Detect temporal ---
    m = re.search(r'temporal_K(\d+)', name, re.IGNORECASE)
    if m:
        info['temporal_K'] = int(m.group(1))

    # --- Detect rate ---
    if '_rate_' in name:
        info['rate_augmented'] = True

    # --- Hidden sizes for MLP (e.g. 16_4, 16_8, 24_12) ---
    if info['arch'] == 'mlp':
        # Match trailing integers separated by underscores (after any prefix/temporal/rate tag)
        # Strategy: split on '_', take the trailing numeric groups
        parts = name.split('_')
        nums = []
        for p in reversed(parts):
            if p.isdigit():
                nums.insert(0, int(p))
            else:
                break
        if len(nums) >= 2:
            info['hidden_sizes'] = nums

    # --- Hidden dim / n_blocks for ResNet (e.g. h16_b2) ---
    if info['arch'] == 'resnet':
        m_h = re.search(r'h(\d+)', name)
        m_b = re.search(r'b(\d+)', name)
        if m_h:
            info['hidden_dim'] = int(m_h.group(1))
        if m_b:
            info['n_blocks'] = int(m_b.group(1))

    return info


# ============================================================================
# Base class
# ============================================================================

class NNTireModel(ABC):
    """Abstract base for all NN tire model variants."""

    # Metadata set by subclasses
    model_type: str            # e.g. 'static_mlp'
    temporal_K: int = 1
    rate_augmented: bool = False
    model_format: str = 'v6'   # 'v6' | 'v6_temporal' | 'v8_rate'
    input_dim: int = 11
    n_params: int = 0
    n_nominal: float = 1.1     # Bekker sinkage exponent

    # Internal
    _weights: dict              # name → numpy array
    _X_mean: np.ndarray
    _X_scale: np.ndarray
    _y_mean: np.ndarray
    _y_scale: np.ndarray
    _terrain_nominals: dict     # {Kphi, Kc, c, phi, k}

    # CasADi functions (built by subclass)
    predict_tire_force: ca.Function   # Scalar symbolic
    _BATCH: int = 8

    def __init__(self, model_dir: str | Path, terrain_params: dict):
        """
        Args:
            model_dir:  Directory containing best_terrain_nn.pt + scalers.pkl
            terrain_params: Dict with keys Kphi, Kc, n, c, phi (degrees), k
        """
        model_dir = Path(model_dir)
        model_path = model_dir / 'best_terrain_nn.pt'
        scaler_path = model_dir / 'scalers.pkl'

        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
        state_dict = checkpoint['model_state_dict'] if isinstance(checkpoint, dict) else checkpoint

        # Remap old-style keys (layer1.weight → layers.0.weight)
        if any(k.startswith('layer') and not k.startswith('layers') for k in state_dict):
            remap = {}
            idx = 0
            while f'layer{idx+1}.weight' in state_dict:
                remap[f'layer{idx+1}.weight'] = f'layers.{idx}.weight'
                remap[f'layer{idx+1}.bias'] = f'layers.{idx}.bias'
                idx += 1
            state_dict = {remap.get(k, k): v for k, v in state_dict.items()}

        self._weights = {k: v.detach().numpy() for k, v in state_dict.items()}

        # Scalers
        with open(scaler_path, 'rb') as f:
            scalers = pickle.load(f)
        self._X_mean = scalers['X'].mean_
        self._X_scale = scalers['X'].scale_
        self._y_mean = scalers['y'].mean_
        self._y_scale = scalers['y'].scale_

        # Terrain nominals
        self.n_nominal = terrain_params.get('n', 1.1)
        self._terrain_nominals = {
            'Kphi': terrain_params['Kphi'],
            'Kc': terrain_params['Kc'],
            'c': terrain_params['c'],
            'phi': terrain_params['phi'],
            'k': terrain_params['k'],
        }

        # Count params
        self.n_params = sum(v.size for v in self._weights.values())

        # Subclass fills the rest via _build()
        self._checkpoint = checkpoint
        self._build()

    @abstractmethod
    def _build(self):
        """Build CasADi symbolic functions. Must set predict_tire_force and predict_batch*."""
        ...

    # ---- convenience wrappers -----------------------------------------------

    def predict(self, alpha, Fz, u, kappa=0.0, n_terrain=None, steering_rate=0.0,
                terrain_params=None, hist=None, rates=None):
        """Single-sample symbolic or numeric evaluation.

        Dispatches to predict_tire_force with correct argument packing.
        """
        if n_terrain is None:
            n_terrain = self.n_nominal
        tp = terrain_params if terrain_params is not None else self._terrain_nominals
        phi_val = np.radians(tp['phi']) if self.model_format in ('v6', 'v6_temporal', 'v8_rate') else tp['phi']

        args = [alpha, Fz, u, kappa, n_terrain, steering_rate]
        if self.temporal_K > 1:
            h = hist if hist is not None else np.zeros((self.temporal_K - 1) * 5)
            args.append(h)
        if self.rate_augmented:
            r = rates if rates is not None else np.zeros(3)
            args.append(r)
        args += [tp['Kphi'], tp['Kc'], tp['c'], phi_val, tp['k']]

        Fx, Fy = self.predict_tire_force(*args)
        return Fx, Fy

    def predict_numeric(self, alpha, Fz, u, kappa=0.0, n_terrain=None,
                        steering_rate=0.0, terrain_params=None, hist=None, rates=None):
        """Numeric (float) convenience wrapper around predict()."""
        Fx, Fy = self.predict(alpha, Fz, u, kappa, n_terrain, steering_rate,
                              terrain_params, hist, rates)
        return float(Fx), float(Fy)


# ============================================================================
# MLP forward-pass helpers (shared by Static / Temporal / Rate)
# ============================================================================

def _mlp_layer_indices(weights: dict) -> list[int]:
    """Discover layer indices from weight dict keys like layers.0.weight."""
    return sorted(set(
        int(k.split('.')[1]) for k in weights if k.startswith('layers.') and 'weight' in k
    ))


def _mlp_forward_casadi(weights: dict, x_scaled, layer_indices: list[int], ncols: int = 1):
    """Evaluate an MLP [layers.{i}.{weight,bias}] in CasADi symbols."""
    h = x_scaled
    for i in layer_indices:
        W = ca.DM(weights[f'layers.{i}.weight'])
        b = ca.DM(weights[f'layers.{i}.bias']).reshape((-1, 1))
        if ncols > 1:
            h = ca.mtimes(W, h) + ca.repmat(b, 1, ncols)
        else:
            h = ca.mtimes(W, h) + b
        if i < layer_indices[-1]:
            h = ca.tanh(h)
    return h


# ============================================================================
# ResNet forward-pass helper
# ============================================================================

def _resnet_forward_casadi(weights: dict, n_blocks: int, x_scaled, ncols: int = 1):
    """Evaluate a ResNet (input_proj → blocks → output_proj) in CasADi symbols."""
    Wi = ca.DM(weights['input_proj.weight'])
    bi = ca.DM(weights['input_proj.bias']).reshape((-1, 1))
    if ncols > 1:
        h = ca.tanh(ca.mtimes(Wi, x_scaled) + ca.repmat(bi, 1, ncols))
    else:
        h = ca.tanh(ca.mtimes(Wi, x_scaled) + bi)

    for blk in range(n_blocks):
        residual = h
        W1 = ca.DM(weights[f'blocks.{blk}.fc1.weight'])
        b1 = ca.DM(weights[f'blocks.{blk}.fc1.bias']).reshape((-1, 1))
        W2 = ca.DM(weights[f'blocks.{blk}.fc2.weight'])
        b2 = ca.DM(weights[f'blocks.{blk}.fc2.bias']).reshape((-1, 1))
        if ncols > 1:
            h = ca.tanh(ca.mtimes(W1, h) + ca.repmat(b1, 1, ncols))
            h = ca.mtimes(W2, h) + ca.repmat(b2, 1, ncols)
        else:
            h = ca.tanh(ca.mtimes(W1, h) + b1)
            h = ca.mtimes(W2, h) + b2
        h = ca.tanh(h + residual)

    Wo = ca.DM(weights['output_proj.weight'])
    bo = ca.DM(weights['output_proj.bias']).reshape((-1, 1))
    if ncols > 1:
        return ca.mtimes(Wo, h) + ca.repmat(bo, 1, ncols)
    else:
        return ca.mtimes(Wo, h) + bo


# ============================================================================
# DenseNet forward-pass helper
# ============================================================================

def _densenet_forward_casadi(weights: dict, n_dense_layers: int, x_scaled, ncols: int = 1):
    """Evaluate a DenseNet (input_proj → dense_layers with concat → output_proj) in CasADi."""
    Wi = ca.DM(weights['input_proj.weight'])
    bi = ca.DM(weights['input_proj.bias']).reshape((-1, 1))
    if ncols > 1:
        h = ca.tanh(ca.mtimes(Wi, x_scaled) + ca.repmat(bi, 1, ncols))
    else:
        h = ca.tanh(ca.mtimes(Wi, x_scaled) + bi)

    features = [h]
    for i in range(n_dense_layers):
        concat = ca.vertcat(*features)
        W = ca.DM(weights[f'dense_layers.{i}.weight'])
        b = ca.DM(weights[f'dense_layers.{i}.bias']).reshape((-1, 1))
        if ncols > 1:
            h = ca.tanh(ca.mtimes(W, concat) + ca.repmat(b, 1, ncols))
        else:
            h = ca.tanh(ca.mtimes(W, concat) + b)
        features.append(h)

    final_concat = ca.vertcat(*features)
    Wo = ca.DM(weights['output_proj.weight'])
    bo = ca.DM(weights['output_proj.bias']).reshape((-1, 1))
    if ncols > 1:
        return ca.mtimes(Wo, final_concat) + ca.repmat(bo, 1, ncols)
    else:
        return ca.mtimes(Wo, final_concat) + bo


# ============================================================================
# Concrete model classes
# ============================================================================

class StaticMLP(NNTireModel):
    """Static MLP tire model (11 inputs → 2 outputs)."""

    model_type = 'static_mlp'
    model_format = 'v6'

    def _build(self):
        self.input_dim = 11
        li = _mlp_layer_indices(self._weights)

        # --- scalar symbolic function ---
        alpha = ca.SX.sym('alpha')
        Fz = ca.SX.sym('Fz')
        u = ca.SX.sym('u')
        kappa = ca.SX.sym('kappa')
        n_t = ca.SX.sym('n_terrain')
        sr = ca.SX.sym('sr')
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        x_in = ca.vertcat(kappa, alpha, u, Fz, sr, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _mlp_forward_casadi(self._weights, x_s, li)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire', [alpha, Fz, u, kappa, n_t, sr, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # --- batched function (8 samples) ---
        self._build_batch(li)

    def _build_batch(self, li):
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us.T, Fzs.T, srs.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X = ca.vertcat(*rows)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        H = (X - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _mlp_forward_casadi(self._weights, H, li, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch = ca.Function(
            'nn_tire_batch',
            [alphas, Fzs, us, kappas, n_ts, srs, Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])


class TemporalMLP(NNTireModel):
    """Temporal MLP: K sliding-window observations + terrain → Fx, Fy."""

    model_type = 'temporal_mlp'
    model_format = 'v6_temporal'

    def _build(self):
        ckpt = self._checkpoint
        self.temporal_K = ckpt.get('temporal_K', 3) if isinstance(ckpt, dict) else 3
        self.input_dim = self.temporal_K * 5 + 6
        li = _mlp_layer_indices(self._weights)

        # scalar function
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        hist = ca.SX.sym('hist', (self.temporal_K - 1) * 5)
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        cur = ca.vertcat(kap, alpha, u, Fz, sr)
        ter = ca.vertcat(Kphi, Kc, n_t, c, phi, k)
        x_in = ca.vertcat(cur, hist, ter)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _mlp_forward_casadi(self._weights, x_s, li)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_temporal',
            [alpha, Fz, u, kap, n_t, sr, hist, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'history',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched temporal: accepts [input_dim x 8] matrix
        B = 8
        X_batch = ca.SX.sym('X_batch', self.input_dim, B)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        Hb = (X_batch - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _mlp_forward_casadi(self._weights, Hb, li, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Yb = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch_temporal = ca.Function(
            'nn_tire_batch_temporal', [X_batch], [Yb[0, :].T, Yb[1, :].T],
            ['X_batch'], ['Fxs', 'Fys'])
        self.predict_batch = None


class RateMLP(NNTireModel):
    """Rate-augmented MLP: 14 inputs (5 ops + 3 rates + 6 terrain)."""

    model_type = 'rate_mlp'
    model_format = 'v8_rate'
    rate_augmented = True

    def _build(self):
        self.input_dim = 14
        li = _mlp_layer_indices(self._weights)

        # scalar
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        rates = ca.SX.sym('rates', 3)  # [dκ, dα, du]
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        x_in = ca.vertcat(kap, alpha, u, Fz, sr, rates, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _mlp_forward_casadi(self._weights, x_s, li)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_rate',
            [alpha, Fz, u, kap, n_t, sr, rates, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'rates',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        dk = ca.SX.sym('dk', B); da = ca.SX.sym('da', B); du = ca.SX.sym('du', B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us.T, Fzs.T, srs.T,
                dk.T, da.T, du.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X = ca.vertcat(*rows)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        H = (X - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _mlp_forward_casadi(self._weights, H, li, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch_rate = ca.Function(
            'nn_tire_batch_rate',
            [alphas, Fzs, us, kappas, n_ts, srs, dk, da, du,
             Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'dk', 'da', 'du',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])
        self.predict_batch = None


class StaticResNet(NNTireModel):
    """Static ResNet tire model."""

    model_type = 'static_resnet'
    model_format = 'v6'

    def _build(self):
        ckpt = self._checkpoint
        self._hidden_dim = ckpt.get('hidden_dim', 16) if isinstance(ckpt, dict) else 16
        self._n_blocks = ckpt.get('n_blocks', 2) if isinstance(ckpt, dict) else 2
        self.input_dim = 11

        # scalar
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        x_in = ca.vertcat(kap, alpha, u, Fz, sr, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _resnet_forward_casadi(self._weights, self._n_blocks, x_s)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_resnet', [alpha, Fz, u, kap, n_t, sr, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us_ = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us_.T, Fzs.T, srs.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X = ca.vertcat(*rows)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        H = (X - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _resnet_forward_casadi(self._weights, self._n_blocks, H, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch = ca.Function(
            'nn_tire_batch_resnet',
            [alphas, Fzs, us_, kappas, n_ts, srs, Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])


class TemporalResNet(NNTireModel):
    """Temporal ResNet: K-window, ResNet backbone."""

    model_type = 'temporal_resnet'
    model_format = 'v6_temporal'

    def _build(self):
        ckpt = self._checkpoint
        self.temporal_K = ckpt.get('temporal_K', 3) if isinstance(ckpt, dict) else 3
        self._hidden_dim = ckpt.get('hidden_dim', 16) if isinstance(ckpt, dict) else 16
        self._n_blocks = ckpt.get('n_blocks', 2) if isinstance(ckpt, dict) else 2
        self.input_dim = self.temporal_K * 5 + 6

        # scalar
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        hist = ca.SX.sym('hist', (self.temporal_K - 1) * 5)
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        cur = ca.vertcat(kap, alpha, u, Fz, sr)
        ter = ca.vertcat(Kphi, Kc, n_t, c, phi, k)
        x_in = ca.vertcat(cur, hist, ter)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _resnet_forward_casadi(self._weights, self._n_blocks, x_s)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_resnet_temporal',
            [alpha, Fz, u, kap, n_t, sr, hist, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'history',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched temporal
        B = 8
        X_batch = ca.SX.sym('X_batch', self.input_dim, B)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        Hb = (X_batch - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _resnet_forward_casadi(self._weights, self._n_blocks, Hb, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Yb = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch_temporal = ca.Function(
            'nn_tire_batch_resnet_temporal', [X_batch], [Yb[0, :].T, Yb[1, :].T],
            ['X_batch'], ['Fxs', 'Fys'])
        self.predict_batch = None


# ============================================================================
# Rate-augmented ResNet (new)
# ============================================================================

class RateResNet(NNTireModel):
    """Rate-augmented ResNet: 14 inputs (5 ops + 3 rates + 6 terrain) → Fx, Fy."""

    model_type = 'rate_resnet'
    model_format = 'v8_rate'
    rate_augmented = True

    def _build(self):
        ckpt = self._checkpoint
        self._hidden_dim = ckpt.get('hidden_dim', 16) if isinstance(ckpt, dict) else 16
        self._n_blocks = ckpt.get('n_blocks', 2) if isinstance(ckpt, dict) else 2
        self.input_dim = 14

        # scalar
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        rates = ca.SX.sym('rates', 3)  # [dκ, dα, du]
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        x_in = ca.vertcat(kap, alpha, u, Fz, sr, rates, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _resnet_forward_casadi(self._weights, self._n_blocks, x_s)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_rate_resnet',
            [alpha, Fz, u, kap, n_t, sr, rates, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'rates',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        dk = ca.SX.sym('dk', B); da = ca.SX.sym('da', B); du = ca.SX.sym('du', B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us.T, Fzs.T, srs.T,
                dk.T, da.T, du.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X = ca.vertcat(*rows)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        H = (X - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _resnet_forward_casadi(self._weights, self._n_blocks, H, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch_rate = ca.Function(
            'nn_tire_batch_rate_resnet',
            [alphas, Fzs, us, kappas, n_ts, srs, dk, da, du,
             Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'dk', 'da', 'du',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])
        self.predict_batch = None


# ============================================================================
# DenseNet concrete class
# ============================================================================

class StaticDenseNet(NNTireModel):
    """Static DenseNet tire model (11 inputs → 2 outputs)."""

    model_type = 'static_densenet'
    model_format = 'v6'

    def _build(self):
        ckpt = self._checkpoint
        self._dense_dim = ckpt.get('dense_dim', 32) if isinstance(ckpt, dict) else 32
        self._n_dense_layers = ckpt.get('n_dense_layers', 4) if isinstance(ckpt, dict) else 4
        self.input_dim = 11

        # scalar
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kap = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain'); sr = ca.SX.sym('sr')
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')

        x_in = ca.vertcat(kap, alpha, u, Fz, sr, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_in - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        y_s = _densenet_forward_casadi(self._weights, self._n_dense_layers, x_s)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_densenet', [alpha, Fz, u, kap, n_t, sr, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # batched
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us_ = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us_.T, Fzs.T, srs.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X = ca.vertcat(*rows)
        Xm = ca.DM(self._X_mean.reshape(-1, 1)); Xs = ca.DM(self._X_scale.reshape(-1, 1))
        H = (X - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Y_s = _densenet_forward_casadi(self._weights, self._n_dense_layers, H, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1)); ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch = ca.Function(
            'nn_tire_batch_densenet',
            [alphas, Fzs, us_, kappas, n_ts, srs, Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])


# ============================================================================
# GRU latent-state observer + MLP decoder
# ============================================================================

class GRUObserverMLP(NNTireModel):
    """GRU latent-state observer with MLP decoder.

    The GRU processes scaled observations at runtime (PyTorch / numpy) to
    produce a latent state h ∈ R^{h_dim}.  The MLP decoder is embedded in
    CasADi and takes [x_scaled(11), h(h_dim)] → [Fx, Fy].

    Inside the MPC horizon h is *frozen* (constant parameter); the GRU is
    only stepped in the controller at each real-time cycle.
    """

    model_type = 'gru_observer_mlp'
    model_format = 'gru_observer'

    # Populated during _build()
    gru_h_dim: int = 0

    def _build(self):
        ckpt = self._checkpoint
        self.gru_h_dim = ckpt['gru_h_dim']
        self.input_dim = ckpt.get('input_size', 11 + self.gru_h_dim)
        self._gru_input_dim = ckpt.get('gru_input_size', 11)
        li = _mlp_layer_indices(self._weights)

        # GRU weights (numpy, for runtime gru_step)
        self._gru_weights = {
            k: v.detach().numpy() for k, v in ckpt['gru_state_dict'].items()
        }
        self.n_params += sum(v.size for v in self._gru_weights.values())

        h_dim = self.gru_h_dim

        # --- Scalar symbolic function ---
        alpha = ca.SX.sym('alpha'); Fz = ca.SX.sym('Fz'); u = ca.SX.sym('u')
        kappa = ca.SX.sym('kappa'); n_t = ca.SX.sym('n_terrain')
        sr = ca.SX.sym('sr')
        Kphi = ca.SX.sym('Kphi'); Kc = ca.SX.sym('Kc')
        c = ca.SX.sym('c'); phi = ca.SX.sym('phi'); k = ca.SX.sym('k')
        h_vec = ca.SX.sym('h', h_dim)

        x_ops = ca.vertcat(kappa, alpha, u, Fz, sr, Kphi, Kc, n_t, c, phi, k)
        x_s = (x_ops - self._X_mean.reshape(-1, 1)) / self._X_scale.reshape(-1, 1)
        dec_in = ca.vertcat(x_s, h_vec)
        y_s = _mlp_forward_casadi(self._weights, dec_in, li)
        y_out = y_s * self._y_scale.reshape(-1, 1) + self._y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_gru',
            [alpha, Fz, u, kappa, n_t, sr, h_vec, Kphi, Kc, c, phi, k],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'sr', 'h',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fx', 'Fy'])

        # --- Batched function (8 samples) ---
        B = 8
        alphas = ca.SX.sym('alphas', B); Fzs = ca.SX.sym('Fzs', B)
        us = ca.SX.sym('us', B); kappas = ca.SX.sym('kappas', B)
        n_ts = ca.SX.sym('n_ts', B); srs = ca.SX.sym('srs', B)
        h_mat = ca.SX.sym('h_mat', h_dim, B)
        Kphi_b = ca.SX.sym('Kphi'); Kc_b = ca.SX.sym('Kc')
        c_b = ca.SX.sym('c'); phi_b = ca.SX.sym('phi'); k_b = ca.SX.sym('k')

        rows = [kappas.T, alphas.T, us.T, Fzs.T, srs.T,
                ca.repmat(Kphi_b, 1, B), ca.repmat(Kc_b, 1, B), n_ts.T,
                ca.repmat(c_b, 1, B), ca.repmat(phi_b, 1, B), ca.repmat(k_b, 1, B)]
        X_ops = ca.vertcat(*rows)                         # (11, B)
        Xm = ca.DM(self._X_mean.reshape(-1, 1))
        Xs = ca.DM(self._X_scale.reshape(-1, 1))
        X_s = (X_ops - ca.repmat(Xm, 1, B)) / ca.repmat(Xs, 1, B)
        Dec_in = ca.vertcat(X_s, h_mat)                   # (11+h_dim, B)
        Y_s = _mlp_forward_casadi(self._weights, Dec_in, li, ncols=B)
        ym = ca.DM(self._y_mean.reshape(-1, 1))
        ys = ca.DM(self._y_scale.reshape(-1, 1))
        Y = Y_s * ca.repmat(ys, 1, B) + ca.repmat(ym, 1, B)

        self._BATCH = B
        self.predict_batch_gru = ca.Function(
            'nn_tire_batch_gru',
            [alphas, Fzs, us, kappas, n_ts, srs, h_mat,
             Kphi_b, Kc_b, c_b, phi_b, k_b],
            [Y[0, :].T, Y[1, :].T],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs', 'h_mat',
             'Kphi', 'Kc', 'c', 'phi', 'k'],
            ['Fxs', 'Fys'])
        self.predict_batch = None

    # --- GRU runtime (numpy) -------------------------------------------------

    def gru_step(self, x_obs, h_prev=None):
        """Run one GRU cell step.  Returns updated hidden state (numpy).

        Args:
            x_obs: (11,) raw observation
                   [kappa, alpha, u, Fz, sr, Kphi, Kc, n, c, phi, k]
            h_prev: (h_dim,) previous hidden state, or None → zeros.
        Returns:
            h_new: (h_dim,) updated hidden state.
        """
        hd = self.gru_h_dim
        if h_prev is None:
            h_prev = np.zeros(hd, dtype=np.float64)

        x = (np.asarray(x_obs, dtype=np.float64) - self._X_mean) / self._X_scale

        W_ih = self._gru_weights['weight_ih_l0']
        W_hh = self._gru_weights['weight_hh_l0']
        b_ih = self._gru_weights['bias_ih_l0']
        b_hh = self._gru_weights['bias_hh_l0']

        gi = W_ih @ x + b_ih           # (3*hd,)
        gh = W_hh @ h_prev + b_hh      # (3*hd,)

        def _sigmoid(v):
            return 1.0 / (1.0 + np.exp(-np.clip(v, -20, 20)))

        r = _sigmoid(gi[:hd] + gh[:hd])
        z = _sigmoid(gi[hd:2*hd] + gh[hd:2*hd])
        n = np.tanh(gi[2*hd:] + r * gh[2*hd:])
        return (1.0 - z) * n + z * h_prev

    # --- predict overrides (so kappa-ref estimation works) -------------------

    def predict(self, alpha, Fz, u, kappa=0.0, n_terrain=None, steering_rate=0.0,
                terrain_params=None, hist=None, rates=None, gru_h=None):
        if n_terrain is None:
            n_terrain = self.n_nominal
        tp = terrain_params if terrain_params is not None else self._terrain_nominals
        phi_val = np.radians(tp['phi'])
        h = gru_h if gru_h is not None else np.zeros(self.gru_h_dim)
        Fx, Fy = self.predict_tire_force(
            alpha, Fz, u, kappa, n_terrain, steering_rate,
            h, tp['Kphi'], tp['Kc'], tp['c'], phi_val, tp['k'])
        return Fx, Fy

    def predict_numeric(self, alpha, Fz, u, kappa=0.0, n_terrain=None,
                        steering_rate=0.0, terrain_params=None, hist=None,
                        rates=None, gru_h=None):
        Fx, Fy = self.predict(alpha, Fz, u, kappa, n_terrain, steering_rate,
                              terrain_params, hist, rates, gru_h=gru_h)
        return float(Fx), float(Fy)


# ============================================================================
# Factory / loader
# ============================================================================

def load_nn_tire_model(model_dir: str | Path, terrain_params: dict) -> NNTireModel:
    """Load any NN tire model from *model_dir* and return the correct subclass.

    Detection priority:
        1. Checkpoint metadata (architecture_type, temporal_K, rate_augmented)
        2. Directory name (parsed for confirmation / fallback)

    Args:
        model_dir: Path to directory containing best_terrain_nn.pt + scalers.pkl
        terrain_params: Dict with Kphi, Kc, n, c, phi (degrees), k
    """
    model_dir = Path(model_dir)
    model_path = model_dir / 'best_terrain_nn.pt'
    checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)

    # --- detect from checkpoint ---
    arch = 'mlp'
    temporal_K = 1
    rate_aug = False
    if isinstance(checkpoint, dict):
        arch = checkpoint.get('architecture_type', 'mlp')
        temporal_K = checkpoint.get('temporal_K', 1)
        rate_aug = checkpoint.get('rate_augmented', False)

    # --- cross-check with name ---
    name_info = parse_model_name(model_dir.name)
    if name_info['arch'] is not None and name_info['arch'] != arch:
        print(f"⚠ Name suggests {name_info['arch']} but checkpoint says {arch}; using checkpoint.")
    if name_info['temporal_K'] > 1 and temporal_K <= 1:
        print(f"⚠ Name suggests temporal K={name_info['temporal_K']} but checkpoint is static.")

    # --- dispatch ---
    if arch == 'gru_observer':
        cls = GRUObserverMLP
    elif arch == 'densenet':
        cls = StaticDenseNet
    elif arch == 'resnet' and temporal_K > 1:
        cls = TemporalResNet
    elif arch == 'resnet' and rate_aug:
        cls = RateResNet
    elif arch == 'resnet':
        cls = StaticResNet
    elif rate_aug:
        cls = RateMLP
    elif temporal_K > 1:
        cls = TemporalMLP
    else:
        cls = StaticMLP

    model = cls(model_dir, terrain_params)
    print(f"✓ Loaded {model.model_type}: {model_dir.name}  "
          f"(K={model.temporal_K}, {model.n_params} params, fmt={model.model_format})")
    return model


# ============================================================================
# CLI smoke test
# ============================================================================

if __name__ == '__main__':
    import argparse, sys
    sys.path.insert(0, str(Path(__file__).parent))
    from param_consistency import TERRAIN_PRESETS

    parser = argparse.ArgumentParser(description='Test NN tire model loading')
    parser.add_argument('model_dir', help='Path to model directory')
    parser.add_argument('--terrain', default='sand', choices=list(TERRAIN_PRESETS.keys()))
    args = parser.parse_args()

    preset = TERRAIN_PRESETS[args.terrain]
    tp = {
        'Kphi': preset['Kphi'], 'Kc': preset['Kc'], 'n': preset['n'],
        'c': preset['cohesion'], 'phi': preset['friction_angle'], 'k': preset['janosi_shear'],
    }

    model = load_nn_tire_model(args.model_dir, tp)
    print(f"\nModel type: {model.model_type}")
    print(f"Input dim:  {model.input_dim}")
    print(f"Params:     {model.n_params}")

    # Numeric test
    Fx, Fy = model.predict_numeric(alpha=0.05, Fz=3000.0, u=5.0)
    print(f"\nTest predict(α=0.05, Fz=3000, u=5): Fx={Fx:.1f} N, Fy={Fy:.1f} N")
