#!/usr/bin/env python3
"""
Path Generation Utilities
=========================

Functions for creating reference paths (sinusoidal, lane change, etc.)
and checking path feasibility for the HMMWV vehicle.
"""

import numpy as np


def find_closest_point_on_sinusoid(x_veh, y_veh, amplitude, wavelength, search_range=10.0, n_samples=100,
                                    x_offset=0.0):
    """
    Find the closest point on a sinusoidal path y = amp*sin(2*pi*(x - x_offset)/wavelength) 
    to the vehicle's current position.  For x < x_offset the path is y=0 (lead-in).
    
    Returns:
        s_closest: The x-coordinate of the closest point on the path
        y_path: The y-coordinate of the closest point
        dist: The distance to the closest point
    """
    # Search window around vehicle's x position
    x_min = x_veh - search_range
    x_max = x_veh + search_range
    
    x_samples = np.linspace(x_min, x_max, n_samples)
    y_samples = np.where(x_samples >= x_offset,
                         amplitude * np.sin(2 * np.pi * (x_samples - x_offset) / wavelength),
                         0.0)
    
    # Find minimum distance point
    distances = np.sqrt((x_samples - x_veh)**2 + (y_samples - y_veh)**2)
    idx_min = np.argmin(distances)
    
    # Refine with local search (Newton-like)
    x_closest = x_samples[idx_min]
    for _ in range(3):  # Few iterations of refinement
        if x_closest < x_offset:
            # In the lead-in region: path is y=0, closest point is directly below
            y_path = 0.0
            dy_dx = 0.0
            d2y_dx2 = 0.0
        else:
            y_path = amplitude * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
            dy_dx = amplitude * 2 * np.pi / wavelength * np.cos(2 * np.pi * (x_closest - x_offset) / wavelength)
            d2y_dx2 = -amplitude * (2 * np.pi / wavelength)**2 * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
        
        # Gradient of distance^2 w.r.t. x_path
        grad = 2 * (x_closest - x_veh) + 2 * (y_path - y_veh) * dy_dx
        
        # Second derivative for Newton step
        hess = 2 + 2 * dy_dx**2 + 2 * (y_path - y_veh) * d2y_dx2
        
        if abs(hess) > 1e-6:
            x_closest = x_closest - 0.5 * grad / hess  # Damped Newton step
    
    if x_closest < x_offset:
        y_closest = 0.0
    else:
        y_closest = amplitude * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
    dist = np.sqrt((x_closest - x_veh)**2 + (y_closest - y_veh)**2)
    
    return x_closest, y_closest, dist


def check_sinusoidal_feasibility(amplitude, wavelength, wheelbase=3.302, delta_max=0.5):
    """
    Check if a sinusoidal path is feasible for the vehicle.
    
    For y = A*sin(2πx/λ), max curvature κ = A*(2π/λ)² at the peaks.
    Vehicle min turning radius R_min = L/tan(δ_max).
    
    Returns:
        (is_feasible, required_R, achievable_R, margin_pct)
    """
    # Max curvature of sinusoid (at peaks where y'' is maximum)
    kappa_max = amplitude * (2 * np.pi / wavelength) ** 2
    required_R = 1.0 / kappa_max if kappa_max > 0 else float('inf')
    
    # Vehicle's minimum turning radius (Ackermann geometry)
    achievable_R = wheelbase / np.tan(delta_max)
    
    # Margin (positive = feasible, negative = impossible)
    margin_pct = (required_R - achievable_R) / achievable_R * 100
    is_feasible = required_R >= achievable_R
    
    return is_feasible, required_R, achievable_R, margin_pct


def suggest_feasible_sine_params(target_amplitude=2.0, wheelbase=3.302, delta_max=0.5, margin=1.2):
    """
    Suggest a feasible wavelength for a given amplitude.
    
    Args:
        target_amplitude: Desired amplitude (m)
        margin: Safety margin (1.2 = 20% easier than limit)
    
    Returns:
        min_wavelength: Minimum feasible wavelength (m)
    """
    achievable_R = wheelbase / np.tan(delta_max)
    # Add margin for controller tracking error
    min_R = achievable_R * margin
    
    # κ_max = A * (2π/λ)² = 1/R_min
    # λ = 2π * sqrt(A * R_min)
    min_wavelength = 2 * np.pi * np.sqrt(target_amplitude * min_R)
    
    return min_wavelength


def make_path_function(path_type='lane_change', v_target=8.0,
                        lead_in=0.0, csv_dir=None, friction_angle_deg=None,
                        ay_safety=0.65, **_kwargs):
    """
    Load a ReferencePath from a CSV waypoint file in ``paths/<path_type>.csv``.

    The CSV must contain ``x,y`` columns (or ``s,x,y,psi`` legacy format).
    If *lead_in* > 0 a straight section is prepended along +x.

    Args:
        path_type: Name of the path (matches ``paths/<name>.csv``).
        v_target: Target longitudinal velocity (m/s).
        lead_in: Straight lead-in distance (m) prepended before path geometry.
        csv_dir: If given, save a copy of the loaded path to this dir.
        friction_angle_deg: Terrain friction angle (degrees) for speed profiler.

    Returns:
        ReferencePath object.
    """
    from pathlib import Path as _P
    from reference_path import ReferencePath

    paths_dir = _P(__file__).resolve().parent.parent / "paths"
    csv_path = paths_dir / f"{path_type}.csv"
    if not csv_path.exists():
        raise FileNotFoundError(
            f"Path CSV not found: {csv_path}\n"
            f"  Available: {[p.stem for p in paths_dir.glob('*.csv')]}")

    ref_path = ReferencePath.from_csv(str(csv_path), v_target,
                                       friction_angle_deg=friction_angle_deg,
                                       ay_safety=ay_safety)

    # Optionally prepend a straight lead-in section
    if lead_in > 0:
        ds = 0.25
        n_lead = max(1, int(lead_in / ds))
        x_lead = np.linspace(0, lead_in, n_lead, endpoint=False)
        y_lead = np.zeros(n_lead)
        x_shifted = ref_path.x_pts + lead_in
        y_shifted = ref_path.y_pts
        x_all = np.concatenate([x_lead, x_shifted])
        y_all = np.concatenate([y_lead, y_shifted])
        ref_path = ReferencePath(x_all, y_all, v_target,
                                  friction_angle_deg=friction_angle_deg,
                                  ay_safety=ay_safety)
        print(f"  Lead-in: {lead_in:.0f}m straight before path starts")

    print(f"  Reference path: {ref_path}")

    # Optionally save a copy
    if csv_dir is not None:
        import os
        os.makedirs(csv_dir, exist_ok=True)
        ref_path.save_csv(os.path.join(csv_dir,
                                       f'reference_path_{path_type}.csv'))

    return ref_path
