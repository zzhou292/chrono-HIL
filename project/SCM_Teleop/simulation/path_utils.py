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


def make_path_function(path_type='lane_change', lane_offset=3.0, v_target=8.0,
                        sine_amplitude=2.0, sine_wavelength=30.0,
                        use_closest_point=True, lead_in=0.0,
                        csv_dir=None, total_length=None):
    """
    Create a ReferencePath for the MPC driver.

    Generates dense waypoints for the chosen path type, fits an arc-length
    parameterised cubic spline, and returns a :class:`ReferencePath` whose
    :meth:`get_reference` method has the same ``(time, z0, N, dt)`` signature
    used by the controller loop.

    Args:
        path_type: 'lane_change', 'double_lane_change', or 'sinusoidal'
        lane_offset: Lateral offset for lane change maneuvers (m)
        v_target: Target longitudinal velocity (m/s)
        sine_amplitude: Amplitude for sinusoidal path (m)
        sine_wavelength: Wavelength for sinusoidal path (m)
        use_closest_point: (kept for CLI compat; now always True internally)
        lead_in: Straight lead-in distance (m) added before path geometry
        csv_dir: If given, save a reference_path_<type>.csv into this dir
        total_length: Override total path length (m); auto-computed if None

    Returns:
        ReferencePath object.  Use ``ref_path.get_reference`` as the
        path callable, and ``ref_path.evaluate_at_x`` for analytics.
    """
    from reference_path import ReferencePath, generate_path_waypoints

    # Check sinusoidal path feasibility
    if path_type == 'sinusoidal':
        is_feasible, req_R, ach_R, margin = check_sinusoidal_feasibility(
            sine_amplitude, sine_wavelength)
        if not is_feasible:
            min_wl = suggest_feasible_sine_params(sine_amplitude)
            print(f"\n  ⚠ WARNING: Sinusoidal path is INFEASIBLE!")
            print(f"     Required turning radius: {req_R:.2f} m")
            print(f"     Vehicle minimum radius:  {ach_R:.2f} m")
            print(f"     Path is {-margin:.0f}% beyond vehicle limits")
            print(f"     Suggestions:")
            print(f"       - Increase wavelength to ≥{min_wl:.1f}m (currently {sine_wavelength}m)")
            print(f"       - Or reduce amplitude to ≤{sine_amplitude * (req_R/ach_R):.2f}m")
            print()
        else:
            print(f"  Path feasibility: OK (margin: +{margin:.0f}%)")

    if lead_in > 0:
        print(f"  Lead-in: {lead_in:.0f}m straight before path starts")

    # Generate dense waypoints
    x_pts, y_pts = generate_path_waypoints(
        path_type, lead_in=lead_in, lane_offset=lane_offset,
        sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength,
        total_length=total_length,
    )

    # Build spline-based reference path
    ref_path = ReferencePath(x_pts, y_pts, v_target)
    print(f"  Reference path: {ref_path}")

    # Optionally save CSV
    if csv_dir is not None:
        import os
        os.makedirs(csv_dir, exist_ok=True)
        ref_path.save_csv(os.path.join(csv_dir,
                                       f'reference_path_{path_type}.csv'))

    return ref_path
