#!/usr/bin/env python3
"""
Chrono Simulation Setup Helpers
================================

Functions for setting up the PyChrono HMMWV vehicle, SCM deformable terrain,
terrain config loading, and trajectory markers.
"""

import numpy as np
import yaml

import pychrono as chrono
import pychrono.vehicle as veh

from param_consistency import TERRAIN_PRESETS, get_bumpiness_params
from terrain_gen import generate_heightmap_bmp


def _viz_type(name: str):
    """Return visualization enum value across Chrono Python API variants."""
    attr = f"VisualizationType_{name}"
    if hasattr(veh, attr):
        return getattr(veh, attr)
    if hasattr(chrono, attr):
        return getattr(chrono, attr)
    raise AttributeError(
        f"PyChrono visualization enum '{attr}' not found in pychrono or pychrono.vehicle"
    )


def setup_chrono_vehicle(visualize=True, payload_mass=0.0, simple_powertrain=False):
    """Setup PyChrono HMMWV vehicle.

    ``payload_mass`` (kg) adds an unmodelled cargo mass to the chassis
    body after initialization. The controller's bicycle model continues
    to use the nominal empty-vehicle mass, so a non-zero payload creates
    a genuine, persistent plant/model mismatch -- the per-deployment
    recalibration scenario used to evaluate online residual learning.
    The chassis rotational inertia is scaled by the same mass ratio so
    the payload is dynamically consistent.
    """

    # Set Chrono data path for mesh files
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # Create vehicle FIRST (it creates its own system internally)
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    # Enable primitive chassis collision so the chassis bumps off rigid
    # obstacles (rocks). HULLS self-collides with the wheels/suspension
    # and prevents normal motion, MESH is too expensive per-step.
    # PRIMITIVES + a higher rock Young's modulus (see sensors/obstacles.py)
    # is the empirical sweet spot.
    vehicle.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
    vehicle.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 1.5),
        chrono.ChQuaterniond(1, 0, 0, 0)
    ))
    if simple_powertrain:
        # Near-direct drive: linear torque engine (Te ~ throttle·T_max, no RPM
        # map) + CVT (no gear-shift discontinuities), so throttle->wheel-torque
        # is ~linear and soil-independent. The soil-dependence stays in the tyre
        # Fx(κ), which is unchanged. This makes throttle an effective torque
        # command -- the actuation map the force-balance NMPC needs.
        vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
        vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_CVT)
    else:
        vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
        vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    # Get system FROM vehicle after initialization
    system = vehicle.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    if payload_mass and payload_mass > 0.0:
        chassis = vehicle.GetChassisBody()
        m0 = chassis.GetMass()
        m1 = m0 + float(payload_mass)
        ratio = m1 / m0
        chassis.SetMass(m1)
        inertia = chassis.GetInertiaXX()
        chassis.SetInertiaXX(chrono.ChVector3d(
            inertia.x * ratio, inertia.y * ratio, inertia.z * ratio))
        print(f"  [PAYLOAD] chassis mass {m0:.0f} -> {m1:.0f} kg "
              f"(+{payload_mass:.0f} kg unmodelled cargo)")
    
    if visualize:
        # MESH for visual quality, PRIMITIVES for less important parts
        vehicle.SetChassisVisualizationType(_viz_type("MESH"))
        vehicle.SetSuspensionVisualizationType(_viz_type("PRIMITIVES"))
        vehicle.SetSteeringVisualizationType(_viz_type("PRIMITIVES"))
        vehicle.SetWheelVisualizationType(_viz_type("MESH"))
        vehicle.SetTireVisualizationType(_viz_type("MESH"))
    else:
        vehicle.SetChassisVisualizationType(_viz_type("PRIMITIVES"))
        vehicle.SetWheelVisualizationType(_viz_type("PRIMITIVES"))
        vehicle.SetTireVisualizationType(_viz_type("PRIMITIVES"))
    
    return system, vehicle


def load_terrain_config(config_path):
    """
    Load terrain configuration from YAML file.
    
    Args:
        config_path: Path to YAML config file
        
    Returns:
        dict with terrain parameters (numeric values converted to float)
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Validate required fields
    required = ['Kphi', 'Kc', 'n', 'cohesion', 'friction_angle', 'janosi_shear']
    for field in required:
        if field not in config:
            raise ValueError(f"Missing required terrain parameter: {field}")
    
    # Convert all values to float (handles scientific notation strings like '2.1e6')
    numeric_fields = ['Kphi', 'Kc', 'n', 'cohesion', 'friction_angle', 'janosi_shear',
                      'elastic_stiffness', 'damping', 'length', 'width', 'mesh_resolution',
                      'bump_amplitude', 'bump_wavelength', 'bump_max_slope']
    for field in numeric_fields:
        if field in config:
            config[field] = float(config[field])
    
    # Integer fields
    int_fields = ['bump_octaves']
    for field in int_fields:
        if field in config:
            config[field] = int(config[field])
    
    return config


def setup_scm_terrain(system, vehicle=None, visualize=True, terrain_preset='sand',
                      terrain_config=None, mesh_resolution=None,
                      bumpiness=0, bump_seed=12345, texture=True,
                      spatial_spec=None):
    """Setup SCM deformable terrain

    Args:
        system: Chrono system
        vehicle: Chrono vehicle (for moving patch optimization)
        visualize: Enable visualization
        terrain_preset: Preset name ('sand', 'clay', 'dirt')
                       Ignored if terrain_config provided.
        terrain_config: Dict with terrain params from config file (overrides preset)
        mesh_resolution: Override mesh spacing (m). Default: 0.08 for headless, 0.05 for vis.
        bumpiness: Terrain bumpiness level 0-10 (0=flat, 10=extreme).
                   Maps to TOPOLOGY_LEVELS in param_consistency.
        bump_seed: Random seed for reproducibility
        texture: Apply dirt texture to terrain mesh
        spatial_spec: Optional SpatialTransitionSpec. When provided, the soil
                   varies with x (one preset, a short blend, then another) via
                   a per-location SCM callback. The uniform SetSoilParameters
                   call still runs as a fallback; pass
                   terrain_preset=spatial_spec.start_preset so the base soil
                   and the callback agree on the start of the patch.
    """
    import tempfile
    
    # Resolve bumpiness level to Perlin noise parameters
    bp = get_bumpiness_params(bumpiness, seed=bump_seed)
    bump_amplitude = bp['bump_amplitude']
    bump_wavelength = bp['bump_wavelength']
    bump_octaves = bp['bump_octaves']
    bump_max_slope = bp['bump_max_slope']
    if bumpiness > 0:
        print(f"  Bumpiness {bumpiness} ({bp['description']}): "
              f"amp={bump_amplitude:.2f}m, wl={bump_wavelength:.0f}m, "
              f"octaves={bump_octaves}, slope={bump_max_slope*100:.0f}%")
    
    terrain = veh.SCMTerrain(system)
    
    # Load params from config or use presets
    if terrain_config is not None:
        Kphi = terrain_config['Kphi']
        Kc = terrain_config['Kc']
        n = terrain_config['n']
        c = terrain_config['cohesion']
        phi = terrain_config['friction_angle']
        k = terrain_config['janosi_shear']
        elastic_stiffness = terrain_config.get('elastic_stiffness', 2e8)
        damping = terrain_config.get('damping', 3e4)
        terrain_name = terrain_config.get('description', 'Custom config')
        print(f"  Terrain: {terrain_name}")
        print(f"    Kphi={Kphi:.2e}, Kc={Kc:.0f}, n={n:.2f}")
        print(f"    cohesion={c:.0f}, friction={phi:.0f}°, janosi={k:.3f}")
    else:
        # Use preset
        if terrain_preset not in TERRAIN_PRESETS:
            raise ValueError(f"Unknown terrain preset: {terrain_preset}. "
                           f"Available: {list(TERRAIN_PRESETS.keys())}")
        preset = TERRAIN_PRESETS[terrain_preset]
        Kphi = preset['Kphi']
        Kc = preset['Kc']
        n = preset['n']
        c = preset['cohesion']
        phi = preset['friction_angle']
        k = preset['janosi_shear']
        elastic_stiffness = preset.get('elastic_stiffness', 2e8)
        damping = preset.get('damping', 3e4)
        print(f"  Terrain: {terrain_preset} - {preset.get('description', '')}")
    
    # SetSoilParameters expects friction angle in DEGREES (not radians!)
    terrain.SetSoilParameters(
        Kphi, Kc, n, c, phi, k, elastic_stiffness, damping
    )

    # Spatially-varying soil: register a per-location callback that blends from
    # one preset to another along +x. SetSoilParameters above is the fallback.
    if spatial_spec is not None:
        from spatial_terrain import TransitionSoilCallback
        soil_cb = TransitionSoilCallback(spatial_spec)
        terrain.RegisterSoilParametersCallback(soil_cb)
        # Keep a Python reference alive: the SWIG director is owned by Python,
        # so without this the callback would be garbage-collected mid-run.
        terrain._soil_param_callback = soil_cb
        print(f"  Spatial soil transition: {spatial_spec.start_preset} -> "
              f"{spatial_spec.end_preset} at x={spatial_spec.transition_x:.1f}m "
              f"(blend {spatial_spec.transition_width:.1f}m)")

    # Mesh resolution: coarser = faster, all modes use 0.12m for real-time performance
    if mesh_resolution is not None:
        print(f"  Mesh: custom resolution {mesh_resolution}m")
        delta = mesh_resolution
    else:
        delta = 0.08  # Fine mesh for accurate terrain
        print(f"  Mesh: {delta}m")
    
    # Terrain dimensions: large visual area (moving patch keeps computation local)
    length, width = 200.0, 80.0  # Large terrain for visualization
    
    # Initialize terrain - flat or bumpy
    if bump_amplitude > 0:
        # Generate Perlin noise heightmap.  Parallel collectors must not share
        # a fixed /tmp filename because Chrono may read while another worker is
        # overwriting the BMP.
        heightmap_tmp = tempfile.NamedTemporaryFile(
            prefix="scm_heightmap_", suffix=".bmp", delete=False
        )
        heightmap_file = heightmap_tmp.name
        heightmap_tmp.close()
        # Image resolution: ~1 pixel per 0.5m for reasonable detail
        img_width = int(length * 2)
        img_height = int(width * 2)
        
        # Convert wavelength to frequency: 
        # wavelength is in meters, frequency is per-pixel
        # With 2 pixels per meter, freq = 1 / (wavelength * 2)
        pixel_frequency = 1.0 / (bump_wavelength * 2)
        
        generate_heightmap_bmp(heightmap_file, img_width, img_height,
                               amplitude=bump_amplitude, octaves=bump_octaves,
                               frequency=pixel_frequency, seed=bump_seed,
                               max_slope=bump_max_slope)
        # Initialize with heightmap: maps pixel values to height range
        terrain.Initialize(heightmap_file, length, width, 
                          0.0, bump_amplitude, delta)
        print(f"  Bumpy terrain: amplitude={bump_amplitude:.2f}m, "
              f"wavelength={bump_wavelength:.0f}m, max_slope={bump_max_slope*100:.0f}%")
    else:
        terrain.Initialize(length, width, delta)
    
    # Per-wheel moving patch: tighter boxes = fewer SCM nodes evaluated per step.
    # Older/newer Chrono Python builds expose either AddActiveDomain or AddMovingPatch.
    if vehicle is not None:
        add_patch = None
        if hasattr(terrain, "AddActiveDomain"):
            add_patch = terrain.AddActiveDomain
        elif hasattr(terrain, "AddMovingPatch"):
            add_patch = terrain.AddMovingPatch
        else:
            print("  WARNING: SCMTerrain moving patch API not found; running without moving patches")

        for ax in vehicle.GetVehicle().GetAxles():
            if add_patch is None:
                break
            add_patch(ax.m_wheels[0].GetSpindle(),
                      chrono.ChVector3d(0, 0, 0),
                      chrono.ChVector3d(1, 0.5, 1))
            add_patch(ax.m_wheels[1].GetSpindle(),
                      chrono.ChVector3d(0, 0, 0),
                      chrono.ChVector3d(1, 0.5, 1))
    
    if visualize:
        terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
        if texture:
            terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 10, 10)
    
    print(f"  SCM mesh: {delta}m, terrain: {length}x{width}m"
          + (", moving patch ON" if vehicle else ""))
    
    return terrain, {'Kphi': Kphi, 'Kc': Kc, 'n': n, 'c': c, 'phi': phi, 'k': k}


def add_trajectory_markers(system, path_type='lane_change', marker_z=None,
                           lead_in=0.0, **_kwargs):
    """
    Add visual sphere markers along the reference path loaded from CSV.

    Loads waypoints from ``paths/<path_type>.csv`` and places markers at
    regular arc-length intervals along the path.

    Args:
        system: Chrono system to add markers to.
        path_type: Name of the CSV file (without extension) in ``paths/``.
        marker_z: Z height for markers (default 0.15).
        lead_in: Optional straight lead-in distance prepended to the path.
    """
    from pathlib import Path as _P

    marker_spacing = 4.0   # arc-length metres between markers
    marker_radius = 0.15
    marker_height = marker_z if marker_z is not None else 0.15

    paths_dir = _P(__file__).resolve().parent.parent / "paths"
    csv_path = paths_dir / f"{path_type}.csv"
    if not csv_path.exists():
        print(f"  WARNING: path CSV not found: {csv_path}, skipping markers")
        return

    data = np.loadtxt(str(csv_path), delimiter=',', skiprows=1)
    if data.shape[1] == 2:
        x_all, y_all = data[:, 0], data[:, 1]
    else:
        x_all, y_all = data[:, 1], data[:, 2]

    # Optionally prepend lead-in straight section
    if lead_in > 0:
        ds = 0.25
        n_lead = max(1, int(lead_in / ds))
        x_lead = np.linspace(0, lead_in, n_lead, endpoint=False)
        y_lead = np.zeros(n_lead)
        x_all = np.concatenate([x_lead, x_all + lead_in])
        y_all = np.concatenate([y_lead, y_all])

    # Compute cumulative arc length
    dx = np.diff(x_all)
    dy = np.diff(y_all)
    ds_arr = np.sqrt(dx ** 2 + dy ** 2)
    s_cum = np.concatenate([[0.0], np.cumsum(ds_arr)])
    s_total = s_cum[-1]

    n_markers = int(s_total / marker_spacing) + 1
    print(f"  Adding {n_markers} trajectory markers for {path_type} ({s_total:.0f}m arc)...")

    # Subsample at regular arc-length intervals
    s_targets = np.linspace(0, s_total, n_markers)

    for i, s_t in enumerate(s_targets):
        idx = int(np.searchsorted(s_cum, s_t, side='right')) - 1
        idx = max(0, min(idx, len(x_all) - 1))
        x = float(x_all[idx])
        y = float(y_all[idx])

        marker = chrono.ChBodyEasySphere(marker_radius, 1000, True, False)
        marker.SetPos(chrono.ChVector3d(x, y, marker_height))
        marker.SetFixed(True)

        # Gradient color: green → yellow → blue along path progress
        t = i / max(n_markers - 1, 1)
        color = chrono.ChColor(0.2 + 0.7 * t, 0.8 - 0.5 * t, 0.2 + 0.6 * t)
        marker.GetVisualShape(0).SetColor(color)
        system.Add(marker)
