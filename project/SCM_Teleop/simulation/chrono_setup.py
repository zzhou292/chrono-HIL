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


def setup_chrono_vehicle(visualize=True):
    """Setup PyChrono HMMWV vehicle."""
    
    # Set Chrono data path for mesh files
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Create vehicle FIRST (it creates its own system internally)
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 1.5),
        chrono.ChQuaterniond(1, 0, 0, 0)
    ))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()
    
    # Get system FROM vehicle after initialization  
    system = vehicle.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    if visualize:
        # MESH for visual quality, PRIMITIVES for less important parts
        vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
        vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
        vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    else:
        vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
    
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
                      bumpiness=0, bump_seed=12345):
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
    
    # Mesh resolution: coarser = faster, all modes use 0.12m for real-time performance
    if mesh_resolution is not None:
        print(f"  Mesh: custom resolution {mesh_resolution}m")
        delta = mesh_resolution
    else:
        delta = 0.08  # Fine mesh for accurate terrain
        print(f"  Mesh: {delta}m")
    
    # Terrain dimensions: large visual area (moving patch keeps computation local)
    length, width = 200.0, 50.0  # Large terrain for visualization
    
    # Initialize terrain - flat or bumpy
    if bump_amplitude > 0:
        # Generate Perlin noise heightmap
        heightmap_file = tempfile.gettempdir() + '/scm_heightmap.bmp'
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
    
    # Moving patch: only compute SCM deformation near the vehicle (huge speedup)
    if vehicle is not None:
        terrain.AddMovingPatch(vehicle.GetChassisBody(),
                               chrono.ChVector3d(0, 0, 0),
                               chrono.ChVector3d(6, 3, 1))
    
    if visualize:
        terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
    
    print(f"  SCM mesh: {delta}m, terrain: {length}x{width}m"
          + (", moving patch ON" if vehicle else ""))
    
    return terrain, {'Kphi': Kphi, 'Kc': Kc, 'n': n, 'c': c, 'phi': phi, 'k': k}


def add_trajectory_markers(system, path_type='lane_change', sim_time=10.0, 
                           v_target=8.0, lane_offset=3.0, marker_z=None,
                           sine_amplitude=2.0, sine_wavelength=30.0, lead_in=0.0):
    """
    Add visual markers on the ground to show the reference trajectory.
    
    Args:
        system: Chrono system
        path_type: 'lane_change', 'double_lane_change', or 'sinusoidal'
        sim_time: Duration to generate markers for
        v_target: Target velocity (used only for estimating marker count)
        lane_offset: Lane change offset (m)
        marker_z: Z height for markers (default: 0.15, set higher for bumpy terrain)
        sine_amplitude: Amplitude for sinusoidal path (m)
        sine_wavelength: Wavelength for sinusoidal path (m)
    """
    marker_spacing = 4.0  # meters between markers (sparser for performance)
    marker_radius = 0.15
    marker_height = marker_z if marker_z is not None else 0.15
    
    # Estimate total distance (just for marker count, not for path positions)
    total_dist = max(v_target * sim_time, 60.0)  # At least 60m to cover path
    n_markers = int(total_dist / marker_spacing) + 1
    
    print(f"  Adding {n_markers} trajectory markers for {path_type}...")
    
    # Path-specific parameters - ALL FIXED POSITIONS (shifted by lead_in)
    # Single lane change
    lc_start = 10.0 + lead_in
    lc_end = 25.0 + lead_in
    
    # Double lane change zones
    dlc_z1_start, dlc_z1_end = 8.0 + lead_in, 18.0 + lead_in
    dlc_z2_start, dlc_z2_end = 28.0 + lead_in, 38.0 + lead_in
    
    for i in range(n_markers):
        x = i * marker_spacing
        
        if path_type == 'lane_change':
            if x < lc_start:
                y = 0.0
                zone = 'start'
            elif x > lc_end:
                y = lane_offset
                zone = 'end'
            else:
                blend = (x - lc_start) / (lc_end - lc_start)
                blend = blend * blend * (3 - 2 * blend)
                y = blend * lane_offset
                zone = 'transition'
                
        elif path_type == 'double_lane_change':
            if x < dlc_z1_start:
                y = 0.0
                zone = 'start'
            elif x < dlc_z1_end:
                blend = (x - dlc_z1_start) / (dlc_z1_end - dlc_z1_start)
                blend = blend * blend * (3 - 2 * blend)
                y = blend * lane_offset
                zone = 'transition1'
            elif x < dlc_z2_start:
                y = lane_offset
                zone = 'middle'
            elif x < dlc_z2_end:
                blend = (x - dlc_z2_start) / (dlc_z2_end - dlc_z2_start)
                blend = blend * blend * (3 - 2 * blend)
                y = lane_offset * (1 - blend)
                zone = 'transition2'
            else:
                y = 0.0
                zone = 'end'
                
        elif path_type == 'sinusoidal':
            # Use parameters passed to function (with lead-in offset)
            if x < lead_in:
                y = 0.0
            else:
                y = sine_amplitude * np.sin(2 * np.pi * (x - lead_in) / sine_wavelength)
            zone = 'sine'
        else:
            y = 0.0
            zone = 'default'
        
        # Create marker
        marker = chrono.ChBodyEasySphere(marker_radius, 1000, True, False)
        marker.SetPos(chrono.ChVector3d(x, y, marker_height))
        marker.SetFixed(True)
        
        # Color by zone
        if zone == 'start':
            color = chrono.ChColor(0.2, 0.8, 0.2)  # Green
        elif zone == 'end':
            color = chrono.ChColor(0.2, 0.2, 0.8)  # Blue
        elif zone in ['transition', 'transition1']:
            color = chrono.ChColor(0.9, 0.9, 0.2)  # Yellow
        elif zone == 'middle':
            color = chrono.ChColor(0.8, 0.4, 0.1)  # Orange
        elif zone == 'transition2':
            color = chrono.ChColor(0.9, 0.5, 0.9)  # Pink
        elif zone == 'sine':
            # Rainbow based on sine phase (using actual amplitude)
            phase = (y / sine_amplitude + 1) / 2  # 0 to 1
            color = chrono.ChColor(0.8 * phase, 0.3, 0.8 * (1-phase))
        else:
            t = i / max(n_markers - 1, 1)
            color = chrono.ChColor(0.2 + 0.6 * t, 0.8 - 0.4 * t, 0.2)
        
        marker.GetVisualShape(0).SetColor(color)
        system.Add(marker)
