# Terrain options
--terrain_type=<flat|heightmap|random>
--terrain_size_x=60.0 --terrain_size_y=60.0
--heightmap_file=<path> --heightmap_min=0.0 --heightmap_max=0.5
--random_seed=12345 --random_amplitude=0.3 --random_octaves=4

# Soil parameters
--bekker_kphi=2e6 --bekker_n=1.1 --mohr_friction=30.0

# Vehicle
--tire_type=<cylindrical|lugged|rigid>
--init_x=-25.0 --init_y=0.0 --cruise_speed=10.0

# Visualization
--vis_mode=<irrlicht|sensor|both>
--wireframe=false --sinkage=true

# Sensors
--enable_camera=true --enable_lidar=true
--lidar_offset_x=5.0 --lidar_range=100.0

# Obstacles
--enable_obstacles=true --num_rocks=20
--rock_min_size=0.3 --rock_max_size=1.5

# Simulation
--delay_val=0.0 --ros_bridge=false