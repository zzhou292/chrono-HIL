// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou (adapted from Teleop and SCM demos)
// =============================================================================
//
// SCM Deformable Terrain Teleoperation Project
// Features:
//   - SCM (Soil Contact Model) deformable terrain
//   - Configurable height map (randomized or from file)
//   - Obstacle/rock placement with collision
//   - Camera and LiDAR sensors
//   - Irrlicht visualization (third-person chase camera)
//   - Sensor visualization (camera and lidar)
//   - SDL2 controller input
//   - ROS2 bridge interface
//   - Configurable simulation delay
//
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>

#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"
#include "chrono_hil/network/sim/ChDelaySim.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "project/SCM_Teleop/Ros2Bridge.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::hil;
using namespace chrono::utils;
using namespace chrono::sensor;

// =============================================================================
// Constants
// =============================================================================
const double RADS_2_RPM = 30 / CH_PI;
const double RADS_2_DEG = 180 / CH_PI;
const double MS_2_MPH = 2.2369;
const double M_2_FT = 3.28084;
const double G_2_MPSS = 9.81;

// =============================================================================
// Configuration Enums
// =============================================================================

// Terrain patch type
enum class TerrainType { FLAT, HEIGHTMAP, RANDOM };

// Tire type
enum class TireType { CYLINDRICAL, LUGGED, RIGID };

// Visualization mode
enum class VisMode { IRRLICHT_ONLY, SENSOR_ONLY, BOTH };

// =============================================================================
// Default Configuration
// =============================================================================

// Terrain configuration
TerrainType terrain_type = TerrainType::FLAT;
double terrain_size_x = 60.0;
double terrain_size_y = 60.0;
double scm_delta = 0.05;  // SCM grid spacing
std::string heightmap_file = "";
double heightmap_min = 0.0;
double heightmap_max = 0.5;

// Random terrain configuration
unsigned int random_seed = 12345;
double random_amplitude = 0.3;
int random_octaves = 4;
double random_frequency = 0.05;

// SCM soil parameters (default: firm soil for easier control)
double bekker_kphi = 5e6;
double bekker_kc = 0;
double bekker_n = 1.1;
double mohr_cohesion = 20000;
double mohr_friction_deg = 35;
double janosi_shear = 0.03;
double elastic_stiffness = 5e8;
double damping = 3e4;

// Vehicle configuration
ChVector3d initLoc(-25.0, 0.0, 0.8);
ChQuaterniond initRot(1, 0, 0, 0);
TireType tire_type = TireType::LUGGED;
double cruise_speed = 10.0;  // m/s

// Simulation configuration
double step_size = 3e-3;
double t_end = 1000;
ChContactMethod contact_method = ChContactMethod::SMC;

// Visualization configuration
VisMode vis_mode = VisMode::BOTH;
bool render_wireframe = false;
bool render_sinkage = true;
double render_fps = 50;

// Sensor configuration
bool enable_camera = true;
bool enable_lidar = true;
double camera_update_rate = 30.0;
double lidar_update_rate = 20.0;
ChVector3d lidar_offset(5.0, 0.0, 1.5);  // 5m in front of vehicle
int lidar_horizontal_samples = 900;
int lidar_vertical_channels = 32;
double lidar_max_range = 100.0;

// Network/delay configuration
float delay_val = 0.0;
float cam_delay_val = 0.0;
std::string delay_config_file = "";
const std::string UNITY_IP_OUT = "127.0.0.1";
const int UNITY_PORT_OUT = 1209;

// ROS2 configuration
bool enable_ros_bridge = false;

// Obstacle configuration
bool enable_obstacles = true;
int num_rocks = 20;
double rock_min_size = 1.0;
double rock_max_size = 5.0;
double obstacle_zone_start_x = -15.0;
double obstacle_zone_end_x = 25.0;
double obstacle_zone_y_range = 20.0;
std::string rock_mesh_path = "/home/kyle/Downloads/NSFDemoDataDir/NSFDemoDataDir/Environments/SCMTeleop/Cliff_Rock_Two/Cliff_Rock_Two_OBJ.obj";
std::string rock_texture_path = "/home/kyle/Downloads/NSFDemoDataDir/NSFDemoDataDir/Environments/SCMTeleop/Cliff_Rock_Two/Cliff_Rock_Two_BaseColor.png";

// =============================================================================
// Forward declarations
// =============================================================================
void AddCommandLineOptions(ChCLI &cli);
void CreateLuggedGeometry(std::shared_ptr<ChBody> wheel_body, std::shared_ptr<ChContactMaterialSMC> wheel_material);
void AddRockObstacles(ChSystem &sys, std::mt19937 &rng);
void GenerateRandomHeightmap(const std::string &filename, int width, int height, 
                             double amplitude, int octaves, double frequency, unsigned int seed);

// =============================================================================
// Helper functions
// =============================================================================

std::vector<char> serializeFloats(const std::vector<float> &floatVec) {
  std::vector<char> byteVec(floatVec.size() * sizeof(float));
  char *bytePointer = byteVec.data();
  for (const float &value : floatVec) {
    std::memcpy(bytePointer, &value, sizeof(float));
    bytePointer += sizeof(float);
  }
  return byteVec;
}

std::vector<float> deserializeFloats(const std::vector<char> &byteVec) {
  std::vector<float> floatVec(byteVec.size() / sizeof(float));
  const char *bytePointer = byteVec.data();
  for (float &value : floatVec) {
    std::memcpy(&value, bytePointer, sizeof(float));
    bytePointer += sizeof(float);
  }
  return floatVec;
}

// Perlin noise implementation for procedural terrain generation
class PerlinNoise {
public:
  PerlinNoise(unsigned int seed = 0) {
    std::mt19937 gen(seed);
    std::iota(p.begin(), p.end(), 0);
    std::shuffle(p.begin(), p.end(), gen);
    for (int i = 0; i < 256; ++i) {
      p[256 + i] = p[i];
    }
  }

  double noise(double x, double y) const {
    int X = static_cast<int>(std::floor(x)) & 255;
    int Y = static_cast<int>(std::floor(y)) & 255;
    x -= std::floor(x);
    y -= std::floor(y);
    double u = fade(x);
    double v = fade(y);
    int A = p[X] + Y;
    int B = p[X + 1] + Y;
    return lerp(v, lerp(u, grad(p[A], x, y), grad(p[B], x - 1, y)),
                   lerp(u, grad(p[A + 1], x, y - 1), grad(p[B + 1], x - 1, y - 1)));
  }

  double octaveNoise(double x, double y, int octaves, double persistence = 0.5) const {
    double total = 0;
    double frequency = 1;
    double amplitude = 1;
    double maxValue = 0;
    for (int i = 0; i < octaves; ++i) {
      total += noise(x * frequency, y * frequency) * amplitude;
      maxValue += amplitude;
      amplitude *= persistence;
      frequency *= 2;
    }
    return total / maxValue;
  }

private:
  std::array<int, 512> p;

  static double fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
  static double lerp(double t, double a, double b) { return a + t * (b - a); }
  static double grad(int hash, double x, double y) {
    int h = hash & 15;
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : (h == 12 || h == 14 ? x : 0);
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
  }
};

// =============================================================================
// Command line options
// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  // Terrain options
  cli.AddOption<std::string>("Terrain", "terrain_type", "Terrain type: flat, heightmap, random", "flat");
  cli.AddOption<double>("Terrain", "terrain_size_x", "Terrain X size", std::to_string(terrain_size_x));
  cli.AddOption<double>("Terrain", "terrain_size_y", "Terrain Y size", std::to_string(terrain_size_y));
  cli.AddOption<double>("Terrain", "scm_delta", "SCM grid spacing", std::to_string(scm_delta));
  cli.AddOption<std::string>("Terrain", "heightmap_file", "Height map file path", heightmap_file);
  cli.AddOption<double>("Terrain", "heightmap_min", "Height map minimum height", std::to_string(heightmap_min));
  cli.AddOption<double>("Terrain", "heightmap_max", "Height map maximum height", std::to_string(heightmap_max));
  
  // Random terrain options
  cli.AddOption<int>("Terrain", "random_seed", "Random seed for terrain generation", std::to_string(random_seed));
  cli.AddOption<double>("Terrain", "random_amplitude", "Random terrain amplitude", std::to_string(random_amplitude));
  cli.AddOption<int>("Terrain", "random_octaves", "Random terrain noise octaves", std::to_string(random_octaves));
  cli.AddOption<double>("Terrain", "random_frequency", "Random terrain noise frequency", std::to_string(random_frequency));

  // SCM soil parameters
  cli.AddOption<double>("Soil", "bekker_kphi", "Bekker Kphi parameter", std::to_string(bekker_kphi));
  cli.AddOption<double>("Soil", "bekker_n", "Bekker n exponent", std::to_string(bekker_n));
  cli.AddOption<double>("Soil", "mohr_friction", "Mohr friction angle (degrees)", std::to_string(mohr_friction_deg));
  cli.AddOption<double>("Soil", "elastic_stiffness", "Elastic stiffness (Pa/m)", std::to_string(elastic_stiffness));

  // Vehicle options
  cli.AddOption<std::string>("Vehicle", "tire_type", "Tire type: cylindrical, lugged, rigid", "lugged");
  cli.AddOption<double>("Vehicle", "init_x", "Initial X position", std::to_string(initLoc.x()));
  cli.AddOption<double>("Vehicle", "init_y", "Initial Y position", std::to_string(initLoc.y()));
  cli.AddOption<double>("Vehicle", "cruise_speed", "Cruise speed (m/s)", std::to_string(cruise_speed));

  // Visualization options
  cli.AddOption<std::string>("Visualization", "vis_mode", "Visualization mode: irrlicht, sensor, both", "both");
  cli.AddOption<bool>("Visualization", "wireframe", "Render SCM terrain as wireframe", "false");
  cli.AddOption<bool>("Visualization", "sinkage", "Render sinkage color map", "true");
  cli.AddOption<double>("Visualization", "render_fps", "Render FPS", std::to_string(render_fps));

  // Sensor options
  cli.AddOption<bool>("Sensor", "enable_camera", "Enable camera sensor", "true");
  cli.AddOption<bool>("Sensor", "enable_lidar", "Enable lidar sensor", "true");
  cli.AddOption<double>("Sensor", "lidar_offset_x", "Lidar X offset from vehicle", std::to_string(lidar_offset.x()));
  cli.AddOption<double>("Sensor", "lidar_range", "Lidar max range", std::to_string(lidar_max_range));

  // Obstacle options
  cli.AddOption<bool>("Obstacles", "enable_obstacles", "Enable rock obstacles", "true");
  cli.AddOption<int>("Obstacles", "num_rocks", "Number of rock obstacles", std::to_string(num_rocks));
  cli.AddOption<double>("Obstacles", "rock_min_size", "Minimum rock size", std::to_string(rock_min_size));
  cli.AddOption<double>("Obstacles", "rock_max_size", "Maximum rock size", std::to_string(rock_max_size));

  // Simulation options
  cli.AddOption<double>("Simulation", "step_size", "Simulation step size", std::to_string(step_size));
  cli.AddOption<double>("Simulation", "t_end", "Simulation end time", std::to_string(t_end));
  cli.AddOption<float>("Simulation", "delay_val", "Control delay (ms)", std::to_string(delay_val));
  cli.AddOption<float>("Simulation", "cam_delay_val", "Camera delay (ms)", std::to_string(cam_delay_val));
  cli.AddOption<std::string>("Simulation", "delay_config", "Delay configuration JSON file", delay_config_file);
  cli.AddOption<bool>("Simulation", "ros_bridge", "Enable ROS2 bridge", "false");
}

// =============================================================================
// Create lugged tire geometry (from SCM demo)
// =============================================================================
void CreateLuggedGeometry(std::shared_ptr<ChBody> wheel_body, std::shared_ptr<ChContactMaterialSMC> wheel_material) {
  std::string lugged_file("hmmwv/lugged_wheel_section.obj");
  ChTriangleMeshConnected lugged_mesh;
  ChConvexDecompositionHACDv2 lugged_convex;
  chrono::utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
  int num_hulls = lugged_convex.GetHullCount();

  // Assemble the tire contact from 15 segments
  for (int iseg = 0; iseg < 15; iseg++) {
    ChQuaternion<> rot = QuatFromAngleY(iseg * 24 * CH_DEG_TO_RAD);
    for (int ihull = 0; ihull < num_hulls; ihull++) {
      std::vector<ChVector3d> convexhull;
      lugged_convex.GetConvexHullResult(ihull, convexhull);
      auto shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(wheel_material, convexhull);
      wheel_body->AddCollisionShape(shape, ChFrame<>(VNULL, rot));
    }
  }

  // Add cylinder for wheel hub
  auto cyl_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_material, 0.223, 0.252);
  wheel_body->AddCollisionShape(cyl_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

  // Visualization
  auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
      vehicle::GetDataFile("hmmwv/lugged_wheel.obj"), false, false);
  auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetMutable(false);
  trimesh_shape->SetName("lugged_wheel");
  trimesh_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
  wheel_body->AddVisualShape(trimesh_shape);
}

// =============================================================================
// Generate random heightmap using Perlin noise
// =============================================================================
void GenerateRandomHeightmap(const std::string &filename, int width, int height,
                             double amplitude, int octaves, double frequency, unsigned int seed) {
  PerlinNoise perlin(seed);
  
  // Create BMP file
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Failed to create heightmap file: " << filename << std::endl;
    return;
  }

  // BMP header
  int rowSize = ((width * 3 + 3) / 4) * 4;  // Row size padded to 4 bytes
  int imageSize = rowSize * height;
  int fileSize = 54 + imageSize;

  unsigned char header[54] = {
    'B', 'M',                           // Signature
    0, 0, 0, 0,                         // File size (filled below)
    0, 0, 0, 0,                         // Reserved
    54, 0, 0, 0,                        // Data offset
    40, 0, 0, 0,                        // Info header size
    0, 0, 0, 0,                         // Width (filled below)
    0, 0, 0, 0,                         // Height (filled below)
    1, 0,                               // Planes
    24, 0,                              // Bits per pixel
    0, 0, 0, 0,                         // Compression
    0, 0, 0, 0,                         // Image size (filled below)
    0, 0, 0, 0,                         // X pixels per meter
    0, 0, 0, 0,                         // Y pixels per meter
    0, 0, 0, 0,                         // Colors used
    0, 0, 0, 0                          // Important colors
  };

  // Fill in sizes
  *reinterpret_cast<int*>(&header[2]) = fileSize;
  *reinterpret_cast<int*>(&header[18]) = width;
  *reinterpret_cast<int*>(&header[22]) = height;
  *reinterpret_cast<int*>(&header[34]) = imageSize;

  file.write(reinterpret_cast<char*>(header), 54);

  // Generate heightmap data
  std::vector<unsigned char> row(rowSize, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double nx = static_cast<double>(x) * frequency;
      double ny = static_cast<double>(y) * frequency;
      double noise = perlin.octaveNoise(nx, ny, octaves);
      noise = (noise + 1.0) / 2.0;  // Normalize to 0-1
      unsigned char gray = static_cast<unsigned char>(noise * 255);
      row[x * 3 + 0] = gray;  // B
      row[x * 3 + 1] = gray;  // G
      row[x * 3 + 2] = gray;  // R
    }
    file.write(reinterpret_cast<char*>(row.data()), rowSize);
  }

  file.close();
  std::cout << "Generated random heightmap: " << filename << std::endl;
}

// =============================================================================
// Add rock obstacles with collision
// =============================================================================
void AddRockObstacles(ChSystem &sys, std::mt19937 &rng) {
  std::uniform_real_distribution<double> dist_x(obstacle_zone_start_x, obstacle_zone_end_x);
  std::uniform_real_distribution<double> dist_y(-obstacle_zone_y_range / 2, obstacle_zone_y_range / 2);
  std::uniform_real_distribution<double> dist_size(rock_min_size, rock_max_size);
  std::uniform_real_distribution<double> dist_yaw(0, CH_2PI);

  auto rock_material = chrono_types::make_shared<ChContactMaterialSMC>();
  rock_material->SetFriction(0.9f);
  rock_material->SetYoungModulus(1e8f);
  rock_material->SetRestitution(0.1f);

  // Load the rock mesh for visualization
  auto rock_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_mesh_path, true, true);
  if (!rock_mesh) {
    std::cerr << "ERROR: Could not load rock mesh from: " << rock_mesh_path << std::endl;
    return;
  }

  // Get mesh bounding box for scaling and centering
  auto aabb = rock_mesh->GetBoundingBox();
  ChVector3d mesh_size = aabb.max - aabb.min;
  ChVector3d mesh_center = aabb.Center();
  double mesh_base_size = std::max({mesh_size.x(), mesh_size.y(), mesh_size.z()});

  std::cout << "Rock Mesh Info:" << std::endl;
  std::cout << "  Bounding Box: Min(" << aabb.min.x() << ", " << aabb.min.y() << ", " << aabb.min.z() << ")" << std::endl;
  std::cout << "  Bounding Box: Max(" << aabb.max.x() << ", " << aabb.max.y() << ", " << aabb.max.z() << ")" << std::endl;
  std::cout << "  Size: " << mesh_size.x() << " x " << mesh_size.y() << " x " << mesh_size.z() << std::endl;
  std::cout << "  Base Size: " << mesh_base_size << std::endl;

  if (mesh_base_size < 1e-6) {
      std::cerr << "WARNING: Rock mesh size is extremely small/zero! Defaulting to 1.0" << std::endl;
      mesh_base_size = 1.0;
  }

  for (int i = 0; i < num_rocks; ++i) {
    double x = dist_x(rng);
    double y = dist_y(rng);
    double target_size = dist_size(rng);
    double yaw_angle = dist_yaw(rng);

    // Calculate scale factor to achieve target size
    double scale = target_size / mesh_base_size;
    
    if (i == 0) {
        std::cout << "  Example Rock 0: Target=" << target_size << " Scale=" << scale << std::endl;
    }

    // Create rock body
    auto rock = chrono_types::make_shared<ChBody>();
    rock->SetPos(ChVector3d(x, y, target_size * 0.3));  // Partially buried
    rock->SetRot(QuatFromAngleZ(yaw_angle));
    rock->SetFixed(true);
    rock->SetMass(2500 * target_size * target_size * target_size);

    // Add collision shape (use ellipsoid approximation for performance)
    ChVector3d collision_size(target_size * 0.5, target_size * 0.5, target_size * 0.4);
    auto coll_shape = chrono_types::make_shared<ChCollisionShapeEllipsoid>(
        rock_material, collision_size);
    rock->AddCollisionShape(coll_shape);
    rock->EnableCollision(true);

    // Create a scaled copy of the mesh for this rock (Irrlicht doesn't respect SetScale)
    auto scaled_mesh = chrono_types::make_shared<ChTriangleMeshConnected>(*rock_mesh);
    scaled_mesh->Transform(-mesh_center, ChMatrix33<>(1));  // Center at origin first
    scaled_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale));  // Then scale

    // Create visual shape from the pre-scaled mesh
    auto rock_vis = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    rock_vis->SetMesh(scaled_mesh);
    rock_vis->SetMutable(false);
    
    // Apply texture (SetTexture takes filename directly)
    rock_vis->SetTexture(rock_texture_path);

    rock->AddVisualShape(rock_vis);

    sys.Add(rock);
  }

  std::cout << "Added " << num_rocks << " rock obstacles (mesh: " << rock_mesh_path << ")" << std::endl;
}

// =============================================================================
// Main function
// =============================================================================
int main(int argc, char *argv[]) {
  std::cout << "SCM Deformable Terrain Teleoperation Demo" << std::endl;
  std::cout << "Copyright (c) 2024 projectchrono.org" << std::endl;
  std::cout << "Chrono version: " << CHRONO_VERSION << std::endl;

  // Parse command line
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);
  if (!cli.Parse(argc, argv, true))
    return 0;

  // Set data paths
  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // Parse terrain options
  std::string terrain_type_str = cli.GetAsType<std::string>("terrain_type");
  if (terrain_type_str == "heightmap") terrain_type = TerrainType::HEIGHTMAP;
  else if (terrain_type_str == "random") terrain_type = TerrainType::RANDOM;
  else terrain_type = TerrainType::FLAT;

  terrain_size_x = cli.GetAsType<double>("terrain_size_x");
  terrain_size_y = cli.GetAsType<double>("terrain_size_y");
  scm_delta = cli.GetAsType<double>("scm_delta");
  heightmap_file = cli.GetAsType<std::string>("heightmap_file");
  heightmap_min = cli.GetAsType<double>("heightmap_min");
  heightmap_max = cli.GetAsType<double>("heightmap_max");
  random_seed = cli.GetAsType<int>("random_seed");
  random_amplitude = cli.GetAsType<double>("random_amplitude");
  random_octaves = cli.GetAsType<int>("random_octaves");
  random_frequency = cli.GetAsType<double>("random_frequency");

  // Parse soil parameters
  bekker_kphi = cli.GetAsType<double>("bekker_kphi");
  bekker_n = cli.GetAsType<double>("bekker_n");
  mohr_friction_deg = cli.GetAsType<double>("mohr_friction");
  elastic_stiffness = cli.GetAsType<double>("elastic_stiffness");

  // Parse vehicle options
  std::string tire_type_str = cli.GetAsType<std::string>("tire_type");
  if (tire_type_str == "cylindrical") tire_type = TireType::CYLINDRICAL;
  else if (tire_type_str == "rigid") tire_type = TireType::RIGID;
  else tire_type = TireType::LUGGED;

  initLoc.x() = cli.GetAsType<double>("init_x");
  initLoc.y() = cli.GetAsType<double>("init_y");
  cruise_speed = cli.GetAsType<double>("cruise_speed");

  // Parse visualization options
  std::string vis_mode_str = cli.GetAsType<std::string>("vis_mode");
  if (vis_mode_str == "irrlicht") vis_mode = VisMode::IRRLICHT_ONLY;
  else if (vis_mode_str == "sensor") vis_mode = VisMode::SENSOR_ONLY;
  else vis_mode = VisMode::BOTH;

  render_wireframe = cli.GetAsType<bool>("wireframe");
  render_sinkage = cli.GetAsType<bool>("sinkage");
  render_fps = cli.GetAsType<double>("render_fps");

  // Parse sensor options
  enable_camera = cli.GetAsType<bool>("enable_camera");
  enable_lidar = cli.GetAsType<bool>("enable_lidar");
  lidar_offset.x() = cli.GetAsType<double>("lidar_offset_x");
  lidar_max_range = cli.GetAsType<double>("lidar_range");

  // Parse obstacle options
  enable_obstacles = cli.GetAsType<bool>("enable_obstacles");
  num_rocks = cli.GetAsType<int>("num_rocks");
  rock_min_size = cli.GetAsType<double>("rock_min_size");
  rock_max_size = cli.GetAsType<double>("rock_max_size");

  // Parse simulation options
  step_size = cli.GetAsType<double>("step_size");
  t_end = cli.GetAsType<double>("t_end");
  delay_val = cli.GetAsType<float>("delay_val");
  cam_delay_val = cli.GetAsType<float>("cam_delay_val");
  delay_config_file = cli.GetAsType<std::string>("delay_config");
  enable_ros_bridge = cli.GetAsType<bool>("ros_bridge");

  // Initialize random number generator
  std::mt19937 rng(random_seed);

  // Generate random heightmap if needed
  std::string generated_heightmap = "";
  if (terrain_type == TerrainType::RANDOM) {
    generated_heightmap = "/tmp/scm_teleop_heightmap.bmp";
    GenerateRandomHeightmap(generated_heightmap, 256, 256, 
                           random_amplitude, random_octaves, random_frequency, random_seed);
    heightmap_file = generated_heightmap;
    terrain_type = TerrainType::HEIGHTMAP;
  }

  // --------------------
  // Create HMMWV vehicle
  // --------------------
  HMMWV_Full hmmwv;
  hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
  hmmwv.SetContactMethod(contact_method);
  hmmwv.SetChassisFixed(false);
  hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  hmmwv.SetEngineType(EngineModelType::SHAFTS);
  hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
  hmmwv.SetDriveType(DrivelineTypeWV::AWD);

  switch (tire_type) {
    case TireType::CYLINDRICAL:
      hmmwv.SetTireType(TireModelType::RIGID_MESH);
      break;
    case TireType::LUGGED:
      hmmwv.SetTireType(TireModelType::RIGID);
      break;
    case TireType::RIGID:
      hmmwv.SetTireType(TireModelType::RIGID);
      break;
  }
  hmmwv.Initialize();

  hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
  hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
  hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);

  ChSystem* sys = hmmwv.GetSystem();

  // -----------------------------------------------------------
  // Set tire contact material and visualization
  // -----------------------------------------------------------
  float Y_t = 1.0e6f;
  float cr_t = 0.1f;
  float mu_t = 0.8f;
  auto wheel_material = chrono_types::make_shared<ChContactMaterialSMC>();
  wheel_material->SetFriction(mu_t);
  wheel_material->SetYoungModulus(Y_t);
  wheel_material->SetRestitution(cr_t);

  switch (tire_type) {
    case TireType::CYLINDRICAL:
    case TireType::RIGID:
      hmmwv.SetTireVisualizationType(VisualizationType::MESH);
      break;
    case TireType::LUGGED:
      hmmwv.SetTireVisualizationType(VisualizationType::NONE);
      for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
        CreateLuggedGeometry(axle->m_wheels[0]->GetSpindle(), wheel_material);
        CreateLuggedGeometry(axle->m_wheels[1]->GetSpindle(), wheel_material);
      }
      break;
  }

  // Create attached body for camera tracking
  auto attached_body = std::make_shared<ChBody>();
  sys->AddBody(attached_body);
  attached_body->EnableCollision(false);
  attached_body->SetFixed(true);

  // ------------------
  // Create SCM terrain
  // ------------------
  SCMTerrain terrain(sys);
  terrain.SetSoilParameters(bekker_kphi,
                            bekker_kc,
                            bekker_n,
                            mohr_cohesion,
                            mohr_friction_deg,
                            janosi_shear,
                            elastic_stiffness,
                            damping);

  // Add active domains around each wheel for efficiency
  for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
    terrain.AddActiveDomain(axle->m_wheels[0]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1.5, 0.75, 1.5));
    terrain.AddActiveDomain(axle->m_wheels[1]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1.5, 0.75, 1.5));
  }

  // Initialize terrain based on type
  switch (terrain_type) {
    case TerrainType::FLAT:
      terrain.Initialize(terrain_size_x, terrain_size_y, scm_delta);
      break;
    case TerrainType::HEIGHTMAP:
      if (!heightmap_file.empty()) {
        terrain.Initialize(heightmap_file, terrain_size_x, terrain_size_y,
                          heightmap_min, heightmap_max, scm_delta);
      } else {
        std::cerr << "Heightmap file not specified, using flat terrain." << std::endl;
        terrain.Initialize(terrain_size_x, terrain_size_y, scm_delta);
      }
      break;
    default:
      terrain.Initialize(terrain_size_x, terrain_size_y, scm_delta);
      break;
  }

  // SCM visualization settings
  terrain.GetMesh()->SetWireframe(render_wireframe);
  if (render_sinkage) {
    terrain.SetColormap(ChColormap::Type::FAST);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
  }

  // ------------------
  // Add rock obstacles (before visualizations so they appear in both)
  // ------------------
  if (enable_obstacles) {
    AddRockObstacles(*sys, rng);
  }

  // ------------------------
  // Create driver system
  // ------------------------
  ChSDLInterface SDLDriver;
  SDLDriver.Initialize();

  std::string joystick_file = std::string(STRINGIFY(HIL_DATA_DIR)) + "/joystick/controller_G29.json";
  SDLDriver.SetJoystickConfigFile(joystick_file);

  int auto_toggle_button = 6;
  SDLDriver.AddCallbackButtons(auto_toggle_button);

  // -------------------------------------------
  // Create Irrlicht visualization (if enabled)
  // -------------------------------------------
  std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> vis_irr;
  if (vis_mode == VisMode::IRRLICHT_ONLY || vis_mode == VisMode::BOTH) {
    vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis_irr->SetWindowTitle("SCM Teleop - Irrlicht View");
    vis_irr->SetWindowSize(1920, 1080);
    vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5);  // Third-person chase view
    vis_irr->Initialize();
    vis_irr->AddLightDirectional();
    vis_irr->AddSkyBox();
    vis_irr->AddLogo();
    vis_irr->AttachVehicle(&hmmwv.GetVehicle());
  }

  // ---------------------------------
  // Create sensor manager and sensors
  // ---------------------------------
  std::shared_ptr<ChSensorManager> manager;
  std::shared_ptr<ChCameraSensor> driver_cam;
  std::shared_ptr<ChCameraSensor> third_person_cam;
  std::shared_ptr<ChLidarSensor> lidar;

  if (vis_mode == VisMode::SENSOR_ONLY || vis_mode == VisMode::BOTH) {
    manager = chrono_types::make_shared<ChSensorManager>(sys);
    
    // Scene setup
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/sky_2_4k.hdr");
    manager->scene->SetBackground(b);
    
    float brightness = 1.5f;
    manager->scene->AddPointLight({0, 0, 10000}, {brightness, brightness, brightness}, 100000);
    manager->scene->SetAmbientLight({0.2f, 0.2f, 0.2f});
    manager->scene->SetSceneEpsilon(1e-3);
    manager->scene->EnableDynamicOrigin(true);
    manager->scene->SetOriginOffsetThreshold(500.f);

    // Driver camera (first-person view)
    if (enable_camera) {
      ChQuaterniond driver_cam_rot;
      driver_cam_rot.SetFromAngleAxis(0, {0, 1, 0});
      
      driver_cam = chrono_types::make_shared<ChCameraSensor>(
          hmmwv.GetChassisBody(),
          camera_update_rate,
          ChFrame<double>({0.5, 0.0, 1.2}, driver_cam_rot),
          1920, 1080,
          CH_PI / 3,  // FOV
          1);
      driver_cam->SetName("DriverCam");
      driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
          1920, 1080, "Driver View", false));
      driver_cam->SetLag(cam_delay_val * 0.001f);
      driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
      manager->AddSensor(driver_cam);

      // Third-person chase camera (sensor-based)
      ChQuaterniond third_person_rot;
      third_person_rot.SetFromCardanAnglesXYZ(ChVector3d(0.0, 0.15, 0.0));
      third_person_cam = chrono_types::make_shared<ChCameraSensor>(
          hmmwv.GetChassisBody(),
          camera_update_rate,
          ChFrame<double>({-8.0, 0.0, 3.0}, third_person_rot),
          1920, 1080,
          CH_PI / 4,
          1);
      third_person_cam->SetName("ThirdPersonCam");
      third_person_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
          1920, 1080, "Third Person View", false));
      manager->AddSensor(third_person_cam);
    }

    // LiDAR sensor (positioned 5m in front of vehicle)
    if (enable_lidar) {
      lidar = chrono_types::make_shared<ChLidarSensor>(
          hmmwv.GetChassisBody(),
          lidar_update_rate,
          ChFrame<double>(lidar_offset, QuatFromAngleAxis(0, ChVector3d(0, 1, 0))),
          lidar_horizontal_samples,
          lidar_vertical_channels,
          static_cast<float>(CH_2PI),              // Horizontal FOV (360 degrees)
          static_cast<float>(CH_PI / 12),          // Max vertical angle
          static_cast<float>(-CH_PI / 6),          // Min vertical angle
          static_cast<float>(lidar_max_range),
          LidarBeamShape::RECTANGULAR,
          2,      // Sample radius
          0.003f, // Vertical divergence
          0.003f, // Horizontal divergence
          LidarReturnMode::STRONGEST_RETURN);
      lidar->SetName("FrontLidar");
      lidar->SetLag(0.01f);
      lidar->SetCollectionWindow(0.04f);
      
      // Lidar filter pipeline
      lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
      lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
      lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(
          1280, 720, 1.0f, "Lidar Point Cloud"));
      
      manager->AddSensor(lidar);
    }

    // Reconstruct sensor scenes to ensure all bodies (including obstacles) are included
    manager->ReconstructScenes();
  }

  // ---------------------
  // Initialize ROS2 bridge
  // ---------------------
#ifdef ENABLE_ROS2_BRIDGE
  std::unique_ptr<Ros2Bridge> ros_bridge;
  if (enable_ros_bridge) {
    std::cout << "Initializing ROS2 bridge..." << std::endl;
    Ros2BridgeConfig ros_config;
    ros_config.enabled = true;
    ros_bridge = Ros2Bridge::Create(ros_config);
    if (!ros_bridge) {
      std::cout << "ROS2 bridge requested but initialization failed; continuing without it." << std::endl;
    }
  }
#else
  if (enable_ros_bridge) {
    std::cout << "ROS2 bridge requested but Chrono was built without ROS2 dependencies." << std::endl;
  }
#endif

  // ---------------------
  // Initialize delay simulation
  // ---------------------
  auto normalDist = std::make_shared<chrono::hil::NormalDistribution>(delay_val, 0.001f);
  ChDelaySim delay_sim(normalDist, 1e9f);

  bool use_json_delay_config = false;
  if (delay_val > 0.0f) {
    std::cout << "Using command-line delay value: " << delay_val << "ms" << std::endl;
  } else if (!delay_config_file.empty()) {
    std::string full_delay_config_path;
    if (delay_config_file[0] == '/') {
      full_delay_config_path = delay_config_file;
    } else {
      full_delay_config_path = std::string(STRINGIFY(HIL_DATA_DIR)) + "/" + delay_config_file;
    }
    
    if (delay_sim.loadDelayConfig(full_delay_config_path)) {
      use_json_delay_config = true;
      delay_sim.setLogging(true);
      std::cout << "Loaded delay configuration from: " << full_delay_config_path << std::endl;
    }
  }

  // ---------------------
  // Solver settings
  // ---------------------
  int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
  sys->SetNumThreads(num_threads_chrono, 1, 1);

  // -----------------
  // Initialize output
  // -----------------
  const std::string out_dir = GetChronoOutputPath() + "SCM_TELEOP";
  if (!filesystem::create_directory(filesystem::path(out_dir))) {
    std::cout << "Warning: Could not create output directory " << out_dir << std::endl;
  }

  // ---------------
  // Simulation loop
  // ---------------
  std::cout << "\n=== Starting Simulation ===" << std::endl;
  std::cout << "Total vehicle mass: " << hmmwv.GetVehicle().GetMass() << " kg" << std::endl;
  std::cout << "Terrain type: " << terrain_type_str << std::endl;
  std::cout << "Terrain size: " << terrain_size_x << " x " << terrain_size_y << " m" << std::endl;
  std::cout << "Press controller button " << auto_toggle_button << " to toggle auto/manual mode" << std::endl;

  ChRealtimeCumulative realtime_timer;
  ChBoostOutStreamer boost_streamer(UNITY_IP_OUT, UNITY_PORT_OUT);

  DriverInputs driver_inputs;
  int step_number = 0;
  int render_frame = 0;
  int auto_mode = 0;
  auto last_auto_toggle = std::chrono::system_clock::now();

  std::vector<int> check_button_idx;
  std::vector<int> check_button_val;

#ifdef ENABLE_ROS2_BRIDGE
  teleop_bridge_msgs::msg::ControlCommand last_safety_cmd;
  double last_safety_time = -1.0;
  const double SAFETY_CMD_TIMEOUT = 0.2;
#endif

  while (true) {
    double time = sys->GetChTime();

    // Check for simulation end
    if (time >= t_end)
      break;

    // Check Irrlicht window
    if (vis_irr && !vis_irr->Run())
      break;

    // Update delay based on time if using JSON config
    if (use_json_delay_config) {
      delay_sim.updateDelayForTime(time);
    }

    // Get vehicle state
    ChVector3d pos = hmmwv.GetChassis()->GetPos();
    ChQuaterniond rot = hmmwv.GetChassis()->GetRot();

    // Update attached body for camera tracking
    auto euler_rot = rot.GetCardanAnglesXYZ();
    euler_rot.x() = 0.0;
    euler_rot.y() = 0.0;
    ChQuaterniond y_0_rot;
    y_0_rot.SetFromCardanAnglesXYZ(euler_rot);
    attached_body->SetPos(pos);
    attached_body->SetRot(y_0_rot);

    // Process control inputs at 50Hz
    if (step_number % 20 == 0) {
      // Create control packet
      std::vector<float> floats = {
        static_cast<float>(auto_mode),
        SDLDriver.GetSteering(),
        SDLDriver.GetThrottle(),
        SDLDriver.GetBraking()
      };

      std::vector<char> serializedData = serializeFloats(floats);
      delay_sim.addPacket(serializedData);

      std::vector<char> receivedData = delay_sim.getDelayedPacket();
      if (!receivedData.empty()) {
        std::vector<float> receivedFloats = deserializeFloats(receivedData);
        if (receivedFloats[0] == 0) {  // Manual mode
          driver_inputs.m_steering = receivedFloats[1];
          driver_inputs.m_throttle = receivedFloats[2];
          driver_inputs.m_braking = receivedFloats[3];
        } else {  // Auto mode - simple cruise control
          driver_inputs.m_steering = 0.0;
          double speed_error = cruise_speed - hmmwv.GetVehicle().GetSpeed();
          driver_inputs.m_throttle = std::clamp(speed_error * 0.5, 0.0, 1.0);
          driver_inputs.m_braking = std::clamp(-speed_error * 0.5, 0.0, 1.0);
        }
      }

#ifdef ENABLE_ROS2_BRIDGE
      if (ros_bridge) {
        if (auto cmd = ros_bridge->GetSafetyCommand()) {
          if (cmd->valid) {
            last_safety_cmd = *cmd;
            last_safety_time = time;
          }
        }
        ros_bridge->PublishDriverInput(time, auto_mode, driver_inputs, driver_inputs);
        ros_bridge->PublishEgoState(time, hmmwv.GetVehicle(), driver_inputs.m_steering, 0.0);
      }
#endif
    }

#ifdef ENABLE_ROS2_BRIDGE
    // Apply safety override
    if (ros_bridge && last_safety_time > 0 && (time - last_safety_time) < SAFETY_CMD_TIMEOUT) {
      double alpha = last_safety_cmd.throttle;
      if (alpha >= 0.0) {
        driver_inputs.m_throttle = alpha;
        driver_inputs.m_braking = 0.0;
      } else {
        driver_inputs.m_throttle = 0.0;
        driver_inputs.m_braking = -alpha;
      }
      driver_inputs.m_steering = last_safety_cmd.steering;
    }
#endif

    // Render Irrlicht view
    if (vis_irr && time >= render_frame / render_fps) {
      vis_irr->BeginScene();
      vis_irr->Render();
      vis_irr->EndScene();
      vis_irr->Synchronize(time, driver_inputs);
      render_frame++;
    }

    // Update modules
    terrain.Synchronize(time);
    hmmwv.Synchronize(time, driver_inputs, terrain);

    // Advance simulation
    terrain.Advance(step_size);
    hmmwv.Advance(step_size);
    if (vis_irr) {
      vis_irr->Advance(step_size);
    }

    // Update sensor manager
    if (manager) {
      manager->Update();
    }

    // Handle controller input
    SDLDriver.GetButtonStatus(check_button_idx, check_button_val);
    auto button_now = std::chrono::system_clock::now();
    for (size_t bi = 0; bi < check_button_idx.size(); ++bi) {
      if (check_button_val[bi] == 1 && check_button_idx[bi] == auto_toggle_button) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(button_now - last_auto_toggle).count() >= 300) {
          auto_mode = (auto_mode + 1) % 2;
          std::cout << "Mode switched to: " << (auto_mode ? "AUTO" : "MANUAL") << std::endl;
          last_auto_toggle = button_now;
        }
      }
    }

    // Stream data to Unity (if connected)
    if (step_number % 50 == 0) {
      boost_streamer.AddData(time);
      boost_streamer.AddData(hmmwv.GetVehicle().GetSpeed() * MS_2_MPH);
      boost_streamer.AddData(hmmwv.GetVehicle().GetEngine()->GetMotorSpeed() * RADS_2_RPM);
      boost_streamer.AddData(pos.x());
      boost_streamer.AddData(pos.y());
      boost_streamer.AddData(driver_inputs.m_throttle);
      boost_streamer.AddData(driver_inputs.m_braking);
      boost_streamer.AddData(driver_inputs.m_steering);
      boost_streamer.AddData(auto_mode);
      boost_streamer.Synchronize();
    }

    // Realtime sync
    if (step_number == 0) {
      realtime_timer.Reset();
    }
    realtime_timer.Spin(time);

    // Check SDL for quit
    if (SDLDriver.Synchronize() == 1)
      break;

    step_number++;

    // Print status periodically
    if (step_number % 1000 == 0) {
      std::cout << "Time: " << std::fixed << std::setprecision(2) << time 
                << "s | Speed: " << hmmwv.GetVehicle().GetSpeed() * MS_2_MPH << " mph"
                << " | Mode: " << (auto_mode ? "AUTO" : "MANUAL") << std::endl;
    }
  }

  std::cout << "\n=== Simulation Complete ===" << std::endl;
  return 0;
}
