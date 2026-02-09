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
// Authors: Jason Zhou
// =============================================================================

#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include <chrono>

#include "chrono_hil/driver/ChIDM_Follower.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_hil/driver/ChCSLDriver.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_hil/driver/ChSDLInterface.h"
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
#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChBodyEasy.h"

#include "project/Teleop/Ros2Bridge.h"
#include "project/Teleop/ActorPlayback.h"
#include "project/Teleop/PathUtils.h"
#include "project/Teleop/ActorConfigLoader.h"

#include "chrono_hil/network/udp/ChBoostOutStreamer.h"
#include "chrono_hil/network/sim/ChDelaySim.h"

// SynChrono includes for distributed simulation
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

// FastDDS Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::hil;
using namespace chrono::utils;
using namespace chrono::sensor;
using namespace chrono::synchrono;

// =============================================================================
// Constants
// =============================================================================

const double RADS_2_RPM = 30 / CH_PI;
const double RADS_2_DEG = 180 / CH_PI;
const double MS_2_MPH = 2.2369;
const double M_2_FT = 3.28084;
const double G_2_MPSS = 9.81;
const double rads2rpm = 30 / CH_PI;

// =============================================================================
// Global Configuration
// =============================================================================

bool render = true;
ChVector3d driver_eyepoint(-0.45, 0.4, 0.98);

// Initial vehicle location and orientation
ChVector3d initLoc(-91.788, 98.647, 0.25);
ChQuaterniond initRot(1, 0, 0, 0);
double cruise_speed = 20.0;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-6;

// Simulation end time
double t_end = 1000;

// Unity dashboard streaming out
const std::string UNITY_IP_OUT = "127.0.0.1";
const int UNITY_PORT_OUT = 1209;

// =============================================================================
// Scenario Configuration
// =============================================================================

std::string scenario_filename = "test_parameters_1.json";
std::vector<std::string> obj_filenames;
std::vector<ChVector3d> obj_pos;
std::vector<ChVector3d> obj_rot;
std::vector<double> obj_scale;
float delay_val = 0.0;
float cam_delay_val = 0.2;
int lane = 0;
std::string delay_config_file = "network/delay_configs/delay_config.json";

// Recording configuration
bool record_mode = false;
bool recording_active = false;
double record_interval = 0.02;
std::string record_output_file = "recorded_path.csv";
int auto_toggle_button = 6;
int record_toggle_button = 7;
int finish_record_button = 11;
bool enable_ros_bridge = false;
std::vector<ChVector3d> recorded_positions;

// Actor configuration
std::string actors_config_file = "";
std::vector<ActorPlayback> playback_actors;
ActorPlayback distributed_actor_state;

// SynChrono configuration
double heartbeat = 0.02; // 50 Hz synchronization
int node_id = 1;
int num_nodes = 1;
int lead_node_id = 2;  // Node ID of the lead vehicle (for Unity streaming)

// Relative timing configuration (actors start when ego starts moving)
bool start_on_ego_input = false;
double ego_start_time = -1.0;  // -1 means ego hasn't started yet
double ego_velocity_threshold = 0.5;  // m/s threshold to detect ego moving

// =============================================================================
void AddCommandLineOptions(ChCLI &cli)
{
  cli.AddOption<std::string>("Simulation", "sim_params",
                             "Path to simulation configuration file",
                             scenario_filename);
  cli.AddOption<float>("Simulation", "delay_val", "Delay value", std::to_string(delay_val));
  cli.AddOption<float>("Simulation", "cam_delay_val", "Camera Delay value", std::to_string(cam_delay_val));
  cli.AddOption<std::string>("Simulation", "delay_config",
                             "Path to delay configuration JSON file",
                             delay_config_file);
  cli.AddOption<bool>("Simulation", "ros_bridge", "Enable ROS2 safety bridge", "false");
  cli.AddOption<bool>("Simulation", "record_mode", "Enable waypoint recording mode", "false");
  cli.AddOption<std::string>("Recording", "record_output", "Output file path for recorded waypoints", record_output_file);
  cli.AddOption<double>("Recording", "record_interval", "Minimum time between recorded samples (seconds)", std::to_string(record_interval));
  cli.AddOption<int>("Recording", "auto_button", "Joystick button index for auto/manual toggle", std::to_string(auto_toggle_button));
  cli.AddOption<int>("Recording", "record_button", "Joystick button index for record toggle", std::to_string(record_toggle_button));
  cli.AddOption<int>("Recording", "finish_button", "Joystick button index to finish recording and exit", std::to_string(finish_record_button));
  cli.AddOption<std::string>("Playback", "actors_config", "Path to actor playback configuration file", actors_config_file);
  
  // SynChrono / DDS options for distributed simulation
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node (1 = ego with SDL, 2+ = path-following actors)", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Total number of Nodes in the simulation", "1");
  cli.AddOption<double>("DDS", "heartbeat", "SynChrono heartbeat interval (seconds)", std::to_string(heartbeat));
  cli.AddOption<int>("DDS", "lead_node_id", "Node ID of the lead vehicle for Unity streaming", std::to_string(lead_node_id));
  cli.AddOption<std::vector<std::string>>("DDS", "ip", "IP Addresses for DDS initialPeersList", "127.0.0.1");
  
  // Relative timing options (actors start when ego starts moving)
  cli.AddOption<bool>("Timing", "start_on_ego_input", "Start actors when ego receives first input (relative timing mode)", "false");
  cli.AddOption<double>("Timing", "ego_velocity_threshold", "Velocity threshold (m/s) to detect ego moving (for actor nodes)", std::to_string(ego_velocity_threshold));
}
// =============================================================================
void ReadParameterFiles()
{
  { // Scenario parameter file
    rapidjson::Document d;
    vehicle::ReadFileJSON(std::string(STRINGIFY(HIL_DATA_DIR)) + std::string("/Environments/nads/parameters/") + scenario_filename, d);

    if (d.HasMember("auto_speed"))
    {
      cruise_speed = d["auto_speed"].GetDouble();
    }

    if (d.HasMember("lane"))
    {
      lane = d["lane"].GetInt();
    }

    if (d.HasMember("ego_loc"))
    {
      auto marr = d["ego_loc"].GetArray();
      for (int j = 0; j < 3; j++)
      {
        initLoc[j] = marr[j].GetDouble();
      }
    }

    if (d.HasMember("ego_rot"))
    {
      auto marr = d["ego_rot"].GetArray();
      ChVector3d euler_rot;
      for (int j = 0; j < 3; j++)
      {
        euler_rot[j] = marr[j].GetDouble();
      }
      initRot.SetFromCardanAnglesXYZ(euler_rot);
    }

    int mesh_ct = 0;
    std::string meshname = "object" + std::to_string(mesh_ct);
    while (d.HasMember(meshname.c_str()))
    {
      if (d[meshname.c_str()].HasMember("filename"))
      {
        obj_filenames.push_back(d[meshname.c_str()]["filename"].GetString());
      }
      if (d[meshname.c_str()].HasMember("positions"))
      {
        auto marr = d[meshname.c_str()]["positions"].GetArray();

        ChVector3d temp_pos;
        for (int j = 0; j < 3; j++)
        {
          temp_pos[j] = marr[j].GetDouble();
        }
        obj_pos.push_back(temp_pos);
      }
      if (d[meshname.c_str()].HasMember("rotations"))
      {
        auto marr = d[meshname.c_str()]["rotations"].GetArray();
        ChVector3d temp_rot;
        for (int j = 0; j < 3; j++)
        {
          temp_rot[j] = marr[j].GetDouble();
        }
        obj_rot.push_back(temp_rot);
      }
      if (d[meshname.c_str()].HasMember("scales"))
      {
        obj_scale.push_back(d[meshname.c_str()]["scales"].GetDouble());
      }

      mesh_ct++;
      meshname = "object" + std::to_string(mesh_ct);
    }
  }
}

// =============================================================================
void addObjs(ChSystem &sys);

// =============================================================================
void AddCommandLineOptions(ChCLI &cli);

// =============================================================================
std::vector<char> serializeFloats(const std::vector<float> &floatVec)
{
  std::vector<char> byteVec(floatVec.size() * sizeof(float));
  char *bytePointer = byteVec.data();

  for (const float &value : floatVec)
  {
    std::memcpy(bytePointer, &value, sizeof(float));
    bytePointer += sizeof(float);
  }

  return byteVec;
}

std::vector<float> deserializeFloats(const std::vector<char> &byteVec)
{
  std::vector<float> floatVec(byteVec.size() / sizeof(float));
  const char *bytePointer = byteVec.data();

  for (float &value : floatVec)
  {
    std::memcpy(&value, bytePointer, sizeof(float));
    bytePointer += sizeof(float);
  }

  return floatVec;
}

// =============================================================================

int main(int argc, char *argv[])
{
  // get cli
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string engine_filename =
      vehicle::GetDataFile("audi/json/audi_EngineSimpleMap.json");
  std::string transmission_filename = vehicle::GetDataFile(
      "audi/json/audi_AutomaticTransmissionSimpleMap.json");
  std::string tire_filename =
      vehicle::GetDataFile("audi/json/audi_TMeasyTire.json");
  // Note: Using Sedan.json as zombie since audi.json doesn't exist in standard Chrono data
  // The Sedan has similar proportions to the Audi for visualization purposes
  std::string zombie_filename =
      CHRONO_DATA_DIR + std::string("synchrono/vehicle/Sedan.json");

  scenario_filename = cli.GetAsType<std::string>("sim_params");
  delay_val = cli.GetAsType<float>("delay_val");
  cam_delay_val = cli.GetAsType<float>("cam_delay_val");
  delay_config_file = cli.GetAsType<std::string>("delay_config");
  record_mode = cli.GetAsType<bool>("record_mode");
  record_output_file = cli.GetAsType<std::string>("record_output");
  record_interval = cli.GetAsType<double>("record_interval");
  auto_toggle_button = cli.GetAsType<int>("auto_button");
  record_toggle_button = cli.GetAsType<int>("record_button");
  finish_record_button = cli.GetAsType<int>("finish_button");
  actors_config_file = cli.GetAsType<std::string>("actors_config");
  enable_ros_bridge = cli.GetAsType<bool>("ros_bridge");
  
  // Parse SynChrono/DDS options
  node_id = cli.GetAsType<int>("node_id");
  num_nodes = cli.GetAsType<int>("num_nodes");
  heartbeat = cli.GetAsType<double>("heartbeat");
  lead_node_id = cli.GetAsType<int>("lead_node_id");
  const std::vector<std::string> ip_list = cli.GetAsType<std::vector<std::string>>("ip");
  
  // Parse relative timing options
  start_on_ego_input = cli.GetAsType<bool>("start_on_ego_input");
  ego_velocity_threshold = cli.GetAsType<double>("ego_velocity_threshold");
  
  // Determine if this node is the ego (node_id == 1) or an actor node
  const bool is_ego_node = (node_id == 1);
  const int actor_index = node_id - 2; // Actor index in config (node 2 = actor 0, node 3 = actor 1, etc.)
  
  std::cout << "=== SynChrono Configuration ===" << std::endl;
  std::cout << "Node ID: " << node_id << " / " << num_nodes << std::endl;
  std::cout << "Role: " << (is_ego_node ? "EGO (SDL driver)" : "ACTOR (path follower, actor index " + std::to_string(actor_index) + ")") << std::endl;
  std::cout << "Heartbeat: " << heartbeat << "s" << std::endl;
  if (start_on_ego_input)
  {
    std::cout << "Relative timing: ENABLED (actors start when ego moves)" << std::endl;
    std::cout << "  Velocity threshold: " << ego_velocity_threshold << " m/s" << std::endl;
  }
  if (!actors_config_file.empty())
  {
    std::cout << "Actors config: " << actors_config_file << std::endl;
  }
  std::cout << "===============================" << std::endl;
  
  // For actor nodes, we need the actors_config to know where to start and what path to follow
  if (!is_ego_node && actors_config_file.empty())
  {
    std::cerr << "ERROR: Actor nodes (node_id > 1) require --actors_config to be specified!" << std::endl;
    return 1;
  }
  
  if (record_interval <= 0.0)
  {
    record_interval = 0.1;
  }
  recorded_positions.clear();
  if (record_mode && is_ego_node)
  {
    std::cout << "Recording mode enabled. Use button " << record_toggle_button
              << " to toggle capture and button " << finish_record_button
              << " to finish and exit.\n";
  }

  ReadParameterFiles();
  
  // -----------------------
  // Create SynChronoManager (only if multi-node)
  // -----------------------
  const bool use_synchrono = (num_nodes > 1);
  std::unique_ptr<SynChronoManager> syn_manager_ptr;
  
  if (use_synchrono)
  {
    DomainParticipantQos qos;
    qos.name("/syn/node/" + std::to_string(node_id) + ".0");
    qos.transport().user_transports.push_back(
        std::make_shared<UDPv4TransportDescriptor>());
    qos.transport().use_builtin_transports = false;
    qos.wire_protocol().builtin.avoid_builtin_multicast = false;

    // Set the initialPeersList
    for (const auto &ip : ip_list)
    {
      Locator_t locator;
      locator.kind = LOCATOR_KIND_UDPv4;
      IPLocator::setIPv4(locator, ip);
      qos.wire_protocol().builtin.initialPeersList.push_back(locator);
    }
    
    auto communicator = chrono_types::make_shared<SynDDSCommunicator>(qos);
    syn_manager_ptr = std::make_unique<SynChronoManager>(node_id, num_nodes, communicator);
    syn_manager_ptr->SetHeartbeat(heartbeat);
  }
  else
  {
    std::cout << "Single-node mode: SynChrono disabled" << std::endl;
  }

  // --------------
  // Create systems
  // --------------
  
  // For actor nodes, get the starting position from the actor config file
  if (!is_ego_node)
  {
    ChVector3d actor_start_pos;
    ChQuaterniond actor_start_rot;
    if (GetActorStartPose(actors_config_file, actor_index, actor_start_pos, actor_start_rot))
    {
      initLoc = actor_start_pos;
      initRot = actor_start_rot;
      std::cout << "Actor node " << node_id << " (actor " << actor_index << ") starting at path position: " 
                << initLoc.x() << ", " << initLoc.y() << ", " << initLoc.z() << std::endl;
    }
    else
    {
      std::cerr << "ERROR: Failed to get start pose for actor " << actor_index << " from config!" << std::endl;
      return 1;
    }
  }

  // Create the Sedan vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = my_vehicle.GetChassis();
  my_vehicle.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
  my_vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
  my_vehicle.GetChassis()->SetFixed(false);

  auto engine = ReadEngineJSON(engine_filename);
  std::shared_ptr<ChTransmission> transmission =
      ReadTransmissionJSON(transmission_filename);
  auto powertrain =
      chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
  my_vehicle.InitializePowertrain(powertrain);
  my_vehicle.SetChassisVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSuspensionVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
  my_vehicle.SetWheelVisualizationType(VisualizationType::MESH);

  // Create and initialize the tires
  for (auto &axle : my_vehicle.GetAxles())
  {
    for (auto &wheel : axle->GetWheels())
    {
      auto tire = ReadTireJSON(tire_filename);
      tire->SetStepsize(tire_step_size);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);

    }
  }

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->EnableCollision(false);
  attached_body->SetFixed(true);

  // Create the terrain
  RigidTerrain terrain(my_vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;

  // add terrain patch (this is used for collision i.e. is the physical terrain that the vehicle interacts with)
  patch = terrain.AddPatch(patch_mat, CSYSNORM,
                           std::string(STRINGIFY(HIL_DATA_DIR)) +
                               "/Environments/nads/newnads/terrain.obj",
                           true, 0, false);

  
  // // add terrain patch (this is used for collision i.e. is the physical terrain that the vehicle interacts with)
  // patch = terrain.AddPatch(patch_mat, CSYSNORM,
  //                          std::string(STRINGIFY(HIL_DATA_DIR)) +
  //                              "/Environments/nads/newnads/terrain.obj",
  //                          true, 0, false);

  // std::cout << "HIL Data Dir: " << STRINGIFY(HIL_DATA_DIR) << std::endl;

  terrain.Initialize();

  // add vis mesh (this is used for visualization only)
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();

  terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/Environments/nads/newnads/terrain.obj",
                                  true, true);
    // terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
    //                                   "/Environments/nads/newnads/terrain.obj",
    //                               true, true);

  terrain_mesh->Transform(ChVector3d(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, -.02});
  // terrain_body->SetRot(Q_from_AngX(CH_PI_2));
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetFixed(true);
  terrain_body->EnableCollision(false);
  my_vehicle.GetSystem()->Add(terrain_body);

  // ------------------------
  // Create a Irrlicht vis
  // ------------------------
  // ChVector3d trackPoint(0.0, 0.0, 1.75);
  // int render_step = 20; 
  // auto vis =
  //     chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  // vis->SetWindowTitle("NADS");
  // vis->SetWindowSize(5760, 1080);
  // vis->SetChaseCamera(trackPoint, 6.0, 0.5);
  // vis->Initialize();
  // vis->AddLightDirectional();
  // vis->AddSkyBox();
  // vis->AddLogo();
  // vis->AttachVehicle(&my_vehicle);

  // ------------------------
  // Create the driver system
  // ------------------------
  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  // Only initialize SDL driver for ego node
  if (is_ego_node)
  {
    SDLDriver.Initialize();

    // std::string joystick_file =
    //     (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
    // std::string joystick_file =
    //     (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G29.json");
    // std::string joystick_file =
    //     (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/ps4_controller.json");
    std::string joystick_file =
        (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
    SDLDriver.SetJoystickConfigFile(joystick_file);
    SDLDriver.AddCallbackButtons(auto_toggle_button);
    if (record_mode)
    {
      SDLDriver.AddCallbackButtons(record_toggle_button);
      SDLDriver.AddCallbackButtons(finish_record_button);
    }
  }

#ifdef ENABLE_ROS2_BRIDGE
  std::unique_ptr<Ros2Bridge> ros_bridge;
  if (enable_ros_bridge && is_ego_node)
  {
    std::cout << "Initializing ROS2 bridge..." << std::endl;
    Ros2BridgeConfig ros_config;
    ros_config.enabled = true;
    ros_bridge = Ros2Bridge::Create(ros_config);
    if (!ros_bridge)
    {
      std::cout << "ROS2 bridge requested but initialization failed; continuing without it." << std::endl;
    }
  }
#else
  if (enable_ros_bridge && is_ego_node)
  {
    std::cout << "ROS2 bridge requested but Chrono was built without ROS2 dependencies." << std::endl;
  }
#endif

  // ---------------------------------
  // Add sensor manager and simulation
  // ---------------------------------

  // Sensor manager is only needed for ego node (visualization)
  std::shared_ptr<ChSensorManager> manager;
  std::shared_ptr<ChCameraSensor> driver_cam;
  std::shared_ptr<ChBodyEasySphere> indicator_green;
  std::shared_ptr<ChBodyEasySphere> indicator_red;
  std::shared_ptr<ChBodyEasySphere> indicator_slow;
  
  if (is_ego_node)
  {
    manager = chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP; // GRADIENT
    b.env_tex =
        std::string(STRINGIFY(HIL_DATA_DIR)) + ("/Environments/sky_2_4k.hdr");
    manager->scene->SetBackground(b);
    float brightness = 1.5f;
    manager->scene->AddPointLight({0, 0, 10000},
                                  {brightness, brightness, brightness}, 100000);
    manager->scene->SetAmbientLight({.1, .1, .1});
    manager->scene->SetSceneEpsilon(1e-3);
    manager->scene->EnableDynamicOrigin(true);
    manager->scene->SetOriginOffsetThreshold(500.f);

    // camera at driver's eye location for Audi
    ChQuaterniond driver_cam_rot;
    driver_cam_rot.SetFromAngleAxis(0, {0, 1, 0});
    driver_cam = chrono_types::make_shared<ChCameraSensor>(
        my_vehicle.GetChassisBody(), // body camera is attached to
        35,                          // update rate in Hz
        chrono::ChFrame<double>({0.54, .381, 1.04},
                                driver_cam_rot), // offset pose
        5760,                                    // image width
        1080,                                    // image height
        3.14 / 1.5,                              // fov
        1);

    driver_cam->SetName("DriverCam");
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        5760, 1080, "Camera1", false));
    driver_cam->SetLag(cam_delay_val * 0.001);
    driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(driver_cam);

    // Create warning indicators (Green and Red spheres)
    indicator_green = chrono_types::make_shared<ChBodyEasySphere>(0.15, 100, true, false);
    indicator_green->SetPos(ChVector3d(0, 0, -100));
    indicator_green->SetFixed(true);
    indicator_green->EnableCollision(false);
    indicator_green->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, 0.0f)); // Green
    my_vehicle.GetSystem()->Add(indicator_green);

    indicator_red = chrono_types::make_shared<ChBodyEasySphere>(0.15, 100, true, false);
    indicator_red->SetPos(ChVector3d(0, 0, -100));
    indicator_red->SetFixed(true);
    indicator_red->EnableCollision(false);
    indicator_red->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 0.0f)); // Red
    my_vehicle.GetSystem()->Add(indicator_red);

    indicator_slow = chrono_types::make_shared<ChBodyEasySphere>(0.15, 100, true, false);
    indicator_slow->SetPos(ChVector3d(0, 0, -100));
    indicator_slow->SetFixed(true);
    indicator_slow->EnableCollision(false);
    indicator_slow->GetVisualShape(0)->SetColor(ChColor(1.0f, 1.0f, 0.0f)); // Yellow
    my_vehicle.GetSystem()->Add(indicator_slow);
  }

  // Initialize simulation frame counters
  int step_number = 0;

  my_vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  ChBoostOutStreamer boost_streamer(UNITY_IP_OUT, UNITY_PORT_OUT);

  DriverInputs driver_inputs;
  DriverInputs raw_inputs;

  // Lead vehicle (zombie) tracking for velocity computation
  ChVector3d prev_lead_pos(0, 0, 0);
  double prev_lead_time = 0.0;
  bool lead_initialized = false;

  addObjs(*my_vehicle.GetSystem());

  auto normalDist = std::make_shared<chrono::hil::NormalDistribution>(delay_val, 0.001f);
  // Initialize delay simulator with normal distribution
  // Very high bandwidth limit (effectively unlimited for most scenarios)
  ChDelaySim sim(normalDist, 1e9f);
  
  // Load delay configuration from JSON file if it exists
  // Priority: If delay_val is explicitly set (non-zero), use it instead of JSON
  bool use_json_delay_config = false;
  if (delay_val > 0.0f) {
    // User specified a manual delay value - use it instead of JSON
    std::cout << "Using command-line delay value: " << delay_val << "ms (ignoring JSON config)" << std::endl;
  } else if (!delay_config_file.empty()) {
    // No manual delay specified, try to load JSON config
    // Check if delay_config_file is an absolute path
    std::string full_delay_config_path;
    if (delay_config_file[0] == '/') {
      // Already an absolute path
      full_delay_config_path = delay_config_file;
    } else {
      // Relative path - prepend HIL_DATA_DIR
      full_delay_config_path = std::string(STRINGIFY(HIL_DATA_DIR)) + "/" + delay_config_file;
    }
    
    std::cout << "Attempting to load delay configuration from: " << full_delay_config_path << std::endl;
    if (sim.loadDelayConfig(full_delay_config_path)) {
      use_json_delay_config = true;
      sim.setLogging(true); // Enable logging when using JSON config
      std::cout << "Successfully loaded delay configuration from JSON file." << std::endl;
    } else {
      std::cout << "Failed to load delay config. Using zero delay." << std::endl;
    }
  } else {
    std::cout << "No delay configuration specified. Using zero delay." << std::endl;
  }

  std::vector<int> check_button_idx;
  std::vector<int> check_button_val;
  int auto_mode = 0;

  std::string steering_controller_file_IG_nl =
      std::string(STRINGIFY(HIL_DATA_DIR)) +
      "/Environments/nads/Driver/SteeringController_IG_nl.json";
  std::string speed_controller_file_IG_nl =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/Environments/nads/Driver/SpeedController_IG_nl.json";
  
  // Outer path only used by ego vehicle in auto mode
  std::string outer_path_file = "";
  std::shared_ptr<ChBezierCurve> outer_path;
  std::vector<double> followerParam = {30, 1.5, 2.0, 5.0, 3.0, 4.0, AUDI_LENGTH};
  
  // Create path follower driver for ego vehicle (used in auto mode)
  std::shared_ptr<ChNSFFollowerDriver> PFdriver;
  
  if (is_ego_node)
  {
    // Ego vehicle uses lane-based outer path for auto mode
    if (lane == 0)
      outer_path_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                        "/Environments/nads/bezier_curve_points.txt";
    else if (lane == 1)
      outer_path_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                        "/Environments/nads/nads_path_5.txt";

    outer_path = ChBezierCurve::Read(outer_path_file, true);
    
    PFdriver = chrono_types::make_shared<ChNSFFollowerDriver>(
        my_vehicle, steering_controller_file_IG_nl, speed_controller_file_IG_nl,
        outer_path, "road", cruise_speed * MPH_TO_MS, followerParam);
    PFdriver->Initialize();
  }

  // For actor nodes, create path follower from actor config
  std::shared_ptr<ChPathFollowerDriver> actor_path_driver;
  if (!is_ego_node)
  {
    // Load actor-specific path from config
    std::string full_actors_config_path;
    if (actors_config_file[0] == '/') {
      full_actors_config_path = actors_config_file;
    } else {
      full_actors_config_path = std::string(STRINGIFY(HIL_DATA_DIR)) + "/" + actors_config_file;
    }
    
    std::ifstream actor_ifs(full_actors_config_path);
    if (!actor_ifs.is_open()) {
      std::cerr << "ERROR: Could not open actor config file for driver: " << full_actors_config_path << std::endl;
      return 1;
    }
    std::stringstream actor_buffer;
    actor_buffer << actor_ifs.rdbuf();
    actor_ifs.close();
    
    rapidjson::Document actors_doc;
    actors_doc.Parse(actor_buffer.str().c_str());
    
    if (actors_doc.HasParseError() || !actors_doc.HasMember("actors") || !actors_doc["actors"].IsArray()) {
      std::cerr << "ERROR: Actor config file missing 'actors' array or has parse error!" << std::endl;
      return 1;
    }
    
    const auto& actors_array = actors_doc["actors"].GetArray();
    if (actor_index >= static_cast<int>(actors_array.Size())) {
      std::cerr << "ERROR: Actor index " << actor_index << " out of range (only " 
                << actors_array.Size() << " actors in config)" << std::endl;
      return 1;
    }
    
    const auto& actor_cfg = actors_array[actor_index];
    
    // Get path file
    std::string actor_path_file;
    if (actor_cfg.HasMember("path_file") && actor_cfg["path_file"].IsString()) {
      actor_path_file = actor_cfg["path_file"].GetString();
      if (actor_path_file[0] != '/') {
        actor_path_file = std::string(STRINGIFY(HIL_DATA_DIR)) + "/" + actor_path_file;
      }
    } else {
      std::cerr << "ERROR: Actor " << actor_index << " missing 'path_file'!" << std::endl;
      return 1;
    }
    
    // Get start_time for distributed actor
    distributed_actor_state.start_time = actor_cfg.HasMember("start_time") ? actor_cfg["start_time"].GetDouble() : 0.0;
    distributed_actor_state.active = false;
    
    // Parse speed profile (object format with type and entries)
    double actor_target_speed = cruise_speed * MPH_TO_MS;
    if (actor_cfg.HasMember("speed_profile") && actor_cfg["speed_profile"].IsObject()) {
      if (!ParseSpeedProfileJSON(actor_cfg["speed_profile"], distributed_actor_state)) {
        std::cerr << "WARNING: Failed to parse speed_profile for actor " << actor_index << ", using default speed" << std::endl;
      } else {
        // For velocity profiles, use the initial speed from the first entry
        // For acceleration profiles, the speed is managed dynamically by EvaluateDesiredSpeed(),
        // so we keep a reasonable default for the driver initialization
        if (distributed_actor_state.profile_type == SpeedProfileType::VELOCITY) {
          actor_target_speed = distributed_actor_state.initial_speed;
        }
        // For acceleration profiles, keep the default cruise_speed as initial target -
        // SetDesiredSpeed() will override this during simulation
        
        std::cout << "  Speed profile type: " << (distributed_actor_state.profile_type == SpeedProfileType::VELOCITY ? "velocity" : "acceleration") << std::endl;
        std::cout << "  Initial speed: " << distributed_actor_state.initial_speed << " m/s" << std::endl;
        std::cout << "  Max decel: " << distributed_actor_state.max_decel << " m/s^2" << std::endl;
      }
    }
    
    // Get driver parameters - support both individual fields and array format
    double look_ahead = 5.0;
    double steering_p = 0.8, steering_i = 0.0, steering_d = 0.0;
    double speed_p = 0.6, speed_i = 0.05, speed_d = 0.0;
    
    if (actor_cfg.HasMember("look_ahead") && actor_cfg["look_ahead"].IsNumber()) {
      look_ahead = actor_cfg["look_ahead"].GetDouble();
    }
    // Support individual steering gain fields (steering_kp, steering_ki, steering_kd)
    if (actor_cfg.HasMember("steering_kp") && actor_cfg["steering_kp"].IsNumber()) {
      steering_p = actor_cfg["steering_kp"].GetDouble();
    }
    if (actor_cfg.HasMember("steering_ki") && actor_cfg["steering_ki"].IsNumber()) {
      steering_i = actor_cfg["steering_ki"].GetDouble();
    }
    if (actor_cfg.HasMember("steering_kd") && actor_cfg["steering_kd"].IsNumber()) {
      steering_d = actor_cfg["steering_kd"].GetDouble();
    }
    // Also support array format for backwards compatibility
    if (actor_cfg.HasMember("steering_gains") && actor_cfg["steering_gains"].IsArray()) {
      const auto& sg = actor_cfg["steering_gains"].GetArray();
      if (sg.Size() >= 3) {
        steering_p = sg[0].GetDouble();
        steering_i = sg[1].GetDouble();
        steering_d = sg[2].GetDouble();
      }
    }
    if (actor_cfg.HasMember("speed_gains") && actor_cfg["speed_gains"].IsArray()) {
      const auto& spg = actor_cfg["speed_gains"].GetArray();
      if (spg.Size() >= 3) {
        speed_p = spg[0].GetDouble();
        speed_i = spg[1].GetDouble();
        speed_d = spg[2].GetDouble();
      }
    }
    
    // Get path processing parameters
    double path_spacing = 0.5;
    double smoothing_window = 0.0;
    if (actor_cfg.HasMember("path_spacing") && actor_cfg["path_spacing"].IsNumber()) {
      path_spacing = std::max(0.05, actor_cfg["path_spacing"].GetDouble());
    }
    if (actor_cfg.HasMember("smooth_window") && actor_cfg["smooth_window"].IsNumber()) {
      smoothing_window = std::max(0.0, actor_cfg["smooth_window"].GetDouble());
    }
    
    // Load waypoints from CSV and build bezier curve (same as local playback actors)
    std::vector<ChVector3d> waypoints;
    if (!LoadWaypointCSV(actor_path_file, waypoints) || waypoints.empty()) {
      std::cerr << "ERROR: Failed to load waypoints from " << actor_path_file << std::endl;
      return 1;
    }
    
    std::vector<ChVector3d> path_points = BuildResampledPoints(waypoints, path_spacing);
    if (smoothing_window > 0.0) {
      path_points = SmoothPathPoints(path_points, path_spacing, smoothing_window);
    }
    
    if (path_points.empty()) {
      std::cerr << "ERROR: No path points after processing for actor " << actor_index << std::endl;
      return 1;
    }
    
    auto actor_path = chrono_types::make_shared<ChBezierCurve>(path_points, false);
    
    actor_path_driver = chrono_types::make_shared<ChPathFollowerDriver>(
        my_vehicle, actor_path, "actor_path", actor_target_speed);
    actor_path_driver->GetSteeringController().SetLookAheadDistance(look_ahead);
    actor_path_driver->GetSteeringController().SetGains(steering_p, steering_i, steering_d);
    actor_path_driver->GetSpeedController().SetGains(speed_p, speed_i, speed_d);
    actor_path_driver->Initialize();
    
    // Store in distributed_actor_state for speed profile evaluation
    distributed_actor_state.path_driver = actor_path_driver;
    distributed_actor_state.waypoints = path_points;
    distributed_actor_state.look_ahead_distance = look_ahead;
    distributed_actor_state.steering_kp = steering_p;
    distributed_actor_state.steering_ki = steering_i;
    distributed_actor_state.steering_kd = steering_d;
    
    std::cout << "Actor node " << node_id << " initialized path follower driver:" << std::endl;
    std::cout << "  Path file: " << actor_path_file << std::endl;
    std::cout << "  Start time: " << distributed_actor_state.start_time << " s" << std::endl;
    std::cout << "  Initial target speed: " << actor_target_speed << " m/s" << std::endl;
    std::cout << "  Look ahead: " << look_ahead << " m" << std::endl;
    std::cout << "  Steering gains (P/I/D): " << steering_p << "/" << steering_i << "/" << steering_d << std::endl;
  }

  // Only load local playback actors for ego node (distributed actors are handled via SynChrono)
  if (is_ego_node && !actors_config_file.empty())
  {
    InitializePlaybackActors(actors_config_file, my_vehicle, vehicle_filename, engine_filename, transmission_filename, tire_filename, steering_controller_file_IG_nl, speed_controller_file_IG_nl, tire_step_size, playback_actors);
  }

  // -----------------------
  // Add vehicle as SynChrono agent and initialize (only if multi-node)
  // -----------------------
  if (use_synchrono && syn_manager_ptr)
  {
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&my_vehicle, zombie_filename);
    syn_manager_ptr->AddAgent(agent);
    syn_manager_ptr->Initialize(my_vehicle.GetSystem());
    std::cout << "SynChrono initialized with " << num_nodes << " node(s)" << std::endl;
  }
  
  // Reset realtime timer AFTER SynChrono initialization completes
  // This ensures all nodes start measuring wall time from the same point,
  // avoiding the offset caused by waiting at SynChrono barriers during init
  realtime_timer.Reset();

  auto last_auto_toggle = std::chrono::system_clock::now();
  auto last_record_toggle = last_auto_toggle;
  auto last_finish_toggle = last_auto_toggle;
  bool finish_requested = false;
  double last_recorded_time = -1.0;

  auto capture_sample = [&](double sample_time, const ChVector3d &sample_pos) {
    recorded_positions.push_back(sample_pos);
    last_recorded_time = sample_time;
  };

#ifdef ENABLE_ROS2_BRIDGE
  teleop_bridge_msgs::msg::ControlCommand last_safety_cmd;
  double last_safety_time = -1.0;
  const double SAFETY_CMD_TIMEOUT = 0.2; // 200ms timeout
  bool warning_active = false;
#endif

  // simulation loop
  // For multi-node: use syn_manager.IsOk() to check for distributed sync status
  // For single-node: run until t_end or user quits
  double max_lag_observed = 0.0;
  double last_timing_report = 0.0;
  const double timing_report_interval = 2.0;  // Report every 2 seconds
  const double warmup_time = 3.0;  // Don't track max lag during warmup (SynChrono sync overhead)
  bool simulation_running = true;
  double step_start_wall = 0.0;
  double total_physics_time = 0.0;
  double total_spin_time = 0.0;
  int timing_sample_count = 0;
  
  while (simulation_running && (!use_synchrono || syn_manager_ptr->IsOk()))
  {
    step_start_wall = realtime_timer.GetTimeSeconds();
    
    auto now = std::chrono::high_resolution_clock::now();
    auto dds_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    double time = my_vehicle.GetSystem()->GetChTime();
    
    // Pre-step wall time vs sim time (this is BEFORE spin from previous iteration)
    double wall_time = realtime_timer.GetTimeSeconds();
    double current_lag = wall_time - time;
    // Only track max lag after warmup period to avoid SynChrono init overhead
    if (time > warmup_time && current_lag > max_lag_observed) {
      max_lag_observed = current_lag;
    }
    
    // Periodic timing report for all nodes - shows ACTUAL performance breakdown
    if (time - last_timing_report >= timing_report_interval && timing_sample_count > 0) {
      double avg_physics_ms = (total_physics_time / timing_sample_count) * 1000.0;
      double avg_spin_ms = (total_spin_time / timing_sample_count) * 1000.0;
      double realtime_factor = time / wall_time;  // >1 = faster than realtime, <1 = slower
      std::cout << "[Node " << node_id << " Timing] SimTime: " << std::fixed << std::setprecision(2) << time 
                << "s | RT Factor: " << std::setprecision(3) << realtime_factor
                << " | Avg Step: " << std::setprecision(2) << avg_physics_ms << "ms physics + " 
                << avg_spin_ms << "ms spin (budget: " << (step_size * 1000.0) << "ms)" << std::endl;
      last_timing_report = time;
      // Reset accumulators
      total_physics_time = 0.0;
      total_spin_time = 0.0;
      timing_sample_count = 0;
    }

    // Update delay based on current simulation time if using JSON config (ego only)
    if (is_ego_node && use_json_delay_config) {
      sim.updateDelayForTime(time);
    }

    ChVector3d pos = my_vehicle.GetChassis()->GetPos();
    ChQuaterniond rot = my_vehicle.GetChassis()->GetRot();

    auto euler_rot = rot.GetCardanAnglesXYZ();
    euler_rot.x() = 0.0;
    euler_rot.y() = 0.0;
    ChQuaterniond y_0_rot;
    y_0_rot.SetFromCardanAnglesXYZ(euler_rot);

    attached_body->SetPos(pos);
    attached_body->SetRot(y_0_rot);
#ifndef USENADS
    // End simulation
    if (time >= t_end)
      break;
#endif

    // =====================================================
    // Driver input handling - differs between ego and actor
    // =====================================================
    if (is_ego_node)
    {
      // EGO NODE: Run Control & Comms at 50Hz (20ms) to match ROS rate
      if (step_number % 20 == 0)
      {
        // Create a vector of floats
        std::vector<float> floats = {static_cast<float>(auto_mode), SDLDriver.GetSteering(), SDLDriver.GetThrottle(), SDLDriver.GetBraking()};

        // Serialize the vector of floats to a vector of chars
        std::vector<char> serializedData = serializeFloats(floats);
        sim.addPacket(serializedData);

        // Get the packet back
        std::vector<char> receivedData = sim.getDelayedPacket();
        if (!receivedData.empty())
        {
          // Deserialize the data back to floats
          std::vector<float> receivedFloats = deserializeFloats(receivedData);
          if (receivedFloats[0] == 0) // manual mode
          {
            driver_inputs.m_steering = receivedFloats[1];
            driver_inputs.m_throttle = receivedFloats[2];
            driver_inputs.m_braking = receivedFloats[3];

            // std::cout << "Manual: " << driver_inputs.m_steering << " " << driver_inputs.m_throttle << " " << driver_inputs.m_braking << std::endl;
          }
          else
          {
            driver_inputs = PFdriver->GetInputs();
          }
        }
        
        // Update raw inputs snapshot for logging/publishing
        raw_inputs = driver_inputs;
        
        // Detect first human input for relative timing mode (ego node)
        if (start_on_ego_input && ego_start_time < 0.0)
        {
          // Check if there's any meaningful input (throttle, braking, or steering)
          if (std::abs(driver_inputs.m_throttle) > 0.01 || 
              std::abs(driver_inputs.m_braking) > 0.01 ||
              std::abs(driver_inputs.m_steering) > 0.05)
          {
            ego_start_time = time;
            std::cout << "[RELATIVE TIMING] Ego start detected at time " << time << "s" << std::endl;
          }
        }

#ifdef ENABLE_ROS2_BRIDGE
        if (ros_bridge)
        {
          // Check for new command
          if (auto cmd = ros_bridge->GetSafetyCommand())
          {
            if (cmd->valid)
            {
              last_safety_cmd = *cmd;
              last_safety_time = time;
            }
          }
          
          ros_bridge->PublishDriverInput(time, auto_mode, raw_inputs, driver_inputs);
          ros_bridge->PublishEgoState(time, my_vehicle, driver_inputs.m_steering, driver_inputs.m_steering);
        }
#endif
      }

#ifdef ENABLE_ROS2_BRIDGE
      // Apply safety command (Persistent override with timeout)
      if (ros_bridge && last_safety_time > 0 && (time - last_safety_time) < SAFETY_CMD_TIMEOUT)
      {
          double alpha = last_safety_cmd.throttle;
          if (alpha >= 0.0)
          {
            driver_inputs.m_throttle = alpha;
            driver_inputs.m_braking = 0.0;
          }
          else
          {
            driver_inputs.m_throttle = 0.0;
            driver_inputs.m_braking = -alpha;
          }
          driver_inputs.m_steering = last_safety_cmd.steering;
      }
#endif
    }
    else
    {
      // ACTOR NODE: Use path follower driver for autonomous driving
      if (actor_path_driver)
      {
        // Detect ego movement for relative timing mode (actor nodes)
        if (start_on_ego_input && ego_start_time < 0.0 && use_synchrono && syn_manager_ptr)
        {
          // Look for ego zombie (node 1)
          for (auto& zombie_pair : syn_manager_ptr->GetZombies())
          {
            if (zombie_pair.first.GetNodeID() != 1)
              continue;
            
            if (auto ego_zombie = std::dynamic_pointer_cast<SynWheeledVehicleAgent>(zombie_pair.second))
            {
              // Static variables to track ego position for velocity calculation
              static ChVector3d prev_ego_zombie_pos;
              static double prev_ego_zombie_time = -1.0;
              static int ego_zombie_sample_count = 0;
              
              ChVector3d ego_pos = ego_zombie->GetZombiePos();
              
              // Need at least 2 samples to calculate velocity
              if (ego_zombie_sample_count >= 1 && time > prev_ego_zombie_time)
              {
                double dt = time - prev_ego_zombie_time;
                if (dt > 0.0001)  // Sanity check for dt
                {
                  double ego_vx = (ego_pos.x() - prev_ego_zombie_pos.x()) / dt;
                  double ego_vy = (ego_pos.y() - prev_ego_zombie_pos.y()) / dt;
                  double ego_speed = std::sqrt(ego_vx * ego_vx + ego_vy * ego_vy);
                  
                  if (ego_speed > ego_velocity_threshold && ego_speed < 100.0)  // Sanity cap at 100 m/s
                  {
                    ego_start_time = time;
                    std::cout << "[RELATIVE TIMING] Actor node " << node_id 
                              << " detected ego movement at time " << time 
                              << "s (speed: " << ego_speed << " m/s)" << std::endl;
                  }
                }
              }
              
              prev_ego_zombie_pos = ego_pos;
              prev_ego_zombie_time = time;
              ego_zombie_sample_count++;
              break;
            }
          }
        }
        
        // Check if actor should be active
        // In relative timing mode, start_time is relative to ego_start_time
        double effective_start_time = distributed_actor_state.start_time;
        if (start_on_ego_input)
        {
          // In relative mode: if ego hasn't started, actor can't start
          // If ego has started, effective_start_time = ego_start_time + config_start_time
          if (ego_start_time < 0.0)
          {
            effective_start_time = std::numeric_limits<double>::infinity();  // Never start until ego moves
          }
          else
          {
            effective_start_time = ego_start_time + distributed_actor_state.start_time;
          }
        }
        
        if (!distributed_actor_state.active && time >= effective_start_time)
        {
          distributed_actor_state.active = true;
          distributed_actor_state.last_profile_time = 0.0;
          std::cout << "Actor node " << node_id << " activated at time " << time;
          if (start_on_ego_input)
          {
            std::cout << " (relative timing: " << distributed_actor_state.start_time << "s after ego start)";
          }
          std::cout << std::endl;
        }
        
        if (distributed_actor_state.active)
        {
          // Calculate local time since actor activation
          // In relative timing mode, this is time since effective_start_time
          double local_time = time - effective_start_time;
          
          // Evaluate speed profile if defined
          if (distributed_actor_state.profile_defined)
          {
            // Check if near end of path
            bool within_stop_zone = false;
            if (!distributed_actor_state.waypoints.empty())
            {
              ChVector3d actor_pos = my_vehicle.GetChassis()->GetPos();
              ChVector3d path_end = distributed_actor_state.waypoints.back();
              double dist_to_end = (actor_pos - path_end).Length();
              double stop_distance = distributed_actor_state.look_ahead_distance > 0.0 
                                     ? distributed_actor_state.look_ahead_distance * 1.5 : 10.0;
              within_stop_zone = (dist_to_end < stop_distance);
              
              if (within_stop_zone && !distributed_actor_state.goal_reached)
              {
                distributed_actor_state.goal_reached = true;
                std::cout << "Actor node " << node_id << " approaching end of path" << std::endl;
              }
            }
            
            // Get desired speed from profile
            double desired_speed = EvaluateDesiredSpeed(distributed_actor_state, local_time, step_size, within_stop_zone);
            actor_path_driver->SetDesiredSpeed(desired_speed);
          }
          
          actor_path_driver->Synchronize(time);
          driver_inputs = actor_path_driver->GetInputs();
        }
        else
        {
          // Actor not yet active - stay stopped
          driver_inputs.m_throttle = 0.0;
          driver_inputs.m_braking = 1.0;
          driver_inputs.m_steering = 0.0;
        }
      }
      else
      {
        // No driver available for actor node - should not happen if config was loaded properly
        std::cerr << "ERROR: Actor node has no path driver!" << std::endl;
        driver_inputs.m_throttle = 0.0;
        driver_inputs.m_braking = 1.0;
        driver_inputs.m_steering = 0.0;
      }
    }

    if (is_ego_node && record_mode && recording_active)
    {
      if (last_recorded_time < 0.0 || (time - last_recorded_time) >= record_interval)
      {
        capture_sample(time, pos);
      }
    }

    // =======================
    // end data stream out section
    // =======================

    // Update modules (process inputs from other modules)
    if (use_synchrono && syn_manager_ptr)
    {
      syn_manager_ptr->Synchronize(time);  // SynChrono synchronization between nodes
    }
    terrain.Synchronize(time);
    my_vehicle.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one time for all modules
    terrain.Advance(step_size);
    my_vehicle.Advance(step_size);
    
    // Advance path follower driver for actor nodes
    if (!is_ego_node && actor_path_driver)
    {
      actor_path_driver->Advance(step_size);
    }
    
    // vis->Advance(step_size);
    
    // Local playback actors (only for ego node, distributed actors handled by SynChrono)
#ifdef ENABLE_ROS2_BRIDGE
    std::vector<TrackedVehicleState> ros_actor_states;
    if (is_ego_node && ros_bridge)
    {
      // Reserve space for local playback actors + SynChrono zombies
      size_t zombie_count = (use_synchrono && syn_manager_ptr) ? syn_manager_ptr->GetZombies().size() : 0;
      ros_actor_states.reserve(playback_actors.size() + zombie_count);
      
      // Add SynChrono zombie vehicles to ROS actor states
      if (use_synchrono && syn_manager_ptr)
      {
        uint32_t zombie_id = 1000;  // Start zombie IDs at 1000 to avoid collision with local actors
        for (auto& zombie_pair : syn_manager_ptr->GetZombies())
        {
          if (auto wheeled_zombie = std::dynamic_pointer_cast<SynWheeledVehicleAgent>(zombie_pair.second))
          {
            TrackedVehicleState state;
            state.id = zombie_id++;
            state.label = "syn_zombie";
            state.active = true;
            state.pos = wheeled_zombie->GetZombiePos();
            state.rot = wheeled_zombie->GetZombieRot();
            state.lin_vel = ChVector3d(0, 0, 0);  // Velocity not directly available from zombie
            ros_actor_states.push_back(state);
          }
        }
      }
    }
#endif
    if (is_ego_node)
    {
      for (auto &actor : playback_actors)
      {
        if (!actor.vehicle || !actor.path_driver)
          continue;

        // Activation Logic
        if (!actor.active)
        {
          if (time >= actor.start_time)
          {
            actor.active = true;
            actor.path_driver->Reset();
            actor.current_speed = actor.initial_speed;
            actor.last_profile_time = 0.0;
            actor.goal_reached = false;
          }
        }

        // Physics Logic (only if active)
        if (actor.active)
        {
          double local_time = time - actor.start_time;
          ChVector3d goal = actor.waypoints.back();
          double dist_to_goal = (actor.vehicle->GetChassis()->GetPos() - goal).Length();
          double slowdown_dist = actor.look_ahead_distance > 0.0 ? std::max(30.0, actor.look_ahead_distance * 3) : 6.0;
        bool within_stop_zone = (!actor.goal_reached && dist_to_goal < slowdown_dist);
        double steering_scale = 1.0;

        double desired_speed = EvaluateDesiredSpeed(actor, local_time, step_size, within_stop_zone);

        if (within_stop_zone && dist_to_goal > 1.0)
        {
          double ramp = std::clamp(dist_to_goal / slowdown_dist, 0.0, 1.0);
          steering_scale = ramp;
          actor.current_speed = desired_speed;
        }

        if (dist_to_goal < 1.0)
        {
          desired_speed = 0.0;
          actor.current_speed = 0.0;
          actor.goal_reached = true;
        }

        DriverInputs actor_inputs;
        if (!actor.goal_reached)
        {
          actor.path_driver->SetDesiredSpeed(desired_speed);
          actor.path_driver->Synchronize(time);
          actor.path_driver->Advance(step_size);
          actor_inputs = actor.path_driver->GetInputs();
          actor_inputs.m_steering *= steering_scale;
        }
        else
        {
          actor_inputs.m_throttle = 0.0;
          actor_inputs.m_braking = 1.0;
          actor_inputs.m_steering = 0.0;
        }
        actor.vehicle->Synchronize(time, actor_inputs, terrain);
        actor.vehicle->Advance(step_size);
      }
      else 
      {
        // Ensure inactive vehicles are synchronized to keep them in the world (visuals/collision)
        // but apply full brakes to keep them in place
        DriverInputs hold_inputs;
        hold_inputs.m_throttle = 0.0;
        hold_inputs.m_braking = 1.0;
        hold_inputs.m_steering = 0.0;
        actor.vehicle->Synchronize(time, hold_inputs, terrain);
        actor.vehicle->Advance(step_size);
      }

#ifdef ENABLE_ROS2_BRIDGE
        if (ros_bridge)
        {
          TrackedVehicleState state;
          state.id = static_cast<uint32_t>(&actor - &playback_actors[0]);
          state.label = "path";
          state.active = actor.active;
          auto body = actor.vehicle->GetChassisBody();
          state.pos = body->GetPos();
          state.rot = body->GetRot();
          state.lin_vel = body->GetPosDt();
          ros_actor_states.push_back(state);
        }
#endif
      } // end for playback_actors
    } // end if (is_ego_node) for local actors

#ifdef ENABLE_ROS2_BRIDGE
    if (is_ego_node && ros_bridge)
    {
      ros_bridge->PublishActors(time, ros_actor_states);
      if (auto warning = ros_bridge->GetWarningStatus())
      {
        warning_active = warning->warning;
        // Debug: Print score to see if it changes
        // std::cout << "[ROS2] Warning: " << warning_active << " Score: " << warning->score << std::endl;
        if (warning->warning)
        {
          std::cout << "[ROS2] Predictive warning score: " << warning->score << std::endl;
        }
      }

      // Update indicator position (fixed relative to chassis, in front of driver)
      // Camera is at {0.54, .381, 1.04}, place sphere 2m in front
      ChVector3d sphere_local_pos(2.54, 0.381, 1.04); 
      ChVector3d sphere_global_pos = my_vehicle.GetChassisBody()->TransformPointLocalToParent(sphere_local_pos);
      
      if (warning_active) {
          indicator_red->SetPos(sphere_global_pos);
          indicator_green->SetPos(ChVector3d(0, 0, -100));
      } else {
          indicator_green->SetPos(sphere_global_pos);
          indicator_red->SetPos(ChVector3d(0, 0, -100));
      }
    }
#endif

    // Update sensor manager (ego node only)
    if (is_ego_node && manager)
    {
      manager->Update();
    }

    // Increment frame number
    step_number++;
    
    // Measure physics time (everything from start of loop to here)
    double pre_spin_wall = realtime_timer.GetTimeSeconds();
    double physics_duration = pre_spin_wall - step_start_wall;

    // Visual indicator for simulation running slower than wall time (ego node only)
    // The timing stats are already printed above for all nodes
    if (is_ego_node && indicator_slow)
    {
      if (current_lag > 1.00) {
          ChVector3d sphere_local_pos_slow(2.54, 0.381 + 0.5, 1.04); 
          ChVector3d sphere_global_pos_slow = my_vehicle.GetChassisBody()->TransformPointLocalToParent(sphere_local_pos_slow);
          indicator_slow->SetPos(sphere_global_pos_slow);
      } else {
          indicator_slow->SetPos(ChVector3d(0, 0, -100));
      }
    }

    // Real-time synchronization (ego node spins, actor nodes run as fast as possible but sync via SynChrono)
    if (is_ego_node)
    {
      realtime_timer.Spin(time);
      
      // Measure spin (wait) time
      double post_spin_wall = realtime_timer.GetTimeSeconds();
      double spin_duration = post_spin_wall - pre_spin_wall;
      
      // Accumulate for averaging
      total_physics_time += physics_duration;
      total_spin_time += spin_duration;
      timing_sample_count++;

      if (step_number % 50 == 0)
      {
        // Stream out data (ego node only)
        double current_time = my_vehicle.GetSystem()->GetChTime();
        
        // 1. sim_time
        boost_streamer.AddData(current_time);
        // 2. latency_condition_ms
        boost_streamer.AddData(delay_val);
        
        // Ego vehicle data
        ChVector3d ego_pos = my_vehicle.GetChassis()->GetPos();
        ChQuaterniond ego_rot = my_vehicle.GetChassis()->GetRot();
        ChVector3d ego_vel = my_vehicle.GetChassisBody()->GetPosDt();
        auto ego_euler = ego_rot.GetCardanAnglesXYZ();
        
        // 3-5. ego_x, ego_y, ego_z
        boost_streamer.AddData(ego_pos.x());
        boost_streamer.AddData(ego_pos.y());
        boost_streamer.AddData(ego_pos.z());
        // 6. ego_yaw (degrees)
        boost_streamer.AddData(ego_euler.z() * RADS_2_DEG);
        // 7-9. ego_vx, ego_vy, ego_vz
        boost_streamer.AddData(ego_vel.x());
        boost_streamer.AddData(ego_vel.y());
        boost_streamer.AddData(ego_vel.z());
        // 10. ego_speed (mph)
        boost_streamer.AddData(my_vehicle.GetSpeed() * MS_TO_MPH);
        // 11. engine_rpm
        boost_streamer.AddData(my_vehicle.GetEngine()->GetMotorSpeed() * rads2rpm);
        // 12. steering_input
        boost_streamer.AddData(driver_inputs.m_steering);
        // 13. throttle_input
        boost_streamer.AddData(driver_inputs.m_throttle);
        // 14. brake_input
        boost_streamer.AddData(driver_inputs.m_braking);
        
        // Lead vehicle data from SynChrono zombie
        float lead_x = 0.0f, lead_y = 0.0f, lead_z = 0.0f;
        float lead_yaw = 0.0f;
        float lead_vx = 0.0f, lead_vy = 0.0f, lead_vz = 0.0f;
        float lead_speed = 0.0f;
        
        if (use_synchrono && syn_manager_ptr && !syn_manager_ptr->GetZombies().empty())
        {
          // Find zombie by lead_node_id
          for (auto& zombie_pair : syn_manager_ptr->GetZombies())
          {
            if (zombie_pair.first.GetNodeID() != lead_node_id)
              continue;
              
            if (auto wheeled_zombie = std::dynamic_pointer_cast<SynWheeledVehicleAgent>(zombie_pair.second))
            {
              ChVector3d lead_pos = wheeled_zombie->GetZombiePos();
              ChQuaterniond lead_rot = wheeled_zombie->GetZombieRot();
              auto lead_euler = lead_rot.GetCardanAnglesXYZ();
              
              lead_x = lead_pos.x();
              lead_y = lead_pos.y();
              lead_z = lead_pos.z();
              lead_yaw = lead_euler.z() * RADS_2_DEG;
              
              // Compute velocity from position delta
              if (lead_initialized && current_time > prev_lead_time)
              {
                double dt = current_time - prev_lead_time;
                lead_vx = (lead_pos.x() - prev_lead_pos.x()) / dt;
                lead_vy = (lead_pos.y() - prev_lead_pos.y()) / dt;
                lead_vz = (lead_pos.z() - prev_lead_pos.z()) / dt;
                lead_speed = std::sqrt(lead_vx * lead_vx + lead_vy * lead_vy + lead_vz * lead_vz) * MS_TO_MPH;
              }
              
              // Update previous state
              prev_lead_pos = lead_pos;
              prev_lead_time = current_time;
              lead_initialized = true;
              break;  // Found the lead vehicle, stop searching
            }
          }
        }
        
        // 15-17. lead_x, lead_y, lead_z
        boost_streamer.AddData(lead_x);
        boost_streamer.AddData(lead_y);
        boost_streamer.AddData(lead_z);
        // 18. lead_yaw (degrees)
        boost_streamer.AddData(lead_yaw);
        // 19-21. lead_vx, lead_vy, lead_vz
        boost_streamer.AddData(lead_vx);
        boost_streamer.AddData(lead_vy);
        boost_streamer.AddData(lead_vz);
        // 22. lead_speed (mph)
        boost_streamer.AddData(lead_speed);

        boost_streamer.Synchronize();
      }

      // SDL button handling (ego node only)
      SDLDriver.GetButtonStatus(check_button_idx, check_button_val);
      auto button_now = std::chrono::system_clock::now();
      for (size_t bi = 0; bi < check_button_idx.size(); ++bi)
      {
        if (check_button_val[bi] != 1)
          continue;

        if (check_button_idx[bi] == auto_toggle_button)
        {
          if (std::chrono::duration_cast<std::chrono::milliseconds>(button_now - last_auto_toggle).count() >= 300)
          {
            auto_mode = (auto_mode + 1) % 2;
            last_auto_toggle = button_now;
          }
        }
        else if (record_mode && check_button_idx[bi] == record_toggle_button)
        {
          if (std::chrono::duration_cast<std::chrono::milliseconds>(button_now - last_record_toggle).count() >= 300)
          {
            recording_active = !recording_active;
            if (recording_active)
            {
              last_recorded_time = -1.0;
              capture_sample(time, pos);
              std::cout << "Recording started: " << record_output_file << std::endl;
            }
            else
            {
              capture_sample(time, pos);
              std::cout << "Recording paused." << std::endl;
            }
            last_record_toggle = button_now;
          }
        }
        else if (record_mode && check_button_idx[bi] == finish_record_button)
        {
          if (std::chrono::duration_cast<std::chrono::milliseconds>(button_now - last_finish_toggle).count() >= 300)
          {
            if (recording_active)
            {
              capture_sample(time, pos);
              recording_active = false;
              std::cout << "Recording stopped by finish command." << std::endl;
            }
            finish_requested = true;
            last_finish_toggle = button_now;
            std::cout << "Finish recording requested." << std::endl;
          }
        }
      }

      if (finish_requested)
      {
        if (record_mode)
        {
          std::cout << "Exiting simulation after recording finish request." << std::endl;
        }
        break;
      }

      // Advance PF driver for ego auto mode
      if (PFdriver)
      {
        PFdriver->Advance(step_size);
        PFdriver->Synchronize(time, step_size);
      }

      if (SDLDriver.Synchronize() == 1)
      {
        break;
      }
    } // end if (is_ego_node) block

    // if (render == true && step_number % render_step == 0)
    // {
    //   vis->BeginScene();
    //   vis->Render();
    //   vis->EndScene();
    //   vis->Synchronize(time, driver_inputs);
    // }
  } // end simulation loop
  
  // Cleanup SynChrono
  if (use_synchrono && syn_manager_ptr)
  {
    syn_manager_ptr->QuitSimulation();
  }
  
  if (is_ego_node && record_mode)
  {
    if (!recorded_positions.empty())
    {
      std::ofstream record_stream(record_output_file);
      if (!record_stream.is_open())
      {
        std::cerr << "Failed to open " << record_output_file << " for waypoint recording output." << std::endl;
      }
      else
      {
        record_stream << std::setprecision(16);
        record_stream << "x,y,z\n";
        for (const auto &pos : recorded_positions)
        {
          record_stream << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
        }
        std::cout << "Waypoint recording written to " << record_output_file << " (" << recorded_positions.size() << " samples)." << std::endl;
      }
    }
    else
    {
      std::cout << "Recording mode enabled but no samples were captured." << std::endl;
    }
  }
  return 0;
}

void addObjs(ChSystem &sys)
{

  for (int i = 0; i < obj_filenames.size(); i++)
  {

    double cone_density = 900;
    std::shared_ptr<ChContactMaterial> rock_mat =
        ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
        obj_filenames[i], false, true);

    double mass;
    ChVector3d cog;
    ChMatrix33<> inertia;
    mesh->ComputeMassProperties(true, mass, cog, inertia);

    mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(obj_scale[i]));
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I,
                                     principal_inertia_rot);

    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    sys.Add(body);
    body->SetFixed(true);
    ChQuaterniond body_rot(1, 0, 0, 0);
    body_rot.SetFromCardanAnglesXYZ(obj_rot[i]);

    body->SetFrameRefToAbs(ChFrame<>(obj_pos[i], body_rot));
    body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
    body->SetMass(mass * cone_density);
    body->SetInertiaXX(cone_density * principal_I);

    auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape->SetMesh(mesh);
    mesh_shape->SetBackfaceCull(true);
    body->AddVisualShape(mesh_shape);
  }
}