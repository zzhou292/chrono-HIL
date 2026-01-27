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

const double RADS_2_RPM = 30 / CH_PI;
const double RADS_2_DEG = 180 / CH_PI;
const double MS_2_MPH = 2.2369;
const double M_2_FT = 3.28084;
const double G_2_MPSS = 9.81;

bool render = true;
ChVector3d driver_eyepoint(-0.45, 0.4, 0.98);

// =============================================================================

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

// Conversion factors
const double rads2rpm = 30 / CH_PI;

// =============================================================================
// std::string scenario_filename = "experiment.json";
std::string scenario_filename = "test_parameters_1.json";
std::vector<std::string> obj_filenames;
std::vector<ChVector3d> obj_pos;
std::vector<ChVector3d> obj_rot;
std::vector<double> obj_scale;
float delay_val = 0.0;
float cam_delay_val = 0.2;
int lane = 0;
std::string delay_config_file = "network/delay_configs/delay_config.json";

bool record_mode = false;
bool recording_active = false;
double record_interval = 0.02;
std::string record_output_file = "recorded_path.csv";
int auto_toggle_button = 6;
int record_toggle_button = 18;
int finish_record_button = 19;
bool enable_ros_bridge = false;

std::vector<ChVector3d> recorded_positions;

enum class SpeedProfileType
{
  VELOCITY,
  ACCELERATION
};

struct SpeedProfileSegment
{
  double start_time = 0.0;
  double end_time = std::numeric_limits<double>::infinity();
  bool until_end = false;
  bool has_explicit_end = false;
  double value = 0.0;
};

struct ActorPlayback
{
  std::shared_ptr<WheeledVehicle> vehicle;
  std::shared_ptr<ChPathFollowerDriver> path_driver;
  std::shared_ptr<ChPowertrainAssembly> powertrain;
  std::vector<ChVector3d> waypoints;
  double start_time = 0.0;
  double look_ahead_distance = -1.0;
  double path_spacing = 0.5;
  double smoothing_window = 0.0;
  double steering_kp = -1.0;
  double steering_ki = 0.0;
  double steering_kd = 0.0;
  bool active = false;
  SpeedProfileType profile_type = SpeedProfileType::VELOCITY;
  std::vector<SpeedProfileSegment> profile_segments;
  double initial_speed = 0.0;
  double current_speed = 0.0;
  double last_profile_time = 0.0;
  bool profile_defined = false;
  bool goal_reached = false;
  double max_decel = 3.0;
};

std::string actors_config_file = "";
std::vector<ActorPlayback> playback_actors;

// SynChrono configuration
double heartbeat = 0.02; // 50 Hz synchronization
int node_id = 1;
int num_nodes = 1;

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
  cli.AddOption<std::vector<std::string>>("DDS", "ip", "IP Addresses for DDS initialPeersList", "127.0.0.1");
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

std::vector<ChVector3d> BuildResampledPoints(const std::vector<ChVector3d> &samples,
                                             double spacing)
{
  std::vector<ChVector3d> points;
  if (samples.empty())
    return points;

  spacing = std::max(1e-3, spacing);
  points.push_back(samples.front());

  for (size_t i = 0; i + 1 < samples.size(); ++i)
  {
    ChVector3d start = samples[i];
    ChVector3d end = samples[i + 1];
    ChVector3d delta = end - start;
    double seg_len = delta.Length();
    if (seg_len < 1e-6)
      continue;

    ChVector3d dir = delta / seg_len;
    double dist = spacing;
    while (dist < seg_len)
    {
      points.push_back(start + dir * dist);
      dist += spacing;
    }
    points.push_back(end);
  }
  return points;
}

std::vector<ChVector3d> SmoothPathPoints(const std::vector<ChVector3d> &points,
                                         double spacing,
                                         double smoothing_window)
{
  if (points.size() <= 2 || smoothing_window <= 0.0)
    return points;

  int window_half = static_cast<int>(std::round(std::max(smoothing_window / spacing, 1.0)));
  window_half = std::max(1, window_half);
  std::vector<ChVector3d> smoothed(points.size());

  for (size_t i = 0; i < points.size(); ++i)
  {
    ChVector3d accum(0, 0, 0);
    int count = 0;
    int start = static_cast<int>(std::max<int>(0, i - window_half));
    int end = static_cast<int>(std::min<int>(points.size() - 1, i + window_half));
    for (int j = start; j <= end; ++j)
    {
      accum += points[j];
      ++count;
    }
    smoothed[i] = accum / static_cast<double>(count);
  }

  // Preserve endpoints exactly to avoid drift
  smoothed.front() = points.front();
  smoothed.back() = points.back();
  return smoothed;
}

ChQuaterniond EstimateInitialRotation(const std::vector<ChVector3d> &points)
{
  if (points.size() < 2)
    return QUNIT;

  ChVector3d dir = points[1] - points[0];
  dir.z() = 0.0;
  if (dir.Length2() < 1e-8)
    return QUNIT;
  dir.Normalize();
  double yaw = std::atan2(dir.y(), dir.x());
  ChQuaterniond rot;
  rot.SetFromAngleZ(yaw);
  return rot;
}

bool LoadWaypointCSV(const std::string &filename, std::vector<ChVector3d> &out_points)
{
  std::ifstream infile(filename);
  if (!infile.is_open())
  {
    std::cerr << "Unable to open waypoint CSV: " << filename << std::endl;
    return false;
  }

  out_points.clear();
  std::string line;
  while (std::getline(infile, line))
  {
    if (line.empty())
      continue;
    if (line[0] == '#')
      continue;
    std::stringstream ss(line);
    double x, y, z;
    char delim;
    if (!(ss >> x))
    {
      continue; // skip header or invalid lines
    }
    if (ss.peek() == ',' || ss.peek() == ';')
      ss >> delim;
    if (!(ss >> y))
      continue;
    if (ss.peek() == ',' || ss.peek() == ';')
      ss >> delim;
    if (!(ss >> z))
      continue;
    out_points.emplace_back(x, y, z);
  }

  if (out_points.size() < 2)
  {
    std::cerr << "Waypoint CSV must contain at least two points: " << filename << std::endl;
    return false;
  }

  return true;
}

bool ParseSpeedProfileJSON(const rapidjson::Value &profile_json, ActorPlayback &actor)
{
  if (!profile_json.IsObject() || !profile_json.HasMember("type") || !profile_json.HasMember("entries"))
  {
    std::cerr << "Speed profile must contain 'type' and 'entries'.\n";
    return false;
  }

  std::string type = profile_json["type"].GetString();
  if (type == "velocity")
  {
    actor.profile_type = SpeedProfileType::VELOCITY;
    actor.initial_speed = 0.0;
    actor.max_decel = profile_json.HasMember("max_decel") ? profile_json["max_decel"].GetDouble() : 3.0;
    if (actor.max_decel < 0.1)
      actor.max_decel = 3.0;
  }
  else if (type == "acceleration")
  {
    actor.profile_type = SpeedProfileType::ACCELERATION;
    actor.initial_speed = profile_json.HasMember("initial_speed") ? profile_json["initial_speed"].GetDouble() : 0.0;
    actor.current_speed = actor.initial_speed;
    actor.max_decel = profile_json.HasMember("max_decel") ? std::max(0.1, profile_json["max_decel"].GetDouble()) : 3.0;
  }
  else
  {
    std::cerr << "Unknown speed profile type: " << type << "\n";
    return false;
  }

  const auto &entries = profile_json["entries"];
  if (!entries.IsArray() || entries.Empty())
  {
    std::cerr << "Speed profile entries must be a non-empty array.\n";
    return false;
  }

  actor.profile_segments.clear();
  for (const auto &entry : entries.GetArray())
  {
    if (!entry.HasMember("start_time") || !entry.HasMember("value"))
    {
      std::cerr << "Each speed profile entry must have 'start_time' and 'value'.\n";
      return false;
    }

    SpeedProfileSegment seg;
    seg.start_time = entry["start_time"].GetDouble();
    seg.value = entry["value"].GetDouble();
    seg.until_end = entry.HasMember("until_end") && entry["until_end"].GetBool();
    if (entry.HasMember("end_time"))
    {
      seg.end_time = entry["end_time"].GetDouble();
      seg.has_explicit_end = true;
    }
    else if (entry.HasMember("duration"))
    {
      seg.end_time = seg.start_time + entry["duration"].GetDouble();
      seg.has_explicit_end = true;
    }
    else if (seg.until_end)
    {
      seg.end_time = std::numeric_limits<double>::infinity();
    }
    actor.profile_segments.push_back(seg);
  }

  std::sort(actor.profile_segments.begin(), actor.profile_segments.end(),
            [](const SpeedProfileSegment &a, const SpeedProfileSegment &b) { return a.start_time < b.start_time; });

  for (size_t i = 0; i + 1 < actor.profile_segments.size(); ++i)
  {
    auto &seg = actor.profile_segments[i];
    auto &next = actor.profile_segments[i + 1];
    if (!seg.until_end && !seg.has_explicit_end)
    {
      seg.end_time = next.start_time;
    }
    else if (!seg.until_end && seg.end_time > next.start_time)
    {
      seg.end_time = next.start_time;
    }
  }

  actor.profile_defined = true;
  actor.last_profile_time = 0.0;
  if (actor.profile_type == SpeedProfileType::VELOCITY)
  {
    actor.initial_speed = actor.profile_segments.front().value;
    actor.current_speed = actor.initial_speed;
  }
  else
  {
    actor.current_speed = actor.initial_speed;
  }
  return true;
}

const SpeedProfileSegment *GetActiveSegment(const ActorPlayback &actor, double local_time)
{
  const SpeedProfileSegment *active = nullptr;
  for (const auto &segment : actor.profile_segments)
  {
    if (local_time < segment.start_time)
      break;
    if (segment.until_end || local_time < segment.end_time)
      active = &segment;
  }
  return active;
}

double EvaluateDesiredSpeed(ActorPlayback &actor, double local_time, double step, bool within_stop_zone)
{
  if (!actor.profile_defined || actor.profile_segments.empty())
    return 0.0;

  const SpeedProfileSegment *segment = GetActiveSegment(actor, local_time);
  if (actor.profile_type == SpeedProfileType::VELOCITY)
  {
    double target = segment ? std::max(0.0, segment->value) : 0.0;
    double dt = step;
    if (within_stop_zone)
    {
      actor.current_speed = std::max(0.0, actor.current_speed - actor.max_decel * dt);
      target = std::min(target, actor.current_speed);
    }
    else
    {
      actor.current_speed = target;
    }
    return target;
  }

  double accel = segment ? segment->value : 0.0;
  double dt = local_time - actor.last_profile_time;
  if (dt < 0.0 || dt > 1.0)
    dt = step;
  if (within_stop_zone && accel > 0)
  {
    accel = -actor.max_decel;
  }
  actor.current_speed = std::max(0.0, actor.current_speed + accel * dt);
  actor.last_profile_time = local_time;
  return actor.current_speed;
}

bool InitializePlaybackActors(const std::string &config_path,
                              WheeledVehicle &reference_vehicle,
                              const std::string &vehicle_filename,
                              const std::string &engine_filename,
                              const std::string &transmission_filename,
                              const std::string &tire_filename,
                              const std::string &steering_file,
                              const std::string &speed_file)
{
  std::ifstream ifs(config_path);
  if (!ifs.is_open())
  {
    std::cerr << "Failed to open actor configuration file: " << config_path << std::endl;
    return false;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  rapidjson::Document d;
  d.Parse(buffer.str().c_str());
  if (d.HasParseError())
  {
    std::cerr << "Failed to parse actor configuration file: " << config_path << std::endl;
    return false;
  }
  if (!d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray())
  {
    std::cerr << "Actor configuration missing 'actors' array: " << config_path << std::endl;
    return false;
  }

  const auto &actors_array = d["actors"].GetArray();
  for (const auto &actor_entry : actors_array)
  {
    if (!actor_entry.IsObject() || !actor_entry.HasMember("path_file"))
      continue;

    ActorPlayback actor;
    actor.start_time = actor_entry.HasMember("start_time") ? actor_entry["start_time"].GetDouble() : 0.0;
    if (actor_entry.HasMember("look_ahead"))
    {
      actor.look_ahead_distance = actor_entry["look_ahead"].GetDouble();
    }
    if (actor_entry.HasMember("path_spacing"))
    {
      actor.path_spacing = std::max(0.05, actor_entry["path_spacing"].GetDouble());
    }
    if (actor_entry.HasMember("smooth_window"))
    {
      actor.smoothing_window = std::max(0.0, actor_entry["smooth_window"].GetDouble());
    }
    if (actor_entry.HasMember("steering_kp"))
    {
      actor.steering_kp = actor_entry["steering_kp"].GetDouble();
    }
    if (actor_entry.HasMember("steering_ki"))
    {
      actor.steering_ki = actor_entry["steering_ki"].GetDouble();
    }
    if (actor_entry.HasMember("steering_kd"))
    {
      actor.steering_kd = actor_entry["steering_kd"].GetDouble();
    }
    std::string path_file = actor_entry["path_file"].GetString();
    if (!LoadWaypointCSV(path_file, actor.waypoints))
    {
      std::cerr << "Skipping actor due to failed path load: " << path_file << std::endl;
      continue;
    }
    if (!actor_entry.HasMember("speed_profile"))
    {
      std::cerr << "Actor entry missing speed_profile; skipping.\n";
      continue;
    }
    if (!ParseSpeedProfileJSON(actor_entry["speed_profile"], actor))
    {
      std::cerr << "Failed to parse speed profile for actor path " << path_file << std::endl;
      continue;
    }

    std::vector<ChVector3d> path_points = BuildResampledPoints(actor.waypoints, actor.path_spacing);
    // if (!path_points.empty())
    // {
    //   ChVector3d last = path_points.back();
    //   if (path_points.size() >= 2)
    //   {
    //     ChVector3d dir = path_points.back() - path_points[path_points.size() - 2];
    //     double len = dir.Length();
    //     if (len > 1e-6)
    //     {
    //       dir /= len;
    //       double tail_length = std::max(3.0, actor.look_ahead_distance > 0.0 ? actor.look_ahead_distance : 3.0);
    //       path_points.push_back(last + dir * tail_length);
    //     }
    //   }
    //   else
    //   {
    //     path_points.push_back(last);
    //   }
    // }
    if (actor.smoothing_window > 0.0)
    {
      path_points = SmoothPathPoints(path_points, actor.path_spacing, actor.smoothing_window);
    }
    auto path_curve = chrono_types::make_shared<ChBezierCurve>(path_points, false);
    actor.waypoints = path_points;

    auto actor_vehicle = chrono_types::make_shared<WheeledVehicle>(reference_vehicle.GetSystem(), vehicle_filename);
    actor_vehicle->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChQuaterniond start_rot = EstimateInitialRotation(path_points);
    actor_vehicle->Initialize(ChCoordsys<>(path_points.front(), start_rot));
    actor_vehicle->GetChassis()->SetFixed(false);
    actor_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    actor_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    actor_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    actor_vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    auto actor_engine = ReadEngineJSON(engine_filename);
    auto actor_transmission = ReadTransmissionJSON(transmission_filename);
    actor.powertrain = chrono_types::make_shared<ChPowertrainAssembly>(actor_engine, actor_transmission);
    actor_vehicle->InitializePowertrain(actor.powertrain);

    for (auto &axle : actor_vehicle->GetAxles())
    {
      for (auto &wheel : axle->GetWheels())
      {
        auto tire = ReadTireJSON(tire_filename);
        tire->SetStepsize(tire_step_size);
        actor_vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
      }
    }
    auto driver = chrono_types::make_shared<ChPathFollowerDriver>(*actor_vehicle, steering_file, speed_file, path_curve,
                                                                  "actor_path", 0.0);
    if (actor.look_ahead_distance > 0.0)
    {
      driver->GetSteeringController().SetLookAheadDistance(actor.look_ahead_distance);
    }
    driver->GetSteeringController().SetGains(actor.steering_kp, actor.steering_ki, actor.steering_kd);
    driver->Initialize();
    actor.path_driver = driver;

    actor.vehicle = actor_vehicle;
    actor.active = (actor.start_time <= 0.0);

    playback_actors.push_back(std::move(actor));
  }

  if (playback_actors.empty())
  {
    std::cerr << "No valid actors were initialized from " << config_path << std::endl;
    return false;
  }

  std::cout << "Loaded " << playback_actors.size() << " playback actor(s) from " << config_path << std::endl;
  return true;
}

// Initialize a single actor for distributed (SynChrono) mode
// Returns the ActorPlayback struct with vehicle and driver initialized
// actor_index is 0-based index into the actors array in the config file
bool InitializeSingleDistributedActor(const std::string &config_path,
                                       int actor_index,
                                       WheeledVehicle &my_vehicle,
                                       const std::string &vehicle_filename,
                                       const std::string &engine_filename,
                                       const std::string &transmission_filename,
                                       const std::string &tire_filename,
                                       const std::string &steering_file,
                                       const std::string &speed_file,
                                       ActorPlayback &out_actor)
{
  std::ifstream ifs(config_path);
  if (!ifs.is_open())
  {
    std::cerr << "Failed to open actor configuration file: " << config_path << std::endl;
    return false;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  rapidjson::Document d;
  d.Parse(buffer.str().c_str());
  if (d.HasParseError())
  {
    std::cerr << "Failed to parse actor configuration file: " << config_path << std::endl;
    return false;
  }
  if (!d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray())
  {
    std::cerr << "Actor configuration missing 'actors' array: " << config_path << std::endl;
    return false;
  }

  const auto &actors_array = d["actors"].GetArray();
  if (actor_index < 0 || actor_index >= static_cast<int>(actors_array.Size()))
  {
    std::cerr << "Actor index " << actor_index << " out of range (0-" << actors_array.Size() - 1 << ")" << std::endl;
    return false;
  }

  const auto &actor_entry = actors_array[actor_index];
  if (!actor_entry.IsObject() || !actor_entry.HasMember("path_file"))
  {
    std::cerr << "Actor entry at index " << actor_index << " is invalid or missing path_file" << std::endl;
    return false;
  }

  ActorPlayback actor;
  actor.start_time = actor_entry.HasMember("start_time") ? actor_entry["start_time"].GetDouble() : 0.0;
  if (actor_entry.HasMember("look_ahead"))
  {
    actor.look_ahead_distance = actor_entry["look_ahead"].GetDouble();
  }
  if (actor_entry.HasMember("path_spacing"))
  {
    actor.path_spacing = std::max(0.05, actor_entry["path_spacing"].GetDouble());
  }
  if (actor_entry.HasMember("smooth_window"))
  {
    actor.smoothing_window = std::max(0.0, actor_entry["smooth_window"].GetDouble());
  }
  if (actor_entry.HasMember("steering_kp"))
  {
    actor.steering_kp = actor_entry["steering_kp"].GetDouble();
  }
  if (actor_entry.HasMember("steering_ki"))
  {
    actor.steering_ki = actor_entry["steering_ki"].GetDouble();
  }
  if (actor_entry.HasMember("steering_kd"))
  {
    actor.steering_kd = actor_entry["steering_kd"].GetDouble();
  }
  std::string path_file = actor_entry["path_file"].GetString();
  if (!LoadWaypointCSV(path_file, actor.waypoints))
  {
    std::cerr << "Failed to load path for actor " << actor_index << ": " << path_file << std::endl;
    return false;
  }
  if (!actor_entry.HasMember("speed_profile"))
  {
    std::cerr << "Actor " << actor_index << " missing speed_profile" << std::endl;
    return false;
  }
  if (!ParseSpeedProfileJSON(actor_entry["speed_profile"], actor))
  {
    std::cerr << "Failed to parse speed profile for actor " << actor_index << std::endl;
    return false;
  }

  std::vector<ChVector3d> path_points = BuildResampledPoints(actor.waypoints, actor.path_spacing);
  if (actor.smoothing_window > 0.0)
  {
    path_points = SmoothPathPoints(path_points, actor.path_spacing, actor.smoothing_window);
  }
  auto path_curve = chrono_types::make_shared<ChBezierCurve>(path_points, false);
  actor.waypoints = path_points;

  // For distributed actor, we use my_vehicle directly (it's already initialized at the right position)
  // Just need to set up the path driver
  auto driver = chrono_types::make_shared<ChPathFollowerDriver>(my_vehicle, steering_file, speed_file, path_curve,
                                                                "actor_path", 0.0);
  if (actor.look_ahead_distance > 0.0)
  {
    driver->GetSteeringController().SetLookAheadDistance(actor.look_ahead_distance);
  }
  driver->GetSteeringController().SetGains(actor.steering_kp, actor.steering_ki, actor.steering_kd);
  driver->Initialize();
  actor.path_driver = driver;
  actor.vehicle = nullptr; // Distributed actor uses my_vehicle directly, not a separate vehicle
  actor.active = true; // Distributed actors start active immediately

  out_actor = std::move(actor);
  std::cout << "Loaded distributed actor " << actor_index << " from " << config_path << std::endl;
  return true;
}

// Get the starting position and rotation for a distributed actor from config
bool GetActorStartPose(const std::string &config_path,
                       int actor_index,
                       ChVector3d &out_pos,
                       ChQuaterniond &out_rot)
{
  std::ifstream ifs(config_path);
  if (!ifs.is_open())
    return false;

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  rapidjson::Document d;
  d.Parse(buffer.str().c_str());
  if (d.HasParseError() || !d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray())
    return false;

  const auto &actors_array = d["actors"].GetArray();
  if (actor_index < 0 || actor_index >= static_cast<int>(actors_array.Size()))
    return false;

  const auto &actor_entry = actors_array[actor_index];
  if (!actor_entry.IsObject() || !actor_entry.HasMember("path_file"))
    return false;

  // Load waypoints to get starting position
  std::vector<ChVector3d> waypoints;
  std::string path_file = actor_entry["path_file"].GetString();
  if (!LoadWaypointCSV(path_file, waypoints) || waypoints.empty())
    return false;

  double path_spacing = actor_entry.HasMember("path_spacing") ? 
      std::max(0.05, actor_entry["path_spacing"].GetDouble()) : 0.5;
  double smoothing = actor_entry.HasMember("smooth_window") ?
      std::max(0.0, actor_entry["smooth_window"].GetDouble()) : 0.0;

  std::vector<ChVector3d> path_points = BuildResampledPoints(waypoints, path_spacing);
  if (smoothing > 0.0)
  {
    path_points = SmoothPathPoints(path_points, path_spacing, smoothing);
  }

  if (path_points.empty())
    return false;

  out_pos = path_points.front();
  out_rot = EstimateInitialRotation(path_points);
  return true;
}

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
  const std::vector<std::string> ip_list = cli.GetAsType<std::vector<std::string>>("ip");
  
  // Determine if this node is the ego (node_id == 1) or an actor node
  const bool is_ego_node = (node_id == 1);
  const int actor_index = node_id - 2; // Actor index in config (node 2 = actor 0, node 3 = actor 1, etc.)
  
  std::cout << "=== SynChrono Configuration ===" << std::endl;
  std::cout << "Node ID: " << node_id << " / " << num_nodes << std::endl;
  std::cout << "Role: " << (is_ego_node ? "EGO (SDL driver)" : "ACTOR (path follower, actor index " + std::to_string(actor_index) + ")") << std::endl;
  std::cout << "Heartbeat: " << heartbeat << "s" << std::endl;
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
  // Create SynChronoManager
  // -----------------------
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
  SynChronoManager syn_manager(node_id, num_nodes, communicator);
  syn_manager.SetHeartbeat(heartbeat);

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
  // patch = terrain.AddPatch(patch_mat, CSYSNORM,
  //                          std::string(STRINGIFY(HIL_DATA_DIR)) +
  //                              "/Environments/map_project/remote.obj",
  //                          true, 0, false);

  
  // add terrain patch (this is used for collision i.e. is the physical terrain that the vehicle interacts with)
  patch = terrain.AddPatch(patch_mat, CSYSNORM,
                           std::string(STRINGIFY(HIL_DATA_DIR)) +
                               "/Environments/nads/newnads/terrain.obj",
                           true, 0, false);

  // std::cout << "HIL Data Dir: " << STRINGIFY(HIL_DATA_DIR) << std::endl;

  terrain.Initialize();

  // add vis mesh (this is used for visualization only)
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  // terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
  //                                     "/Environments/map_project/remote.obj",
  //                                 true, true);
    terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/Environments/nads/newnads/terrain.obj",
                                  true, true);
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
    std::string joystick_file =
        (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G29.json");
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
    
    // Get speed (default to cruise_speed)
    double actor_target_speed = cruise_speed * MPH_TO_MS;
    if (actor_cfg.HasMember("speed_profile") && actor_cfg["speed_profile"].IsArray()) {
      const auto& speed_arr = actor_cfg["speed_profile"].GetArray();
      if (speed_arr.Size() >= 2) {
        // Use the second element as target speed in m/s
        actor_target_speed = speed_arr[1].GetDouble();
      }
    }
    
    // Get driver parameters
    double look_ahead = 5.0;
    double steering_p = 0.8, steering_i = 0.0, steering_d = 0.0;
    double speed_p = 0.6, speed_i = 0.05, speed_d = 0.0;
    
    if (actor_cfg.HasMember("look_ahead") && actor_cfg["look_ahead"].IsNumber()) {
      look_ahead = actor_cfg["look_ahead"].GetDouble();
    }
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
    
    std::cout << "Actor node " << node_id << " initialized path follower driver:" << std::endl;
    std::cout << "  Path file: " << actor_path_file << std::endl;
    std::cout << "  Target speed: " << actor_target_speed << " m/s" << std::endl;
    std::cout << "  Look ahead: " << look_ahead << " m" << std::endl;
  }

  // Only load local playback actors for ego node (distributed actors are handled via SynChrono)
  if (is_ego_node && !actors_config_file.empty())
  {
    InitializePlaybackActors(actors_config_file, my_vehicle, vehicle_filename, engine_filename, transmission_filename, tire_filename, steering_controller_file_IG_nl, speed_controller_file_IG_nl);
  }

  // -----------------------
  // Add vehicle as SynChrono agent and initialize
  // -----------------------
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&my_vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(my_vehicle.GetSystem());
  
  std::cout << "SynChrono initialized with " << num_nodes << " node(s)" << std::endl;

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

  // simulation loop - use syn_manager.IsOk() to check for distributed sync status
  while (syn_manager.IsOk())
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto dds_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    double time = my_vehicle.GetSystem()->GetChTime();

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
        actor_path_driver->Synchronize(time);
        driver_inputs = actor_path_driver->GetInputs();
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
    syn_manager.Synchronize(time);  // SynChrono synchronization between nodes
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
      ros_actor_states.reserve(playback_actors.size());
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

    if (step_number == 0)
    {
      realtime_timer.Reset();
    }

    // if (step_number % 10 == 0) {

    // Check for simulation running slower than wall time (ego node only)
    if (is_ego_node && indicator_slow)
    {
      if (realtime_timer.GetTimeSeconds() > time + 1.00) {
          ChVector3d sphere_local_pos_slow(2.54, 0.381 + 0.5, 1.04); 
          ChVector3d sphere_global_pos_slow = my_vehicle.GetChassisBody()->TransformPointLocalToParent(sphere_local_pos_slow);
          indicator_slow->SetPos(sphere_global_pos_slow);
          
          if (step_number % 50 == 0) {
               std::cout << "[Slow Warning] Wall: " << realtime_timer.GetTimeSeconds() << " Sim: " << time << " Lag: " << (realtime_timer.GetTimeSeconds() - time) << "s" << std::endl;
          }
      } else {
          indicator_slow->SetPos(ChVector3d(0, 0, -100));
      }
    }

    // Real-time synchronization (ego node spins, actor nodes run as fast as possible but sync via SynChrono)
    if (is_ego_node)
    {
      realtime_timer.Spin(time);

      if (step_number % 50 == 0)
      {
        // Stream out data (ego node only)
        boost_streamer.AddData(my_vehicle.GetSystem()->GetChTime()); // sim time
        boost_streamer.AddData(my_vehicle.GetSpeed() *
                               MS_TO_MPH); // vehicle speed
        boost_streamer.AddData(my_vehicle.GetEngine()->GetMotorSpeed() *
                               rads2rpm); // RPM
        boost_streamer.AddData(
            my_vehicle.GetChassis()->GetPos().x()); // vehicle x pos
        boost_streamer.AddData(
            my_vehicle.GetChassis()->GetPos().y());       // vehicle y pos
        boost_streamer.AddData(driver_inputs.m_throttle); // throttle data
        boost_streamer.AddData(driver_inputs.m_braking);  // brake data
        boost_streamer.AddData(driver_inputs.m_steering); // steering data
        boost_streamer.AddData(auto_mode);                // auto mode

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
  syn_manager.QuitSimulation();
  
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