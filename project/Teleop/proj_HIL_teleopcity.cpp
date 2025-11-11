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
#include "chrono_hil/driver/ChRecordedDriver.h"
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

#include "chrono_hil/network/udp/ChBoostOutStreamer.h"
#include "chrono_hil/network/sim/ChDelaySim.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::hil;
using namespace chrono::utils;
using namespace chrono::sensor;

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
std::string record_output_file = "recorded_path.json";
int auto_toggle_button = 6;
int record_toggle_button = 2;
int finish_record_button = 1;
bool use_recorded_inputs_default = true;

struct WaypointSample
{
  double time;
  ChVector3d pos;
  ChQuaterniond rot;
  DriverInputs inputs;
};

std::vector<WaypointSample> recorded_samples;

struct ActorPlayback
{
  std::shared_ptr<WheeledVehicle> vehicle;
  std::shared_ptr<ChPathFollowerDriver> path_driver;
  std::shared_ptr<ChPowertrainAssembly> powertrain;
  std::vector<WaypointSample> samples;
  std::vector<double> sample_times;
  std::vector<double> segment_speeds;
  double start_time = 0.0;
  double speed_override = -1.0; // meters per second, negative uses recorded profile
  double look_ahead_distance = -1.0;
  double path_spacing = 0.5;
  double smoothing_window = 0.0;
  double steering_kp = -1.0;
  double steering_ki = -1.0;
  double steering_kd = -1.0;
  bool active = false;
  std::vector<ChRecordedDriver::Sample> input_samples;
  std::shared_ptr<ChRecordedDriver> recorded_driver;
  bool use_recorded_inputs = false;
};

std::string actors_config_file = "";
std::vector<ActorPlayback> playback_actors;
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
  cli.AddOption<bool>("Recording", "record_mode", "Enable waypoint recording mode", "false");
  cli.AddOption<std::string>("Recording", "record_output", "Output file path for recorded waypoints", record_output_file);
  cli.AddOption<double>("Recording", "record_interval", "Minimum time between recorded samples (seconds)", std::to_string(record_interval));
  cli.AddOption<int>("Recording", "auto_button", "Joystick button index for auto/manual toggle", std::to_string(auto_toggle_button));
  cli.AddOption<int>("Recording", "record_button", "Joystick button index for record toggle", std::to_string(record_toggle_button));
  cli.AddOption<int>("Recording", "finish_button", "Joystick button index to finish recording and exit", std::to_string(finish_record_button));
  cli.AddOption<bool>("Playback", "use_recorded_inputs_default", "Use recorded inputs when available (fallback to path follower otherwise)", use_recorded_inputs_default ? "true" : "false");
  cli.AddOption<std::string>("Playback", "actors_config", "Path to actor playback configuration file", actors_config_file);
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

std::vector<ChVector3d> BuildResampledPoints(const std::vector<WaypointSample> &samples,
                                             double spacing)
{
  std::vector<ChVector3d> points;
  if (samples.empty())
    return points;

  spacing = std::max(1e-3, spacing);
  points.push_back(samples.front().pos);

  for (size_t i = 0; i + 1 < samples.size(); ++i)
  {
    ChVector3d start = samples[i].pos;
    ChVector3d end = samples[i + 1].pos;
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

bool LoadRecordedPathFile(const std::string &filename, std::vector<WaypointSample> &out_samples, bool &out_has_inputs)
{
  std::ifstream ifs(filename);
  if (!ifs.is_open())
  {
    std::cerr << "Unable to open recorded waypoint file: " << filename << std::endl;
    return false;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();

  rapidjson::Document d;
  d.Parse(buffer.str().c_str());
  if (d.HasParseError())
  {
    std::cerr << "Failed to parse waypoint file: " << filename << std::endl;
    return false;
  }
  if (!d.IsObject() || !d.HasMember("samples") || !d["samples"].IsArray())
  {
    std::cerr << "Waypoint file missing 'samples' array: " << filename << std::endl;
    return false;
  }

  out_samples.clear();
  out_has_inputs = false;
  const auto &samples = d["samples"].GetArray();
  out_samples.reserve(samples.Size());
  for (const auto &entry : samples)
  {
    if (!entry.IsObject())
      continue;
    if (!entry.HasMember("time") || !entry.HasMember("pos") || !entry.HasMember("rot"))
      continue;
    const auto &pos_arr = entry["pos"];
    const auto &rot_arr = entry["rot"];
    if (!pos_arr.IsArray() || pos_arr.Size() != 3 || !rot_arr.IsArray() || rot_arr.Size() != 4)
      continue;

    WaypointSample sample;
    sample.time = entry["time"].GetDouble();
    sample.pos = ChVector3d(pos_arr[0].GetDouble(), pos_arr[1].GetDouble(), pos_arr[2].GetDouble());
    sample.rot = ChQuaterniond(rot_arr[0].GetDouble(), rot_arr[1].GetDouble(), rot_arr[2].GetDouble(), rot_arr[3].GetDouble());
    if (entry.HasMember("inputs"))
    {
      const auto &inp_arr = entry["inputs"];
      if (inp_arr.IsArray() && inp_arr.Size() == 3)
      {
        sample.inputs.m_steering = inp_arr[0].GetDouble();
        sample.inputs.m_throttle = inp_arr[1].GetDouble();
        sample.inputs.m_braking = inp_arr[2].GetDouble();
        out_has_inputs = true;
      }
      else
      {
        sample.inputs = DriverInputs();
      }
    }
    else
    {
      sample.inputs = DriverInputs();
    }

    out_samples.push_back(sample);
  }

  if (out_samples.empty())
  {
    std::cerr << "No valid samples found in " << filename << std::endl;
    return false;
  }

  double t0 = out_samples.front().time;
  for (auto &sample : out_samples)
  {
    sample.time -= t0;
  }

  return true;
}

void BuildPlaybackTiming(ActorPlayback &actor)
{
  actor.sample_times.clear();
  actor.segment_speeds.clear();

  if (actor.samples.empty())
    return;

  actor.sample_times.reserve(actor.samples.size());
  for (const auto &sample : actor.samples)
  {
    actor.sample_times.push_back(sample.time);
  }

  for (size_t i = 0; i + 1 < actor.samples.size(); ++i)
  {
    double dt = actor.samples[i + 1].time - actor.samples[i].time;
    double dist = (actor.samples[i + 1].pos - actor.samples[i].pos).Length();
    if (dt <= 1e-6)
      actor.segment_speeds.push_back(0.0);
    else
      actor.segment_speeds.push_back(dist / dt);
  }

  if (!actor.segment_speeds.empty())
  {
    actor.segment_speeds.push_back(actor.segment_speeds.back());
  }
}

double RecordedSpeedAtTime(const ActorPlayback &actor, double local_time)
{
  if (actor.segment_speeds.empty() || actor.sample_times.empty())
    return 0.0;

  if (local_time <= actor.sample_times.front())
    return actor.segment_speeds.front();

  for (size_t i = 0; i + 1 < actor.sample_times.size(); ++i)
  {
    if (local_time < actor.sample_times[i + 1])
      return actor.segment_speeds[i];
  }

  return actor.segment_speeds.back();
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
    actor.speed_override = -1.0;
    if (actor_entry.HasMember("target_speed_mps"))
    {
      actor.speed_override = actor_entry["target_speed_mps"].GetDouble();
    }
    else if (actor_entry.HasMember("target_speed_mph"))
    {
      actor.speed_override = actor_entry["target_speed_mph"].GetDouble() * MPH_TO_MS;
    }
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
    bool actor_force_path = false;
    if (actor_entry.HasMember("force_path_follower"))
    {
      actor_force_path = actor_entry["force_path_follower"].GetBool();
    }

    std::string path_file = actor_entry["path_file"].GetString();
    bool has_input_data = false;
    if (!LoadRecordedPathFile(path_file, actor.samples, has_input_data))
    {
      std::cerr << "Skipping actor due to failed path load: " << path_file << std::endl;
      continue;
    }
    BuildPlaybackTiming(actor);

    actor.input_samples.clear();
    actor.input_samples.reserve(actor.samples.size());
    for (const auto &sample : actor.samples)
    {
      ChRecordedDriver::Sample rs;
      rs.time = sample.time;
      rs.inputs = sample.inputs;
      actor.input_samples.push_back(rs);
    }
    actor.use_recorded_inputs = (!actor_force_path) && use_recorded_inputs_default && has_input_data && actor.input_samples.size() > 1;

    std::vector<ChVector3d> path_points = BuildResampledPoints(actor.samples, actor.path_spacing);
    if (actor.smoothing_window > 0.0)
    {
      path_points = SmoothPathPoints(path_points, actor.path_spacing, actor.smoothing_window);
    }
    auto path_curve = chrono_types::make_shared<ChBezierCurve>(path_points, false);

    auto actor_vehicle = chrono_types::make_shared<WheeledVehicle>(reference_vehicle.GetSystem(), vehicle_filename);
    actor_vehicle->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    actor_vehicle->Initialize(ChCoordsys<>(actor.samples.front().pos, actor.samples.front().rot));
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

    if (actor.use_recorded_inputs)
    {
      actor.recorded_driver = chrono_types::make_shared<ChRecordedDriver>(*actor_vehicle, actor.input_samples);
    }
    else
    {
      auto driver = chrono_types::make_shared<ChPathFollowerDriver>(*actor_vehicle, steering_file, speed_file, path_curve,
                                                                    "actor_path", std::max(0.0, actor.speed_override));
      if (actor.look_ahead_distance > 0.0)
      {
        driver->GetSteeringController().SetLookAheadDistance(actor.look_ahead_distance);
      }
      if (actor.steering_kp >= 0.0)
      {
        driver->GetSteeringController().SetGains(actor.steering_kp, actor.steering_ki, actor.steering_kd);
      }
      driver->Initialize();
      if (actor.speed_override < 0 && !actor.segment_speeds.empty())
      {
        driver->SetDesiredSpeed(actor.segment_speeds.front());
      }
      else if (actor.speed_override >= 0)
      {
        driver->SetDesiredSpeed(actor.speed_override);
      }
      actor.path_driver = driver;
    }

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
  use_recorded_inputs_default = cli.GetAsType<bool>("use_recorded_inputs_default");
  if (record_interval <= 0.0)
  {
    record_interval = 0.1;
  }
  recorded_samples.clear();
  if (record_mode)
  {
    std::cout << "Recording mode enabled. Use button " << record_toggle_button
              << " to toggle capture and button " << finish_record_button
              << " to finish and exit.\n";
  }

  ReadParameterFiles();
  // --------------
  // Create systems
  // --------------

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

  // std::cout << "HIL Data Dir: " << STRINGIFY(HIL_DATA_DIR) << std::endl;

  terrain.Initialize();

  // add vis mesh (this is used for visualization only)
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
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

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/ps4_controller.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);
  SDLDriver.AddCallbackButtons(auto_toggle_button);
  if (record_mode)
  {
    SDLDriver.AddCallbackButtons(record_toggle_button);
    SDLDriver.AddCallbackButtons(finish_record_button);
  }

  // ---------------------------------
  // Add sensor manager and simulation
  // ---------------------------------

  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
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
  auto driver_cam = chrono_types::make_shared<ChCameraSensor>(
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

  // Initialize simulation frame counters
  int step_number = 0;

  my_vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  ChBoostOutStreamer boost_streamer(UNITY_IP_OUT, UNITY_PORT_OUT);

  DriverInputs driver_inputs;

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
  std::string outer_path_file = "";

  if (lane == 0)
    outer_path_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                      "/Environments/nads/bezier_curve_points.txt";
  else if (lane == 1)
    outer_path_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                      "/Environments/nads/nads_path_5.txt";

  auto outer_path = ChBezierCurve::Read(outer_path_file, true);
  std::vector<double>
      followerParam = {30, 1.5, 2.0, 5.0, 3.0, 4.0, AUDI_LENGTH};

  std::shared_ptr<ChNSFFollowerDriver> PFdriver = chrono_types::make_shared<ChNSFFollowerDriver>(
      my_vehicle, steering_controller_file_IG_nl, speed_controller_file_IG_nl,
      outer_path, "road", cruise_speed * MPH_TO_MS, followerParam);
  PFdriver->Initialize();

  if (!actors_config_file.empty())
  {
    InitializePlaybackActors(actors_config_file, my_vehicle, vehicle_filename, engine_filename, transmission_filename, tire_filename, steering_controller_file_IG_nl, speed_controller_file_IG_nl);
  }

  auto last_auto_toggle = std::chrono::system_clock::now();
  auto last_record_toggle = last_auto_toggle;
  auto last_finish_toggle = last_auto_toggle;
  bool finish_requested = false;
  double last_recorded_time = -1.0;

  auto capture_sample = [&](double sample_time, const ChVector3d &sample_pos,
                            const ChQuaterniond &sample_rot, const DriverInputs &sample_inputs) {
    recorded_samples.push_back({sample_time, sample_pos, sample_rot, sample_inputs});
    last_recorded_time = sample_time;
  };

  // simulation loop
  while (true)
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto dds_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    double time = my_vehicle.GetSystem()->GetChTime();

    // Update delay based on current simulation time if using JSON config
    if (use_json_delay_config) {
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

    if (step_number % 10 == 0)
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
        if (receivedFloats[0] == 0)
        {
          driver_inputs.m_steering = receivedFloats[1];
          driver_inputs.m_throttle = receivedFloats[2];
          driver_inputs.m_braking = receivedFloats[3];
        }
        else
        {
          driver_inputs = PFdriver->GetInputs();
          // std::cout << driver_inputs.m_steering << " " << driver_inputs.m_throttle << " " << driver_inputs.m_braking << std::endl;
        }
      }
    }

    if (record_mode && recording_active)
    {
      if (last_recorded_time < 0.0 || (time - last_recorded_time) >= record_interval)
      {
        capture_sample(time, pos, rot, driver_inputs);
      }
    }

    // =======================
    // end data stream out section
    // =======================

    // Update modules (process inputs from other modules)
    terrain.Synchronize(time);
    my_vehicle.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one time for all modules
    terrain.Advance(step_size);
    my_vehicle.Advance(step_size);
    // vis->Advance(step_size);
    for (auto &actor : playback_actors)
    {
      if (!actor.vehicle || (!actor.path_driver && !actor.recorded_driver))
        continue;

      if (!actor.active)
      {
        if (time >= actor.start_time)
        {
          actor.active = true;
          if (actor.recorded_driver)
          {
            actor.recorded_driver->Reset();
          }
          if (actor.path_driver)
          {
            actor.path_driver->Reset();
            double initial_speed = (actor.speed_override >= 0.0) ? actor.speed_override
                                                                : (!actor.segment_speeds.empty() ? actor.segment_speeds.front() : 0.0);
            actor.path_driver->SetDesiredSpeed(initial_speed);
          }
        }
        else
        {
          continue;
        }
      }

      double local_time = time - actor.start_time;

      if (actor.recorded_driver)
      {
        actor.recorded_driver->Synchronize(local_time);
        actor.recorded_driver->Advance(step_size);
        DriverInputs actor_inputs = actor.recorded_driver->GetInputs();
        actor.vehicle->Synchronize(time, actor_inputs, terrain);
        actor.vehicle->Advance(step_size);
      }
      else if (actor.path_driver)
      {
        double desired_speed = (actor.speed_override >= 0.0) ? actor.speed_override : RecordedSpeedAtTime(actor, local_time);
        actor.path_driver->SetDesiredSpeed(desired_speed);
        actor.path_driver->Synchronize(time);
        actor.path_driver->Advance(step_size);
        DriverInputs actor_inputs = actor.path_driver->GetInputs();
        actor.vehicle->Synchronize(time, actor_inputs, terrain);
        actor.vehicle->Advance(step_size);
      }
    }

    manager->Update();

    // Increment frame number
    step_number++;

    if (step_number == 0)
    {
      realtime_timer.Reset();
    }

    // if (step_number % 10 == 0) {
    realtime_timer.Spin(time);

    if (step_number % 50 == 0)
    {

      // Stream out data
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
            capture_sample(time, pos, rot, driver_inputs);
            std::cout << "Recording started: " << record_output_file << std::endl;
          }
          else
          {
            capture_sample(time, pos, rot, driver_inputs);
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
            capture_sample(time, pos, rot, driver_inputs);
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

    PFdriver->Advance(step_size);
    PFdriver->Synchronize(time, step_size);

    if (SDLDriver.Synchronize() == 1)
    {
      break;
    }

    // if (render == true && step_number % render_step == 0)
    // {
    //   vis->BeginScene();
    //   vis->Render();
    //   vis->EndScene();
    //   vis->Synchronize(time, driver_inputs);
    // }
  }
  if (record_mode)
  {
    if (!recorded_samples.empty())
    {
      std::ofstream record_stream(record_output_file);
      if (!record_stream.is_open())
      {
        std::cerr << "Failed to open " << record_output_file << " for waypoint recording output." << std::endl;
      }
      else
      {
        record_stream << "{\n  \"samples\": [\n";
        record_stream << std::setprecision(16);
        for (size_t i = 0; i < recorded_samples.size(); ++i)
        {
          const auto &sample = recorded_samples[i];
          record_stream << "    {\"time\": " << sample.time << ", \"pos\": [" << sample.pos.x() << ", " << sample.pos.y() << ", " << sample.pos.z()
                        << "], \"rot\": [" << sample.rot.e0() << ", " << sample.rot.e1() << ", " << sample.rot.e2() << ", " << sample.rot.e3()
                        << "], \"inputs\": [" << sample.inputs.m_steering << ", " << sample.inputs.m_throttle << ", " << sample.inputs.m_braking << "]}";
          if (i + 1 < recorded_samples.size())
          {
            record_stream << ",";
          }
          record_stream << "\n";
        }
        record_stream << "  ]\n}\n";
        std::cout << "Waypoint recording written to " << record_output_file << " (" << recorded_samples.size() << " samples)." << std::endl;
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
