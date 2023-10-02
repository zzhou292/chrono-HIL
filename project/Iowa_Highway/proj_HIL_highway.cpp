// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Simone Benatti, Jason Zhou
// =============================================================================
//
// Iowa Highway Simulation
// This demo includes IG interactive vehicle, autonomous dynamic vehicles and
// autonomous dummy vehicles
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include <irrlicht.h>
#include <limits>
#include <time.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

//#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/driver/ChCSLDriver.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#ifdef CHRONO_IRRKLANG
#include "chrono_hil/sound/ChCSLSoundEngine.h"
#endif

#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::synchrono;
using namespace chrono::hil;

#define PORT_OUT 1209
#define IP_OUT "128.104.188.63"

// -----------------------------------------------------------------------------
// rad to RPM conversion parameters
// -----------------------------------------------------------------------------
const double rads2rpm = 30 / CH_C_PI;

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Initial vehicle location and orientation
ChVector<> initLoc(5011.5, -445, 100.75); // near mile marker

ChQuaternion<> initRot = Q_from_AngZ(-CH_C_PI_2);

ChVector<> driver_eye(-.3, .4, .98);

ChQuaternion<> driver_view_direction = Q_from_AngAxis(0, {1, 0, 0});

enum DriverMode { HUMAN, AUTONOMOUS };
DriverMode driver_mode = AUTONOMOUS;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, HULLS, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;       // terrain height (FLAT terrain only)
double terrainLength = 40000.0; // size in X direction
double terrainWidth = 40000.0;  // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Global Variable to track miles the car traveled
utils::ChRunningAverage IG_speed_avg(100);

bool render = true;

// Whether to enable realtime in the simulation
bool enable_realtime = true;

// -----------------------------------------------------------------------------
// Steering wheel parameters
// -----------------------------------------------------------------------------
// steering wheel button settings
// change mapping if on a different steering wheel
int r_3 = 19;
int r_1 = 6;

// camera parameters
float frame_rate = 30.0;
int super_samples = 2;
unsigned int image_width = 3840 / 2; // 1920;   // / 2;
unsigned int image_height = 720 / 2; // / 2;
unsigned int fullscreen_image_width = 3840;
unsigned int fullscreen_image_height = 720;
float cam_fov = 1.608f;
// float cam_fov = .524;
bool use_fullscreen = false;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step sizes
double step_size = 1e-3;

// Simulation end time
double t_end = 10000;

// Save sensor data
bool sensor_save = false;

// Visualize sensor data
bool sensor_vis = true;

std::string demo_data_path = std::string(STRINGIFY(HIL_DATA_DIR));

// Driver parameters
std::vector<double> followerParam;
std::string scenario_parameters = "scenario_parameters.json";
std::string simulation_parameters = "simulation_parameters.json";
std::string lead_parameters = "lead_parameters_nolead.json";
// cruise speed [mph]
double cruise_speed = 45;

int correct_ratio = 200;

// Data saving
bool save_driver = true;
// time interval between data savings
double tsave = 1e-2;

// Fog parameters
bool fog_enabled = false;
ChVector<float> fog_color = {.8, .8, .8};
float fog_distance = 2000.0;

// mirrors position and rotations
ChVector<> mirror_rearview_pos = {0.0, 0.0, 0.0};
ChQuaternion<> mirror_rearview_rot = {1.0, 0.0, 0.0, 0.0};
ChVector<> mirror_wingleft_pos = {0.0, 0.0, 0.0};
ChQuaternion<> mirror_wingleft_rot = {1.0, 0.0, 0.0, 0.0};
ChVector<> mirror_wingright_pos = {0.0, 0.0, 0.0};
ChQuaternion<> mirror_wingright_rot = {1.0, 0.0, 0.0, 0.0};

ChVector<> arrived_sign_pos = {0.0, 0.0, 0.0};
ChQuaternion<> arrived_sign_rot = {1.0, 0.0, 0.0, 0.0};

using namespace std::chrono;

// Define all lead vehicles
int lead_count = 0;

// Dynamic Leaders
std::vector<ChVector<>> dynamic_pos;
std::vector<float> dynamic_cruise_speed;
std::vector<int> dynamic_lane; // 0 for inner, 1 for outer
// 0 for no control(constant speed), 1 for distance control, 2 for time control
std::vector<int> dynamic_control;
// time control variables
std::vector<std::vector<float>> dynamic_control_x;
std::vector<std::vector<float>> dynamic_control_y;

std::vector<std::vector<std::vector<double>>> leaderParam;

// Joystick file
std::string joystick_filename;

// distance variable
float IG_dist = 0;
ChVector<> IG_prev_pos;

// driver global variables
// TODO: maybe there is a better way to handle this
std::shared_ptr<ChNSFFollowerDriver> PF_driver_ptr;
float cur_follower_speed;

// Experiment parameters
int meet_time = 4;     // this meet time is defined in full minutes
double eta_dist = 3.6; // eta counter distance

// Start Stamp
float start_sim_time;
float start_wall_time;

// =============================================================================

// distribution of trees near the road
void AddTrees(ChSystem *chsystem);
// terrain and texture system
void AddTerrain(ChSystem *chsystem);
// signs along the road and the road itself
void AddRoadway(ChSystem *chsystem);
// buildings in the environment
void AddBuildings(ChSystem *chsystem);

// compute desired speed for dynamic vehicles
// based on json defined, time-dependent, piecewise speed data
float ControlFindSpeed_x_y(std::vector<float> time_vec,
                           std::vector<float> speed_vec, float time,
                           float default_speed);

void ReadParameterFiles() {
  { // Simulation parameter file
    rapidjson::Document d;
    vehicle::ReadFileJSON(simulation_parameters, d);

    if (d.HasMember("TimeStep")) {
      step_size = d["TimeStep"].GetDouble();
    }

    if (d.HasMember("Camera")) {
      const rapidjson::Value &camera_params = d["Camera"];
      if (camera_params.HasMember("DriverEye")) {
        driver_eye = vehicle::ReadVectorJSON(camera_params["DriverEye"]);
      }
      if (camera_params.HasMember("FrameRate")) {
        frame_rate = camera_params["FrameRate"].GetFloat();
      }
      if (camera_params.HasMember("SuperSamples")) {
        super_samples = camera_params["SuperSamples"].GetInt();
      }
      if (camera_params.HasMember("FieldOfView")) {
        cam_fov = camera_params["FieldOfView"].GetFloat() * CH_C_DEG_TO_RAD;
      }
    }
    if (d.HasMember("Fog")) {
      const rapidjson::Value &fog_params = d["Fog"];
      if (fog_params.HasMember("Enabled")) {
        fog_enabled = fog_params["Enabled"].GetBool();
      }
      if (fog_params.HasMember("Color")) {
        fog_color = vehicle::ReadVectorJSON(fog_params["Color"]);
      }
      if (fog_params.HasMember("Distance")) {
        fog_distance = fog_params["Distance"].GetFloat();
      }
    }

    if (d.HasMember("Joystick")) {
      joystick_filename = demo_data_path + d["Joystick"].GetString();
    }

    if (d.HasMember("Mirrors")) {
      const rapidjson::Value &mirror_params = d["Mirrors"];
      if (mirror_params.HasMember("Rearview")) {
        const rapidjson::Value &rearview_params = mirror_params["Rearview"];
        if (rearview_params.HasMember("Position")) {
          mirror_rearview_pos =
              vehicle::ReadVectorJSON(rearview_params["Position"]);
        }
        if (rearview_params.HasMember("Rotation")) {
          mirror_rearview_rot = Q_from_Euler123(
              CH_C_DEG_TO_RAD *
              vehicle::ReadVectorJSON(rearview_params["Rotation"]));
        }
      }
      if (mirror_params.HasMember("WingLeft")) {
        const rapidjson::Value &wingleft_params = mirror_params["WingLeft"];
        if (wingleft_params.HasMember("Position")) {
          mirror_wingleft_pos =
              vehicle::ReadVectorJSON(wingleft_params["Position"]);
        }
        if (wingleft_params.HasMember("Rotation")) {
          mirror_wingleft_rot = Q_from_Euler123(
              CH_C_DEG_TO_RAD *
              vehicle::ReadVectorJSON(wingleft_params["Rotation"]));
        }
      }
      if (mirror_params.HasMember("WingRight")) {
        const rapidjson::Value &wingright_params = mirror_params["WingRight"];
        if (wingright_params.HasMember("Position")) {
          mirror_wingright_pos =
              vehicle::ReadVectorJSON(wingright_params["Position"]);
        }
        if (wingright_params.HasMember("Rotation")) {
          mirror_wingright_rot = Q_from_Euler123(
              CH_C_DEG_TO_RAD *
              vehicle::ReadVectorJSON(wingright_params["Rotation"]));
        }
      }
    }
  }

  { // Scenario parameter file
    rapidjson::Document d;
    vehicle::ReadFileJSON(scenario_parameters, d);

    if (d.HasMember("StartLocation")) {
      initLoc = vehicle::ReadVectorJSON(d["StartLocation"]);
    }
    if (d.HasMember("EndTime")) {
      t_end = d["EndTime"].GetDouble();
    }

    if (d.HasMember("MeetTime")) {
      meet_time = d["MeetTime"].GetInt();
    }

    if (d.HasMember("ETADist")) {
      eta_dist = d["ETADist"].GetDouble();
    }
    // TODO: figure out what is happening there, not sure necessary
    if (d.HasMember("FollowerDriverParam")) {
      auto marr = d["FollowerDriverParam"].GetArray();
      int msize = marr.Size();
      assert(msize == 7);
      followerParam.resize(msize);
      for (int j = 0; j < marr.Size(); j++) {
        followerParam[j] = marr[j].GetDouble();
      }
    } else {
      followerParam.resize(6);
      followerParam = {30, 1.5, 2.0, 5.0, 3.0, 4.0, AUDI_LENGTH};
    }

    if (d.HasMember("CruiseSpeed")) {
      cruise_speed = d["CruiseSpeed"].GetDouble();
    }
    if (d.HasMember("ArrivedSign")) {
      const rapidjson::Value &arrived_sign_params = d["ArrivedSign"];
      if (arrived_sign_params.HasMember("Position")) {
        arrived_sign_pos =
            vehicle::ReadVectorJSON(arrived_sign_params["Position"]);
      }
      if (arrived_sign_params.HasMember("Rotation")) {
        arrived_sign_rot = Q_from_Euler123(
            CH_C_DEG_TO_RAD *
            vehicle::ReadVectorJSON(arrived_sign_params["Rotation"]));
      }
    }
  }

  {
    // Leading Vehicles Scenario parameter file
    // Scenario parameter file
    rapidjson::Document d;
    vehicle::ReadFileJSON(lead_parameters, d);

    while (true) {
      std::string entry_name = "lead_" + std::to_string(lead_count);

      std::cout << entry_name << std::endl;

      if (d.HasMember(entry_name.c_str())) {
        std::cout << "lead_count:" << lead_count << std::endl;

        dynamic_pos.push_back(
            vehicle::ReadVectorJSON(d[entry_name.c_str()]["initial_pos"]));
        dynamic_cruise_speed.push_back(
            d[entry_name.c_str()]["cruise_speed"].GetDouble());
        dynamic_lane.push_back(d[entry_name.c_str()]["lane"].GetInt());

        // read speed control
        if (d[entry_name.c_str()].HasMember("time_speed_control")) {
          dynamic_control.push_back(2);
          std::vector<float> temp_time;
          std::vector<float> temp_speed;

          auto marr = d[entry_name.c_str()]["time_speed_control"].GetArray();
          for (int i = 0; i < marr[0].Size(); i++) {
            for (int j = 0; j < marr[1].Size(); j++) {
              if (i == 0) {
                temp_time.push_back(marr[i][j].GetDouble());
              } else if (i == 1) {
                temp_speed.push_back(marr[i][j].GetDouble());
              }
            }
          }
          dynamic_control_x.push_back(temp_time);
          dynamic_control_y.push_back(temp_speed);
        }

        if (d[entry_name.c_str()].HasMember("LeaderDriverParam")) {
          std::vector<std::vector<double>> temp_leaderParam;
          auto marr = d[entry_name.c_str()]["LeaderDriverParam"].GetArray();
          int msize0 = marr.Size();
          int msize1 = marr[0].Size();
          assert(msize1 == 6);
          temp_leaderParam.resize(msize0);
          // printf("ARRAY DIM = %i \n", msize);
          for (auto it = marr.begin(); it != marr.end(); ++it) {
            auto i = std::distance(marr.begin(), it);
            temp_leaderParam[i].resize(msize1);
            for (int j = 0; j < marr[i].Size(); j++) {
              temp_leaderParam[i][j] = marr[i][j].GetDouble();
            }
          }
          leaderParam.push_back(temp_leaderParam);
        } else {
          std::vector<std::vector<double>> temp_leaderParam;
          temp_leaderParam.resize(1);
          temp_leaderParam[0] = {0.5, 1.5, 55.0, 5.0, 628.3, 0.0};
          leaderParam.push_back(temp_leaderParam);
        }
      } else {
        break;
      }
      lead_count++;
    }
  }
}

void AddCommandLineOptions(ChCLI &cli) {
  cli.AddOption<double>("Simulation", "s,step_size", "Step size",
                        std::to_string(step_size));
  cli.AddOption<double>("Simulation", "e,end_time", "End time",
                        std::to_string(t_end));

  // options for human driver
  cli.AddOption<bool>("Simulation", "nojoystick", "Turn off joystick control",
                      "false");
  cli.AddOption<bool>("Simulation", "lbj",
                      "Switch Joystick axes as used by LBJ", "false");
  cli.AddOption<bool>("Simulation", "fullscreen",
                      "Use full screen camera display",
                      std::to_string(use_fullscreen));
  cli.AddOption<bool>("Simulation", "record",
                      "Record human driver inputs to file", "false");
  cli.AddOption<bool>("Simulation", "replay",
                      "Replay human driver inputs from file", "false");

  cli.AddOption<bool>("Simulation", "birdseye", "Enable birds eye camera",
                      "false");

  cli.AddOption<std::string>("Simulation", "scenario_params",
                             "Path to scenario configuration file",
                             scenario_parameters);
  cli.AddOption<std::string>("Simulation", "sim_params",
                             "Path to simulation configuration file",
                             simulation_parameters);
  cli.AddOption<std::string>("Simulation", "lead_params",
                             "Path to lead configuration file",
                             lead_parameters);
  cli.AddOption<int>("Simulation", "image_width", "x resolution",
                     std::to_string(image_width));
  cli.AddOption<int>("Simulation", "image_height", "y resolution",
                     std::to_string(image_height));
  cli.AddOption<bool>(
      "Simulation", "enable_realtime",
      "Use a cumulative realtimer to sync sim_time and wall_time every 0.01s",
      "true");
}

int main(int argc, char *argv[]) {
  // create cli tool
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  // parse command line inputs
  step_size = cli.GetAsType<double>("step_size");
  t_end = cli.GetAsType<double>("end_time");
  enable_realtime = cli.GetAsType<bool>("enable_realtime");

  // parse image resolution
  image_height = cli.GetAsType<int>("image_height");
  image_width = cli.GetAsType<int>("image_width");

  std::cout << "enable_realtime: " << enable_realtime << std::endl;

  use_fullscreen = cli.GetAsType<bool>("fullscreen");
  if (use_fullscreen) {
    image_width = fullscreen_image_width;
    image_height = fullscreen_image_height;
    cam_fov = 1.608f;
  }
  bool disable_joystick = cli.GetAsType<bool>("nojoystick");
  bool lbj_joystick = cli.GetAsType<bool>("lbj");

  std::cout << "disable_joystick=" << disable_joystick << std::endl;

  scenario_parameters = demo_data_path + "/Environments/Iowa/parameters/" +
                        cli.GetAsType<std::string>("scenario_params");
  simulation_parameters = demo_data_path + "/Environments/Iowa/parameters/" +
                          cli.GetAsType<std::string>("sim_params");
  lead_parameters = demo_data_path + "/Environments/Iowa/parameters/" +
                    cli.GetAsType<std::string>("lead_params");

  std::cout << "Scen params: " << scenario_parameters << "\n";
  std::cout << "Sim params:" << simulation_parameters << "\n";
  std::cout << "Lead params:" << lead_parameters << "\n";

  ReadParameterFiles();

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string engine_filename =
      vehicle::GetDataFile("audi/json/audi_EngineSimpleMap.json");
  std::string transmission_filename = vehicle::GetDataFile(
      "audi/json/audi_AutomaticTransmissionSimpleMap.json");
  std::string tire_file = vehicle::GetDataFile("audi/json/audi_Pac02Tire.json");

  std::string outer_path_file =
      demo_data_path + "/Environments/Iowa/Driver/OnOuterLane.txt";
  auto outer_path = ChBezierCurve::read(outer_path_file, true);
  auto inner_path_file =
      demo_data_path + "/Environments/Iowa/Driver/OnInnerLane.txt";
  auto inner_path = ChBezierCurve::read(inner_path_file, true);

  // IG vehicle lane number tracker
  // lane 1 - inner lane; lane 2 - outer lanes
  auto lane_0_path_file =
      demo_data_path + "/Environments/Iowa/Driver/OnInnerLane.txt";
  auto lane_0_path = ChBezierCurve::read(lane_0_path_file, true);
  auto lane_1_path_file =
      demo_data_path + "/Environments/Iowa/Driver/OnOuterLane.txt";
  auto lane_1_path = ChBezierCurve::read(lane_1_path_file, true);

  WheeledVehicle vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = vehicle.GetChassis();
  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
  vehicle.GetChassis()->SetFixed(false);
  vehicle.SetChassisVisualizationType(chassis_vis_type);
  vehicle.SetSuspensionVisualizationType(suspension_vis_type);
  vehicle.SetSteeringVisualizationType(steering_vis_type);
  vehicle.SetWheelVisualizationType(wheel_vis_type);

  auto engine = ReadEngineJSON(engine_filename);
  auto transmission = ReadTransmissionJSON(transmission_filename);
  auto powertrain =
      chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);

  vehicle.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (auto &axle : vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(tire_file);
      tire->SetStepsize(step_size / 20);
      vehicle.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // change the ego vehicle vis out for windowless audi
  vehicle.GetChassisBody()->GetVisualModel()->Clear();

  auto audi_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  audi_mesh->LoadWavefrontMesh(
      demo_data_path +
          "/Environments/Iowa/vehicles/audi_chassis_windowless_2.obj",
      false, true);
  audi_mesh->Transform(ChVector<>(0, 0, 0),
                       ChMatrix33<>(1)); // scale to a different size
  auto audi_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  audi_shape->SetMesh(audi_mesh);
  audi_shape->SetName("Windowless Audi");
  audi_shape->SetMutable(false);
  // audi_shape->SetStatic(true);
  vehicle.GetChassisBody()->AddVisualShape(audi_shape);

  // add rearview mirror
  auto mirror_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  mirror_mesh->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/vehicles/audi_rearview_mirror.obj",
      false, true);
  mirror_mesh->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size

  auto mirror_mat = chrono_types::make_shared<ChVisualMaterial>();
  mirror_mat->SetDiffuseColor({0.2f, 0.2f, 0.2f});
  mirror_mat->SetRoughness(0.f);
  mirror_mat->SetMetallic(1.0f);
  mirror_mat->SetUseSpecularWorkflow(false);

  auto rvw_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  rvw_mirror_shape->SetMesh(mirror_mesh);
  rvw_mirror_shape->SetName("Windowless Audi");
  rvw_mirror_shape->SetMutable(false);
  rvw_mirror_shape->SetScale({1, 1.8, 1.2});
  rvw_mirror_shape->GetMaterials()[0] = mirror_mat;
  vehicle.GetChassisBody()->AddVisualShape(
      rvw_mirror_shape, ChFrame<>(mirror_rearview_pos, mirror_rearview_rot));

  // add left wing mirror
  auto lwm_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  lwm_mesh->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/vehicles/audi_left_wing_mirror.obj",
      false, true);
  lwm_mesh->Transform(ChVector<>(0, 0, 0),
                      ChMatrix33<>(1)); // scale to a different size

  auto lwm_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  lwm_mirror_shape->SetMesh(lwm_mesh);
  lwm_mirror_shape->SetName("Windowless Audi");
  lwm_mirror_shape->GetMaterials()[0] = mirror_mat;
  lwm_mirror_shape->SetMutable(false);
  vehicle.GetChassisBody()->AddVisualShape(
      lwm_mirror_shape, ChFrame<>(mirror_wingleft_pos, mirror_wingleft_rot));

  // add left wing mirror
  auto rwm_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  rwm_mesh->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/vehicles/audi_right_wing_mirror.obj",
      false, true);
  rwm_mesh->Transform(ChVector<>(0, 0, 0),
                      ChMatrix33<>(1)); // scale to a different size

  auto rwm_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  rwm_mirror_shape->SetMesh(rwm_mesh);
  rwm_mirror_shape->SetName("Windowless Audi");
  rwm_mirror_shape->SetScale({1, .98, .98});
  rwm_mirror_shape->GetMaterials()[0] = mirror_mat;
  rwm_mirror_shape->SetMutable(false);
  vehicle.GetChassisBody()->AddVisualShape(
      rwm_mirror_shape, ChFrame<>(mirror_wingright_pos, mirror_wingright_rot));

  // Create the terrain
  RigidTerrain terrain(vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  ChVector<> normal = ChVector<>({0, 0, 1});
  ChVector<> up = normal.GetNormalized();
  ChVector<> lateral = Vcross(up, ChWorldFrame::Forward());
  lateral.Normalize();
  ChVector<> forward = Vcross(lateral, up);
  ChMatrix33<> rot;
  rot.Set_A_axis(forward, lateral, up);

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model) {
  case RigidTerrain::PatchType::BOX:
    patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth,
                             2, false, 1, false);
    break;
  case RigidTerrain::PatchType::HEIGHT_MAP:
    patch = terrain.AddPatch(
        patch_mat, CSYSNORM,
        vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128, 128, 0, 4);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16,
                      16);
    break;
  case RigidTerrain::PatchType::MESH:
    // patch = terrain.AddPatch(patch_mat, CSYSNORM,
    // vehicle::GetDataFile("terrain/meshes/test.obj"));
    patch = terrain.AddPatch(patch_mat, CSYSNORM,
                             demo_data_path + "/Environments/Iowa/road.obj");
    break;
  }

  terrain.Initialize();

  // add terrain with field and grass textures
  AddTerrain(vehicle.GetSystem());

  // add signs along the road
  AddRoadway(vehicle.GetSystem());

  // Add trees near the road
  AddTrees(vehicle.GetSystem());

  // Add buildings away from the road to break up the horizon
  AddBuildings(vehicle.GetSystem());

  // Add lead vehicles
  std::string rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";
  float init_height = 0.20;
  std::vector<std::shared_ptr<Ch_8DOF_vehicle>> lead_vec;
  std::vector<std::shared_ptr<ChROM_PathFollowerDriver>> driver_vec;

  for (int i = 0; i < lead_count; i++) {
    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(rom_json, init_height,
                                                   step_size, true);

    rom_veh->SetInitPos(dynamic_pos[i]);
    rom_veh->SetInitRot(-CH_C_PI_2);
    rom_veh->Initialize(vehicle.GetSystem());
    lead_vec.push_back(rom_veh);

    std::string path_file;
    std::shared_ptr<ChBezierCurve> path;
    if (dynamic_lane[i] == 0) {
      path_file = STRINGIFY(HIL_DATA_DIR) +
                  std::string("/Environments/Iowa/Driver/OnInnerLane.txt");
      path = ChBezierCurve::read(path_file, true);
    } else if (dynamic_lane[i] == 1) {
      path_file = STRINGIFY(HIL_DATA_DIR) +
                  std::string("/Environments/Iowa/Driver/OnOuterLane.txt");
      path = ChBezierCurve::read(path_file, true);
    }

    std::shared_ptr<ChROM_PathFollowerDriver> driver =
        chrono_types::make_shared<ChROM_PathFollowerDriver>(
            lead_vec[i], path, dynamic_cruise_speed[i], 15.0, 0.15, 0.0, 0.0,
            0.3, 0.0, 0.0);
    driver_vec.push_back(driver);
  }

  // ---------------
  // Simulation loop
  // ---------------

  // output vehicle mass
  std::cout << "VEHICLE MASS: " << vehicle.GetMass() << std::endl;

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  int render_frame = 0;
  double sim_time = 0;

  // ---------------------------------------------
  // Create a sensor manager and add a point light
  // ---------------------------------------------
  auto manager =
      chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
  float intensity = 2.0;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  // Set environment map
  Background b;
  b.mode = BackgroundMode::ENVIRONMENT_MAP;
  b.env_tex = GetChronoDataFile("sensor/textures/sunflowers_4k.hdr");
  manager->scene->SetBackground(b);
  if (fog_enabled) {
    manager->scene->SetFogScatteringFromDistance(fog_distance);
    manager->scene->SetFogColor(fog_color);
  }

  // ------------------------------------------------
  // Create a camera and add it to the sensor manager
  // ------------------------------------------------

  auto cam = chrono_types::make_shared<ChCameraSensor>(
      vehicle.GetChassisBody(), // body camera is attached to
      10,                       // update rate in Hz
      chrono::ChFrame<double>(
          {0, 0, 3000}, Q_from_AngAxis(CH_C_PI_2, {0, 1, 0})), // offset pose
      1920,                                                    // image width
      1080,                                                    // image height
      CH_C_PI_4,
      super_samples); // fov, lag, exposure
  cam->SetName("Camera Sensor");
  if (sensor_vis)
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));

    // add sensor to the manager
    if (cli.GetAsType<bool>("birdseye"))
      manager->AddSensor(cam);

  // -------------------------------------------------------
  // Create a second camera and add it to the sensor manager
  // -------------------------------------------------------
  auto cam2 = chrono_types::make_shared<ChCameraSensor>(
      vehicle.GetChassisBody(), // body camera is attached to
      frame_rate,               // update rate in Hz
      chrono::ChFrame<double>(driver_eye, driver_view_direction), // offset pose
      image_width,                                                // image width
      image_height, // image height
      cam_fov,
      super_samples); // fov, lag, exposure
  cam2->SetName("Camera Sensor");
  if (sensor_vis)
    cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        image_width, image_height, "Driver View", use_fullscreen));

  // add sensor to the manager
  manager->AddSensor(cam2);

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);

  // ---------------
  // Simulate system
  // ---------------
  float orbit_radius = 1000.f;
  float orbit_rate = .5;

  double extra_time = 0.0;
  double last_sim_sync = 0;

  ChVector<> prev_IG_pos;
  bool IG_started_driving = false;

  auto t0 = high_resolution_clock::now();
  ChRealtimeCumulative realtime_timer;

  vehicle.EnableRealtime(false);

  // create boost data streaming interface
  ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);

  while (true) {
    sim_time = vehicle.GetSystem()->GetChTime();

    // End simulation
    if (sim_time >= t_end)
      break;

    // Collect output data from modules (for inter-module communication)
    DriverInputs driver_inputs;
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    auto ig_speed = vehicle.GetSpeed() * MS_TO_MPH;
    auto wall_time = high_resolution_clock::now();
    printf("Sim Time=%f, \tWall Time=%f, \tExtra Time=%f, \tIG_Speed mph=%f\n",
           sim_time, duration_cast<duration<double>>(wall_time - t0).count(),
           extra_time, ig_speed);

    // update current vehicle speed
    cur_follower_speed = vehicle.GetSpeed();

    if (SDLDriver.Synchronize() == 1) {
      break;
    }

    terrain.Synchronize(sim_time);
    vehicle.Synchronize(sim_time, driver_inputs, terrain);
    for (int i = 0; i < lead_count; i++) {
      float target_spd =
          ControlFindSpeed_x_y(dynamic_control_x[i], dynamic_control_y[i],
                               sim_time, dynamic_cruise_speed[i]);
      driver_vec[i]->SetCruiseSpeed(target_spd);
      driver_vec[i]->Advance(step_size);
      DriverInputs lead_inputs = driver_vec[i]->GetDriverInput();
      lead_vec[i]->Advance(sim_time, lead_inputs);
    }

#ifdef CHRONO_IRRKLANG
    soundEng.Synchronize(sim_time);
#endif
    // Advance simulation for one timestep for all modules
    double step = step_size;

    terrain.Advance(step);
    vehicle.Advance(step);

    // Update the sensor manager
    if (render) {
      manager->Update();
    }

    if (step_number == 0) {
      if (enable_realtime) {
        realtime_timer.Reset();
      }

      t0 = high_resolution_clock::now();
    }

    // Stream out data
    boost_streamer.AddData(vehicle.GetSystem()->GetChTime());
    boost_streamer.AddData(vehicle.GetSpeed() * MS_TO_MPH);
    boost_streamer.AddData(vehicle.GetEngine()->GetMotorSpeed() * rads2rpm);
    boost_streamer.Synchronize();

    // Increment frame number
    step_number++;

    if (step_number % 5 == 0 && enable_realtime) {
      auto tt0 = high_resolution_clock::now();
      realtime_timer.Spin(sim_time);
      auto tt1 = high_resolution_clock::now();
      extra_time += duration_cast<duration<double>>(tt1 - tt0).count();
    }

    auto t1 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t1 - t0);
    std::cout << "Simulation Time: " << sim_time
              << ", Wall Time: " << time_span.count() << std::endl;
  }
  return 0;
}

void AddTrees(ChSystem *chsystem) {
  // add in tress next to road
  auto tree_mesh_0 = chrono_types::make_shared<ChTriangleMeshConnected>();
  tree_mesh_0->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/trees/tree_01.obj", false, true);
  tree_mesh_0->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size

  auto tree_mesh_1 = chrono_types::make_shared<ChTriangleMeshConnected>();
  tree_mesh_1->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/foliage/trees/oaktree_01.obj", false,
      true);
  tree_mesh_1->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size

  auto tree_mesh_2 = chrono_types::make_shared<ChTriangleMeshConnected>();
  tree_mesh_2->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/foliage/trees/oaktree_02.obj", false,
      true);
  tree_mesh_2->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size

  auto tree_mesh_3 = chrono_types::make_shared<ChTriangleMeshConnected>();
  tree_mesh_3->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/foliage/trees/oaktree_03.obj", false,
      true);
  tree_mesh_3->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size

  std::vector<std::shared_ptr<ChTriangleMeshConnected>> tree_meshes = {
      tree_mesh_0, tree_mesh_1, tree_mesh_2, tree_mesh_3};

  ChSetRandomSeed(4);

  // tree placement parameters
  double x_step = 90;
  double y_step = 400;
  int x_count = 2;
  int y_count = 50;
  float y_variation = 300.f;
  float scale_nominal = .5;
  float scale_variation = .4f;
  float x_variation = .1f;

  double x_start = 4000.0 - 45; // initLoc.x()-38;
  double y_start = -10000.0;    // initLoc.y()-7500;

  for (int i = 0; i < x_count; i++) {
    for (int j = 0; j < y_count; j++) {
      auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
      trimesh_shape->SetMesh(
          tree_meshes[int(ChRandom() * tree_meshes.size() - .001)]);
      trimesh_shape->SetName("Tree");
      float scale = scale_nominal + scale_variation * (ChRandom() - .5);
      trimesh_shape->SetScale({scale, scale, scale});
      trimesh_shape->SetMutable(false);

      auto mesh_body = chrono_types::make_shared<ChBody>();
      mesh_body->SetPos({i * x_step + x_start + x_variation * (ChRandom() - .5),
                         j * y_step + y_start + y_variation * (ChRandom() - .5),
                         0.0});
      mesh_body->SetRot(Q_from_AngZ(CH_C_PI_2 * ChRandom()));
      mesh_body->AddVisualShape(trimesh_shape);
      mesh_body->SetBodyFixed(true);
      mesh_body->SetCollide(false);
      chsystem->Add(mesh_body);
    }
  }

  x_start = -4052.0 - 45; // initLoc.x()-38;
  y_start = -10000.0;     // initLoc.y()-7500;

  for (int i = 0; i < x_count; i++) {
    for (int j = 0; j < y_count; j++) {
      auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
      trimesh_shape->SetMesh(
          tree_meshes[int(ChRandom() * tree_meshes.size() - .001)]);
      trimesh_shape->SetName("Tree");
      float scale = scale_nominal + scale_variation * (ChRandom() - .5);
      trimesh_shape->SetScale({scale, scale, scale});
      trimesh_shape->SetMutable(false);

      auto mesh_body = chrono_types::make_shared<ChBody>();
      mesh_body->SetPos({i * x_step + x_start + x_variation * (ChRandom() - .5),
                         j * y_step + y_start + y_variation * (ChRandom() - .5),
                         0.0});
      mesh_body->SetRot(Q_from_AngZ(CH_C_PI_2 * ChRandom()));
      mesh_body->AddVisualShape(trimesh_shape);
      mesh_body->SetBodyFixed(true);
      mesh_body->SetCollide(false);
      chsystem->Add(mesh_body);
    }
  }
}

void AddRoadway(ChSystem *chsystem) {
  std::vector<std::string> environment_meshes = {
      "/Environments/Iowa/signs/mile_markers_inner.obj",
      "/Environments/Iowa/signs/mile_markers_outer.obj",
      "/Environments/Iowa/terrain/oval_highway.obj"};
  std::vector<ChVector<>> offsets = {
      {0, 0, -128.22}, {0, 0, 0.0}, {0, 0, 0.01}};

  for (int i = 0; i < environment_meshes.size();
       i++) { // auto file_name : environment_meshes) {
    // additional environment assets
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(demo_data_path + environment_meshes[i], true,
                               true);
    trimesh->Transform(ChVector<>(0, 0, 0),
                       ChMatrix33<>(1)); // scale to a different size
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(environment_meshes[i]);
    trimesh_shape->SetMutable(false);

    if (i == 2) {
      auto road_tex = chrono_types::make_shared<ChVisualMaterial>();
      road_tex->SetKdTexture(
          demo_data_path +
          "/Environments/Iowa/terrain/T_Road002_Dashed_Dirt_Color2.png");
      road_tex->SetRoughnessTexture(
          demo_data_path +
          "/Environments/Iowa/terrain/T_Road002_Dashed_Dirt_Roughness2.png");
      road_tex->SetNormalMapTexture(
          demo_data_path +
          "/Environments/Iowa/terrain/T_Road002_Dashed_Dirt_Normal2.png");
      road_tex->SetMetallicTexture(
          demo_data_path +
          "/Environments/Iowa/terrain/T_Road002_Dashed_Dirt_Metallic2.png");
      trimesh_shape->GetMaterials()[0] = road_tex;
    }

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos(offsets[i]);
    mesh_body->AddVisualShape(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mesh_body->SetCollide(false);
    chsystem->Add(mesh_body);
  }

  // Add arrived sign
  {
    std::string meshname = "/Environments/Iowa/signs/arrived.obj";
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(demo_data_path + meshname, false, true);
    trimesh->Transform(ChVector<>(0, 0, 0),
                       ChMatrix33<>(1)); // scale to a different size
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(meshname);
    trimesh_shape->SetMutable(false);
    // trimesh_shape->SetStatic(true);
    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos(arrived_sign_pos);
    mesh_body->SetRot(arrived_sign_rot);
    mesh_body->AddVisualShape(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mesh_body->SetCollide(false);
    chsystem->Add(mesh_body);
  }
}

void AddBuildings(ChSystem *chsystem) {
  std::vector<std::string> environment_meshes = {
      "/Environments/Iowa/buildings/farm_01.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_01.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_01.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //
      "/Environments/Iowa/buildings/farm_01.obj", //
      "/Environments/Iowa/buildings/farm_04.obj", //
      "/Environments/Iowa/buildings/farm_02.obj", //
      "/Environments/Iowa/buildings/farm_01.obj", //
      "/Environments/Iowa/buildings/farm_03.obj", //

      "/Environments/Iowa/buildings/radio_tower.obj", //
      "/Environments/Iowa/buildings/radio_tower.obj", //
      "/Environments/Iowa/buildings/radio_tower.obj", //
      "/Environments/Iowa/buildings/radio_tower.obj", //
      "/Environments/Iowa/buildings/water_tower.obj", //
      "/Environments/Iowa/buildings/water_tower.obj"};
  std::vector<ChVector<>> offsets = {
      {4150, 12776, 0},           // farm 4000
      {3800, 10209, 0},           // farm
      {3920, 8372, 0},            // farm
      {4200, 5251, 0},            // farm
      {3850, 500, 0.0},           // farm
      {4120, -1787, 0},           // farm
      {3900, -3623, 0},           // farm
      {3860, -5458, 0},           // farm
      {4180, -8000, 0},           // farm
      {3510, -14004, 0},          // farm
      {-1832, -15665, 0},         // farm
      {-4050 + 200, -10654, 0.0}, // farm -4050 200
      {-4050 + 180, -8683, 0.0},  // farm 180
      {-4050 - 120, -6634, 0.0},  // farm -120
      {-4050 + 150, -2990, 0.0},  // farm 150
      {-4050 - 120, -1040, 0.0},  // farm -120
      {-4050 - 180, -797, 0.0},   // farm -180
      {-4050 + 160, 2626, 0.0},   // farm 160
      {-4050 - 110, 4461, 0.0},   // farm -110
      {-4050 + 130, 6292, 0.0},   // farm 130
      {-4050 + 100, 8730, 0.0},   // farm 100
      {-2602, 15320, 0.0},        // farm
      {-1312, 15459, 0.0},        // farm
      {-4167, 7087, 0.0},         // radio tower
      {-4167, 10630, 0.0},        // radio tower
      {4100, -10630, 0.0},        // radio tower
      {4100, 3543, 0.0},          // radio tower
      {2657, 14488, 0.0},         // water tower
      {-2922, -14611, 0.0}        // water tower
  };

  if (offsets.size() != environment_meshes.size()) {
    std::cout << "ERROR: incorrect number of offsets for building meshes\n";
    return;
  }
  for (int i = 0; i < environment_meshes.size(); i++) {
    // additional environment assets
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(demo_data_path + environment_meshes[i], false,
                               true);
    trimesh->Transform(ChVector<>(0, 0, 0),
                       ChMatrix33<>(1)); // scale to a different size
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(environment_meshes[i]);
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetScale({3, 3, 3});
    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos(offsets[i]);
    mesh_body->SetRot(Q_from_AngZ(CH_C_2PI * ChRandom()));
    mesh_body->AddVisualShape(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mesh_body->SetCollide(false);
    chsystem->Add(mesh_body);
  }
}

void AddTerrain(ChSystem *chsystem) {
  // add terrain with weighted textures
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  terrain_mesh->LoadWavefrontMesh(
      demo_data_path + "/Environments/Iowa/terrain/terrain_v2.obj", false,
      true);
  terrain_mesh->Transform(ChVector<>(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto gravel_tex = chrono_types::make_shared<ChVisualMaterial>();
  gravel_tex->SetKdTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Grass/GroundMudTireTracks001_COL_500.jpg");
  gravel_tex->SetRoughnessTexture(demo_data_path +
                                  "/Environments/Iowa/terrain/Grass/"
                                  "GroundMudTireTracks001_ROUGH_500.png");
  gravel_tex->SetNormalMapTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Grass/GroundMudTireTracks001_NRM_500.jpg");
  gravel_tex->SetWeightTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Terrain_Weightmap_Gravel_v3.png");
  gravel_tex->SetSpecularColor({.0f, .0f, .0f});
  // gravel_tex->SetTextureScale({1000.0, 1000.0, 1.0});
  gravel_tex->SetRoughness(1.f);
  gravel_tex->SetUseSpecularWorkflow(false);
  terrain_shape->GetMaterials()[0] = gravel_tex;

  auto grass_tex_1 = chrono_types::make_shared<ChVisualMaterial>();
  grass_tex_1->SetKdTexture(demo_data_path + "/Environments/Iowa/terrain/Grass/"
                                             "GroundGrassGreen001_COL_500.jpg");
  grass_tex_1->SetRoughnessTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Grass/GroundGrassGreen001_ROUGH_500.jpg");
  grass_tex_1->SetNormalMapTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Grass/GroundGrassGreen001_NRM_500.jpg");
  grass_tex_1->SetWeightTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Terrain_Weightmap_Grass_A_v3.png");
  // grass_tex_1->SetTextureScale({1000.0, 1000.0, 1.0});
  grass_tex_1->SetSpecularColor({.0f, .0f, .0f});
  grass_tex_1->SetRoughness(1.f);
  grass_tex_1->SetUseSpecularWorkflow(false);
  terrain_shape->AddMaterial(grass_tex_1);

  auto grass_tex_2 = chrono_types::make_shared<ChVisualMaterial>();
  grass_tex_2->SetKdTexture(demo_data_path +
                            "/Environments/Iowa/terrain/Grass/"
                            "GroundGrassGreenPatchy002_COL_500.jpg");
  grass_tex_2->SetRoughnessTexture(demo_data_path +
                                   "/Environments/Iowa/terrain/Grass/"
                                   "GroundGrassGreenPatchy002_ROUGH_500.png");
  grass_tex_2->SetNormalMapTexture(demo_data_path +
                                   "/Environments/Iowa/terrain/Grass/"
                                   "GroundGrassGreenPatchy002_NRM_500.jpg");
  grass_tex_2->SetWeightTexture(
      demo_data_path +
      "/Environments/Iowa/terrain/Terrain_Weightmap_Grass_B_v3.png");
  grass_tex_2->SetSpecularColor({.0f, .0f, .0f});
  // grass_tex_2->SetTextureScale({1000.0, 1000.0, 1.0});
  grass_tex_2->SetRoughness(1.f);
  grass_tex_2->SetUseSpecularWorkflow(false);
  terrain_shape->AddMaterial(grass_tex_2);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, -.01});
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetBodyFixed(true);
  terrain_body->SetCollide(false);
  chsystem->Add(terrain_body);
}

float ControlFindSpeed_x_y(std::vector<float> time_vec,
                           std::vector<float> speed_vec, float time,
                           float default_speed) {
  int n = time_vec.size();
  int target_idx = n;
  // find out which range the 'time' argument falls into
  for (int i = 0; i < n; i++) {
    if (time <= time_vec[i]) {
      target_idx = i;
      break;
    }
  }
  if (target_idx == 0) {
    return default_speed;
  } else {
    return speed_vec[target_idx - 1];
  }
}
