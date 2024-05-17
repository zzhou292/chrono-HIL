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

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include <chrono>

#include "chrono/utils/ChFilters.h"

#include "chrono_hil/driver/ChIDM_Follower.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_hil/driver/ChCSLDriver.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
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
ChVector3<> driver_eyepoint(-0.45, 0.4, 0.98);

// =============================================================================

// Initial vehicle location and orientation
ChVector3<> initLoc(-91.788, 98.647, 0.25);
ChQuaternion<> initRot(1, 0, 0, 0);
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
std::vector<ChVector3<>> obj_pos;
std::vector<ChVector3<>> obj_rot;
std::vector<double> obj_scale;
float delay_val = 0.0;
float cam_delay_val = 0.2;
int lane = 0;
// =============================================================================
void AddCommandLineOptions(ChCLI &cli)
{
  cli.AddOption<std::string>("Simulation", "sim_params",
                             "Path to simulation configuration file",
                             scenario_filename);
  cli.AddOption<float>("Simulation", "delay_val", "Delay value", std::to_string(delay_val));
  cli.AddOption<float>("Simulation", "cam_delay_val", "Camera Delay value", std::to_string(cam_delay_val));
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
      ChVector3<> euler_rot;
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

        ChVector3<> temp_pos;
        for (int j = 0; j < 3; j++)
        {
          temp_pos[j] = marr[j].GetDouble();
        }
        obj_pos.push_back(temp_pos);
      }
      if (d[meshname.c_str()].HasMember("rotations"))
      {
        auto marr = d[meshname.c_str()]["rotations"].GetArray();
        ChVector3<> temp_rot;
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

  ReadParameterFiles();

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

  patch = terrain.AddPatch(patch_mat, CSYSNORM,
                           std::string(STRINGIFY(HIL_DATA_DIR)) +
                               "/Environments/nads/newnads/terrain.obj",
                           true, 0, false);

  terrain.Initialize();

  // add vis mesh
  auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
  terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/Environments/nads/newnads/terrain.obj",
                                  true, true);
  terrain_mesh->Transform(ChVector3<>(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, -.01});
  // terrain_body->SetRot(Q_from_AngX(CH_PI_2));
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetFixed(true);
  terrain_body->EnableCollision(false);
  my_vehicle.GetSystem()->Add(terrain_body);

  // ------------------------
  // Create a Irrlicht vis
  // ------------------------
  // ChVector3<> trackPoint(0.0, 0.0, 1.75);
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
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G29.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);
  SDLDriver.AddCallbackButtons(6);

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
  ChQuaternion<> driver_cam_rot;
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
  ChDelaySim sim(normalDist, 1000000000.f);

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

  static auto last_invoked_1 =
      std::chrono::system_clock::now().time_since_epoch();

  // simulation loop
  while (true)
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto dds_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    double time = my_vehicle.GetSystem()->GetChTime();

    ChVector3<> pos = my_vehicle.GetChassis()->GetPos();
    ChQuaternion<> rot = my_vehicle.GetChassis()->GetRot();

    auto euler_rot = rot.GetCardanAnglesXYZ();
    euler_rot.x() = 0.0;
    euler_rot.y() = 0.0;
    ChQuaternion<> y_0_rot;
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
      std::vector<float> floats = {auto_mode, SDLDriver.GetSteering(), SDLDriver.GetThrottle(), SDLDriver.GetBraking()};

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
    if (check_button_val[0] == 1)
    {
      auto current_invoke_1 =
          std::chrono::system_clock::now().time_since_epoch();

      if (std::chrono::duration_cast<std::chrono::seconds>(current_invoke_1 -
                                                           last_invoked_1)
              .count() >= 1.0)
      {
        auto_mode = ((int)(auto_mode + 1.0)) % 2;

        last_invoked_1 = current_invoke_1;
      }
      last_invoked_1 =
          std::chrono::system_clock::now().time_since_epoch();
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
    ChVector3<> cog;
    ChMatrix33<> inertia;
    mesh->ComputeMassProperties(true, mass, cog, inertia);

    mesh->Transform(ChVector3<>(0, 0, 0), ChMatrix33<>(obj_scale[i]));
    ChMatrix33<> principal_inertia_rot;
    ChVector3<> principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I,
                                     principal_inertia_rot);

    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    sys.Add(body);
    body->SetFixed(true);
    ChQuaternion<> body_rot(1, 0, 0, 0);
    body_rot.SetFromCardanAnglesXYZ(obj_rot[i]);

    body->SetFrameRefToAbs(ChFrame<>(ChVector3<>(obj_pos[i]), body_rot));
    body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
    body->SetMass(mass * cone_density);
    body->SetInertiaXX(cone_density * principal_I);

    auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape->SetMesh(mesh);
    mesh_shape->SetBackfaceCull(true);
    body->AddVisualShape(mesh_shape);
  }
}
