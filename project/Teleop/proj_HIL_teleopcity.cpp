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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include <chrono>

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

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

#include "chrono_hil/network/udp/ChBoostInStreamer.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::utils;
using namespace chrono::sensor;

const double RADS_2_RPM = 30 / CH_C_PI;
const double RADS_2_DEG = 180 / CH_C_PI;
const double MS_2_MPH = 2.2369;
const double M_2_FT = 3.28084;
const double G_2_MPSS = 9.81;

bool render = true;
ChVector<> driver_eyepoint(-0.45, 0.4, 0.98);

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-91.788, 98.647, 0.25);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-6;

// Simulation end time
double t_end = 1000;

// =============================================================================
void AddCommandLineOptions(ChCLI &cli);
int main(int argc, char *argv[]) {

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

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = my_vehicle.GetChassis();
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
  for (auto &axle : my_vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(tire_filename);
      tire->SetStepsize(tire_step_size);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

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
  terrain_mesh->Transform(ChVector<>(0, 0, 0),
                          ChMatrix33<>(1)); // scale to a different size
  auto terrain_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  terrain_shape->SetMesh(terrain_mesh);
  terrain_shape->SetName("terrain");
  terrain_shape->SetMutable(false);

  auto terrain_body = chrono_types::make_shared<ChBody>();
  terrain_body->SetPos({0, 0, -.01});
  // terrain_body->SetRot(Q_from_AngX(CH_C_PI_2));
  terrain_body->AddVisualShape(terrain_shape);
  terrain_body->SetBodyFixed(true);
  terrain_body->SetCollide(false);
  my_vehicle.GetSystem()->Add(terrain_body);

  // ------------------------
  // Create a Irrlicht vis
  // ------------------------
  ChVector<> trackPoint(0.0, 0.0, 1.75);
  // int render_step = 20;
  // auto vis =
  // chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
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
  auto driver_cam = chrono_types::make_shared<ChCameraSensor>(
      my_vehicle.GetChassisBody(), // body camera is attached to
      35,                          // update rate in Hz
      chrono::ChFrame<double>({0.54, .381, 1.04},
                              Q_from_AngAxis(0, {0, 1, 0})), // offset pose
      5760,                                                  // image width
      1080,                                                  // image height
      3.14 / 1.5,                                            // fov
      2);

  driver_cam->SetName("DriverCam");
  driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      5760, 1080, "Camera1", false));
  driver_cam->SetLag(0.2f);
  driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(driver_cam);

  // Initialize simulation frame counters
  int step_number = 0;

  my_vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  ChBoostInStreamer in_streamer(1214, 3);

  DriverInputs driver_inputs;

  // simulation loop
  while (true) {
    auto now = std::chrono::high_resolution_clock::now();
    auto dds_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    double time = my_vehicle.GetSystem()->GetChTime();

    ChVector<> pos = my_vehicle.GetChassis()->GetPos();
    ChQuaternion<> rot = my_vehicle.GetChassis()->GetRot();

    auto euler_rot = Q_to_Euler123(rot);
    euler_rot.x() = 0.0;
    euler_rot.y() = 0.0;
    auto y_0_rot = Q_from_Euler123(euler_rot);

    attached_body->SetPos(pos);
    attached_body->SetRot(y_0_rot);
#ifndef USENADS
    // End simulation
    if (time >= t_end)
      break;
#endif

    // Get driver inputs

    if (step_number % 50 == 0) {
      in_streamer.Synchronize();

      std::vector<float> recv_data = in_streamer.GetRecvData();

      driver_inputs.m_steering = recv_data[0];
      driver_inputs.m_throttle = recv_data[1];
      driver_inputs.m_braking = recv_data[2];
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

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    // if (step_number % 10 == 0) {
    realtime_timer.Spin(time);

    if (SDLDriver.Synchronize() == 1) {
      break;
    }

    // if (render == true && step_number % render_step == 0) {
    //   // vis->BeginScene();
    //   // vis->Render();
    //   // vis->EndScene();
    //   // vis->Synchronize(time, driver_inputs);
    // }
  }
  return 0;
}
