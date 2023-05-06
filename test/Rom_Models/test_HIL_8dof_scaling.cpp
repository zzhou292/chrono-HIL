// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// Jason Zhou
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Author: Jason Zhou
// =============================================================================
// This demo conducts a scaling test on the 8dof vehicle model
// =============================================================================

#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <irrlicht.h>
#include <time.h>

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/core/ChBezierCurve.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

#define IP_OUT "128.104.190.187"
#define PORT_OUT 1204
// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;
using namespace chrono::hil;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

ChVector<> initLoc(0, 0, 1.4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

int main(int argc, char *argv[]) {
  ChSystemSMC my_system;
  int num_rom = 300;

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::vector<std::shared_ptr<Ch_8DOF_vehicle>> rom_vec;

  float init_height = 0.45;
  std::string rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";

  for (int i = 0; i < num_rom; i++) {
    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(rom_json, init_height,
                                                   step_size);
    rom_veh->SetInitPos(initLoc + ChVector<>(0.0, 0.0 + i * 2.0, init_height));
    rom_veh->SetInitRot(0.0);
    rom_veh->Initialize(&my_system);
    rom_vec.push_back(rom_veh);
    std::cout << "initialize: " << i << std::endl;
  }

  // Initialize terrain
  RigidTerrain terrain(&my_system);

  double terrainLength = 200.0; // size in X direction
  double terrainWidth = 200.0;  // size in Y direction

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200,
                    200);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
  terrain.Initialize();

  // Create a body that camera attaches to
  auto attached_body = std::make_shared<ChBody>();
  my_system.AddBody(attached_body);
  attached_body->SetPos(ChVector<>(0.0, 0.0, 0.0));
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Create camera
  // Create the camera sensor
  auto manager = chrono_types::make_shared<ChSensorManager>(&my_system);
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  auto cam = chrono_types::make_shared<ChCameraSensor>(
      attached_body, // body camera is attached to
      35,            // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(20.0, -35.0, 1.0),
          Q_from_Euler123(ChVector<>(0.0, 0.0, C_PI / 2))), // offset pose
      1920,                                                 // image width
      1080,                                                 // image
      1.608f, 1); // fov, lag, exposure cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);
  // manager->Update();

  // Set the time response for steering and throttle keyboard inputs.
  double steering_time = 1.0; // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0; // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;
  double time = 0.0;

  // Simulation end time
  double t_end = 0.0;

  t_end = 14.0;

  auto tt_0 = std::chrono::high_resolution_clock::now();
  double start_time;

  // create boost data streaming interface
  ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);

  while (time <= t_end) {

    time = my_system.GetChTime();
    // std::cout << "time: " << time << std::endl;

    // End simulation
    if (time >= t_end)
      break;

    // Driver inputs
    DriverInputs driver_inputs;

    if (time < 3.0f) {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    } else if (time >= 3.0f && time < 8.0f) {
      driver_inputs.m_throttle = 0.5;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    } else if (time >= 8.0f && time < 12.0f) {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.6;
      driver_inputs.m_steering = 0.0;
    } else {
      driver_inputs.m_throttle = 0.0;
      driver_inputs.m_braking = 0.0;
      driver_inputs.m_steering = 0.0;
    }

    // Update modules (process inputs from other modules)
    // terrain.Synchronize(time);

    // Advance simulation for one timestep for all modules
    for (int i = 0; i < num_rom; i++) {
      rom_vec[i]->Advance(time, driver_inputs);
    }
    terrain.Advance(step_size);
    my_system.DoStepDynamics(step_size);

    // manager->Update();

    if (step_number == 0) {
      auto tt_1 = std::chrono::high_resolution_clock::now();
      start_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(tt_1 - tt_0)
              .count();
    }

    if (step_number != 0 && step_number % 2000 == 0) {
      auto tt_2 = std::chrono::high_resolution_clock::now();
      auto wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(tt_2 - tt_0)
              .count() -
          start_time;
      std::cout << "RTF: " << wall_time / time << std::endl;
    }

    // Send out data
    for (int i = 0; i < num_rom; i++) {
      boost_streamer.AddData(rom_vec[i]->GetPos().x()); // 18 - current RPM
    }

    boost_streamer.Synchronize();

    // Increment frame number
    step_number++;
  }
}