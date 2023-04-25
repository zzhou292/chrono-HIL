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
// Author: Jason Zhou
// =============================================================================
// Introduces ROM distibutor to Synchrono framework to allow parallel
// computation
// This is the ROM distributor side
// simply launch this program, it will distribute simulation data to synchrono
// rank 1 and rank 2, encapsulated in "test_HIL_rom_synchrono.cpp"
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

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <irrlicht.h>
#include <time.h>

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/syn/Ch_8DOF_zombie.h"
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

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_hil/network/tcp/ChTCPClient.h"
#include "chrono_hil/network/tcp/ChTCPServer.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"
// =============================================================================

#define IP_OUT "127.0.0.1"
#define PORT_IN_1 1204
#define PORT_IN_2 1205
#define PORT_IN_3 1206

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::hil;
using namespace chrono::vehicle;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = 1e-3;
float heartbeat = 1e-2;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

ChVector<> initLoc(0, 0, 1.4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =====================================================

int main(int argc, char *argv[]) {

  ChSystemSMC my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
  int num_rom = 2500; // number of rom on each distributor

  std::vector<std::shared_ptr<Ch_8DOF_vehicle>>
      rom_vec; // rom vector, for node 0

  float init_height = 0.45;
  std::string rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";

  for (int i = 0; i < num_rom; i++) {
    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(rom_json, init_height,
                                                   step_size, false);
    rom_veh->SetInitPos(initLoc + ChVector<>(0.0, 0.0 + i * 3.0, init_height));
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
  double t_end = 14.0;

  ChTCPServer rom_distributor_1(
      PORT_IN_2,
      3); // create the 1st TCP server to send data to synchrono rank 1

  rom_distributor_1.Initialize();

  while (time <= t_end) {

    time = my_system.GetChTime();

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

    if (step_number % 20 == 0) {
      // send data to chrono
      std::vector<float> data_to_send;
      for (int i = 0; i < num_rom; i++) {
        ChVector<> rom_pos = rom_vec[i]->GetPos();
        ChQuaternion<> rom_rot = rom_vec[i]->GetRot();
        ChVector<> rom_rot_vec = rom_rot.Q_to_Euler123();
        DriverInputs rom_inputs = rom_vec[i]->GetDriverInputs();

        data_to_send.push_back(rom_pos.x());
        data_to_send.push_back(rom_pos.y());
        data_to_send.push_back(rom_pos.z());
        data_to_send.push_back(rom_rot_vec.x());
        data_to_send.push_back(rom_rot_vec.y());
        data_to_send.push_back(rom_rot_vec.z());
        data_to_send.push_back(rom_inputs.m_steering);
        data_to_send.push_back(rom_vec[i]->GetTireRotation(0));
        data_to_send.push_back(rom_vec[i]->GetTireRotation(1));
        data_to_send.push_back(rom_vec[i]->GetTireRotation(2));
        data_to_send.push_back(rom_vec[i]->GetTireRotation(3));
      }
      rom_distributor_1.Write(data_to_send);
      // receive data from chrono_1
      rom_distributor_1.Read();
      std::vector<float> recv_data;
      recv_data = rom_distributor_1.GetRecvData();
      for (int i = 0; i < recv_data.size(); i++) {
        // std::cout << recv_data[i] << ",";
      }
      // std::cout << std::endl;
    }

    for (int i = 0; i < num_rom; i++) {
      rom_vec[i]->Advance(time, driver_inputs);
    }

    terrain.Advance(step_size);
    my_system.DoStepDynamics(step_size);

    // std::cout << "time:" << time << std::endl;

    // Increment frame number
    step_number++;
  }
}