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
// This is the Synchrono side
// Launch 2 ranks with rank_id as 1 and 2
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
// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

#define IP_OUT "127.0.0.1"
#define PORT_IN_1 1204
#define PORT_IN_2 1205
#define PORT_IN_3 1206
#define PORT_IN_4 1207
#define PORT_IN_5 1208
#define PORT_IN_6 1209
#define PORT_IN_7 1210
#define PORT_IN_8 1211
#define PORT_IN_9 1212
#define PORT_IN_10 1213
// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;
using namespace chrono::hil;
using namespace chrono::synchrono;

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

// =============================================================================
// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;
// =============================================================================

// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("syn", "d,node_id", "ID for this Node", "0");
  cli.AddOption<int>("syn", "n,num_nodes", "Number of Nodes", "1");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
}

int main(int argc, char *argv[]) {
  // read from cli
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");
  const std::vector<std::string> ip_list =
      cli.GetAsType<std::vector<std::string>>("ip");

  ChSystemSMC my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
  int num_rom = 2500;
  int num_dis = 16;

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // -----------------------
  // Create SynChronoManager
  // -----------------------

  // Use UDP4
  DomainParticipantQos qos;
  qos.name("/syn/node/" + std::to_string(node_id) + ".0");
  qos.transport().user_transports.push_back(
      std::make_shared<UDPv4TransportDescriptor>());

  qos.transport().use_builtin_transports = false;
  qos.wire_protocol().builtin.avoid_builtin_multicast = false;

  // Set the initialPeersList
  for (const auto &ip : ip_list) {
    Locator_t locator;
    locator.kind = LOCATOR_KIND_UDPv4;
    IPLocator::setIPv4(locator, ip);
    qos.wire_protocol().builtin.initialPeersList.push_back(locator);
  }
  auto communicator = chrono_types::make_shared<SynDDSCommunicator>(qos);
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  syn_manager.SetHeartbeat(heartbeat);

  // Vehicle, node_id = 1
  std::string vehicle_filename =
      vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
  std::string tire_filename =
      vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("hmmwv/powertrain/HMMWV_ShaftsPowertrain.json");
  std::string zombie_filename =
      CHRONO_DATA_DIR + std::string("synchrono/vehicle/HMMWV.json");

  // Create the HMMWV vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle((ChSystem *)&my_system, vehicle_filename);
  std::vector<std::shared_ptr<Ch_8DOF_zombie>> zombie_vec;

  auto ego_chassis = my_vehicle.GetChassis();
  my_vehicle.Initialize(ChCoordsys<>(
      initLoc + ChVector<>(0.0, 0.0 + (num_rom + (node_id - 1)) * 3.0, 0.25),
      initRot));
  my_vehicle.GetChassis()->SetFixed(false);
  auto powertrain = ReadPowertrainJSON(powertrain_filename);
  my_vehicle.InitializePowertrain(powertrain);
  my_vehicle.SetChassisVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSuspensionVisualizationType(VisualizationType::MESH);
  my_vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
  my_vehicle.SetWheelVisualizationType(VisualizationType::MESH);

  // Create and initialize the tires
  for (auto &axle : my_vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(tire_filename);
      tire->SetStepsize(step_size / 2);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  // Initialize ROM zombie
  float init_height = 0.45;
  std::string rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";

  for (int i = 0; i < num_rom * num_dis; i++) {
    auto rom_zombie =
        chrono_types::make_shared<Ch_8DOF_zombie>(rom_json, init_height, false);
    zombie_vec.push_back(rom_zombie);
    rom_zombie->Initialize(&my_system);
    std::cout << "zombie: " << i << std::endl;
  }

  // Add vehicle as an agent and initialize SynChronoManager
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(my_vehicle.GetSystem());

  // Initialize terrain
  RigidTerrain terrain(&my_system);

  double terrainLength = 500.0; // size in X direction
  double terrainWidth = 500.0;  // size in Y direction

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
      25,            // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(20.0, -13.0, 18.0),
          Q_from_Euler123(ChVector<>(0.0, C_PI / 8, C_PI / 2))), // offset pose
      1920,                                                      // image width
      1080,                                                      // image
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
  double t_end = 14.0;

  std::vector<ChTCPClient> client_vec;

  ChTCPClient chrono_1("127.0.0.1", PORT_IN_1,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_2("127.0.0.1", PORT_IN_2,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_3("127.0.0.1", PORT_IN_3,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_4("127.0.0.1", PORT_IN_4,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_5("127.0.0.1", PORT_IN_5,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_6("127.0.0.1", PORT_IN_6,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_7("128.104.190.187", PORT_IN_1,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_8("128.104.190.187", PORT_IN_2,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_9("128.104.190.187", PORT_IN_3,
                       num_rom * 11); // create a TCP client
  ChTCPClient chrono_10("128.104.190.187", PORT_IN_4,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_11("128.104.190.187", PORT_IN_5,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_12("128.104.190.187", PORT_IN_6,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_13("128.104.190.187", PORT_IN_7,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_14("128.104.190.187", PORT_IN_8,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_15("128.104.190.187", PORT_IN_9,
                        num_rom * 11); // create a TCP client
  ChTCPClient chrono_16("128.104.190.187", PORT_IN_10,
                        num_rom * 11); // create a TCP client
  client_vec.push_back(chrono_1);
  client_vec.push_back(chrono_2);
  client_vec.push_back(chrono_3);
  client_vec.push_back(chrono_4);
  client_vec.push_back(chrono_5);
  client_vec.push_back(chrono_6);
  client_vec.push_back(chrono_7);
  client_vec.push_back(chrono_8);
  client_vec.push_back(chrono_9);
  client_vec.push_back(chrono_10);
  client_vec.push_back(chrono_11);
  client_vec.push_back(chrono_12);
  client_vec.push_back(chrono_13);
  client_vec.push_back(chrono_14);
  client_vec.push_back(chrono_15);
  client_vec.push_back(chrono_16);
  for (int j = 0; j < num_dis; j++) {
    client_vec[j].Initialize();
  }

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

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

    // ==================================================
    // TCP Synchronization Section
    // ==================================================
    // read data from rom distributor 1
    if (step_number % 20 == 0) {

      for (int j = 0; j < num_dis; j++) {
        client_vec[j].Read();

        std::vector<float> recv_data;
        recv_data = client_vec[j].GetRecvData();

        for (int i = 0; i < num_rom; i++) {
          zombie_vec[i + j * num_dis]->Update(
              ChVector<>(recv_data[0 + i * 11], recv_data[1 + i * 11],
                         recv_data[2 + i * 11]),
              ChVector<>(recv_data[3 + i * 11], recv_data[4 + i * 11],
                         recv_data[5 + i * 11]),
              recv_data[6 + i * 11], recv_data[7 + i * 11],
              recv_data[8 + i * 11], recv_data[9 + i * 11],
              recv_data[10 + i * 11]);
        }

        // send data to distributor 1
        std::vector<float> data_to_send;
        data_to_send.push_back(my_vehicle.GetChassis()->GetPos().x());
        data_to_send.push_back(my_vehicle.GetChassis()->GetPos().y());
        data_to_send.push_back(my_vehicle.GetChassis()->GetPos().z());

        client_vec[j].Write(data_to_send);
      }
    }
    // ==================================================
    // END OF TCP Synchronization Section
    // ==================================================
    my_vehicle.Synchronize(time, driver_inputs, terrain);
    my_vehicle.Advance(step_size);

    terrain.Advance(step_size);
    my_system.DoStepDynamics(step_size);

    if (step_number % 500 == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      SynLog() << (wall_time.count()) / (time - last_time) << "\n";
      last_time = time;
      start = std::chrono::high_resolution_clock::now();
    }

    if (node_id == 1 || node_id == 2 || node_id == 3) {
      syn_manager.Synchronize(time);
    }

    if (node_id == 1) {
      // manager->Update();
    }

    // Increment frame number
    step_number++;
  }
}
