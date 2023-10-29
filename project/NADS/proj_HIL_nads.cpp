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

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/network/udp/ChBoostInStreamer.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::utils;
using namespace chrono::synchrono;

const double RADS_2_RPM = 30 / CH_C_PI;
const double RADS_2_DEG = 180 / CH_C_PI;
const double MS_2_MPH = 2.2369;
const double M_2_FT = 3.28084;
const double G_2_MPSS = 9.81;

#undef USENADS

#ifdef USENADS
#define PORT_IN 9090
#define PORT_OUT 9091
#define PORT_OUT_2 9092
#define IP_OUT "90.0.0.125"
#define IP_OUT_2 "90.0.0.120"
#else
#define PORT_IN 1209
#define PORT_OUT 1204
#define PORT_OUT_2 9092
#define IP_OUT "127.0.0.1"
#define IP_OUT_2 "127.0.0.1"
#endif

bool render = false;
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

  // ==========================================================================
  ChCLI cli(argv[0]);

  AddCommandLineOptions(cli);
  if (!cli.Parse(argc, argv, false, false))
    return 0;

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");

  std::cout << "id:" << node_id << std::endl;
  std::cout << "num:" << num_nodes << std::endl;

  // =============================================================================

  // -----------------------
  // Create SynChronoManager
  // -----------------------
  auto communicator = chrono_types::make_shared<SynDDSCommunicator>(node_id);
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  // Change SynChronoManager settings
  float heartbeat = 0.02f;
  syn_manager.SetHeartbeat(heartbeat);

  // ========================================================================

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

  // Add vehicle as an agent and initialize SynChronoManager
  std::string zombie_filename =
      CHRONO_DATA_DIR + std::string("synchrono/vehicle/audi.json");
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(my_vehicle.GetSystem());

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
  auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
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
  int render_step = 20;
  auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("NADS");
  vis->SetChaseCamera(trackPoint, 6.0, 0.5);
  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(&my_vehicle);

  // ------------------------
  // Create the driver system
  // ------------------------

  ChBoostInStreamer in_streamer(PORT_IN, 4);
  std::vector<float> recv_data;

  // ---------------
  // Simulation loop
  // ---------------
  std::cout << "\nVehicle mass: " << my_vehicle.GetMass() << std::endl;
  std::cout << "\nIP_OUT: " << IP_OUT << std::endl;

  // Initialize simulation frame counters
  int step_number = 0;

  my_vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  // create boost data streaming interface
  ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);
  ChBoostOutStreamer boost_traffic_streamer(IP_OUT_2, PORT_OUT_2);

  // obtain and initiate all zombie instances
  std::map<AgentKey, std::shared_ptr<SynAgent>> zombie_map;
  std::map<int, std::shared_ptr<SynWheeledVehicleAgent>> id_map;

  // declare a set of moving average filter for data smoothing
  ChRunningAverage acc_x(250);
  ChRunningAverage acc_y(250);
  ChRunningAverage acc_z(250);

  ChRunningAverage ang_vel_x(250);
  ChRunningAverage ang_vel_y(250);
  ChRunningAverage ang_vel_z(250);

  ChRunningAverage eyepoint_vel_x(250);
  ChRunningAverage eyepoint_vel_y(250);
  ChRunningAverage eyepoint_vel_z(250);

  ChRunningAverage eyepoint_acc_x(250);
  ChRunningAverage eyepoint_acc_y(250);
  ChRunningAverage eyepoint_acc_z(250);

  ChRunningAverage lf_wheel_vel(250);
  ChRunningAverage rf_wheel_vel(250);
  ChRunningAverage lr_wheel_vel(250);
  ChRunningAverage rr_wheel_vel(250);

  // simulation loop
  while (vis->Run() && syn_manager.IsOk()) {
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
    DriverInputs driver_inputs;

    if (step_number % 4 == 0) {
      in_streamer.Synchronize();
      recv_data = in_streamer.GetRecvData();
    }

    driver_inputs.m_throttle = recv_data[0];
    driver_inputs.m_steering = recv_data[1];
    driver_inputs.m_braking = recv_data[2];

    // this might be problematic
    float gear = recv_data[3];
    auto trans = my_vehicle.GetTransmission();
    int gear_to_send = 0;
    if (gear == 0.0) {
      driver_inputs.m_braking = 1.0;
      driver_inputs.m_throttle = 0.0;

      gear_to_send = 0;
    } else if (gear == 1.0) {
      if (trans->GetCurrentGear() <= 0) {
        trans->SetGear(1);
      }

      gear_to_send = trans->GetCurrentGear();
    } else if (gear == 2.0) {
      trans->SetGear(-1);

      gear_to_send = -1;
    } else if (gear == 3.0) {
      driver_inputs.m_throttle = 0.0;

      gear_to_send = 0;
    }

    // =======================
    // data stream out section
    // =======================
    if (step_number % 4 == 0) {
      // Time
      boost_streamer.AddData((float)time); // 0 - time

      // Chassis location
      boost_streamer.AddData(pos.x() * M_2_FT);  // 1
      boost_streamer.AddData(-pos.y() * M_2_FT); // 2
      boost_streamer.AddData(-pos.z() * M_2_FT); // 3

      // Eyepoint position
      ChVector<> eyepoint_global = my_vehicle.GetPointLocation(driver_eyepoint);

      boost_streamer.AddData(-eyepoint_global.y() * M_2_FT); // 4
      boost_streamer.AddData(eyepoint_global.x() * M_2_FT);  // 5
      boost_streamer.AddData(eyepoint_global.z() * M_2_FT);  // 6

      // Eyepoint orientation
      auto eu_rot = Q_to_Euler123(rot);

      boost_streamer.AddData(eu_rot.z() * RADS_2_DEG);  // 7 - yaw
      boost_streamer.AddData(-eu_rot.y() * RADS_2_DEG); // 8 - pitch
      boost_streamer.AddData(eu_rot.x() * RADS_2_DEG);  // 9 - roll

      // Chassis angular velocity
      auto ang_vel = my_vehicle.GetChassis()->GetBody()->GetWvel_loc();
      auto ang_vel_x_filtered = ang_vel_x.Add(ang_vel.x());
      auto ang_vel_y_filtered = ang_vel_y.Add(ang_vel.y());
      auto ang_vel_z_filtered = ang_vel_z.Add(ang_vel.z());

      boost_streamer.AddData(ang_vel_x_filtered * RADS_2_DEG);  // 10
      boost_streamer.AddData(-ang_vel_y_filtered * RADS_2_DEG); // 11
      boost_streamer.AddData(-ang_vel_z_filtered * RADS_2_DEG); // 12

      // Chassis velocity
      auto vel =
          my_vehicle.GetChassis()->GetBody()->GetFrame_REF_to_abs().GetPos_dt();

      boost_streamer.AddData(vel.x() * M_2_FT);  // 13
      boost_streamer.AddData(-vel.y() * M_2_FT); // 14
      boost_streamer.AddData(-vel.z() * M_2_FT); // 15

      // Eyepoint velocity
      ChVector<> eyepoint_velocity =
          my_vehicle.GetPointVelocity(driver_eyepoint);
      auto eyepoint_velocity_x_filtered =
          eyepoint_vel_x.Add(-eyepoint_velocity.y());
      auto eyepoint_velocity_y_filtered =
          eyepoint_vel_y.Add(eyepoint_velocity.x());
      auto eyepoint_velocity_z_filtered =
          eyepoint_vel_z.Add(eyepoint_velocity.z());

      boost_streamer.AddData(eyepoint_velocity_x_filtered * M_2_FT);  // 16
      boost_streamer.AddData(-eyepoint_velocity_y_filtered * M_2_FT); // 17
      boost_streamer.AddData(-eyepoint_velocity_z_filtered * M_2_FT); // 18

      // Chassis local acceleration
      auto acc_local = my_vehicle.GetPointAcceleration(
          my_vehicle.GetChassis()->GetCOMFrame().GetPos());
      auto acc_loc_x_filtered = acc_x.Add(acc_local.x());
      auto acc_loc_y_filtered = acc_y.Add(-acc_local.y());
      auto acc_loc_z_filtered = acc_z.Add(-acc_local.z());

      boost_streamer.AddData(acc_loc_x_filtered * M_2_FT); // 19
      boost_streamer.AddData(acc_loc_y_filtered * M_2_FT); // 20
      boost_streamer.AddData(acc_loc_z_filtered * M_2_FT); // 21

      // Eyepoint specific force
      // rotation matrix A -> from local to global
      auto A_REF_to_abs =
          my_vehicle.GetChassis()->GetBody()->GetFrame_REF_to_abs().GetA();

      // inverse rotation matrix invA -> from global to local
      ChMatrix33<> inv_A_REF_to_abs = A_REF_to_abs.inverse();

      // local gravity
      auto local_g = inv_A_REF_to_abs * ChVector<>(0.0, 0.0, -9.81);

      auto eye_acc_x_filtered = eyepoint_acc_x.Add(acc_local.x() + local_g.x());
      auto eye_acc_y_filtered =
          eyepoint_acc_y.Add(-acc_local.y() + local_g.y());
      auto eye_acc_z_filtered =
          eyepoint_acc_z.Add(-acc_local.z() + local_g.z());

      boost_streamer.AddData(eye_acc_x_filtered / G_2_MPSS); // 22
      boost_streamer.AddData(eye_acc_y_filtered / G_2_MPSS); // 23
      boost_streamer.AddData(eye_acc_z_filtered / G_2_MPSS); // 24

      // wheel center locations
      auto wheel_LF_state = my_vehicle.GetWheel(0, LEFT)->GetState();
      auto wheel_RF_state = my_vehicle.GetWheel(0, RIGHT)->GetState();
      auto wheel_LR_state = my_vehicle.GetWheel(1, LEFT)->GetState();
      auto wheel_RR_state = my_vehicle.GetWheel(1, RIGHT)->GetState();

      boost_streamer.AddData(
          wheel_RF_state.pos.x()); // 25 - RF wheel center pos x - global
      boost_streamer.AddData(
          wheel_RF_state.pos.y()); // 26 - RF wheel center pos y - global
      boost_streamer.AddData(
          wheel_RF_state.pos.x()); // 27 - LF wheel center pos x - global
      boost_streamer.AddData(
          wheel_LF_state.pos.y()); // 28 - LF wheel center pos y - global
      boost_streamer.AddData(
          wheel_RR_state.pos.x()); // 29 - RR wheel center pos x - global
      boost_streamer.AddData(
          wheel_RR_state.pos.y()); // 30 - RR wheel center pos y - global
      boost_streamer.AddData(
          wheel_LR_state.pos.x()); // 31 - LR wheel center pos x - global
      boost_streamer.AddData(
          wheel_LR_state.pos.y()); // 32 - LR wheel center pos y - global

      // wheel rotational velocity
      auto lf_omega_filtered = lf_wheel_vel.Add(wheel_RF_state.omega);
      auto rf_omega_filtered = rf_wheel_vel.Add(wheel_LF_state.omega);
      auto lr_omega_filtered = lr_wheel_vel.Add(wheel_RR_state.omega);
      auto rr_omega_filtered = rr_wheel_vel.Add(wheel_LR_state.omega);

      boost_streamer.AddData(
          lf_omega_filtered); // 33 - RF wheel rot vel - in rad/s
      boost_streamer.AddData(
          rf_omega_filtered); // 34 - LF wheel rot vel - in rad/s
      boost_streamer.AddData(
          lr_omega_filtered); // 35 - RR wheel rot vel - in rad/s
      boost_streamer.AddData(
          rr_omega_filtered); // 36 - LR wheel rot vel - in rad/s

      boost_streamer.AddData(
          my_vehicle.GetTransmission()->GetCurrentGear()); // 37 - current gear

      boost_streamer.AddData(
          (float)(my_vehicle.GetSpeed() * MS_2_MPH)); // 38 - speed (m/s)

      boost_streamer.AddData(my_vehicle.GetEngine()->GetMotorSpeed() *
                             RADS_2_RPM); // 39 - current RPM

      boost_streamer.AddData(
          my_vehicle.GetEngine()
              ->GetOutputMotorshaftTorque()); // 40 - Engine Torque - in N-m

      // Send the data
      boost_streamer.Synchronize();
    }

    if (step_number == 0) {
<<<<<<< HEAD
      zombie_map = syn_manager.GetZombies();
      std::cout << "zombie size: " << zombie_map.size() << std::endl;
      std::cout << "agent size: " << syn_manager.GetAgents().size()
                << std::endl;
      for (std::map<AgentKey, std::shared_ptr<SynAgent>>::iterator it =
               zombie_map.begin();
           it != zombie_map.end(); ++it) {
        std::shared_ptr<SynAgent> temp_ptr = it->second;
        std::shared_ptr<SynWheeledVehicleAgent> converted_ptr =
            std::dynamic_pointer_cast<SynWheeledVehicleAgent>(temp_ptr);
        id_map.insert(std::make_pair(it->first.GetNodeID(), converted_ptr));
      }
    }

    // obtain map
    if (num_nodes > 1) {
      if (step_number % 16 == 0) {
        int traf_id = 1;
        for (std::map<int, std::shared_ptr<SynWheeledVehicleAgent>>::iterator
                 it = id_map.begin();
             it != id_map.end(); ++it) {

          ChronoVehicleInfo info;
          info.vehicle_id = traf_id;
          info.time_stamp = dds_time_stamp;

          ChVector<double> chassis_pos = it->second->GetZombiePos();
          ChVector<double> chassis_rot =
              it->second->GetZombieRot().Q_to_Euler123();

          // converting chassis
          info.position[0] =
              -chassis_pos.y() * M_2_FT; // chassis_pos.x()*M_2_FT;
          info.position[1] =
              chassis_pos.x() * M_2_FT; //-chassis_pos.y()*M_2_FT;
          info.position[2] =
              chassis_pos.z() * M_2_FT; //-chassis_pos.z()*M_2_FT;

          info.orientation[0] = CH_C_PI - chassis_rot.x(); // chassis_rot.x();
          info.orientation[1] = -chassis_rot.y();          //-chassis_rot.y();
          info.orientation[2] =
              CH_C_PI / 2.0 + chassis_rot.z(); //-chassis_rot.z();

          info.steering_angle =
              driver_inputs.m_steering * double(30.0 / 180.0) * CH_C_PI * 2;
          info.wheel_rotations[0] = 0.0;
          info.wheel_rotations[1] = 0.0;
          info.wheel_rotations[2] = 0.0;
          info.wheel_rotations[3] = 0.0;

          boost_traffic_streamer.AddChronoVehicleInfo(info);
          /*
                          for(int j = 0; j < 4; j++){
                              ChVector<long long> wheel_pos =
             it->second->GetZombieWheelPos(j); ChVector<long long> wheel_rot =
             it->second->GetZombieWheelRot(j).Q_to_Euler123();

                              // converting wheels

                              wheel_rot.y() = -wheel_rot.y();
                              wheel_rot.z() = -wheel_rot.z();

                              boost_traffic_streamer.AddLongLongVector(wheel_pos);
                              boost_traffic_streamer.AddLongLongVector(wheel_rot);
                          }
                          */

          traf_id++;

          // std::cout << info.vehicle_id <<", "<< info.time_stamp << ", " <<
          // info.position[0]<<", "<<info.position[1] << ", " <<
          // info.position[2] << ", " << info.orientation[0] << ",
          // "<<info.orientation[1] << ", "<<info.orientation[2] << ", " <<
          // info.steering_angle << ", " << info.wheel_rotations[0] <<
          // std::endl;
        }
        boost_traffic_streamer.Synchronize();
      }
    }

    == == == = zombie_map = syn_manager.GetZombies();
    std::cout << "zombie size: " << zombie_map.size() << std::endl;
    std::cout << "agent size: " << syn_manager.GetAgents().size() << std::endl;
    for (std::map<AgentKey, std::shared_ptr<SynAgent>>::iterator it =
             zombie_map.begin();
         it != zombie_map.end(); ++it) {
      std::shared_ptr<SynAgent> temp_ptr = it->second;
      std::shared_ptr<SynWheeledVehicleAgent> converted_ptr =
          std::dynamic_pointer_cast<SynWheeledVehicleAgent>(temp_ptr);
      id_map.insert(std::make_pair(it->first.GetNodeID(), converted_ptr));
    }
  }

  // obtain map
  if (num_nodes > 1) {
    if (step_number % 10 == 0) {
      int traf_id = 1;
      for (std::map<int, std::shared_ptr<SynWheeledVehicleAgent>>::iterator it =
               id_map.begin();
           it != id_map.end(); ++it) {

        ChronoVehicleInfo info;
        info.vehicle_id = traf_id;
        info.time_stamp = std::chrono::high_resolution_clock::now()
                              .time_since_epoch()
                              .count();

        ChVector<double> chassis_pos = it->second->GetZombiePos();
        ChVector<double> chassis_rot =
            it->second->GetZombieRot().Q_to_Euler123();

        // converting chassis
        info.position[0] = chassis_pos.x() * M_2_FT;
        info.position[1] = -chassis_pos.y() * M_2_FT;
        info.position[2] = -chassis_pos.z() * M_2_FT;

        info.orientation[0] = chassis_rot.x();
        info.orientation[1] = -chassis_rot.y();
        info.orientation[2] = -chassis_rot.z();

        info.steering_angle =
            driver_inputs.m_steering * double(30.0 / 180.0) * CH_C_PI * 2;
        info.wheel_rotations[0] = 0.0;
        info.wheel_rotations[1] = 0.0;
        info.wheel_rotations[2] = 0.0;
        info.wheel_rotations[3] = 0.0;

        boost_traffic_streamer.AddVehicleStruct(info);
        traf_id++;
      }
      boost_traffic_streamer.Synchronize();
    }
  }

>>>>>>> main
  // =======================
  // end data stream out section
  // =======================

  // Update modules (process inputs from other modules)
  terrain.Synchronize(time);
  my_vehicle.Synchronize(time, driver_inputs, terrain);
  syn_manager.Synchronize(time); // Synchronize between nodes

  // Advance simulation for one time for all modules
  terrain.Advance(step_size);
  my_vehicle.Advance(step_size);
  vis->Advance(step_size);

  // Increment frame number
  step_number++;

  if (step_number == 0) {
    realtime_timer.Reset();
  }

  // if (step_number % 10 == 0) {
  realtime_timer.Spin(time);
  //}

  if (step_number % 500 == 0) {
    std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time =
        std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    std::cout << "elapsed time = " << (wall_time.count()) / (time - last_time)
              << ", t = " << time << ", gear = "
              << gear
              //<< ", Sp Frc Z = " << (eye_acc_z_filtered / G_2_MPSS)
              << "\n";

    last_time = time;
    start = std::chrono::high_resolution_clock::now();
  }

  if (render == true && step_number % render_step == 0) {
    vis->BeginScene();
    vis->Render();
    vis->EndScene();
    vis->Synchronize(time, driver_inputs);
  }
}
syn_manager.QuitSimulation();
return 0;
}

void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
}
