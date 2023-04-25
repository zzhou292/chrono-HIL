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

#include <fstream>
#include <sstream>
#include <vector>

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

// ==============================================================================
int output = 0;
int focus_idx[10] = {5, 22, 34, 38, 45, 61, 73, 92, 104, 140};

// =======================================================================

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

std::string demo_data_path = std::string(STRINGIFY(HIL_DATA_DIR));

// =====================================================

struct rom_item {
  int type;
  ChVector<double> pos;
  ChVector<double> rot;
  int path;
  int ld_id;
  int idm_type;
};

// =====================================================

void ReadRomInitFile(std::string csv_filename, std::vector<rom_item> &rom_arr);

// =====================================================

int main(int argc, char *argv[]) {

  std::vector<rom_item> rom_data;
  std::string filename = std::string(STRINGIFY(HIL_DATA_DIR)) +
                         "/Environments/SanFrancisco/rom_init/test.csv";
  std::cout << filename << std::endl;
  ReadRomInitFile(filename, rom_data);

  ChSystemSMC my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));

  std::string hmmwv_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
  std::string sedan_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";
  std::string patrol_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";
  std::string audi_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";

  std::vector<std::shared_ptr<Ch_8DOF_vehicle>>
      rom_vec; // rom vector, for node 0
  std::vector<std::shared_ptr<ChROM_PathFollowerDriver>>
      driver_vec; // rom driver vector
  std::vector<std::shared_ptr<ChROM_IDMFollower>>
      idm_vec; // rom idm driver vector

  std::string output_file_path = "./rom_output.csv";
  std::ofstream output_filestream = std::ofstream(output_file_path);
  std::stringstream output_buffer;

  std::vector<DriverInputs> input_record;

  // initialize vehicle and drivers
  for (int i = 0; i < rom_data.size(); i++) {
    std::string rom_json;
    if (rom_data[i].type == 0) {
      rom_json = hmmwv_json;
    } else if (rom_data[i].type == 1) {
      rom_json = sedan_json;
    } else if (rom_data[i].type == 2) {
      rom_json = patrol_json;
    } else if (rom_data[i].type == 3) {
      rom_json = audi_json;
    }

    // ROM_VEHICLE
    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(
            rom_json, rom_data[i].pos.z(), step_size);
    rom_veh->SetInitPos(rom_data[i].pos);
    rom_veh->SetInitRot(rom_data[i].rot.z());
    rom_veh->Initialize(&my_system);
    rom_vec.push_back(rom_veh);

    // DRIVER
    std::shared_ptr<ChBezierCurve> path;
    if (rom_data[i].path == 2) {
      path = ChBezierCurve::read(demo_data_path + "/paths/2.txt", true);
    } else if (rom_data[i].path == 3) {
      path = ChBezierCurve::read(demo_data_path + "/paths/3.txt", true);
    } else if (rom_data[i].path == 5) {
      path = ChBezierCurve::read(demo_data_path + "/paths/5.txt", true);
    } else if (rom_data[i].path == 1) {
      path = ChBezierCurve::read(demo_data_path + "/paths/1.txt", true);
    } else if (rom_data[i].path == 6) {
      path = ChBezierCurve::read(demo_data_path + "/paths/6.txt", true);
    } else if (rom_data[i].path == 7) {
      path = ChBezierCurve::read(demo_data_path + "/paths/7.txt", true);
    }

    std::shared_ptr<ChROM_PathFollowerDriver> driver;
    if (rom_data[i].path == 3) {
      driver = chrono_types::make_shared<ChROM_PathFollowerDriver>(
          rom_vec[i], path, 6.0, 10.0, 0.1, 0.0, 0.0, 0.5, 0.0, 0.0);
    } else {
      driver = chrono_types::make_shared<ChROM_PathFollowerDriver>(
          rom_vec[i], path, 6.0, 10.0, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0);
    }

    driver_vec.push_back(driver);

    std::vector<double> params;
    if (rom_data[i].idm_type == 0) {
      params.push_back(10.0);
      params.push_back(0.1);
      params.push_back(5.0);
      params.push_back(3.5);
      params.push_back(2.5);
      params.push_back(4.0);
      params.push_back(6.0);

    } else if (rom_data[i].idm_type == 1) {
      params.push_back(9.0);
      params.push_back(0.2);
      params.push_back(6.0);
      params.push_back(3.0);
      params.push_back(2.1);
      params.push_back(4.0);
      params.push_back(6.0);
    } else if (rom_data[i].idm_type == 2) {
      params.push_back(7.5);
      params.push_back(0.7);
      params.push_back(8.0);
      params.push_back(2.5);
      params.push_back(1.5);
      params.push_back(4.0);
      params.push_back(6.0);
    }
    std::shared_ptr<ChROM_IDMFollower> idm_controller =
        chrono_types::make_shared<ChROM_IDMFollower>(rom_vec[i], driver_vec[i],
                                                     params);
    idm_vec.push_back(idm_controller);
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

  // create boost data streaming interface
  ChRealtimeCumulative realtime_timer;

  ChTCPServer rom_distributor_0(
      1204,
      3); // create the 1st TCP server to send data to synchrono rank 1
  ChTCPServer rom_distributor_1(
      1205,
      3); // creat the 2nd TCP server to send data to synchrono rank 2
  ChTCPServer rom_distributor_2(
      1206,
      3); // creat the 2nd TCP server to send data to synchrono rank 2

  rom_distributor_0.Initialize(); // initialize connection to synchrono rank 1
  rom_distributor_1.Initialize(); // initialize connection to synchrono rank 2
  rom_distributor_2.Initialize(); // initialize connection to synchrono rank 2

  while (true) {

    if (step_number == 0) {
      rom_distributor_0.Read();
      std::vector<float> recv_data_0;
      recv_data_0 = rom_distributor_0.GetRecvData();

      rom_distributor_1.Read();
      std::vector<float> recv_data_1;
      recv_data_1 = rom_distributor_1.GetRecvData();

      rom_distributor_2.Read();
      std::vector<float> recv_data_2;
      recv_data_2 = rom_distributor_2.GetRecvData();
    }

    time = my_system.GetChTime();

    if (step_number == 1) {
      realtime_timer.Reset();
    } else if (step_number != 1 && step_number != 0) {
      realtime_timer.Spin(time);
    }

    // Advance simulation for one timestep for all modules

    std::vector<float> data_to_send;
    if (step_number % 20 == 0) {
      // send data to chrono
      for (int i = 0; i < rom_data.size(); i++) {
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
      rom_distributor_0.Write(
          data_to_send); // send rom data to synchrono rank 1
      rom_distributor_1.Write(
          data_to_send); // send rom data to synchrono rank 1
      rom_distributor_2.Write(
          data_to_send); // send rom data to synchrono rank 1

      // receive data from chrono_0
      rom_distributor_0.Read();
      // receive data from chrono_1
      rom_distributor_1.Read();
      // receive data from chrono_2
      rom_distributor_2.Read();
      std::vector<float> recv_data_0;
      recv_data_0 = rom_distributor_0.GetRecvData();
      std::vector<float> recv_data_1;
      recv_data_1 = rom_distributor_1.GetRecvData();
      std::vector<float> recv_data_2;
      recv_data_2 = rom_distributor_2.GetRecvData();
    }

    for (int i = 0; i < rom_data.size(); i++) {
      // Driver inputs
      DriverInputs driver_inputs;
      idm_vec[i]->Synchronize(
          time, step_size,
          (rom_vec[rom_data[i].ld_id]->GetPos() - rom_vec[i]->GetPos())
              .Length(),
          (rom_vec[rom_data[i].ld_id]->GetVel()).Length());
      // driver_vec[i]->Advance(step_size);
      driver_inputs = driver_vec[i]->GetDriverInput();
      if (abs(driver_inputs.m_steering) > 0.05 &&
          rom_vec[i]->GetVel().Length() > 4.0) {
        driver_inputs.m_throttle = 0.0;
        driver_inputs.m_braking = 0.2;
      }

      if (output == 1) {
        if (i == focus_idx[0] || i == focus_idx[1] || i == focus_idx[2] ||
            i == focus_idx[3] || i == focus_idx[4] || i == focus_idx[5] ||
            i == focus_idx[6] || i == focus_idx[7] || i == focus_idx[8] ||
            i == focus_idx[9]) {
          if (step_number % 20 == 0) {
            input_record.push_back(driver_inputs);
          }
        }
      }

      rom_vec[i]->Advance(time, driver_inputs);
    }

    terrain.Advance(step_size);
    my_system.DoStepDynamics(step_size);

    std::cout << "time:" << time << std::endl;

    if (output == 1) {
      if (step_number % 20 == 0) {
        output_buffer << time << ",";

        for (int j = 0; j < 10; j++) {
          output_buffer << (rom_vec[focus_idx[j]]->GetVel()).Length() << ",";

          output_buffer << input_record[j].m_throttle << ",";
          output_buffer << input_record[j].m_braking << ",";
          output_buffer << input_record[j].m_steering << ",";

          ChQuaternion<> temp_qua = rom_vec[focus_idx[j]]->GetRot();
          ChVector<> temp_vec = temp_qua.Q_to_Euler123();
          output_buffer << temp_vec.x() << ",";
          output_buffer << rom_vec[focus_idx[j]]->GetGear() << ",";
          output_buffer << rom_vec[focus_idx[j]]->GetMotorSpeed() << ",";
        }
        output_buffer << std::endl;

        std::cout << "record size: " << input_record.size() << std::endl;
        input_record.clear();
      }

      if (step_number % 10000 == 0) {
        std::cout << "Writing to output file..." << std::endl;
        output_filestream << output_buffer.rdbuf();
        output_buffer.str("");
      }
    }

    // Increment frame number
    step_number++;
  }
}

void ReadRomInitFile(std::string csv_filename, std::vector<rom_item> &rom_vec) {
  std::ifstream inputFile(csv_filename);

  if (!inputFile.is_open()) {
    std::cout << "Failed to open the file." << std::endl;
  }

  std::vector<std::vector<float>> data; // 2D vector to store the float numbers

  std::string line;

  int line_count = 0;

  while (std::getline(inputFile, line)) {
    if (line_count != 0) {
      std::istringstream iss(line);
      std::string token;
      std::vector<float> row; // Vector to store a row of float numbers
      while (std::getline(iss, token, ',')) { // Assuming comma as the delimiter
        float number;
        try {
          number = std::stof(token); // Convert string to float
          row.push_back(number);     // Store the float number in the row vector
        } catch (const std::exception &e) {
          // Failed to convert to float, ignore and continue
        }
      }
      data.push_back(row); // Add the row vector to the 2D vector
    }

    line_count++;
  }

  inputFile.close(); // Close the input file

  for (int i = 0; i < data.size(); i++) {
    rom_item temp_struct;
    temp_struct.type = int(data[i][1]);
    temp_struct.pos =
        ChVector<>(float(data[i][2]), float(data[i][3]), float(data[i][4]));

    temp_struct.rot =
        ChVector<>(float(data[i][5]), float(data[i][6]), float(data[i][7]));

    temp_struct.path = int(data[i][8]);
    temp_struct.ld_id = int(data[i][9]);
    temp_struct.idm_type = int(data[i][10]);
    rom_vec.push_back(temp_struct);
  }

  // Print the float numbers in the 2D vector
  std::cout << "test struct print" << std::endl;
  for (auto temp_data : rom_vec) {
    std::cout << temp_data.type << ", " << temp_data.pos << ", "
              << temp_data.rot << ", " << temp_data.path << std::endl;
  }
}