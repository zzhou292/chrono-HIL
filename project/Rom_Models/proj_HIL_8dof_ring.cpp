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

#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <irrlicht.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/core/ChBezierCurve.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::vehicle;
using namespace chrono::sensor;

enum VEH_TYPE { HMMWV, PATROL, AUDI, SEDAN };

enum IDM_TYPE {
  AGG,
  NORMAL,
  CONS
}; // aggressive, normal, or conservative driver

std::vector<VEH_TYPE> vehicle_types;
std::vector<IDM_TYPE> idm_types;

std::random_device rd{};
std::mt19937 gen{rd()};

void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<std::string>("simulation", "conf", "configuration file name",
                             "test.csv");
  cli.AddOption<std::string>("simulation", "record_name",
                             "record output file name", "record");
}

void ReadConfFile(std::string csv_filename,
                  std::vector<VEH_TYPE> &vehicle_types,
                  std::vector<IDM_TYPE> &idm_types);

int main(int argc, char *argv[]) {
  // read from cli
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  std::string conf_name = cli.GetAsType<std::string>("conf");
  std::string output_name = cli.GetAsType<std::string>("record_name");

  std::string record_file_path = "./" + output_name + ".csv";
  std::ofstream record_filestream = std::ofstream(record_file_path);
  std::stringstream record_buffer;

  std::cout << "conf_name:" << conf_name << std::endl;

  ReadConfFile(conf_name, vehicle_types, idm_types);

  // Create a physical system
  ChSystemSMC sys;
  std::vector<std::shared_ptr<Ch_8DOF_vehicle>> rom_vec; // rom vehicle vector
  std::vector<std::shared_ptr<ChROM_PathFollowerDriver>>
      driver_vec; // rom driver vector
  std::vector<std::shared_ptr<ChROM_IDMFollower>> idm_vec;

  // input sec
  int num_rom = vehicle_types.size();
  float ring_radius = 600.0;
  std::string path_file =
      STRINGIFY(HIL_DATA_DIR) + std::string("/ring/terrain0103/600_ring.txt");

  // now lets run our simulation
  float time = 0;
  int step_number = 0; // time step counter
  float step_size = 1e-3;

  // Create the terrain
  RigidTerrain terrain(&sys);

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, 5000, 5000);
  patch->SetColor(ChColor(0.5f, 0.5f, 0.5f));

  terrain.Initialize();

  /*
    // add terrain with weighted textures
    auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    terrain_mesh->LoadWavefrontMesh(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                        "/ring/terrain0103/ring_terrain_50.obj",
                                    false, true);
    terrain_mesh->Transform(ChVector<>(0, 0, 0),
                            ChMatrix33<>(1)); // scale to a different size
    auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    terrain_shape->SetMesh(terrain_mesh);
    terrain_shape->SetName("terrain");
    terrain_shape->SetMutable(false);

    auto terrain_body = chrono_types::make_shared<ChBody>();
    terrain_body->SetPos({0, 0, 0.0});
    terrain_body->AddVisualShape(terrain_shape);
    terrain_body->SetBodyFixed(true);
    terrain_body->SetCollide(false);

    sys.AddBody(terrain_body);
  */
  std::string hmmwv_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
  std::string patrol_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";
  std::string audi_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";
  std::string sedan_rom_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";

  // initialize all idm roms
  for (int i = 0; i < num_rom; i++) {
    std::cout << "loading: " << i << std::endl;
    std::string rom_json;
    float init_height;

    if (vehicle_types[i] == HMMWV) {
      rom_json = hmmwv_rom_json;
      init_height = 0.45;
    } else if (vehicle_types[i] == PATROL) {
      rom_json = patrol_rom_json;
      init_height = 0.45;
    } else if (vehicle_types[i] == AUDI) {
      rom_json = audi_rom_json;
      init_height = 0.20;
    } else if (vehicle_types[i] == SEDAN) {
      rom_json = sedan_rom_json;
      init_height = 0.20;
    }

    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(rom_json, init_height,
                                                   step_size, true);

    // determine initial position and initial orientation
    float deg_sec = (CH_C_PI * 1.8) / (num_rom);
    ChVector<> initLoc = ChVector<>(ring_radius * cos(deg_sec * i),
                                    ring_radius * sin(deg_sec * i), 0.5);
    float rot_deg = deg_sec * i + CH_C_PI_2;
    if (rot_deg > CH_C_2PI) {
      rot_deg = rot_deg - CH_C_2PI;
    }
    rom_veh->SetInitPos(initLoc);
    rom_veh->SetInitRot(rot_deg);
    rom_veh->Initialize(&sys);
    rom_vec.push_back(rom_veh);

    // initialize driver
    std::shared_ptr<ChBezierCurve> path = ChBezierCurve::read(path_file, true);

    std::shared_ptr<ChROM_PathFollowerDriver> driver =
        chrono_types::make_shared<ChROM_PathFollowerDriver>(
            rom_vec[i], path, 2.0, 6.0, 0.4, 0.0, 0.0, 0.4, 0.0, 0.0);
    driver_vec.push_back(driver);

    // initialize idm control
    std::vector<double> params;
    if (idm_types[i] == AGG) {
      params.push_back(13.4112);
      params.push_back(0.1);
      params.push_back(5.0);
      params.push_back(3.5);
      params.push_back(2.5);
      params.push_back(4.0);
      params.push_back(6.5);
    } else if (idm_types[i] == CONS) {
      params.push_back(8.9408);
      params.push_back(0.7);
      params.push_back(8.0);
      params.push_back(2.5);
      params.push_back(1.5);
      params.push_back(4.0);
      params.push_back(6.5);
    } else if (idm_types[i] == NORMAL) {
      params.push_back(11.176);
      params.push_back(0.2);
      params.push_back(6.0);
      params.push_back(3.0);
      params.push_back(2.1);
      params.push_back(4.0);
      params.push_back(6.5);
    }

    std::shared_ptr<ChROM_IDMFollower> idm_controller =
        chrono_types::make_shared<ChROM_IDMFollower>(rom_vec[i], driver_vec[i],
                                                     params);
    // if (i != num_rom - 1) {
    //  idm_controller->SetSto(true, 0.1, 0.8, 0.2, 0.2);
    //}

    idm_vec.push_back(idm_controller);
  }

  auto attached_body = std::make_shared<ChBody>();
  sys.AddBody(attached_body);
  attached_body->SetPos(ChVector<>(0.0, 0.0, 0.0));
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Create the camera sensor
  auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
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
          ChVector<>(0.0, 0.0, 1100.0),
          Q_from_Euler123(ChVector<>(0.0, C_PI / 2, 0.0))), // offset pose
      1280,                                                 // image width
      720,                                                  // image
      1.608f, 1); // fov, lag, exposure cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);

  // manager->Update();

  std::vector<ChVector<>> prev_pos_vec;
  std::vector<float> rom_dis_vec;

  for (int i = 0; i < num_rom; i++) {
    prev_pos_vec.push_back(ChVector<>(0.0, 0.0, 0.0));
    rom_dis_vec.push_back(0.0);
  }

  while (time < 900.0) {

    // get the controls for this time step
    // Driver inputs

    for (int i = 0; i < num_rom; i++) {
      // update idm
      int ld_idx = (i + 1) % num_rom;

      // Compute critical information
      float raw_dis =
          (rom_vec[ld_idx]->GetPos() - rom_vec[i]->GetPos()).Length();
      float temp = 1 - (raw_dis * raw_dis) / (2.0 * ring_radius * ring_radius);
      if (temp > 1) {
        temp = 1;
      } else if (temp < -1) {
        temp = -1;
      }

      float theta = abs(acos(temp));
      float act_dis = theta * ring_radius;

      idm_vec[i]->Synchronize(time, step_size, act_dis,
                              (rom_vec[ld_idx]->GetVel()).Length());

      DriverInputs driver_inputs;
      driver_inputs = driver_vec[i]->GetDriverInput();

      rom_vec[i]->Advance(time, driver_inputs);

      if (step_number == 0) {
        prev_pos_vec[i] = rom_vec[i]->GetPos();
      } else {
        rom_dis_vec[i] =
            rom_dis_vec[i] + (rom_vec[i]->GetPos() - prev_pos_vec[i]).Length();
        prev_pos_vec[i] = rom_vec[i]->GetPos();
      }
    }

    if (step_number % 10 == 0) {
      for (int k = 0; k < num_rom; k++) {
        record_buffer << std::to_string(rom_dis_vec[k]) + ",";
        record_buffer << std::to_string((rom_vec[k]->GetVel()).Length()) + ",";
        record_buffer << std::to_string(rom_vec[k]->GetMotorSpeed()) + ",";
        record_buffer << std::to_string(rom_vec[k]->GetGear()) + ",";
      }
      record_buffer << std::endl;
      if (step_number % 5000 == 0) {
        std::cout << "Writing to record file..." << std::endl;
        record_filestream << record_buffer.rdbuf();
        record_buffer.str("");
        std::cout << "time: " << time << std::endl;
      }
    }

    time += step_size;
    step_number += 1;

    sys.DoStepDynamics(step_size);
    // manager->Update();
  }
  return 0;
}

void ReadConfFile(std::string csv_filename,
                  std::vector<VEH_TYPE> &vehicle_types,
                  std::vector<IDM_TYPE> &idm_types) {
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
    VEH_TYPE temp_veh;
    IDM_TYPE temp_idm;
    if (data[i][0] == 0) {
      temp_veh = HMMWV;
    } else if (data[i][0] == 1) {
      temp_veh = PATROL;
    } else if (data[i][0] == 2) {
      temp_veh = AUDI;
    } else if (data[i][0] == 3) {
      temp_veh = SEDAN;
    }

    if (data[i][1] == 0) {
      temp_idm = AGG;
    } else if (data[i][1] == 1) {
      temp_idm = NORMAL;
    } else if (data[i][1] == 2) {
      temp_idm = CONS;
    }

    vehicle_types.push_back(temp_veh);
    idm_types.push_back(temp_idm);
  }
}