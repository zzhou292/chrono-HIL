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

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <irrlicht.h>
#include <limits>
#include <time.h>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
//#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/driver/ChCSLDriver.h"
#include "chrono_hil/driver/ChNSF_Drivers.h"
#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_hil/ROM/driver/ChROM_IDMFollower.h"
#include "chrono_hil/ROM/driver/ChROM_PathFollowerDriver.h"
#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::synchrono;
using namespace chrono::hil;

std::string json_parameters;

float MPH2MS = 0.44704;

struct IG_vehicle {
  std::string vehicle_file;
  std::string powertrain_file;
  std::string tire_file;
  ChVector<> initial_pos;
  ChQuaternion<> initial_rot;
  std::string joystick;
};

struct Terrain {
  RigidTerrain::PatchType terrain_model;
  std::string terrain_mesh;
  ChVector<> pos;
  float length;
  float width;
};

struct Asset {
  std::string name;
  ChVector<> pos;
  ChVector<> rot;
  float scale;
  std::string mesh_file;
  bool mutable_ind; // 0 for mutable false. 1 for mutable true
};

struct LdVehicle {
  std::string vehicle_file;
  ChVector<> pos;
  ChVector<> rot;
  float speed;
  std::string path_file;
};

struct Camera {
  ChVector<> pos;
  ChVector<> rot;
  bool attach_pos;
  int resolution_x;
  int resolution_y;
  int supersampling;
  int fps;
  bool fullscreen;
  std::string name;
};

// variable initial declaration
IG_vehicle my_ig_vehicle;
Terrain my_terrain;
std::vector<Asset> my_assets;
std::vector<Camera> my_cameras;
std::vector<LdVehicle> my_lds;

ChContactMethod contact_method = ChContactMethod::SMC;
float step_size = 1e-3;

void PrintErrorMsg(int num);

void ReadParameterFiles() {
  rapidjson::Document d;
  std::cout << "json: " << json_parameters << std::endl;
  vehicle::ReadFileJSON(json_parameters, d);

  // ===================================
  // Simulation parameters specification
  // ===================================
  if (d.HasMember("Simulation")) {
    if (d["Simulation"].HasMember("step_size")) {
      step_size = d["Simulation"]["step_size"].GetFloat();
    }
  }

  // ===================================
  // Camera specification
  // ===================================
  int camera_index = 0;
  while (d.HasMember(
      (std::string("Camera") + std::to_string(camera_index)).c_str())) {
    Camera temp_camera;
    std::string cameraname =
        std::string("Camera") + std::to_string(camera_index);
    if (d[cameraname.c_str()].HasMember("pos")) {
      auto marr = d[cameraname.c_str()]["pos"].GetArray();
      temp_camera.pos = ChVector<>(marr[0].GetFloat(), marr[1].GetFloat(),
                                   marr[2].GetFloat());
    } else {
      temp_camera.pos = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d[cameraname.c_str()].HasMember("rot")) {
      auto marr = d[cameraname.c_str()]["rot"].GetArray();
      temp_camera.rot = ChVector<>(marr[0].GetFloat(), marr[1].GetFloat(),
                                   marr[2].GetFloat());
    } else {
      temp_camera.rot = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d[cameraname.c_str()].HasMember("resolution_x")) {
      temp_camera.resolution_x = d[cameraname.c_str()]["resolution_x"].GetInt();
    } else {
      temp_camera.resolution_x = 50;
    }

    if (d[cameraname.c_str()].HasMember("resolution_y")) {
      temp_camera.resolution_y = d[cameraname.c_str()]["resolution_y"].GetInt();
    } else {
      temp_camera.resolution_y = 50;
    }

    if (d[cameraname.c_str()].HasMember("supersampling")) {
      temp_camera.supersampling =
          d[cameraname.c_str()]["supersampling"].GetInt();
    } else {
      temp_camera.supersampling = 1;
    }

    if (d[cameraname.c_str()].HasMember("fps")) {
      temp_camera.fps = d[cameraname.c_str()]["fps"].GetInt();
    } else {
      temp_camera.fps = 50;
    }

    if (d[cameraname.c_str()].HasMember("attach_pos")) {
      temp_camera.attach_pos = d[cameraname.c_str()]["attach_pos"].GetBool();
    } else {
      temp_camera.attach_pos = false;
    }

    if (d[cameraname.c_str()].HasMember("name")) {
      temp_camera.name = d[cameraname.c_str()]["name"].GetString();
    } else {
      temp_camera.name = "Default Cam View";
    }

    if (d[cameraname.c_str()].HasMember("fullscreen")) {
      temp_camera.fullscreen = d[cameraname.c_str()]["fullscreen"].GetBool();
    } else {
      temp_camera.fullscreen = false;
    }

    my_cameras.push_back(temp_camera);
    camera_index++;
  }

  // ===================================
  // IG_vehicle specification
  // ===================================
  if (d.HasMember("IG_Vehicle")) {
    if (d["IG_Vehicle"].HasMember("vehicle_file")) {
      my_ig_vehicle.vehicle_file = d["IG_Vehicle"]["vehicle_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("powertrain_file")) {
      my_ig_vehicle.powertrain_file =
          d["IG_Vehicle"]["powertrain_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("tire_file")) {
      my_ig_vehicle.tire_file = d["IG_Vehicle"]["tire_file"].GetString();
    } else {
      PrintErrorMsg(1);
    }

    if (d["IG_Vehicle"].HasMember("initial_pos")) {
      auto marr = d["IG_Vehicle"]["initial_pos"].GetArray();
      my_ig_vehicle.initial_pos = ChVector<>(
          marr[0].GetDouble(), marr[1].GetDouble(), marr[2].GetDouble());
    } else {
      my_ig_vehicle.initial_pos = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d["IG_Vehicle"].HasMember("initial_rot")) {
      auto marr = d["IG_Vehicle"]["initial_rot"].GetArray();
      my_ig_vehicle.initial_rot =
          ChQuaternion<>(marr[0].GetDouble(), marr[1].GetDouble(),
                         marr[2].GetDouble(), marr[3].GetDouble());
    } else {
      my_ig_vehicle.initial_rot = ChQuaternion<>(1.0, 0.0, 0.0, 0.0);
    }

    if (d["IG_Vehicle"].HasMember("joystick")) {
      my_ig_vehicle.joystick = d["IG_Vehicle"]["joystick"].GetString();
    }
  } else {
    PrintErrorMsg(0);
  }

  // ===================================
  // Terrain specification
  // ====================================
  if (d.HasMember("Terrain")) {
    if (d["Terrain"].HasMember("type")) {
      std::string type_string = d["Terrain"]["type"].GetString();
      if (type_string == "BOX") {
        my_terrain.terrain_model = RigidTerrain::PatchType::BOX;
      } else if (type_string == "MESH") {
        my_terrain.terrain_model = RigidTerrain::PatchType::MESH;
      }
    } else {
      PrintErrorMsg(3);
    }

    if (d["Terrain"].HasMember("terrain_mesh")) {
      my_terrain.terrain_mesh = d["Terrain"]["terrain_mesh"].GetString();
    }

    if (d["Terrain"].HasMember("pos")) {
      auto marr = d["Terrain"]["pos"].GetArray();
      my_terrain.pos = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                                  marr[2].GetDouble());
    } else {
      my_terrain.pos = ChVector<>(0.0, 0.0, 0.0);
    }

    if (d["Terrain"].HasMember("length")) {
      my_terrain.length = d["Terrain"]["length"].GetDouble();
    } else {
      my_terrain.length = 300.0;
    }

    if (d["Terrain"].HasMember("width")) {
      my_terrain.width = d["Terrain"]["width"].GetDouble();
    } else {
      my_terrain.width = 300.0;
    }

  } else {
    PrintErrorMsg(2);
  }

  // ===================================
  // Assets specification
  // ===================================
  int asset_index = 0;
  while (d.HasMember(
      (std::string("Asset") + std::to_string(asset_index)).c_str())) {
    std::string assetname = std::string("Asset") + std::to_string(asset_index);
    Asset temp_asset;
    if (d[assetname.c_str()].HasMember("name")) {
      temp_asset.name = d[assetname.c_str()]["name"].GetString();
    }

    if (d[assetname.c_str()].HasMember("pos")) {
      auto marr = d[assetname.c_str()]["pos"].GetArray();
      temp_asset.pos = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                                  marr[2].GetDouble());
    }

    if (d[assetname.c_str()].HasMember("rot")) {
      auto marr = d[assetname.c_str()]["rot"].GetArray();
      temp_asset.rot = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                                  marr[2].GetDouble());
    }

    if (d[assetname.c_str()].HasMember("mesh")) {
      temp_asset.mesh_file = d[assetname.c_str()]["mesh"].GetString();
    }

    if (d[assetname.c_str()].HasMember("mutable")) {
      temp_asset.mutable_ind = d[assetname.c_str()]["mutable"].GetBool();
    }

    if (d[assetname.c_str()].HasMember("scale")) {
      temp_asset.scale = d[assetname.c_str()]["scale"].GetFloat();
    }

    my_assets.push_back(temp_asset);
    asset_index++;
  }

  // ===================================
  // Lead Vehicle specification
  // ====================================
  int ld_index = 0;
  while (d.HasMember(
      (std::string("Ld_Vehicle") + std::to_string(ld_index)).c_str())) {
    LdVehicle temp_ld;
    std::string ldname = std::string("Ld_Vehicle") + std::to_string(ld_index);

    if (d[ldname.c_str()].HasMember("vehicle_type")) {
      std::string type = d[ldname.c_str()]["vehicle_type"].GetString();
      if (type == "HMMWV") {
        temp_ld.vehicle_file =
            std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
      } else if (type == "Sedan") {
        temp_ld.vehicle_file =
            std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";
      } else if (type == "Audi") {
        temp_ld.vehicle_file =
            std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";
      } else if (type == "Patrol") {
        temp_ld.vehicle_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                               "/rom/patrol/patrol_rom.json";
      }
    }

    if (d[ldname.c_str()].HasMember("pos")) {
      auto marr = d[ldname.c_str()]["pos"].GetArray();
      temp_ld.pos = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                               marr[2].GetDouble());
    }

    if (d[ldname.c_str()].HasMember("rot")) {
      auto marr = d[ldname.c_str()]["rot"].GetArray();
      temp_ld.rot = ChVector<>(marr[0].GetDouble(), marr[1].GetDouble(),
                               marr[2].GetDouble());
    }

    if (d[ldname.c_str()].HasMember("path")) {
      temp_ld.path_file = d[ldname.c_str()]["path"].GetString();
    }

    if (d[ldname.c_str()].HasMember("speed")) {
      temp_ld.speed = d[ldname.c_str()]["speed"].GetInt();
    }

    my_lds.push_back(temp_ld);
    ld_index++;
  }
}

void AddCommandLineOptions(ChCLI &cli) {
  cli.AddOption<std::string>(
      "Simulation", "json_parameters",
      "point to the simulation parameter file which defines environment",
      json_parameters);
}

int main(int argc, char *argv[]) {
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  if (!cli.Parse(argc, argv, true))
    return 0;

  json_parameters = std::string(STRINGIFY(HIL_DATA_DIR)) +
                    cli.GetAsType<std::string>("json_parameters");

  ReadParameterFiles();

  // setup IG_vehicle
  WheeledVehicle vehicle(std::string(STRINGIFY(HIL_DATA_DIR)) +
                             my_ig_vehicle.vehicle_file,
                         ChContactMethod::SMC);
  auto ego_chassis = vehicle.GetChassis();

  vehicle.Initialize(
      ChCoordsys<>(my_ig_vehicle.initial_pos, my_ig_vehicle.initial_rot));
  vehicle.GetChassis()->SetFixed(false);
  auto powertrain = ReadPowertrainJSON(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                       my_ig_vehicle.powertrain_file);
  vehicle.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (auto &axle : vehicle.GetAxles()) {
    for (auto &wheel : axle->GetWheels()) {
      auto tire = ReadTireJSON(std::string(STRINGIFY(HIL_DATA_DIR)) +
                               my_ig_vehicle.tire_file);
      tire->SetStepsize(step_size);
      vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  vehicle.SetChassisVisualizationType(VisualizationType::MESH);
  vehicle.SetSuspensionVisualizationType(VisualizationType::MESH);
  vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
  vehicle.SetWheelVisualizationType(VisualizationType::MESH);

  std::cout << "Finished IG_Vehicle Setup" << std::endl;
  // setup terrain

  auto terrain = std::make_shared<RigidTerrain>(vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (my_terrain.terrain_model) {
  case RigidTerrain::PatchType::BOX:
    patch = terrain->AddPatch(patch_mat, CSYSNORM, my_terrain.length,
                              my_terrain.width);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  case RigidTerrain::PatchType::MESH:
    patch = terrain->AddPatch(patch_mat, CSYSNORM, my_terrain.terrain_mesh);
    // add vis mesh
    auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    terrain_mesh->LoadWavefrontMesh(my_terrain.terrain_mesh, true, true);
    terrain_mesh->Transform(ChVector<>(0, 0, 0),
                            ChMatrix33<>(1)); // scale to a different size
    auto terrain_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    terrain_shape->SetMesh(terrain_mesh);
    terrain_shape->SetName("terrain");
    terrain_shape->SetMutable(false);

    auto terrain_body = chrono_types::make_shared<ChBody>();
    terrain_body->SetPos({0, 0, -.01});
    terrain_body->AddVisualShape(terrain_shape);
    terrain_body->SetBodyFixed(true);
    terrain_body->SetCollide(false);
    vehicle.GetSystem()->Add(terrain_body);

    break;
  }
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain->Initialize();

  std::cout << "Finished Terrain Setup" << std::endl;

  // setup ld_vehicles

  std::vector<std::shared_ptr<Ch_8DOF_vehicle>> rom_vec;
  std::vector<std::shared_ptr<ChROM_PathFollowerDriver>> driver_vec;
  for (int i = 0; i < my_lds.size(); i++) {

    std::shared_ptr<Ch_8DOF_vehicle> rom_veh =
        chrono_types::make_shared<Ch_8DOF_vehicle>(
            my_lds[i].vehicle_file, my_lds[i].pos.z(), step_size);
    rom_veh->SetInitPos(my_lds[i].pos);
    rom_veh->SetInitRot(my_lds[i].rot.z());
    rom_veh->Initialize(vehicle.GetSystem());

    std::shared_ptr<ChBezierCurve> path =
        ChBezierCurve::read(my_lds[i].path_file, true);

    std::shared_ptr<ChROM_PathFollowerDriver> driver =
        chrono_types::make_shared<ChROM_PathFollowerDriver>(
            rom_veh, path, my_lds[i].speed * MPH2MS, 6.0, 0.06, 0.0, 0.0, 0.4,
            0.0, 0.0);
    rom_vec.push_back(rom_veh);
    driver_vec.push_back(driver);
  }

  std::cout << "Finished Ld Vehicle Setup" << std::endl;

  // setup assets
  for (int i = 0; i < my_assets.size(); i++) {
    auto body_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    body_mesh->LoadWavefrontMesh(my_assets[i].mesh_file, false, true);
    body_mesh->Transform(
        ChVector<>(0, 0, 0),
        ChMatrix33<>(my_assets[i].scale)); // scale to a different size
    auto body_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    body_shape->SetMesh(body_mesh);
    body_shape->SetName(my_assets[i].name);
    body_shape->SetMutable(my_assets[i].mutable_ind);

    auto body_body = chrono_types::make_shared<ChBody>();
    body_body->SetPos(my_assets[i].pos);
    body_body->SetRot(Q_from_Euler123(my_assets[i].rot));
    body_body->AddVisualShape(body_shape);
    body_body->SetBodyFixed(true);
    body_body->SetCollide(false);

    vehicle.GetSystem()->AddBody(body_body);
  }
  std::cout << "Finished Asset Setup" << std::endl;

  // ========================================

  auto manager =
      chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
  float intensity = 2.0;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  // -------------------------------------------------------
  // Create a camera and add it to the sensor manager
  // -------------------------------------------------------
  for (int i = 0; i < my_cameras.size(); i++) {
    if (my_cameras[i].attach_pos == true) {
      auto cam = chrono_types::make_shared<ChCameraSensor>(
          vehicle.GetChassisBody(), // body camera is attached to
          my_cameras[i].fps,        // update rate in Hz
          chrono::ChFrame<double>(
              my_cameras[i].pos,
              Q_from_Euler123(my_cameras[i].rot)), // offset pose
          my_cameras[i].resolution_x,              // image width
          my_cameras[i].resolution_y,              // image height
          1.608f,
          my_cameras[i].supersampling); // fov, lag, exposure
      cam->SetName("Camera Sensor");

      cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
          my_cameras[i].resolution_x, my_cameras[i].resolution_y,
          my_cameras[i].name, my_cameras[i].fullscreen));
      // Provide the host access to the RGBA8 buffer
      cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
      manager->AddSensor(cam);
    } else {
      auto temp_body = chrono_types::make_shared<ChBody>();
      temp_body->SetPos(my_cameras[i].pos);
      temp_body->SetBodyFixed(true);
      vehicle.GetSystem()->AddBody(temp_body);
      auto cam = chrono_types::make_shared<ChCameraSensor>(
          temp_body,         // body camera is attached to
          my_cameras[i].fps, // update rate in Hz
          chrono::ChFrame<double>(
              my_cameras[i].pos,
              Q_from_Euler123(my_cameras[i].rot)), // offset pose
          my_cameras[i].resolution_x,              // image width
          my_cameras[i].resolution_y,              // image height
          1.608f,
          my_cameras[i].supersampling); // fov, lag, exposure
      cam->SetName("Camera Sensor");
      cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
          my_cameras[i].resolution_x, my_cameras[i].resolution_y,
          my_cameras[i].name, my_cameras[i].fullscreen));
      cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
      manager->AddSensor(cam);
    }
  }

  manager->Update();

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                  my_ig_vehicle.joystick);

  // Simulation Loop
  int step_number = 0;
  while (true) {
    double time = vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_steering = -SDLDriver.GetSteering();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    // Update modules (process inputs from other modules)
    int status = SDLDriver.Synchronize();
    if (status == 1) {
      return -1;
    }

    // Update leads
    for (int i = 0; i < rom_vec.size(); i++) {
      driver_vec[i]->Advance(step_size);
      DriverInputs ld_di = driver_vec[i]->GetDriverInput();
      rom_vec[i]->Advance(time, ld_di);
    }

    terrain->Synchronize(time);
    vehicle.Synchronize(time, driver_inputs, *terrain);

    // Advance simulation for one timestep for all modules
    // driver.Advance(step_size);
    terrain->Advance(step_size);
    vehicle.Advance(step_size);

    manager->Update();

    // Increment frame number
    step_number++;
  }
}

void PrintErrorMsg(int num) {
  if (num == 0) {
    std::cout << std::endl << "ERROR: missing IG_vehicle field" << std::endl;
  } else if (num == 1) {
    std::cout << std::endl
              << "ERROR: missing IG_vehicle type specification" << std::endl;
  } else if (num == 2) {
    std::cout << std::endl
              << "ERROR: missing terrain specification" << std::endl;
  } else if (num == 3) {
    std::cout << std::endl << "ERROR: missing terrain type" << std::endl;
  }
}
