#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_hil/driver/ChSDLInterface.h"

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

#include "chrono_hil/ROM/veh/Ch_8DOF_vehicle.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::hil;
using namespace chrono::vehicle;
using namespace chrono::sensor;

enum VEH_TYPE { HMMWV, PATROL, AUDI, SEDAN };

int main(int argc, char *argv[]) {

  // Create a ChronoENGINE physical system
  ChSystemSMC sys;

  // Create the terrain
  RigidTerrain terrain(&sys);

  // Define ROM vehicle type
  VEH_TYPE rom_type = VEH_TYPE::AUDI;
  float init_height = 0.45;

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, 300, 300);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  // now lets run our simulation
  float time = 0;
  int step_number = 0; // time step counter
  float step_size = 1e-3;

  std::string rom_json;
  switch (rom_type) {
  case VEH_TYPE::HMMWV:
    rom_json =
        std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
    init_height = 0.45;
    break;
  case VEH_TYPE::PATROL:
    rom_json =
        std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";
    init_height = 0.45;
    break;
  case VEH_TYPE::AUDI:
    rom_json = std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";
    init_height = 0.20;
    break;
  case VEH_TYPE::SEDAN:
    rom_json =
        std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";
    init_height = 0.18;
    break;
  default:
    return -1;
  }

  Ch_8DOF_vehicle rom_veh(rom_json, init_height, step_size, true);

  rom_veh.Initialize(&sys);

  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);

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
      rom_veh.GetChassisBody(), // body camera is attached to
      35,                       // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(0.0, 6.0, 2.5),
          Q_from_Euler123(ChVector<>(0.0, 0.15, -C_PI / 2))), // offset pose
      1280,                                                   // image width
      720,                                                    // image height
      1.608f,
      1); // fov, lag, exposure
  cam->SetName("Camera Sensor");

  cam->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam);

  auto cam2 = chrono_types::make_shared<ChCameraSensor>(
      rom_veh.GetChassisBody(), // body camera is attached to
      35,                       // update rate in Hz
      chrono::ChFrame<double>(
          ChVector<>(0.0, -5.0, 0.0),
          Q_from_Euler123(ChVector<>(0.0, 0.25, C_PI / 2))), // offset pose
      1280,                                                  // image width
      720,                                                   // image height
      1.608f,
      1); // fov, lag, exposure
  cam2->SetName("Camera Sensor");

  cam2->PushFilter(
      chrono_types::make_shared<ChFilterVisualize>(1280, 720, "test", false));
  // Provide the host access to the RGBA8 buffer
  // cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
  manager->AddSensor(cam2);
  ChRealtimeCumulative realtime_timer;

  manager->Update();

  while (true) {

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    // get the controls for this time step
    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    rom_veh.Advance(time, driver_inputs);

    time += step_size;
    step_number += 1;

    sys.DoStepDynamics(step_size);
    manager->Update();

    realtime_timer.Spin(time);

    if (SDLDriver.Synchronize() == 1) {
      break;
    }
  }
  return 0;
}