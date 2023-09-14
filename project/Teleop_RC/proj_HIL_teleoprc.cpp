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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Main driver function for the RCCar model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/rccar/RCCar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::rccar;
using namespace chrono::hil;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;     // terrain height (FLAT terrain only)
double terrainLength = 100.0; // size in X direction
double terrainWidth = 100.0;  // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "RCCar";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1; // FPS = 1

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  RCCar my_rccar;
  my_rccar.SetContactMethod(contact_method);
  my_rccar.SetChassisCollisionType(chassis_collision_type);
  my_rccar.SetChassisFixed(false);
  my_rccar.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  my_rccar.SetTireType(tire_model);
  my_rccar.SetTireStepSize(tire_step_size);
  my_rccar.Initialize();

  VisualizationType tire_vis_type = VisualizationType::MESH;

  my_rccar.SetChassisVisualizationType(chassis_vis_type);
  my_rccar.SetSuspensionVisualizationType(suspension_vis_type);
  my_rccar.SetSteeringVisualizationType(steering_vis_type);
  my_rccar.SetWheelVisualizationType(wheel_vis_type);
  my_rccar.SetTireVisualizationType(tire_vis_type);

  // Create the terrain
  RigidTerrain terrain(my_rccar.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model) {
  case RigidTerrain::PatchType::BOX:
    patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  case RigidTerrain::PatchType::HEIGHT_MAP:
    patch = terrain.AddPatch(
        patch_mat, CSYSNORM,
        vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128, 128, 0, 4);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16,
                      16);
    break;
  case RigidTerrain::PatchType::MESH:
    patch = terrain.AddPatch(patch_mat, CSYSNORM,
                             vehicle::GetDataFile("terrain/meshes/test.obj"));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100,
                      100);
    break;
  }
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  // Create the vehicle Irrlicht interface
  auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("RCCar Demo");
  vis->SetChaseCamera(trackPoint, 1.5, 0.05);
  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(&my_rccar.GetVehicle());

  // ---------------------------------------------
  // Create a sensor manager and add a point light
  // ---------------------------------------------
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_rccar.GetSystem());
  float intensity = 2.0;
  manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  // ------------------------------------------------
  // Create a camera and add it to the sensor manager
  // ------------------------------------------------

  auto cam = chrono_types::make_shared<ChCameraSensor>(
      my_rccar.GetVehicle().GetChassisBody(), // body camera is attached to
      25,                                     // update rate in Hz
      chrono::ChFrame<double>({-0.05, 0, 0.07},
                              Q_from_AngAxis(0, {0, 1, 0})), // offset pose
      1280,                                                  // image width
      720,                                                   // image height
      CH_C_PI_4,
      1); // fov, lag, exposure
  cam->SetName("Camera Sensor");
  cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      1280, 720, "Driver View", false));
  cam->SetLag(0.05f);
  manager->AddSensor(cam);

  // -----------------
  // Initialize output
  // -----------------

  if (!filesystem::create_directory(filesystem::path(out_dir))) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if (povray_output) {
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
      std::cout << "Error creating directory " << pov_dir << std::endl;
      return 1;
    }
    terrain.ExportMeshPovray(out_dir);
  }

  // ------------------------
  // Create the driver system
  // ------------------------
  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);

  // ---------------
  // Simulation loop
  // ---------------
  // output vehicle mass
  std::cout << "VEHICLE MASS: " << my_rccar.GetVehicle().GetMass() << std::endl;

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);
  int debug_steps = (int)std::ceil(debug_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;

  if (contact_vis) {
    vis->SetSymbolScale(1e-4);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
  }

  my_rccar.GetVehicle().EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;

  while (true) {
    double time = my_rccar.GetSystem()->GetChTime();

    std::cout << cam->GetLag() << std::endl;
    if (step_number == 5000) {
      cam->SetLag(0.08f);
    }

    manager->Update();

    if (step_number == 0) {
      realtime_timer.Reset();
    }

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (step_number % render_steps == 0) {
      vis->BeginScene();
      vis->Render();
      vis->EndScene();

      if (povray_output) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(),
                render_frame + 1);
        utils::WriteVisualizationAssets(my_rccar.GetSystem(), filename);
      }

      render_frame++;
    }

    // Debug logging
    if (debug_output && step_number % debug_steps == 0) {
      GetLog() << "\n\n============ System Information ============\n";
      GetLog() << "Time = " << time << "\n\n";
      my_rccar.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
    }

    // get the controls for this time step
    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    // Update modules (process inputs from other modules)
    terrain.Synchronize(time);
    my_rccar.Synchronize(time, driver_inputs, terrain);
    vis->Synchronize(time, driver_inputs);

    // Advance simulation for one timestep for all modules
    terrain.Advance(step_size);
    my_rccar.Advance(step_size);
    vis->Advance(step_size);

    // Increment frame number
    step_number++;

    realtime_timer.Spin(time);

    if (SDLDriver.Synchronize() == 1) {
      break;
    }
  }

  return 0;
}
