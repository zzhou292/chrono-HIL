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
// Authors: Json Zhou
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_hil/network/sim/ChDelaySim.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::hil;
using namespace chrono::sensor;
using namespace chrono::synchrono;

// =============================================================================

// Initial vehicle location and orientation
ChVector3<> initLoc(1000, 0, 0.5);
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
ChVector3<> trackPoint(0.0, 0.0, 0.2);

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

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1; // FPS = 1

// =============================================================================
// path files
std::string lane_0_path = (STRINGIFY(HIL_DATA_DIR)) + std::string("/large_ring/lane_0.txt");
std::string lane_1_path = (STRINGIFY(HIL_DATA_DIR)) + std::string("/large_ring/lane_1.txt");
std::string lane_2_path = (STRINGIFY(HIL_DATA_DIR)) + std::string("/large_ring/lane_2.txt");

// =============================================================================
void addTerrain(ChSystem &sys);
// Forward declares for straight forward helper functions
ChCoordsys<> GetVehicleConfig(int node_id,
                              std::string &vehicle,
                              std::string &engine,
                              std::string &transmission,
                              std::string &tire,
                              std::string &zombie);

int main(int argc, char *argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("synchrono/"));

  auto lane_0 = ChBezierCurve::Read(lane_0_path, true);
  auto lane_1 = ChBezierCurve::Read(lane_1_path, true);
  auto lane_2 = ChBezierCurve::Read(lane_2_path, true);

  // -----------------------
  // Create SynChronoManager
  // -----------------------
  auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
  int node_id = communicator->GetRank();
  int num_nodes = communicator->GetNumRanks();
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  // Change SynChronoManager settings
  syn_manager.SetHeartbeat(0.01);

  // --------------
  // Create systems
  // --------------
  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // Get the vehicle JSON filenames and initial locations
  std::string vehicle_filename, engine_filename, transmission_filename, tire_filename, zombie_filename;
  auto initPos = GetVehicleConfig(node_id,               //
                                  vehicle_filename,      //
                                  engine_filename,       //
                                  transmission_filename, //
                                  tire_filename,         //
                                  zombie_filename);      //

  // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
  VisualizationType chassis_vis_type = VisualizationType::MESH;
  VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
  VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
  VisualizationType wheel_vis_type = VisualizationType::MESH;
  VisualizationType tire_vis_type = VisualizationType::MESH;

  WheeledVehicle vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = vehicle.GetChassis();
  vehicle.Initialize(initPos);
  vehicle.GetChassis()->SetFixed(false);
  vehicle.SetChassisVisualizationType(chassis_vis_type);
  vehicle.SetSuspensionVisualizationType(suspension_vis_type);
  vehicle.SetSteeringVisualizationType(steering_vis_type);
  vehicle.SetWheelVisualizationType(wheel_vis_type);

  auto engine = ReadEngineJSON(engine_filename);
  auto transmission = ReadTransmissionJSON(transmission_filename);
  auto powertrain =
      chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);

  vehicle.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (auto &axle : vehicle.GetAxles())
  {
    for (auto &wheel : axle->GetWheels())
    {
      auto tire = ReadTireJSON(tire_filename);
      tire->SetStepsize(step_size / 20);
      vehicle.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // Set associated collision detection system
  vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

  // Add vehicle as an agent and initialize SynChronoManager
  syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, zombie_filename));
  syn_manager.Initialize(vehicle.GetSystem());

  // Create the terrain
  RigidTerrain terrain(vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model)
  {
  case RigidTerrain::PatchType::BOX:
    patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3<>(0.0, 0.0, 0.2), ChQuaternion<>(1, 0, 0, 0)), terrainLength, terrainWidth, 1, false, 1, false);

    break;
  }
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  if (node_id == 0)
    addTerrain(*vehicle.GetSystem());

  // Create the vehicle Irrlicht interface
  auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  if (node_id == 0)
  {
    vis->SetWindowTitle("RCCar Demo");
    vis->SetChaseCamera(trackPoint, 20, 6.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);
  }

  // ---------------------------------------------
  // Create a sensor manager and add a point light
  // ---------------------------------------------
  auto manager =
      chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
  if (node_id == 0)
  {
    float intensity = 2.0;
    manager->scene->AddPointLight({0, 0, 1e8}, {intensity, intensity, intensity},
                                  1e12);
    manager->scene->SetAmbientLight({0.5, 0.5, 0.5});
    manager->scene->SetSceneEpsilon(1e-3);
    manager->scene->EnableDynamicOrigin(true);
    manager->scene->SetOriginOffsetThreshold(500.f);

    // Set environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/sunflowers_4k.hdr");
    manager->scene->SetBackground(b);

    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    // camera parameters
    float frame_rate = 25.0;
    int super_samples = 2;
    unsigned int image_width = 1280;
    unsigned int image_height = 720;
    float cam_fov = 1.608f;
    // float cam_fov = .524;

    ChVector3<> driver_eye(2.0, .0, 1.0);
    ChQuaternion<> driver_view_direction(1, 0, 0, 0);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        vehicle.GetChassisBody(),                                   // body camera is attached to
        frame_rate,                                                 // update rate in Hz
        chrono::ChFrame<double>(driver_eye, driver_view_direction), // offset pose
        image_width,                                                // image width
        image_height,                                               // image height
        cam_fov,
        super_samples); // fov, lag, exposure
    cam->SetName("Camera Sensor");
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        image_width, image_height, "Driver View", false));

    // add sensor to the manager
    manager->AddSensor(cam);
  }

  // ------------------------
  // Create the driver system
  // ------------------------
  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  if (node_id == 0)
  {
    SDLDriver.Initialize();

    std::string joystick_file =
        (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
    SDLDriver.SetJoystickConfigFile(joystick_file);
  }

  // ------------------------
  // Create the path follower system
  // ------------------------
  // Make node_id 2 slower so the passing looks nice, other parameters are normal car-following settings
  double target_speed = node_id == 2 ? 16 : 12;
  double target_following_time = 1.2;
  double target_min_distance = 10;
  double current_distance = 100;
  std::shared_ptr<ChDriver> driver;
  if (node_id != 0)
  {
    std::vector<std::shared_ptr<ChBezierCurve>> paths = {lane_0, lane_1, lane_2};
    auto acc_driver = chrono_types::make_shared<ChMultiPathFollowerACCDriver>(
        vehicle, paths, "Highway", target_speed, target_following_time, target_min_distance, current_distance);

    acc_driver->GetSpeedController().SetGains(0.6, 0.0, 0.0);
    acc_driver->GetSteeringController().SetGains(0.1, 0.01, 0.0);
    acc_driver->GetSteeringController().SetLookAheadDistance(10);

    if (node_id == 1)
    {
      acc_driver->changePath(1);
    }
    else if (node_id == 2)
    {
      acc_driver->changePath(2);
    }
    else if (node_id == 3)
    {
      acc_driver->changePath(0);
    }

    driver = acc_driver;
  }

  // ---------------
  // Simulation loop
  // ---------------

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);
  int debug_steps = (int)std::ceil(debug_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;

  vehicle.EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;

  DriverInputs driver_inputs;

  if (node_id == 0)
    manager->Update();

  while (true)
  {
    if (syn_manager.IsOk() == false)
      break;

    double time = vehicle.GetSystem()->GetChTime();

    if (step_number == 0)
    {
      realtime_timer.Reset();
    }

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (step_number % render_steps == 0 && node_id == 0)
    {
      vis->BeginScene();
      vis->Render();
      vis->EndScene();

      render_frame++;
    }

    // get the controls for this time step
    // Get driver inputs

    if (step_number % 10 == 0 && node_id == 0)
    {

      driver_inputs.m_steering = SDLDriver.GetSteering();
      driver_inputs.m_throttle = SDLDriver.GetThrottle();
      driver_inputs.m_braking = SDLDriver.GetBraking();
    }

    if (node_id != 0)
      driver_inputs = driver->GetInputs();

    // Update modules (process inputs from other modules)
    syn_manager.Synchronize(time); // Synchronize between nodes
    terrain.Synchronize(time);
    vehicle.Synchronize(time, driver_inputs, terrain);
    if (node_id == 0)
      vis->Synchronize(time, driver_inputs);
    if (node_id != 0)
      driver->Synchronize(time);

    // Advance simulation for one timestep for all modules
    terrain.Advance(step_size);
    vehicle.Advance(step_size);
    if (node_id == 0)
      vis->Advance(step_size);
    if (node_id != 0)
      driver->Advance(step_size);
    // Increment frame number
    step_number++;

    realtime_timer.Spin(time);

    // node_id 2 switch lane
    // randomization happens here
    if (node_id == 2 && std::abs(vehicle.GetSystem()->GetChTime() - 5.0) < 1e-2)
      std::dynamic_pointer_cast<ChMultiPathFollowerACCDriver>(driver)->changePath(1);

    if (node_id == 0)
      manager->Update();

    if (node_id == 0 && SDLDriver.Synchronize() == 1)
    {
      break;
    }
  }

  syn_manager.QuitSimulation();

  return 0;
}

void addTerrain(ChSystem &sys)
{
  std::vector<std::shared_ptr<ChBodyAuxRef>> terrain;
  double terrain_density = 900;
  std::shared_ptr<ChContactMaterial> terrain_mat =
      ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

  std::string terrain_file = std::string(STRINGIFY(HIL_DATA_DIR)) +
                             std::string("/large_ring/new/road_new.obj");

  auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
      terrain_file, true, true);

  double mass;
  ChVector3<> cog;
  ChMatrix33<> inertia;
  mesh->ComputeMassProperties(true, mass, cog, inertia);
  ChMatrix33<> principal_inertia_rot;
  ChVector3<> principal_I;
  ChInertiaUtils::PrincipalInertia(inertia, principal_I,
                                   principal_inertia_rot);

  auto body = chrono_types::make_shared<ChBodyAuxRef>();
  sys.Add(body);
  body->SetFixed(true);
  body->SetFrameRefToAbs(ChFrame<>(ChVector3<>(0, 0, 0), QUNIT));
  body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
  body->SetMass(mass * terrain_density);
  body->SetInertiaXX(terrain_density * principal_I);
  body->EnableCollision(false);

  auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  mesh_shape->SetMesh(mesh);
  mesh_shape->SetMutable(false);
  mesh_shape->SetBackfaceCull(true);
  body->AddVisualShape(mesh_shape);
}

ChCoordsys<> GetVehicleConfig(int node_id,
                              std::string &vehicle,
                              std::string &engine,
                              std::string &transmission,
                              std::string &tire,
                              std::string &zombie)
{
  ChVector3d initLoc;
  ChQuaternion<> initRot;
  switch (node_id)
  {
  case 0:
    vehicle = vehicle::GetDataFile("truck/json/truck_Vehicle.json");
    engine = vehicle::GetDataFile("truck/json/truck_EngineSimple.json");
    transmission = vehicle::GetDataFile("truck/json/truck_AutomaticTransmissionSimpleMap.json");
    tire = vehicle::GetDataFile("truck/json/truck_TMeasyTire.json");
    zombie = synchrono::GetDataFile("vehicle/truck.json");
    initLoc = ChVector3d(1000, 0, 0.5);
    initRot = QuatFromAngleZ(90 * CH_DEG_TO_RAD);
    break;
  case 1:
    vehicle = vehicle::GetDataFile("citybus/vehicle/CityBus_Vehicle.json");
    engine = vehicle::GetDataFile("citybus/powertrain/CityBus_EngineSimpleMap.json");
    transmission = vehicle::GetDataFile("citybus/powertrain/CityBus_AutomaticTransmissionSimpleMap.json");
    tire = vehicle::GetDataFile("citybus/tire/CityBus_TMeasyTire.json");
    zombie = synchrono::GetDataFile("vehicle/CityBus.json");
    initLoc = ChVector3d(1004, 35, 0.3);
    initRot = QuatFromAngleZ(90 * CH_DEG_TO_RAD);
    break;
  case 2:
    vehicle = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
    engine = vehicle::GetDataFile("sedan/powertrain/Sedan_EngineSimpleMap.json");
    transmission = vehicle::GetDataFile("sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json");
    tire = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
    zombie = synchrono::GetDataFile("vehicle/Sedan.json");
    initLoc = ChVector3d(1006.5, 72.1, 0.15);
    initRot = QuatFromAngleZ(90 * CH_DEG_TO_RAD);
    break;
  case 3:
    vehicle = vehicle::GetDataFile("Nissan_Patrol/json/suv_Vehicle.json");
    engine = vehicle::GetDataFile("Nissan_Patrol/json/suv_EngineSimple.json");
    transmission = vehicle::GetDataFile("Nissan_Patrol/json/suv_AutomaticTransmissionSimpleMap.json");
    tire = vehicle::GetDataFile("Nissan_Patrol/json/suv_TMeasyTire.json");
    zombie = vehicle::GetDataFile("Nissan_Patrol/json/suv.json");
    initLoc = ChVector3d(994, 110.6, 0.15);
    initRot = QuatFromAngleZ(90 * CH_DEG_TO_RAD);
    break;
  }

  return ChCoordsys<>(initLoc, initRot);
}
