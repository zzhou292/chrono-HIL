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

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_hil/network/sim/ChDelaySim.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::artcar;
using namespace chrono::hil;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector3<> initLoc(0, 0, 0.2);
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
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::MESH;
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

// Output directories
const std::string out_dir = "ARTCar";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1; // FPS = 1

void addCones(ChSystem &sys, std::vector<std::string> &cone_files,
              std::vector<ChVector3<>> &cone_pos);

// =============================================================================
std::vector<char> serializeFloats(const std::vector<float> &floatVec)
{
  std::vector<char> byteVec(floatVec.size() * sizeof(float));
  char *bytePointer = byteVec.data();

  for (const float &value : floatVec)
  {
    std::memcpy(bytePointer, &value, sizeof(float));
    bytePointer += sizeof(float);
  }

  return byteVec;
}

std::vector<float> deserializeFloats(const std::vector<char> &byteVec)
{
  std::vector<float> floatVec(byteVec.size() / sizeof(float));
  const char *bytePointer = byteVec.data();

  for (float &value : floatVec)
  {
    std::memcpy(&value, bytePointer, sizeof(float));
    bytePointer += sizeof(float);
  }

  return floatVec;
}

// =============================================================================
int main(int argc, char *argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  ARTcar my_rccar;
  my_rccar.SetContactMethod(contact_method);
  my_rccar.SetChassisCollisionType(chassis_collision_type);
  my_rccar.SetChassisFixed(false);
  my_rccar.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  my_rccar.SetTireType(tire_model);
  my_rccar.SetTireStepSize(tire_step_size);
  my_rccar.Initialize();

  my_rccar.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
  my_rccar.GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
  my_rccar.GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  my_rccar.GetSystem()->GetSolver()->AsIterative()->SetMaxIterations(150);
  my_rccar.GetSystem()->SetMaxPenetrationRecoverySpeed(4.0);

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
  switch (terrain_model)
  {
  case RigidTerrain::PatchType::BOX:
    patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20,
                      20);
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
                             std::string(STRINGIFY(HIL_DATA_DIR)) + std::string("/Environments/me3038/rm3038_v1.obj"));
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

  // -----------------------
  // Adding cone objects
  // -----------------------

  // Create obstacles
  std::vector<std::string> cone_meshfile = {
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //

      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //

      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //

      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //

      "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj", //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
      "sensor/cones/red_cone.obj",                                //
  };
  std::vector<ChVector3<>> cone_pos = {
      ChVector3<>(-1.48, -4.54, -0.24),
      ChVector3<>(-4.0872, -2.0322, -0.24),
      ChVector3<>(-1.42, -3.53, -0.24),
      ChVector3<>(-4.07, -0.97672, -0.24),
      ChVector3<>(-1.7255, -2.8225, -0.24),
      ChVector3<>(-3.9574, 0.50855, -0.24),
      ChVector3<>(-1.9213, -1.9518, -0.24),
      ChVector3<>(-3.9172, 2.2994, -0.24),
      ChVector3<>(-2.207, -1.0438, -0.24),
      ChVector3<>(-2.4924, 2.778, -0.24),
      ChVector3<>(-2.6502, -0.5412, -0.24),
      ChVector3<>(-0.89042, 2.7256, -0.24),
      ChVector3<>(-2.6556, 0.23594, -0.24),
      ChVector3<>(0.24018, 1.5736, -0.24),
      ChVector3<>(-2.30, 1.077, -0.24),
      ChVector3<>(0.94455, 0.85916, -0.24),

      ChVector3<>(-1.7513, 0.88291, -0.24),
      ChVector3<>(-1.2546, 0.08584, -0.24),
      ChVector3<>(-1.0548, 0.6463, -0.24),
      ChVector3<>(1.3596, -0.71367, -0.24),
      ChVector3<>(-0.422, 0.49659, -0.24),
      ChVector3<>(1.7232, -1.5822, -0.24),
      ChVector3<>(0.048359, 0.21137, -0.24),
      ChVector3<>(1.5273, -2.5145, -0.24),
      ChVector3<>(0.26932, -0.4039, -0.24),
      ChVector3<>(1.4294, -3.7086, -0.24),
      ChVector3<>(0.45935, -1.3053, -0.24),
      ChVector3<>(1.4357, -5.4693, -0.24),
      ChVector3<>(0.43702, -2.2048, -0.24),
      ChVector3<>(0.42572, -5.9832, -0.24),
      ChVector3<>(0.29, -2.9696, -0.24),
      ChVector3<>(-0.31011, -6.2391, -0.24),

      ChVector3<>(0.2511, -4.6653, -0.24),
      ChVector3<>(-0.84832, -6.4183, -0.24),
      ChVector3<>(-0.17387, -5.0, -0.24),
      ChVector3<>(-1.4886, -6.2794, -0.24),
      ChVector3<>(-0.519, -5.2637, -0.24),
      ChVector3<>(-2.7153, -5.7845, -0.24),
      ChVector3<>(-0.8987, -5.3569, -0.24),
      ChVector3<>(-3.455, -5.6976, -0.24),
      ChVector3<>(-1.5731, -5.4358, -0.24),
      ChVector3<>(-3.8393, -5.4056, -0.24),
      ChVector3<>(-2.0647, -5.16, -0.24),
      ChVector3<>(-4.4641, -5.2733, -0.24),
      ChVector3<>(-2.4413, -5.164, -0.24),
      ChVector3<>(-4.8451, -4.9955, -0.24),
      ChVector3<>(-2.8449, -5.1021, -0.24),
      ChVector3<>(-5.1941, -4.6861, -0.24),

      ChVector3<>(-3.5, -4.9796, -0.24),
      ChVector3<>(-0.84832, -6.4183, -0.24),
      ChVector3<>(-3.8379, -4.7264, -0.24),
      ChVector3<>(-1.4886, -6.2794, -0.24),
      ChVector3<>(-4.1242, -4.572, -0.24),
      ChVector3<>(-2.7153, -5.7845, -0.24),
      ChVector3<>(-4.5491, -4.2879, -0.24),
      ChVector3<>(-3.455, -5.6976, -0.24),
      ChVector3<>(-4.4093, -4.2266, -0.24),
      ChVector3<>(-3.8393, -5.4056, -0.24),
      ChVector3<>(-4.1837, -4.3619, -0.24),
      ChVector3<>(-4.4641, -5.2733, -0.24),
      ChVector3<>(-3.7624, -4.6589, -0.24),
      ChVector3<>(-4.8451, -4.9955, -0.24),
      ChVector3<>(-3.0384, -4.9674, -0.24),
      ChVector3<>(-5.1941, -4.6861, -0.24),

      ChVector3<>(-2.351, -4.9441, -0.24),
      ChVector3<>(-5.2459, -3.9942, -0.24),
      ChVector3<>(-5.1885, -3.3619, -0.24),
      ChVector3<>(-4.6484, -3.0405, -0.24),
      ChVector3<>(-4.0247, -3.2546, -0.24),
      ChVector3<>(-3.5223, -3.6743, -0.24),
      ChVector3<>(-3.0137, -4.0342, -0.24),
      ChVector3<>(-3.2747, -3.6302, -0.24),
      ChVector3<>(-3.9402, -2.6624, -0.24),
  };
  addCones((*my_rccar.GetSystem()), cone_meshfile, cone_pos);

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
  ChQuaternion<> cam_rot;
  cam_rot.SetFromAngleAxis(0, {0, 1, 0});
  auto cam = chrono_types::make_shared<ChCameraSensor>(
      my_rccar.GetVehicle().GetChassisBody(), // body camera is attached to
      35,                                     // update rate in Hz
      chrono::ChFrame<double>({-0.02, 0, 0.07},
                              cam_rot), // offset pose
      1280,                             // image width
      720,                              // image height
      CH_PI_2 / 1.5,
      2); // fov, lag, exposure
  cam->SetName("Camera Sensor");
  cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
      1280, 720, "Driver View - front", false));
  cam->SetLag(0.05f);
  manager->AddSensor(cam);

  // ------------------------------------------------
  // Create a back-view camera and add it to the sensor manager
  // ------------------------------------------------
  // ChQuaternion<> cam_rot2;
  // cam_rot2.SetFromAngleAxis(CH_PI, {0, 0, 1});
  // auto cam2 = chrono_types::make_shared<ChCameraSensor>(
  //     my_rccar.GetVehicle().GetChassisBody(), // body camera is attached to
  //     25,                                     // update rate in Hz
  //     chrono::ChFrame<double>(
  //         {-0.1, 0, 0.07}, cam_rot2), // offset pose
  //     1280,                           // image width
  //     720,                            // image height
  //     CH_PI_4,
  //     1); // fov, lag, exposure
  // cam2->SetName("Camera Sensor - back");
  // cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
  //     1280, 720, "Driver View - back", false));
  // cam2->SetLag(0.05f);
  // manager->AddSensor(cam2);

  // -----------------
  // Initialize output
  // -----------------

  if (!filesystem::create_directory(filesystem::path(out_dir)))
  {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }

  utils::ChWriterCSV csv_1(" ");
  utils::ChWriterCSV csv_2(" ");

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

  if (contact_vis)
  {
    vis->SetSymbolScale(1e-4);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
  }

  my_rccar.GetVehicle().EnableRealtime(false);

  ChRealtimeCumulative realtime_timer;

  DriverInputs driver_inputs;

  auto normalDist = std::make_shared<chrono::hil::NormalDistribution>(600.0f, 8.0f);
  // Initialize delay simulator with normal distribution
  ChDelaySim sim(normalDist, 500.0f);
  sim.setLogging(true);

  while (true)
  {
    double time = my_rccar.GetSystem()->GetChTime();

    // std::cout << cam->GetLag() << std::endl;
    if (step_number == 5000)
    {
      cam->SetLag(0.08f);
    }

    manager->Update();

    if (step_number == 0)
    {
      realtime_timer.Reset();
    }

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (step_number % render_steps == 0)
    {
      vis->BeginScene();
      vis->Render();
      vis->EndScene();

      render_frame++;
    }

    // get the controls for this time step
    // Get driver inputs

    if (step_number % 10 == 0)
    {
      // Create a vector of floats
      std::vector<float> floats = {SDLDriver.GetSteering(), SDLDriver.GetThrottle(), SDLDriver.GetBraking()};

      // Serialize the vector of floats to a vector of chars
      std::vector<char> serializedData = serializeFloats(floats);
      sim.addPacket(serializedData);

      // Get the packet back
      std::vector<char> receivedData = sim.getDelayedPacket();
      if (!receivedData.empty())
      {
        // Deserialize the data back to floats
        std::vector<float> receivedFloats = deserializeFloats(receivedData);
        driver_inputs.m_steering = receivedFloats[0];
        driver_inputs.m_throttle = receivedFloats[1] * 0.2;
        driver_inputs.m_braking = receivedFloats[2];
      }

      // write drive torques of all four wheels into file
      std::vector<float> delay_buffer = sim.getDelayBuffer();

      for (int i = 0; i < delay_buffer.size(); i++)
      {
        csv_1 << time << "," << delay_buffer[i] << std::endl;
      }
    }

    if (step_number % 100 == 0)
    {
      int drop_ct = sim.getPacketDropCount();
      csv_2 << time << "," << drop_ct << std::endl;
    }

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

    if (SDLDriver.Synchronize() == 1)
    {
      break;
    }
  }

  csv_1.WriteToFile(out_dir + "/output1.dat");
  csv_2.WriteToFile(out_dir + "/output2.dat");
  return 0;
}

void addCones(ChSystem &sys, std::vector<std::string> &cone_files,
              std::vector<ChVector3<>> &cone_pos)
{
  SetChronoDataPath(CHRONO_DATA_DIR);
  std::vector<std::shared_ptr<ChBodyAuxRef>> cone;
  double cone_density = 900;
  std::shared_ptr<ChContactMaterial> rock_mat =
      ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

  for (int i = 0; i < cone_files.size(); i++)
  {
    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
        GetChronoDataFile(cone_files[i]), true, true);

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
    body->SetFrameRefToAbs(ChFrame<>(ChVector3<>(cone_pos[i]), QUNIT));
    body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
    body->SetMass(mass * cone_density);
    body->SetInertiaXX(cone_density * principal_I);
    body->EnableCollision(false);

    auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape->SetMesh(mesh);
    mesh_shape->SetBackfaceCull(true);
    body->AddVisualShape(mesh_shape);
  }
}
