// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// Jason Zhou
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
// This is a demo showing chrono-unity vehicle coupling using an UDP port
// note: this demo requires boost library (if on Windows an extra external
// library needs to be used)
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <cmath>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

using boost::asio::ip::address;
using boost::asio::ip::udp;

// ====================================================================================
// Gear Settings. The current sedan has four drive mode - Park, Drive, Reverse,
// Neutral gear_button is the button number (need to be changed if controller
// changed)

int gear_mode = 3; // 0=D, 1=R, 2=N, 3=P (N+full brake)
int gear_button = 6;

void ReverseButtonCallback();
// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::MESH;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 5000.0; // size in X direction
double terrainWidth = 5000.0;  // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 5e-4;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Sedan";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1; // FPS = 1

// POV-Ray output
bool povray_output = false;

// param
const double RADS_2_RPM = 30 / CH_C_PI;
const double MS_2_MPH = 2.2369;

// read ip address and port from a json file
std::string couple_data_json = "/unity/sedan_couple_data.json";
std::string end_ip_addr;
int port;
int udp_fp = 10; // udp communication resolution, how many sim steps do we send
                 // UDP package once?
bool render = true; // Irrlicht rendering option

// if the terrain mode is MESH
// mesh_number = 0 -> customized long-distance curved highway
// mesh_number = 1 -> chrono default test obj terrain
int mesh_number = 1;

// =============================================================================

void readParamFile() {
  rapidjson::Document d;
  vehicle::ReadFileJSON(std::string(STRINGIFY(HIL_DATA_DIR)) + couple_data_json,
                        d);

  if (d.HasMember("Address")) {
    end_ip_addr = d["Address"].GetString();
  }

  if (d.HasMember("Port")) {
    port = d["Port"].GetInt();
  }

  if (d.HasMember("Mesh")) {
    mesh_number = d["Mesh"].GetInt();
  }

  if (d.HasMember("Button")) {
    gear_button = d["Button"].GetInt();
  }
}

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  // Read json steering control json file
  readParamFile();
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));

  switch (terrain_model) {
  case RigidTerrain::PatchType::BOX:
    initLoc = ChVector<>(0, 0, 0.5);
    initRot = ChQuaternion<>(1, 0, 0, 0);
    break;
  case RigidTerrain::PatchType::MESH:
    // if long-distance curved highway mesh
    if (mesh_number == 0) {
      initLoc = ChVector<>(0, -580, 1.5);
      initRot = ChQuaternion<>(1, 0, 0, 0);
      initRot.Q_from_AngZ(CH_C_PI / 2);
    } else if (mesh_number == 1) {
      // if default test obj terrain
      initLoc = ChVector<>(0, 0, 0.5);
      initRot = ChQuaternion<>(1, 0, 0, 0);
    }

    break;
  }

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  Sedan my_sedan;
  my_sedan.SetContactMethod(contact_method);
  my_sedan.SetChassisCollisionType(chassis_collision_type);
  my_sedan.SetChassisFixed(false);
  my_sedan.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  my_sedan.SetTireType(tire_model);
  my_sedan.SetTireStepSize(tire_step_size);

  my_sedan.Initialize();

  my_sedan.SetChassisVisualizationType(chassis_vis_type);
  my_sedan.SetSuspensionVisualizationType(suspension_vis_type);
  my_sedan.SetSteeringVisualizationType(steering_vis_type);
  my_sedan.SetWheelVisualizationType(wheel_vis_type);
  my_sedan.SetTireVisualizationType(tire_vis_type);

  // Create the terrain
  RigidTerrain terrain(my_sedan.GetSystem());

  MaterialInfo minfo;
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
    if (mesh_number == 0) {
      patch =
          terrain.AddPatch(patch_mat, CSYSNORM,
                           vehicle::GetDataFile("terrain/meshes/Model.obj"));
      patch->SetTexture(
          vehicle::GetDataFile("terrain/textures/Concrete1_Diff.png"), 100,
          100);
    } else if (mesh_number == 1) {
      patch = terrain.AddPatch(patch_mat, CSYSNORM,
                               vehicle::GetDataFile("terrain/meshes/test.obj"));
      patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 100,
                        100);
    }

    break;
  }
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

  terrain.Initialize();

  // Create the vehicle Irrlicht interface
  auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();

  vis->SetWindowTitle("Sedan Demo");
  vis->SetChaseCamera(trackPoint, 6.0, 0.5);
  vis->Initialize();
  vis->AddTypicalLights();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(&my_sedan.GetVehicle());

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

  std::string driver_file = out_dir + "/driver_inputs.txt";
  utils::CSV_writer driver_csv(" ");

  // ------------------------
  // Create the driver system
  // ------------------------

  // Create the interactive driver system
  ChIrrGuiDriver driver(*vis);

  driver.SetButtonCallback(gear_button, &ReverseButtonCallback);

  // Set the time response for steering and throttle keyboard inputs.
  double steering_time = 1.0; // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0; // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1
  driver.SetSteeringDelta(render_step_size / steering_time);
  driver.SetThrottleDelta(render_step_size / throttle_time);
  driver.SetBrakingDelta(render_step_size / braking_time);

  // If in playback mode, attach the data file to the driver system and
  // force it to playback the driver inputs.
  if (driver_mode == PLAYBACK) {
    driver.SetInputDataFile(driver_file);
    driver.SetInputMode(ChIrrGuiDriver::InputMode::DATAFILE);
  }

  driver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                               std::string("/unity/controller_G27.json"));

  driver.Initialize();

  // ---------------
  // Simulation loop
  // ---------------

  if (debug_output) {
    GetLog() << "\n\n============ System Configuration ============\n";
    my_sedan.LogHardpointLocations();
    std::cout << "\nVehicle mass: " << my_sedan.GetVehicle().GetMass()
              << std::endl;
  }
  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);
  int debug_steps = (int)std::ceil(debug_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;

  if (contact_vis && render) {
    vis->SetSymbolScale(1e-4);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
  }

  ChRealtimeStepTimer realtime_timer;
  utils::ChRunningAverage RTF_filter(50);

  boost::asio::io_service io_service;
  udp::socket socket(io_service);
  udp::endpoint remote_endpoint =
      udp::endpoint(address::from_string(end_ip_addr), port);
  socket.open(udp::v4());

  while (vis->Run()) {
    double time = my_sedan.GetSystem()->GetChTime();

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (step_number % render_steps == 0) {
      if (render) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
      }

      if (povray_output) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(),
                render_frame + 1);
        utils::WriteVisualizationAssets(my_sedan.GetSystem(), filename);
      }

      render_frame++;
    }

    // Debug logging
    if (debug_output && step_number % debug_steps == 0) {
      GetLog() << "\n\n============ System Information ============\n";
      GetLog() << "Time = " << time << "\n\n";
      my_sedan.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
    }

    // Get driver inputs
    DriverInputs driver_inputs = driver.GetInputs();

    // Drive mode switch
    if (gear_mode == 0) {
      my_sedan.GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
    } else if (gear_mode == 1) {
      my_sedan.GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
    } else if (gear_mode == 2) {
      my_sedan.GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
    } else {
      my_sedan.GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
      driver_inputs.m_braking = 1.0;
    }

    // Driver output
    if (driver_mode == RECORD) {
      driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle
                 << driver_inputs.m_braking << std::endl;
    }

    // Update modules (process inputs from other modules)
    driver.Synchronize(time);
    terrain.Synchronize(time);
    my_sedan.Synchronize(time, driver_inputs, terrain);
    vis->Synchronize(driver.GetInputModeAsString(), driver_inputs);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    terrain.Advance(step_size);
    my_sedan.Advance(step_size);
    vis->Advance(step_size);

    // Increment frame number
    step_number++;

    // Spin in place for real time to catch up
    realtime_timer.Spin(step_size);
    // std::cout << RTF_filter.Add(realtime_timer.RTF) << std::endl;

    // prepare for data output
    ChQuaternion<> dir = my_sedan.GetVehicle().GetRot();
    ChVector<> euler_dir = dir.Q_to_Euler123();

    float udp_float_arr[23];

    udp_float_arr[0] = time;
    udp_float_arr[1] = step_size;
    udp_float_arr[2] = my_sedan.GetVehicle().GetPos().x();
    udp_float_arr[3] = my_sedan.GetVehicle().GetPos().y();
    udp_float_arr[4] = my_sedan.GetVehicle().GetPos().z();

    udp_float_arr[5] = my_sedan.GetVehicle().GetSpeed();
    udp_float_arr[6] = 0; // heading ?
    udp_float_arr[7] = driver.GetSteering();
    udp_float_arr[8] = driver.GetThrottle();
    udp_float_arr[9] = driver.GetBraking();

    udp_float_arr[10] = 0; // state1 ?
    udp_float_arr[11] = 0; // state2 ?
    udp_float_arr[12] = 0; // state3 ?
    udp_float_arr[13] = 0; // state4 ?
    udp_float_arr[14] = 0; // state5 ?
    udp_float_arr[15] = 0; // state6 ?

    udp_float_arr[16] = euler_dir.x();
    udp_float_arr[17] = euler_dir.y();
    udp_float_arr[18] = euler_dir.z();

    udp_float_arr[19] = my_sedan.GetPowertrain()->GetMotorSpeed() * RADS_2_RPM;
    udp_float_arr[20] = my_sedan.GetPowertrain()->GetCurrentTransmissionGear();
    udp_float_arr[21] = 0; // left signal ?
    udp_float_arr[22] = 0; // right signal ?

    // send udp package - the 23*4 bytes float array
    if (step_number % udp_fp == 0) {
      boost::system::error_code err;
      auto sent =
          socket.send_to(boost::asio::buffer(udp_float_arr, sizeof(float) * 23),
                         remote_endpoint, 0, err);
    }
  }

  socket.close();

  if (driver_mode == RECORD) {
    driver_csv.write_to_file(driver_file);
  }

  return 0;
}

void ReverseButtonCallback() {
  // anti bounce
  // switch driving mode
  static auto last_invoked =
      std::chrono::system_clock::now().time_since_epoch();
  static int count = 0;
  auto current_invoke = std::chrono::system_clock::now().time_since_epoch();
  if (count == 0 || std::chrono::duration_cast<std::chrono::seconds>(
                        current_invoke - last_invoked)
                            .count() > 0.5) {
    gear_mode = (gear_mode + 1) % 4;
    last_invoked = current_invoke;
    count = count + 1;
  }
}