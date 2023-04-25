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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChIDM_Follower.h"
#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLineBezier.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::sensor;
using namespace chrono::hil;

using namespace std::chrono;
// =============================================================================
// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;
// =============================================================================

float radius = 50.f;

// Initial vehicle location and orientation
ChVector<> initLoc(0, radius, 0.5);
ChQuaternion<> initRot = Q_from_AngZ(0);

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
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
double terrainHeight = 0;   // terrain height (FLAT terrain only)
double terrainLength = 0.0; // size in X direction
double terrainWidth = 0.0;  // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double sim_time = 1200.0;
double heartbeat = 0.04;
double step_size = 2e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50; // FPS = 50

int render_scene = 0;
int fps = 25;
int output = 0;

int use_control = 0; // whether to use slow start control
utils::ChRunningAverage IG_speed_avg(10000);

std::string path_file(std::string(STRINGIFY(HIL_DATA_DIR)) +
                      "/ring/terrain0103/ring50_closed.txt");

const std::string out_dir = GetChronoOutputPath() + "ring_out";

// reply data file
std::vector<float> throttles;
std::vector<float> brakes;
std::vector<float> steerings;

// =============================================================================
void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
  cli.AddOption<int>("Simulation", "v,vehicle",
                     "Vehicle Type: 1-sedan, 2-audi, 3-hmmwv",
                     "must have this number");
  cli.AddOption<int>("Simulation", "s,drive_type",
                     "drive type: 0-idm 1-sdl 2-readfrom file",
                     "must have this number");
  cli.AddOption<int>("Simulation", "i,idm",
                     "idm Type: 1-normal, 2-aggressive, 3-conservative",
                     "must have this number");
  cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat",
                        std::to_string(heartbeat));
  cli.AddOption<int>("Render", "r,render", "whether_to_render",
                     std::to_string(render_scene));
  cli.AddOption<int>("Simulation", "realtime", "run real time", "0");
  cli.AddOption<int>("Simulation", "record", "record driver input", "0");
  cli.AddOption<int>("Simulation", "output", "output exp data", "0");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
  cli.AddOption<int>("Simulation", "control", "to use slow-start control", "0");
}

void readvectors(std::vector<float> &throttle_ref,
                 std::vector<float> &brakes_ref,
                 std::vector<float> &steerings_ref) {
  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;
  std::string line, word;

  std::fstream file("input.csv", std::ios::in);
  if (file.is_open()) {
    while (getline(file, line)) {
      row.clear();

      std::stringstream str(line);

      while (getline(str, word, ','))
        row.push_back(word);
      content.push_back(row);
    }
  } else
    std::cout << "Could not open the file\n";

  for (int i = 0; i < content.size(); i++) {
    throttle_ref.push_back(std::stof(content[i][0]));
    brakes_ref.push_back(std::stof(content[i][1]));
    steerings_ref.push_back(std::stof(content[i][2]));
  }

  std::cout << "done reading records" << std::endl;
}

// =============================================================================

int main(int argc, char *argv[]) {
  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";
  ChCLI cli(argv[0]);
  AddCommandLineOptions(cli);

  if (!cli.Parse(argc, argv, true))
    return 0;

  const int vehicle_type = cli.GetAsType<int>("vehicle");
  const int idm_type = cli.GetAsType<int>("idm");
  const int drive_type = cli.GetAsType<int>("drive_type");

  const int real_time = cli.GetAsType<int>("realtime");
  const int record = cli.GetAsType<int>("record");
  const int output = cli.GetAsType<int>("output");

  if (drive_type == 2) {
    readvectors(throttles, brakes, steerings);
  }

  SetChronoDataPath(CHRONO_DATA_DIR);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("synchrono/"));

  std::string vehicle_filename =
      vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
  std::string zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");

  if (vehicle_type == 1) {
    vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
    tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
    powertrain_filename =
        vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
    zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
  } else if (vehicle_type == 2) {
    vehicle_filename = vehicle::GetDataFile("audi/json/audi_Vehicle.json");
    tire_filename = vehicle::GetDataFile("audi/json/audi_TMeasyTire.json");
    powertrain_filename =
        vehicle::GetDataFile("audi/json/audi_SimpleMapPowertrain.json");
    zombie_filename = synchrono::GetDataFile("vehicle/audi.json");
  } else if (vehicle_type == 3) {
    vehicle_filename = vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
    tire_filename = vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");
    powertrain_filename =
        vehicle::GetDataFile("hmmwv/powertrain/HMMWV_ShaftsPowertrain.json");
    zombie_filename = synchrono::GetDataFile("vehicle/HMMWV.json");
  }

  std::string steer_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SteeringController.json";
  std::string speed_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SpeedController.json";

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");
  const std::vector<std::string> ip_list =
      cli.GetAsType<std::vector<std::string>>("ip");
  render_scene = cli.GetAsType<int>("render");
  use_control = cli.GetAsType<int>("control");

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
  std::cout << "node_id: " << node_id << ", num_nodes:" << num_nodes
            << std::endl;
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  // Change SynChronoManager settings
  syn_manager.SetHeartbeat(heartbeat);

  // Decide Vehicle Locations
  float deg_sec = (CH_C_PI * 1.6) / num_nodes;

  initLoc = ChVector<>(radius * cos(deg_sec * node_id),
                       radius * sin(deg_sec * node_id), 0.5);
  float rot_deg = deg_sec * node_id + CH_C_PI_2;
  if (rot_deg > CH_C_2PI) {
    rot_deg = rot_deg - CH_C_2PI;
  }
  initRot = Q_from_AngZ(rot_deg);

  // --------------
  // Create systems
  // --------------

  // Create the Sedan vehicle, set parameters, and initialize
  WheeledVehicle my_vehicle(vehicle_filename, ChContactMethod::SMC);
  auto ego_chassis = my_vehicle.GetChassis();
  my_vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
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
      my_vehicle.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // Create the terrain
  RigidTerrain terrain(my_vehicle.GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.01f;
  minfo.Y = 2e7f;
  auto patch_mat = minfo.CreateMaterial(contact_method);

  std::shared_ptr<RigidTerrain::Patch> patch;
  patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);

  terrain.Initialize();

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

  my_vehicle.GetSystem()->AddBody(terrain_body);

  // add dummy body
  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetPos({0.0, 0.0, 0.0});
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);

  // Add vehicle as an agent and initialize SynChronoManager
  syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename));
  syn_manager.Initialize(my_vehicle.GetSystem());

  // Create a sensor manager
  // ====================================================
  auto manager =
      chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
  float intensity = 1.2;
  manager->scene->AddPointLight({0, 0, 1e8}, {1.0, 1.0, 1.0}, 1e12);
  manager->scene->SetAmbientLight({.1, .1, .1});
  manager->scene->SetSceneEpsilon(1e-3);
  manager->scene->EnableDynamicOrigin(true);
  manager->scene->SetOriginOffsetThreshold(500.f);

  if (render_scene == 1) {

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        30,            // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(0.0, 0.0, 100.0),
            Q_from_Euler123(ChVector<>(0.0, CH_C_PI_2, 0.0))), // offset pose
        1280,                                                  // image width
        720,                                                   // image height
        1.608f,
        1); // fov, lag, exposure
    cam->SetName("Camera Sensor");

    // cam->PushFilter(
    //     chrono_types::make_shared<ChFilterVisualize>(1280, 720, "fov",
    //     false));
    //  Provide the host access to the RGBA8 buffer
    cam->PushFilter(
        chrono_types::make_shared<ChFilterVisualize>(1280, 720, "fov", false));
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // cam->PushFilter(chrono_types::make_shared<ChFilterSave>("cam1/"));
    manager->AddSensor(cam);

  } else if (render_scene == 2) {
    // mirrors position and rotations
    ChVector<> mirror_rearview_pos = {0.253, 0.0, 1.10};
    ChQuaternion<> mirror_rearview_rot = Q_from_Euler123(
        ChVector<>(2.0 * 0.01745, -6.0 * 0.01745, -12.0 * 0.01745));
    ChVector<> mirror_wingleft_pos = {0.47, 0.945, 0.815};
    ChQuaternion<> mirror_wingleft_rot =
        Q_from_Euler123(ChVector<>(0.0, 5.0 * 0.01745, 17.0 * 0.01745));
    ChVector<> mirror_wingright_pos = {0.4899, -0.95925, 0.80857};
    ChQuaternion<> mirror_wingright_rot =
        Q_from_Euler123(ChVector<>(0.0, 3.5 * 0.01745, -28.0 * 0.01745));

    // change the ego vehicle vis out for windowless audi
    my_vehicle.GetChassisBody()->GetVisualModel()->Clear();

    auto audi_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    audi_mesh->LoadWavefrontMesh(
        std::string(STRINGIFY(HIL_DATA_DIR)) +
            "/Environments/Iowa/vehicles/audi_chassis_windowless_2.obj",
        false, true);
    audi_mesh->Transform(ChVector<>(0, 0, 0),
                         ChMatrix33<>(1)); // scale to a different size
    auto audi_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    audi_shape->SetMesh(audi_mesh);
    audi_shape->SetName("Windowless Audi");
    audi_shape->SetMutable(false);
    // audi_shape->SetStatic(true);
    my_vehicle.GetChassisBody()->AddVisualShape(audi_shape);

    // add rearview mirror
    auto mirror_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mirror_mesh->LoadWavefrontMesh(
        std::string(STRINGIFY(HIL_DATA_DIR)) +
            "/Environments/Iowa/vehicles/audi_rearview_mirror.obj",
        false, true);
    mirror_mesh->Transform(ChVector<>(0, 0, 0),
                           ChMatrix33<>(1)); // scale to a different size

    auto mirror_mat = chrono_types::make_shared<ChVisualMaterial>();
    mirror_mat->SetDiffuseColor({0.2f, 0.2f, 0.2f});
    mirror_mat->SetRoughness(0.f);
    mirror_mat->SetMetallic(1.0f);
    mirror_mat->SetUseSpecularWorkflow(false);

    auto rvw_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    rvw_mirror_shape->SetMesh(mirror_mesh);
    rvw_mirror_shape->SetName("Windowless Audi");
    rvw_mirror_shape->SetMutable(false);
    rvw_mirror_shape->SetScale({1, 1.8, 1.2});
    rvw_mirror_shape->GetMaterials()[0] = mirror_mat;
    my_vehicle.GetChassisBody()->AddVisualShape(
        rvw_mirror_shape, ChFrame<>(mirror_rearview_pos, mirror_rearview_rot));

    // add left wing mirror
    auto lwm_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    lwm_mesh->LoadWavefrontMesh(
        std::string(STRINGIFY(HIL_DATA_DIR)) +
            "/Environments/Iowa/vehicles/audi_left_wing_mirror.obj",
        false, true);
    lwm_mesh->Transform(ChVector<>(0, 0, 0),
                        ChMatrix33<>(1)); // scale to a different size

    auto lwm_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    lwm_mirror_shape->SetMesh(lwm_mesh);
    lwm_mirror_shape->SetName("Windowless Audi");
    lwm_mirror_shape->GetMaterials()[0] = mirror_mat;
    lwm_mirror_shape->SetMutable(false);
    my_vehicle.GetChassisBody()->AddVisualShape(
        lwm_mirror_shape, ChFrame<>(mirror_wingleft_pos, mirror_wingleft_rot));

    // add left wing mirror
    auto rwm_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    rwm_mesh->LoadWavefrontMesh(
        std::string(STRINGIFY(HIL_DATA_DIR)) +
            "/Environments/Iowa/vehicles/audi_right_wing_mirror.obj",
        false, true);
    rwm_mesh->Transform(ChVector<>(0, 0, 0),
                        ChMatrix33<>(1)); // scale to a different size

    auto rwm_mirror_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    rwm_mirror_shape->SetMesh(rwm_mesh);
    rwm_mirror_shape->SetName("Windowless Audi");
    rwm_mirror_shape->SetScale({1, .98, .98});
    rwm_mirror_shape->GetMaterials()[0] = mirror_mat;
    rwm_mirror_shape->SetMutable(false);
    my_vehicle.GetChassisBody()->AddVisualShape(
        rwm_mirror_shape,
        ChFrame<>(mirror_wingright_pos, mirror_wingright_rot));

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        my_vehicle.GetChassisBody(), // body camera is attached to
        fps,                         // update rate in Hz
        chrono::ChFrame<double>(ChVector<>(-.3, .4, .98),
                                Q_from_AngAxis(0, {1, 0, 0})), // offset pose
        1920 * 3,                                              // image width
        1080,                                                  // image height
        3.2f,
        1); // fov, lag, exposure
    cam->SetName("Camera Sensor");
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
        1920 * 3, 1080, "Driver View", false));

    // add sensor to the manager
    manager->AddSensor(cam);
  } else if (render_scene == 3) {

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        fps,           // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(0.0, 0.0, 100.0),
            Q_from_Euler123(ChVector<>(0.0, CH_C_PI_2, 0.0))), // offset pose
        1920,                                                  // image width
        1080,                                                  // image height
        1.608f,
        2); // fov, lag, exposure
    cam->SetName("Camera Sensor");

    // cam->PushFilter(
    //     chrono_types::make_shared<ChFilterVisualize>(1280, 720, "fov",
    //     false));
    //  Provide the host access to the RGBA8 buffer
    // cam->PushFilter(
    //    chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "fov",
    //    false));
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("cam1/"));
    manager->AddSensor(cam);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        my_vehicle.GetChassisBody(), // body camera is attached to
        fps,                         // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(-6.0, 0.0, 2.5),
            Q_from_Euler123(ChVector<>(0.0, 0.3, 0.0))), // offset
        1920,                                            // image width
        1080,                                            // image
        1.608f, 2); // fov, lag, exposure cam2->SetName("Camera Sensor

    // cam2->PushFilter(
    //     chrono_types::make_shared<ChFilterVisualize>(1920, 1080, "fov",
    //     false));
    //   Provide the host access to the RGBA8 buffer
    // cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam2->PushFilter(chrono_types::make_shared<ChFilterSave>("cam2/"));
    manager->AddSensor(cam2);

    auto cam3 = chrono_types::make_shared<ChCameraSensor>(
        attached_body, // body camera is attached to
        fps,           // update rate in Hz
        chrono::ChFrame<double>(
            ChVector<>(55.0, -55.0, 9.0),
            Q_from_Euler123(ChVector<>(0.0, 0.3, CH_C_PI * 5 / 6))), // offset
        1920,     // image width
        1080,     // image
        1.3f, 2); // fov, lag, exposure cam2->SetName("Camera Sensor

    // cam3->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
    //     1920, 1080, "stand", false));
    //   Provide the host access to the RGBA8 buffer
    // cam3->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam3->PushFilter(chrono_types::make_shared<ChFilterSave>("cam3/"));
    manager->AddSensor(cam3);
  }

  // -----------------
  // Initialize output
  // -----------------

  // Initialize output

  if (!filesystem::create_directory(filesystem::path(out_dir))) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }

  std::string record_file_path = "./record.csv";
  std::ofstream record_filestream = std::ofstream(record_file_path);
  std::stringstream record_buffer;

  std::string render_file_path = "./render.csv";
  std::ofstream render_filestream = std::ofstream(render_file_path);
  std::stringstream render_buffer;

  std::string output_file_path = "./output.csv";
  std::ofstream output_filestream = std::ofstream(output_file_path);
  std::stringstream output_buffer;

  // ------------------------
  // Create the driver system
  // ------------------------

  // read from a bezier curve, and form a closed loop
  auto path = ChBezierCurve::read(path_file, true);

  // idm parameters
  std::vector<double> followerParam;
  if (idm_type == 1) {
    followerParam.push_back(8.9408);
    followerParam.push_back(0.1);
    followerParam.push_back(5.0);
    followerParam.push_back(3.5);
    followerParam.push_back(2.5);
    followerParam.push_back(4.0);
    followerParam.push_back(4.86);

  } else if (idm_type == 2) {
    followerParam.push_back(8.9408);
    followerParam.push_back(0.2);
    followerParam.push_back(6.0);
    followerParam.push_back(3.0);
    followerParam.push_back(2.1);
    followerParam.push_back(4.0);
    followerParam.push_back(4.86);
  } else if (idm_type == 3) {
    followerParam.push_back(8.9408);
    followerParam.push_back(0.7);
    followerParam.push_back(8.0);
    followerParam.push_back(2.5);
    followerParam.push_back(1.5);
    followerParam.push_back(4.0);
    followerParam.push_back(4.86);
  }

  ChIDMFollower driver(my_vehicle, steer_controller, speed_controller, path,
                       "road", 20.0 * MPH_TO_MS, followerParam);
  // ChPathFollowerDriver driver(my_vehicle, path, "my_path", 20.0 *
  // MPH_TO_MS);
  driver.GetSteeringController().SetLookAheadDistance(5);
  driver.GetSteeringController().SetGains(0.8, 0, 0);
  driver.GetSpeedController().SetGains(0.4, 0, 0);
  ChSDLInterface SDLDriver;
  driver.Initialize();

  if (drive_type == 1) {
    SDLDriver.Initialize();
    SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                    "/joystick/controller_G29.json");
  }

  // ---------------
  // Simulation loop
  // ---------------

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Initialize simulation frame counters
  int step_number = 0;
  int render_frame = 0;

  utils::ChRunningAverage RTF_filter(50);

  float *all_x = new float[num_nodes];
  float *all_y = new float[num_nodes];
  float *all_prev_x = new float[num_nodes];
  float *all_prev_y = new float[num_nodes];
  float *all_speed = new float[num_nodes];

  double time = 0.0;

  // obtain and initiate all zombie instances
  std::map<AgentKey, std::shared_ptr<SynAgent>> zombie_map;
  std::map<int, std::shared_ptr<SynWheeledVehicleAgent>> id_map;

  int lead_idx = (node_id + 1) % num_nodes;
  float act_dis = 0;

  ChRealtimeCumulative realtime_timer;
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double last_time = 0;

  my_vehicle.EnableRealtime(false);

  manager->Update();

  while (time <= sim_time && syn_manager.IsOk()) {
    time = my_vehicle.GetSystem()->GetChTime();

    if (step_number == 1) {
      realtime_timer.Reset();
    } else if (step_number != 1 && step_number != 0) {
      if (real_time == 1) {
        realtime_timer.Spin(time);
      }
    }

    if (step_number % 500 == 0 && node_id == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      SynLog() << (wall_time.count()) / (time - last_time) << "\n";
      last_time = time;
      start = std::chrono::high_resolution_clock::now();
    }

    // slow-start control
    if (use_control == 1 && drive_type == 0) {
      float avg_speed = IG_speed_avg.Add(my_vehicle.GetSpeed());
      if (avg_speed < (8.9408 / 4.0)) {
        driver.Set_CruiseSpeed(8.9408 / 3.0);
        // std::cout << "spd:" << 8.9408 / 3.0 << std::endl;
      } else if (avg_speed < (8.9408 / 3.0)) {
        driver.Set_CruiseSpeed(8.9408 / 2.0);
        // std::cout << "spd:" << 8.9408 / 2.0 << std::endl;
      } else {
        driver.Set_CruiseSpeed(8.9408);
        // std::cout << "spd:" << 8.9408 << std::endl;
      }
    }

    // obtain map
    if (step_number == 0) {
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
        std::cout << it->first.GetNodeID() << std::endl;
        id_map.insert(std::make_pair(it->first.GetNodeID(), converted_ptr));
      }
    }

    // update necessary zombie info for IDM
    if (step_number % int(heartbeat / step_size) == 0) {
      for (int i = 0; i < num_nodes; i++) {
        if (i != node_id) {
          ChVector<> temp_pos = id_map.at(i)->GetZombiePos();
          all_x[i] = temp_pos.x();
          all_y[i] = temp_pos.y();

          if (step_number == 0) {
            all_prev_x[i] = all_x[i];
            all_prev_y[i] = all_y[i];
          }
          all_speed[i] =
              sqrt((all_x[i] - all_prev_x[i]) * (all_x[i] - all_prev_x[i]) +
                   (all_y[i] - all_prev_y[i]) * (all_y[i] - all_prev_y[i])) /
              heartbeat;
          all_prev_x[i] = all_x[i];
          all_prev_y[i] = all_y[i];
        }
      }

      ChVector<> veh_pos = my_vehicle.GetPos();
      all_x[node_id] = veh_pos.x();
      all_y[node_id] = veh_pos.y();
      all_speed[node_id] = my_vehicle.GetSpeed();

      // Compute critical information
      float raw_dis = sqrt((all_x[node_id] - all_x[lead_idx]) *
                               (all_x[node_id] - all_x[lead_idx]) +
                           (all_y[node_id] - all_y[lead_idx]) *
                               (all_y[node_id] - all_y[lead_idx]));
      float temp = 1 - (raw_dis * raw_dis) / (2.0 * radius * radius);
      if (temp > 1) {
        temp = 1;
      } else if (temp < -1) {
        temp = -1;
      }

      float theta = abs(acos(temp));
      act_dis = theta * radius;

      // store rendering data
      if (render_scene == 4) {
        for (int i = 0; i < num_nodes; i++) {
          if (i != node_id) {
            // body
            ChVector<> temp_pos = id_map.at(i)->GetZombiePos();
            ChQuaternion<> temp_rot = id_map.at(i)->GetZombieRot();
            ChVector<> temp_rot_euler = temp_rot.Q_to_Euler123();

            render_buffer << std::to_string(temp_pos.x()) + ",";
            render_buffer << std::to_string(temp_pos.y()) + ",";
            render_buffer << std::to_string(temp_pos.z()) + ",";
            render_buffer << std::to_string(temp_rot_euler.x()) + ",";
            render_buffer << std::to_string(temp_rot_euler.y()) + ",";
            render_buffer << std::to_string(temp_rot_euler.z()) + ",";

            // wheels
            for (int j = 0; j < 4; j++) {
              ChVector<> temp_wheel_pos = id_map.at(i)->GetZombieWheelPos(j);
              ChQuaternion<> temp_wheel_rot =
                  id_map.at(i)->GetZombieWheelRot(j);
              ChVector<> temp_wheel_rot_euler = temp_wheel_rot.Q_to_Euler123();

              render_buffer << std::to_string(temp_wheel_pos.x()) + ",";
              render_buffer << std::to_string(temp_wheel_pos.y()) + ",";
              render_buffer << std::to_string(temp_wheel_pos.z()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.x()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.y()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.z()) + ",";
            }
          } else {
            // ego body
            ChVector<> temp_pos = my_vehicle.GetChassis()->GetPos();
            ChQuaternion<> temp_rot = my_vehicle.GetChassis()->GetRot();
            ChVector<> temp_rot_euler = temp_rot.Q_to_Euler123();

            render_buffer << std::to_string(temp_pos.x()) + ",";
            render_buffer << std::to_string(temp_pos.y()) + ",";
            render_buffer << std::to_string(temp_pos.z()) + ",";
            render_buffer << std::to_string(temp_rot_euler.x()) + ",";
            render_buffer << std::to_string(temp_rot_euler.y()) + ",";
            render_buffer << std::to_string(temp_rot_euler.z()) + ",";

            // ego_wheels
            for (int j = 0; j < 4; j++) {
              std::shared_ptr<ChWheel> temp_wheel;
              if (j % 2 == 0) {
                temp_wheel = my_vehicle.GetWheel(int(j / 2), VehicleSide::LEFT);
              } else {
                temp_wheel =
                    my_vehicle.GetWheel(int(j / 2), VehicleSide::RIGHT);
              }

              ChVector<> temp_wheel_pos = temp_wheel->GetState().pos;
              ChQuaternion<> temp_wheel_rot = temp_wheel->GetState().rot;
              ChVector<> temp_wheel_rot_euler = temp_wheel_rot.Q_to_Euler123();

              render_buffer << std::to_string(temp_wheel_pos.x()) + ",";
              render_buffer << std::to_string(temp_wheel_pos.y()) + ",";
              render_buffer << std::to_string(temp_wheel_pos.z()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.x()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.y()) + ",";
              render_buffer << std::to_string(temp_wheel_rot_euler.z()) + ",";
            }
          }
        }

        render_buffer << std::endl;

        SynLog() << ("Writing to render file...") << "\n";
        render_filestream << render_buffer.rdbuf();
        render_buffer.str("");
      }
    }

    // Render scene and output POV-Ray data
    if (render_scene != 0 && (step_number % 20) == 0) {
      manager->Update();
    }

    // Get driver inputs
    DriverInputs driver_inputs = driver.GetInputs();
    if (drive_type == 1) {
      driver_inputs.m_throttle = SDLDriver.GetThrottle();
      driver_inputs.m_steering = SDLDriver.GetSteering();
      driver_inputs.m_braking = SDLDriver.GetBraking();
    }

    if (drive_type == 2) {
      driver_inputs.m_throttle = throttles[step_number];
      driver_inputs.m_steering = steerings[step_number];
      driver_inputs.m_braking = brakes[step_number];
    }

    if (record == 1) {
      record_buffer << std::to_string(driver_inputs.m_throttle) + ",";
      record_buffer << std::to_string(driver_inputs.m_braking) + ",";
      record_buffer << std::to_string(driver_inputs.m_steering);
      record_buffer << std::endl;
      if (step_number % 5000 == 0) {
        SynLog() << ("Writing to record file...") << "\n";
        record_filestream << record_buffer.rdbuf();
        record_buffer.str("");
      }
    }

    // record ring experiment output data
    if (output == 1) {
      if (step_number % 10 == 0) {
        output_buffer << time << ",";
        for (int j = 0; j < num_nodes; j++) {
          output_buffer << std::to_string(all_x[j]) + ",";
          output_buffer << std::to_string(all_y[j]) + ",";
          output_buffer << std::to_string(all_speed[j]);
          if (j != num_nodes - 1) {
            output_buffer << ",";
          }
        }
        output_buffer << std::endl;
      }

      if (step_number % 10000 == 0) {
        SynLog() << ("Writing to output file...") << "\n";
        output_filestream << output_buffer.rdbuf();
        output_buffer.str("");
      }
    }

    // Update modules (process inputs from other modules)
    syn_manager.Synchronize(time);
    driver.Synchronize(time, step_size, act_dis, all_speed[lead_idx]);
    // driver.Synchronize(time);
    terrain.Synchronize(time);
    my_vehicle.Synchronize(time, driver_inputs, terrain);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    terrain.Advance(step_size);
    my_vehicle.Advance(step_size);

    if (drive_type == 1 && (step_number % 20) == 0) {
      if (SDLDriver.Synchronize() == 1) {
        break;
      }
    }

    // std::cout << my_sedan.GetVehicle().GetPos() << std::endl;

    // Increment frame number
    step_number++;

    if (!syn_manager.IsOk()) {
      syn_manager.QuitSimulation();
    }
  }

  return 0;
}