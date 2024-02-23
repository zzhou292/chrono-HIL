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
// =============================================================================
//
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include <chrono>
#include <functional>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"

#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/controller/SynControllerFunctions.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_hil/timer/ChRealtimeCumulative.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include "chrono_hil/driver/ChIDM_Follower.h"
#include "chrono_hil/driver/ChLidarWaypointDriver.h"
#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_hil/ROM/syn/Ch_8DOF_zombie.h"

#include "chrono_hil/network/tcp/ChTCPClient.h"
#include "chrono_hil/network/tcp/ChTCPServer.h"

// =============================================================================

// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::hil;
// =============================================================================
const double RADS_2_RPM = 30 / CH_C_PI;
const double MS_2_MPH = 2.2369;
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
void AddCommandLineOptions(ChCLI &cli);

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

// =====================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of tire model
TireModelType tire_model = TireModelType::TMEASY;

// Type of vehicle
enum VehicleType { HMMWV, SEDAN, CITYBUS, AUDI, SUV, TRUCK };

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 2e-3;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 4e-2;

bool use_fullscreen = false;

ChVector<> simulation_center = {826.734, -37.97, -64.8};

double loading_radius = 1000;

float resolution_x = 1920;
float resolution_y = 1080;
int supersample = 1;
std::string joystick_filename;
int driver_type = 0; // 0 for data driven, 1 for SDL driven, 2 for stream driven
bool render = false;
int refresh_rate = 35;
float cam_offset = 0.54;

int port_id = 6078;

std::string demo_data_path = std::string(STRINGIFY(HIL_DATA_DIR));

double suv_lookahead = 5.0;
double audi_tight_lookahead = 6.0;
double suv_pgain = .5;
double audi_pgain = .5;

std::vector<float> throttles;
std::vector<float> brakes;
std::vector<float> steerings;

// =============================================================================

// Forward declares for straight forward helper functions

void AddSceneMeshes(ChSystem *chsystem, RigidTerrain *terrain);

void VehicleProcessMessageCallback(
    std::shared_ptr<SynMessage> message, WheeledVehicle &vehicle,
    std::shared_ptr<SynWheeledVehicleAgent> agent,
    std::shared_ptr<ChLidarWaypointDriver> driver);

// =============================================================================

int main(int argc, char *argv[]) {

  ChCLI cli(argv[0]);

  AddCommandLineOptions(cli);
  if (!cli.Parse(argc, argv, true))
    return 0;

  const int node_id = cli.GetAsType<int>("node_id");
  const int num_nodes = cli.GetAsType<int>("num_nodes");
  const std::vector<std::string> ip_list =
      cli.GetAsType<std::vector<std::string>>("ip");
  const int record = cli.GetAsType<int>("record");
  const int output_state = cli.GetAsType<int>("output");

  std::string output_file_path =
      "./syn_output" + std::to_string(node_id) + ".csv";
  std::ofstream output_filestream = std::ofstream(output_file_path);
  std::stringstream output_buffer;

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
  SynChronoManager syn_manager(node_id, num_nodes, communicator);

  syn_manager.SetHeartbeat(heartbeat);

  // ===========================================

  // all the demo data will be in user-specified location
  SetChronoDataPath(demo_data_path);
  vehicle::SetDataPath(CHRONO_DATA_DIR + std::string("vehicle/"));
  synchrono::SetDataPath(CHRONO_DATA_DIR + std::string("synchrono/"));

  // read rom data csv
  std::vector<rom_item> rom_data;
  std::string filename = std::string(STRINGIFY(HIL_DATA_DIR)) +
                         "/Environments/SanFrancisco/rom_init/test.csv";
  std::cout << filename << std::endl;
  ReadRomInitFile(filename, rom_data);

  if (record == 2) {
    readvectors(throttles, brakes, steerings);
  }

  // --------------
  // Create vehicle
  // --------------

  std::string vehicle_filename =
      vehicle::GetDataFile("audi/json/audi_Vehicle.json");
  std::string powertrain_filename =
      vehicle::GetDataFile("audi/json/audi_SimpleMapPowertrain.json");
  std::string tire_filename =
      vehicle::GetDataFile("audi/json/audi_Pac02Tire.json");
  std::string zombie_filename =
      CHRONO_DATA_DIR + std::string("vehicle/audi/json/audi.json");

  // Initial vehicle location and orientation
  ChVector<> initLoc;
  ChQuaternion<> initRot;
  if (node_id == 0) {
    initLoc = ChVector<>(930.434, 0, -65.2);
    initRot = Q_from_AngZ(3.14 / 2);
  } else if (node_id == 1) {
    initLoc = ChVector<>(930.434, -50.87, -65.2);
    initRot = Q_from_AngZ(3.14 / 2);
  } else if (node_id == 2) {
    initLoc = ChVector<>(930.434, -150.87, -65.2);
    initRot = Q_from_AngZ(3.14 / 2);
  }

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
      tire->SetStepsize(step_size / 5.f);
      my_vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
    }
  }

  // Add vehicle as an agent and initialize SynChronoManager
  auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(
      &my_vehicle, zombie_filename);
  syn_manager.AddAgent(agent);
  syn_manager.Initialize(my_vehicle.GetSystem());

  auto attached_body = std::make_shared<ChBody>();
  my_vehicle.GetSystem()->AddBody(attached_body);
  attached_body->SetCollide(false);
  attached_body->SetBodyFixed(true);
  auto box = chrono_types::make_shared<ChBoxShape>();
  box->GetBoxGeometry().Size = ChVector<>(10000.0, 10000.0, 0.001);
  auto black_mat = chrono_types::make_shared<ChVisualMaterial>();
  black_mat->SetDiffuseColor(ChColor(0.3f, 0.3f, 0.3f));
  box->AddMaterial(black_mat);
  attached_body->AddVisualShape(box, ChFrame<>(ChVector<>(0, 0, -65.6), QUNIT));

  // --------------
  // Create systems
  // --------------

  RigidTerrain terrain(my_vehicle.GetSystem());
  AddSceneMeshes(my_vehicle.GetSystem(), &terrain);

  ChContactMaterialData minfo; // values from RigidPlane.json
  minfo.mu = 0.9;              // coefficient of friction
  minfo.cr = 0.01;             // coefficient of restitution
  minfo.Y = 2e7;               // Young's modulus
  minfo.nu = 0.3;              // Poisson ratio
  minfo.kn = 2e5;              // normal stiffness
  minfo.gn = 40.0;             // normal viscous damping
  minfo.kt = 2e5;              // tangential stiffness
  minfo.gt = 20.0;             // tangential viscous damping
  auto patch_mat = minfo.CreateMaterial(contact_method);

  ChVector<> normal = ChVector<>({0, 0, 1});
  ChVector<> up = normal.GetNormalized();
  ChVector<> lateral = Vcross(up, ChWorldFrame::Forward());
  lateral.Normalize();
  ChVector<> forward = Vcross(lateral, up);
  ChMatrix33<> rot;
  rot.Set_A_axis(forward, lateral, up);

  auto patch = terrain.AddPatch(
      patch_mat,
      ChCoordsys<>(ChVector<>({0, 0, -65.554}), rot.Get_A_quaternion()),
      10000.0, 10000.0, 2, false, 1, false);
  terrain.Initialize();

  // Create Zombies
  int num_rom = rom_data.size();

  std::string hmmwv_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/hmmwv/hmmwv_rom.json";
  std::string sedan_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/sedan/sedan_rom.json";
  std::string patrol_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/patrol/patrol_rom.json";
  std::string audi_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/rom/audi/audi_rom.json";

  std::vector<std::shared_ptr<Ch_8DOF_zombie>> zombie_vec;
  for (int i = 0; i < num_rom; i++) {
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

    auto rom_zombie =
        chrono_types::make_shared<Ch_8DOF_zombie>(rom_json, 0.0, true);
    zombie_vec.push_back(rom_zombie);
    rom_zombie->Initialize(my_vehicle.GetSystem());
    std::cout << "zombie: " << i << std::endl;
  }

  // --------------
  // Create cam
  // --------------
  // add a sensor manager
  std::shared_ptr<ChSensorManager> manager;

  if (node_id == 0) {
    /*
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
    auto audi_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
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

    auto lwm_mirror_shape =
    chrono_types::make_shared<ChVisualShapeTriangleMesh>();
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
*/
    manager =
        chrono_types::make_shared<ChSensorManager>(my_vehicle.GetSystem());
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP; // GRADIENT
    b.env_tex = GetChronoDataFile("/Environments/sky_2_4k.hdr");
    manager->scene->SetBackground(b);
    float brightness = 1.5f;
    manager->scene->AddPointLight({0, 0, 10000},
                                  {brightness, brightness, brightness}, 100000);
    manager->scene->SetAmbientLight({.1, .1, .1});
    manager->scene->SetSceneEpsilon(1e-3);
    manager->scene->EnableDynamicOrigin(true);
    manager->scene->SetOriginOffsetThreshold(500.f);

    /*
        // camera at driver's eye location for Audi
        auto driver_cam = chrono_types::make_shared<ChCameraSensor>(
            my_vehicle.GetChassisBody(), // body camera is attached to
                                         // my_vehicle.GetChassisBody()
            25,                          // update rate in Hz
            chrono::ChFrame<double>({-6.0, 0.0, 2.5},
                                    Q_from_AngAxis(0.0, {0, 1, 0})), // offset
       pose 640,                                                     // image
       width 320,                                                     // image
       height 3.14 / 1.5,                                              // fov
            1);

        driver_cam->SetName("DriverCam");
        driver_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
            1920, 1080, "Camera1", false));
        //
       driver_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        //
       driver_cam->PushFilter(chrono_types::make_shared<ChFilterSave>("rom/"));
        */

    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        my_vehicle.GetChassisBody(), // body lidar is attached to
        25,                          // scanning rate in Hz
        chrono::ChFrame<double>({0.0, 0.0, 3.5},
                                Q_from_AngAxis(0.0, {0, 1, 0})), // offset pose
        1280,                 // number of horizontal samples
        720,                  // numberof vertical channels
        (float)(2 * CH_C_PI), // horizontal field of view
        (float)CH_C_PI / 12, (float)-CH_C_PI / 3,
        200.0f // vertical field of view
    );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.02f);

    // Create a filter graph for post-processing the data from the lidar
    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    // Render the point cloud
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(
        1280, 720, 1.0, "Lidar Point Cloud"));
    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());

    // manager->AddSensor(driver_cam);
    manager->AddSensor(lidar);
  }

  // --------------
  // Create control
  // --------------

  ChSDLInterface SDLDriver;

  if (node_id == 1) {
    SDLDriver.Initialize();

    SDLDriver.SetJoystickConfigFile(
        std::string(STRINGIFY(HIL_DATA_DIR)) +
        std::string("/joystick/controller_G29.json"));
  }

  std::string path_file("paths/output.txt");

  auto path = ChBezierCurve::read(demo_data_path + "/paths/output.txt", true);

  ChPathFollowerDriver driver(my_vehicle, path, "my_path", 6.0);
  driver.GetSteeringController().SetLookAheadDistance(5);
  driver.GetSteeringController().SetGains(0.2, 0, 0);
  driver.GetSpeedController().SetGains(0.4, 0, 0);
  driver.Initialize();

  std::vector<double> followerParam;
  followerParam.push_back(8.9408);
  followerParam.push_back(0.2);
  followerParam.push_back(6.0);
  followerParam.push_back(3.0);
  followerParam.push_back(2.1);
  followerParam.push_back(4.0);
  followerParam.push_back(4.86);
  std::string steer_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SteeringController.json";
  std::string speed_controller =
      std::string(STRINGIFY(HIL_DATA_DIR)) + "/ring/SpeedController.json";
  ChIDMFollower idm_driver(my_vehicle, steer_controller, speed_controller, path,
                           "road", 20.0 * MPH_TO_MS, followerParam);
  idm_driver.Initialize();

  // Create TCP Tunnel
  ChTCPClient chrono_0("127.0.0.1", 1204,
                       num_rom * 11); // create a TCP client for rank 0
  ChTCPClient chrono_1("127.0.0.1", 1205,
                       num_rom * 11); // create a TCP client for rank 1
  ChTCPClient chrono_2("127.0.0.1", 1206,
                       num_rom * 11); // create a TCP client for rank 2

  if (node_id == 0) {
    chrono_0.Initialize(); // initialize TCP connection for rank 1
  } else if (node_id == 1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        5000)); // wait for 2000 seconds to prevent duplicated connection
    chrono_1.Initialize(); // initialize TCP connection for rank 2
  } else if (node_id == 2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        10000)); // wait for 2000 seconds to prevent duplicated connection
    chrono_2.Initialize(); // initialize TCP connection for rank 2
  }

  double sim_time = 0.f;
  double last_time = 0.f;
  int step_number = 0;
  ChVector<> ego_cur_pos;
  ChVector<> ego_prev_pos;
  float ego_spd;

  // obtain and initiate all zombie instances
  std::map<AgentKey, std::shared_ptr<SynAgent>> zombie_map;
  std::map<int, std::shared_ptr<SynWheeledVehicleAgent>> id_map;

  std::string record_file_path = "./record.csv";
  std::ofstream record_filestream = std::ofstream(record_file_path);
  std::stringstream record_buffer;

  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

  while (syn_manager.IsOk()) {
    sim_time = my_vehicle.GetSystem()->GetChTime();

    // Get driver inputs
    DriverInputs driver_inputs;

    if (node_id == 1) {
      if (record == 2) {
        driver_inputs.m_throttle = throttles[step_number];
        driver_inputs.m_steering = steerings[step_number];
        driver_inputs.m_braking = brakes[step_number];

      } else {
        driver_inputs.m_throttle = SDLDriver.GetThrottle();
        driver_inputs.m_steering = SDLDriver.GetSteering();
        driver_inputs.m_braking = SDLDriver.GetBraking();
      }

    } else if (node_id == 0) {
      driver_inputs = driver.GetInputs();
    } else if (node_id == 2) {
      driver_inputs = idm_driver.GetInputs();
    }

    if (node_id == 1) {
      if (SDLDriver.Synchronize() == 1)
        break;
    }

    // obtain map
    if (step_number == 0 && node_id == 2) {
      zombie_map = syn_manager.GetZombies();
      for (std::map<AgentKey, std::shared_ptr<SynAgent>>::iterator it =
               zombie_map.begin();
           it != zombie_map.end(); ++it) {
        std::shared_ptr<SynAgent> temp_ptr = it->second;
        std::shared_ptr<SynWheeledVehicleAgent> converted_ptr =
            std::dynamic_pointer_cast<SynWheeledVehicleAgent>(temp_ptr);
        id_map.insert(std::make_pair(it->first.GetNodeID(), converted_ptr));
      }
    }

    // update necessary zombie info for IDM
    if (step_number % int(heartbeat / step_size) == 0 && node_id == 2) {
      ego_cur_pos = id_map.at(1)->GetZombiePos();
      if (step_number == 0) {
        ego_prev_pos = ego_cur_pos;
      }
      ego_spd = (ego_cur_pos - ego_prev_pos).Length() / heartbeat;
      ego_prev_pos = ego_cur_pos;
    }

    // Update modules (process inputs from other modules)
    if (node_id == 0)
      driver.Synchronize(sim_time);
    else if (node_id == 2)
      idm_driver.Synchronize(
          sim_time, step_size,
          (my_vehicle.GetChassis()->GetPos() - ego_cur_pos).Length(), ego_spd);
    my_vehicle.Synchronize(sim_time, driver_inputs, terrain);
    terrain.Synchronize(sim_time);
    syn_manager.Synchronize(sim_time);

    // Advance simulation for one timestep for all modules
    if (node_id == 0) {
      driver.Advance(step_size);
    } else if (node_id == 2) {
      idm_driver.Advance(step_size);
    }

    my_vehicle.Advance(step_size);
    terrain.Advance(step_size);

    if (node_id == 0 && step_number % int(heartbeat / step_size) == 0) {
      manager->Update();
    }

    // Synchronize
    if (step_number == 0) {
      std::vector<float> data_to_send;
      data_to_send.push_back(1.0);
      data_to_send.push_back(1.0);
      data_to_send.push_back(1.0);
      if (node_id == 0) {
        chrono_0.Write(data_to_send);
      } else if (node_id == 1) {
        chrono_1.Write(data_to_send);
      } else if (node_id == 2) {
        chrono_2.Write(data_to_send);
      }
    }

    // Update zombies
    // ==================================================
    // TCP Synchronization Section
    // ==================================================
    // read data from rom distributor 1
    if (step_number % 20 == 0) {
      if (node_id == 0) {
        chrono_0.Read();
      } else if (node_id == 1) {
        chrono_1.Read();
      } else if (node_id == 2) {
        chrono_2.Read();
      }

      std::vector<float> recv_data;
      if (node_id == 0) {
        recv_data = chrono_0.GetRecvData();
      } else if (node_id == 1) {
        recv_data = chrono_1.GetRecvData();
      } else if (node_id == 2) {
        recv_data = chrono_2.GetRecvData();
      }

      for (int i = 0; i < num_rom; i++) {
        zombie_vec[i]->Update(
            ChVector<>(recv_data[0 + i * 11], recv_data[1 + i * 11],
                       recv_data[2 + i * 11]),
            ChVector<>(recv_data[3 + i * 11], recv_data[4 + i * 11],
                       recv_data[5 + i * 11]),
            recv_data[6 + i * 11], recv_data[7 + i * 11], recv_data[8 + i * 11],
            recv_data[9 + i * 11], recv_data[10 + i * 11]);
      }

      // send data to distributor 1
      std::vector<float> data_to_send;
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().x());
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().y());
      data_to_send.push_back(my_vehicle.GetChassis()->GetPos().z());

      if (node_id == 0) {
        chrono_0.Write(data_to_send);
      } else if (node_id == 1) {
        chrono_1.Write(data_to_send);
      } else if (node_id == 2) {
        chrono_2.Write(data_to_send);
      }
    }

    if (output_state == 1 && step_number % 20 == 0) {
      output_buffer << sim_time << ",";
      output_buffer << my_vehicle.GetSpeed() << ",";
      ChQuaternion<> temp_qua = my_vehicle.GetRot();
      ChVector<> temp_vec = temp_qua.Q_to_Euler123();
      output_buffer << temp_vec.x() << ",";
      output_buffer << temp_vec.y() << ",";
      output_buffer << temp_vec.z() << ",";
      output_buffer << driver_inputs.m_throttle << ",";
      output_buffer << driver_inputs.m_braking << ",";
      output_buffer << driver_inputs.m_steering << ",";
      output_buffer << my_vehicle.GetPowertrain()->GetMotorSpeed() << ",";
      output_buffer << my_vehicle.GetPowertrain()->GetCurrentTransmissionGear();
      output_buffer << std::endl;
      if (step_number % 10000 == 0) {
        std::cout << "Writing to output file..." << std::endl;
        output_filestream << output_buffer.rdbuf();
        output_buffer.str("");
      }
    }

    // Increment frame number
    step_number++;

    // Log clock time
    if (step_number % 500 == 0) {
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> wall_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                    start);

      SynLog() << (wall_time.count()) / (sim_time - last_time) << "\n";
      last_time = sim_time;
      start = std::chrono::high_resolution_clock::now();
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
  }
  // Properly shuts down other ranks when one rank ends early
  syn_manager.QuitSimulation();
  return 0;
}

void AddSceneMeshes(ChSystem *chsystem, RigidTerrain *terrain) {
  // load all meshes in input file, using instancing where possible
  std::string base_path =
      GetChronoDataFile("/Environments/SanFrancisco/components_new/");
  std::string input_file = base_path + "instance_map_03.csv";
  // std::string input_file = base_path + "instance1280_map_roads_only.csv";

  std::ifstream infile(input_file);
  if (!infile.is_open())
    throw std::runtime_error("Could not open file " + input_file);
  std::string line, col;
  std::vector<std::string> result;

  std::unordered_map<std::string, std::shared_ptr<ChTriangleMeshConnected>>
      mesh_map;

  auto mesh_body = chrono_types::make_shared<ChBody>();
  mesh_body->SetBodyFixed(true);
  mesh_body->SetCollide(false);
  chsystem->Add(mesh_body);

  int meshes_added = 0;
  int mesh_offset = 0;
  int num_meshes = 20000;

  if (infile.good()) {
    int mesh_count = 0;
    int mesh_limit = mesh_offset + num_meshes;

    while (std::getline(infile, line) && mesh_count < mesh_limit) {

      if (mesh_count < mesh_offset) {
        mesh_count++;
      } else {
        mesh_count++;
        result.clear();
        std::stringstream ss(line);
        while (std::getline(ss, col, ',')) {
          result.push_back(col);
        }
        std::string mesh_name = result[0];
        std::string mesh_obj = base_path + result[1] + ".obj";

        // std::cout << mesh_name << std::endl;
        if (mesh_name.find("EmissionOn") ==
            std::string::npos) { // exlude items with
                                 // emission on

          if (true || mesh_name.find("Road") != std::string::npos) {
            ChVector<double> pos = {std::stod(result[2]), std::stod(result[3]),
                                    std::stod(result[4])};

            if ((pos - simulation_center).Length() < loading_radius) {
              // check if mesh is in map
              bool instance_found = false;
              std::shared_ptr<ChTriangleMeshConnected> mmesh;
              if (mesh_map.find(mesh_obj) != mesh_map.end()) {
                mmesh = mesh_map[mesh_obj];
                instance_found = true;
              } else {
                mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
                mmesh->LoadWavefrontMesh(mesh_obj, false, true);
                mesh_map[mesh_obj] = mmesh;
              }

              ChQuaternion<double> rot = {
                  std::stod(result[5]), std::stod(result[6]),
                  std::stod(result[7]), std::stod(result[8])};
              ChVector<double> scale = {std::stod(result[9]),
                                        std::stod(result[10]),
                                        std::stod(result[11])};

              // if not road, only add visualization with new pos,rot,scale
              auto trimesh_shape =
                  chrono_types::make_shared<ChVisualShapeTriangleMesh>();
              trimesh_shape->SetMesh(mmesh);
              trimesh_shape->SetName(mesh_name);
              trimesh_shape->SetScale(scale);
              trimesh_shape->SetMutable(false);

              mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(pos, rot));

              meshes_added++;
            }
          }
        }
      }
    }
    std::cout << "Total meshes: " << meshes_added
              << " | Unique meshes: " << mesh_map.size() << std::endl;
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

void AddCommandLineOptions(ChCLI &cli) {
  // DDS Specific
  cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
  cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
  cli.AddOption<std::vector<std::string>>(
      "DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
  cli.AddOption<int>("Simulation", "record", "record driver input",
                     "0"); // record 0 - normal
                           // record 1 - record input
                           // record 2 - replay
  cli.AddOption<int>("Simulation", "output", "output vehicle state", "0");
}
