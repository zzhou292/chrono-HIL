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
// Authors: Jason Zhou
// =============================================================================
//
// This is a class which encapsulates the 8dof vehicle model
//
// =============================================================================
#include "Ch_8DOF_vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;

Ch_8DOF_vehicle::Ch_8DOF_vehicle(std::string rom_json, float z_plane,
                                 float step_size, bool vis) {

  preload_vis_mesh = false;

  rom_z_plane = z_plane;
  enable_vis = vis;

  rapidjson::Document d;
  vehicle::ReadFileJSON(rom_json, d);

  if (d.HasParseError()) {
    std::cout << "Error with 8DOF Json file:" << std::endl
              << d.GetParseError() << std::endl;
  }

  vehicle_dyn_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Dynamic_File"].GetString();

  tire_json = std::string(STRINGIFY(HIL_DATA_DIR)) + d["Tire_File"].GetString();
  engine_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Engine_File"].GetString();

  m_chassis_mesh =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Chassis_Mesh"].GetString();
  m_wheel_mesh =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Wheel_Mesh"].GetString();

  // 1 -> LF
  // 2 -> RF
  // 3 -> LR
  // 4 -> RR
  wheels_offset_pos[0] = vehicle::ReadVectorJSON(d["Wheel_Pos_0"]);
  wheels_offset_pos[1] = vehicle::ReadVectorJSON(d["Wheel_Pos_1"]);
  wheels_offset_pos[2] = vehicle::ReadVectorJSON(d["Wheel_Pos_2"]);
  wheels_offset_pos[3] = vehicle::ReadVectorJSON(d["Wheel_Pos_3"]);

  // initialization of vehicle's tire rotation angle on Y direction
  prev_tire_rotation[0] = 0.0;
  prev_tire_rotation[1] = 0.0;
  prev_tire_rotation[2] = 0.0;
  prev_tire_rotation[3] = 0.0;

  wheels_offset_rot[0].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_0"]));
  wheels_offset_rot[1].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_1"]));
  wheels_offset_rot[2].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_2"]));
  wheels_offset_rot[3].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_3"]));

  rapidjson::Document d_dyn;
  vehicle::ReadFileJSON(vehicle_dyn_json, d_dyn);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Dyn Json file:" << std::endl
              << d_dyn.GetParseError() << std::endl;
  }

  rapidjson::Document d_eng;
  vehicle::ReadFileJSON(engine_json, d_eng);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Engine Json file:" << std::endl
              << d_eng.GetParseError() << std::endl;
  }

  rapidjson::Document d_tire;
  vehicle::ReadFileJSON(tire_json, d_tire);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Tire Json file:" << std::endl
              << d_tire.GetParseError() << std::endl;
  }

  // Set vehicle parameters from JSON file
  setVehParamsJSON(veh1_param, d_dyn);
  setEngParamsJSON(veh1_param, d_eng);

  vehInit(veh1_st, veh1_param, step_size);

  // set the tire parameters from a JSON file
  setTireParamsJSON(tire_param, d_tire);

  // now we initialize each of our parameters
  tireInit(tire_param, step_size);
}

Ch_8DOF_vehicle::Ch_8DOF_vehicle(
    std::string rom_json, float z_plane, float step_size,
    std::shared_ptr<ChTriangleMeshConnected> chassis_mesh,
    std::shared_ptr<ChTriangleMeshConnected> wheel_mesh_l,
    std::shared_ptr<ChTriangleMeshConnected> wheel_mesh_r, bool vis) {

  preload_vis_mesh = true;
  m_chassis_trimesh = chassis_mesh;
  m_wheel_trimesh_l = wheel_mesh_l;
  m_wheel_trimesh_r = wheel_mesh_r;

  rom_z_plane = z_plane;
  enable_vis = vis;

  rapidjson::Document d;
  vehicle::ReadFileJSON(rom_json, d);

  if (d.HasParseError()) {
    std::cout << "Error with 8DOF Json file:" << std::endl
              << d.GetParseError() << std::endl;
  }

  vehicle_dyn_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Dynamic_File"].GetString();

  tire_json = std::string(STRINGIFY(HIL_DATA_DIR)) + d["Tire_File"].GetString();
  engine_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Engine_File"].GetString();

  m_wheel_mesh =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Wheel_Mesh"].GetString();

  // 1 -> LF
  // 2 -> RF
  // 3 -> LR
  // 4 -> RR
  wheels_offset_pos[0] = vehicle::ReadVectorJSON(d["Wheel_Pos_0"]);
  wheels_offset_pos[1] = vehicle::ReadVectorJSON(d["Wheel_Pos_1"]);
  wheels_offset_pos[2] = vehicle::ReadVectorJSON(d["Wheel_Pos_2"]);
  wheels_offset_pos[3] = vehicle::ReadVectorJSON(d["Wheel_Pos_3"]);

  // initialization of vehicle's tire rotation angle on Y direction
  prev_tire_rotation[0] = 0.0;
  prev_tire_rotation[1] = 0.0;
  prev_tire_rotation[2] = 0.0;
  prev_tire_rotation[3] = 0.0;

  wheels_offset_rot[0].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_0"]));
  wheels_offset_rot[1].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_1"]));
  wheels_offset_rot[2].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_2"]));
  wheels_offset_rot[3].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_3"]));

  rapidjson::Document d_dyn;
  vehicle::ReadFileJSON(vehicle_dyn_json, d_dyn);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Dyn Json file:" << std::endl
              << d_dyn.GetParseError() << std::endl;
  }

  rapidjson::Document d_eng;
  vehicle::ReadFileJSON(engine_json, d_eng);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Engine Json file:" << std::endl
              << d_eng.GetParseError() << std::endl;
  }

  rapidjson::Document d_tire;
  vehicle::ReadFileJSON(tire_json, d_tire);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Tire Json file:" << std::endl
              << d_tire.GetParseError() << std::endl;
  }

  // Set vehicle parameters from JSON file
  setVehParamsJSON(veh1_param, d_dyn);
  setEngParamsJSON(veh1_param, d_eng);

  vehInit(veh1_st, veh1_param, step_size);

  // set the tire parameters from a JSON file
  setTireParamsJSON(tire_param, d_tire);

  // now we initialize each of our parameters
  tireInit(tire_param, step_size);
}

void Ch_8DOF_vehicle::Initialize(ChSystem *sys) {

  if (enable_vis) {

    chassis_body = chrono_types::make_shared<ChBodyAuxRef>();

    chassis_body->SetCollide(false);

    chassis_body->SetBodyFixed(true);

    // initializing visualization assets for chassis
    if (preload_vis_mesh == true) {
      auto chassis_trimesh_shape =
          chrono_types::make_shared<ChTriangleMeshShape>();
      chassis_trimesh_shape->SetMesh(m_chassis_trimesh);
      chassis_trimesh_shape->SetMutable(false);

      chassis_body->AddVisualShape(chassis_trimesh_shape);

      sys->AddBody(chassis_body);
    } else {
      auto chassis_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
      chassis_mmesh->LoadWavefrontMesh(m_chassis_mesh, false, true);

      auto chassis_trimesh_shape =
          chrono_types::make_shared<ChTriangleMeshShape>();
      chassis_trimesh_shape->SetMesh(chassis_mmesh);
      chassis_trimesh_shape->SetMutable(false);

      chassis_body->AddVisualShape(chassis_trimesh_shape);
    }

    sys->AddBody(chassis_body);

    // Express relative frame in global
    ChFrame<> X_LF = chassis_body->GetFrame_REF_to_abs() *
                     ChFrame<>(wheels_offset_pos[0], wheels_offset_rot[0]);
    ChFrame<> X_RF = chassis_body->GetFrame_REF_to_abs() *
                     ChFrame<>(wheels_offset_pos[1], wheels_offset_rot[1]);
    ChFrame<> X_LR = chassis_body->GetFrame_REF_to_abs() *
                     ChFrame<>(wheels_offset_pos[2], wheels_offset_rot[2]);
    ChFrame<> X_RR = chassis_body->GetFrame_REF_to_abs() *
                     ChFrame<>(wheels_offset_pos[3], wheels_offset_rot[3]);

    // initializing visualization assets for wheels
    for (int i = 0; i < 4; i++) {

      wheels_body[i] = chrono_types::make_shared<ChBodyAuxRef>();
      wheels_body[i]->SetCollide(false);

      wheels_body[i]->SetBodyFixed(true);

      if (enable_vis) {
        if (preload_vis_mesh) {
          if (i % 2 == 0) {
            auto wheel_trimesh_shape =
                chrono_types::make_shared<ChTriangleMeshShape>();
            wheel_trimesh_shape->SetMesh(m_wheel_trimesh_l);
            wheel_trimesh_shape->SetMutable(false);
            wheels_body[i]->AddVisualShape(wheel_trimesh_shape);
          } else {
            auto wheel_trimesh_shape =
                chrono_types::make_shared<ChTriangleMeshShape>();
            wheel_trimesh_shape->SetMesh(m_wheel_trimesh_r);
            wheel_trimesh_shape->SetMutable(false);
            wheels_body[i]->AddVisualShape(wheel_trimesh_shape);
          }

        } else {
          auto wheel_mmesh =
              chrono_types::make_shared<ChTriangleMeshConnected>();
          wheel_mmesh->LoadWavefrontMesh(m_wheel_mesh, false, true);

          // transform all wheel rotations, to the meshes
          wheel_mmesh->Transform(ChVector<>(0.0, 0.0, 0.0),
                                 wheels_offset_rot[i]);

          auto wheel_trimesh_shape =
              chrono_types::make_shared<ChTriangleMeshShape>();
          wheel_trimesh_shape->SetMesh(wheel_mmesh);
          wheel_trimesh_shape->SetMutable(false);
          wheels_body[i]->AddVisualShape(wheel_trimesh_shape);
        }
      }

      if (i == 0) {
        wheels_body[i]->SetPos(X_LF.GetPos());
        wheels_body[i]->SetRot(X_LF.GetRot());
      }

      if (i == 1) {
        wheels_body[i]->SetPos(X_RF.GetPos());
        wheels_body[i]->SetRot(X_RF.GetRot());
      }
      if (i == 2) {
        wheels_body[i]->SetPos(X_LR.GetPos());
        wheels_body[i]->SetRot(X_LR.GetRot());
      }
      if (i == 3) {
        wheels_body[i]->SetPos(X_RR.GetPos());
        wheels_body[i]->SetRot(X_RR.GetRot());
      }

      sys->AddBody(wheels_body[i]);
    }
  }
}

void Ch_8DOF_vehicle::Advance(float time, DriverInputs inputs) {

  // limitation boundary
  std::vector<double> controls(4, 0);

  controls[0] = time;
  controls[1] = inputs.m_steering;
  controls[2] = inputs.m_throttle;
  controls[3] = inputs.m_braking;

  m_inputs.m_steering = inputs.m_steering;
  m_inputs.m_throttle = inputs.m_throttle;
  m_inputs.m_braking = inputs.m_braking;

  // transform velocities and other needed quantities from
  // vehicle frame to tire frame
  vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                     veh1_param, controls);

  // advance our 4 tires
  tireAdv(tirelf_st, tire_param, veh1_st, veh1_param, controls, 0);
  tireAdv(tirerf_st, tire_param, veh1_st, veh1_param, controls, 1);

  // modify controls for our rear tires as they dont take steering
  std::vector<double> mod_controls = {controls[0], 0, controls[2], controls[3]};
  tireAdv(tirelr_st, tire_param, veh1_st, veh1_param, mod_controls, 2);
  tireAdv(tirerr_st, tire_param, veh1_st, veh1_param, mod_controls, 3);

  // transform tire forces to vehicle frame
  tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh1_st,
                     veh1_param, controls);

  // copy the useful stuff that needs to be passed onto the vehicle
  std::vector<double> fx = {tirelf_st.m_fx, tirerf_st.m_fx, tirelr_st.m_fx,
                            tirerr_st.m_fx};
  std::vector<double> fy = {tirelf_st.m_fy, tirerf_st.m_fy, tirelr_st.m_fy,
                            tirerr_st.m_fy};
  double huf = tirelf_st.m_rStat;
  double hur = tirerr_st.m_rStat;

  vehAdv(veh1_st, veh1_param, fx, fy, huf, hur);

  if (enable_vis) {
    chassis_body->SetPos(this->GetPos());

    chassis_body->SetRot(this->GetRot());

    ChFrame<> chassis_body_fr = ChFrame<>(this->GetPos(), this->GetRot());

    ChFrame<> X_LF =
        chassis_body_fr * ChFrame<>(wheels_offset_pos[0], wheels_offset_rot[0]);
    ChFrame<> X_RF =
        chassis_body_fr * ChFrame<>(wheels_offset_pos[1], wheels_offset_rot[1]);
    ChFrame<> X_LR =
        chassis_body_fr * ChFrame<>(wheels_offset_pos[2], wheels_offset_rot[2]);
    ChFrame<> X_RR =
        chassis_body_fr * ChFrame<>(wheels_offset_pos[3], wheels_offset_rot[3]);

    for (int i = 0; i < 4; i++) {
      // 1 - vehicle rotation
      // step one to obtain vehicle chassis orientation and wheel offset
      ChQuaternion<> rot_operator = chassis_body_fr.GetRot();

      // 2 - steer offset
      // step two only applies to front wheels which need to take care of
      // steering
      if (i == 0 || i == 1) {
        ChQuaternion<> temp = ChQuaternion<>(1, 0, 0, 0);
        temp.Q_from_AngZ(inputs.m_steering * veh1_param.m_maxSteer);
        rot_operator = rot_operator * temp;
      }

      // 3 - take into tire rotation
      // apply to all tires
      ChQuaternion<> temp(1, 0, 0, 0);
      temp.Q_from_AngY(prev_tire_rotation[i] +
                       veh1_param.m_step * tirelf_st.m_omega);
      prev_tire_rotation[i] =
          prev_tire_rotation[i] + veh1_param.m_step * tirelf_st.m_omega;
      if (prev_tire_rotation[i] > C_2PI) {
        prev_tire_rotation[i] = 0.f;
      }
      rot_operator = rot_operator * temp;

      // final rotation step
      if (i == 0) {
        wheels_body[i]->SetPos(X_LF.GetPos());
        wheels_body[i]->SetRot(rot_operator);
      }

      if (i == 1) {
        wheels_body[i]->SetPos(X_RF.GetPos());
        wheels_body[i]->SetRot(rot_operator);
      }
      if (i == 2) {
        wheels_body[i]->SetPos(X_LR.GetPos());
        wheels_body[i]->SetRot(rot_operator);
      }
      if (i == 3) {
        wheels_body[i]->SetPos(X_RR.GetPos());
        wheels_body[i]->SetRot(rot_operator);
      }
    }
  }
}

ChVector<> Ch_8DOF_vehicle::GetPos() {
  return ChVector<>(veh1_st.m_x, veh1_st.m_y, rom_z_plane);
}

ChQuaternion<> Ch_8DOF_vehicle::GetRot() {
  ChQuaternion<> ret_rot = ChQuaternion<>(1, 0, 0, 0);
  ret_rot.Q_from_Euler123(ChVector<>(veh1_st.m_phi, 0, veh1_st.m_psi));
  return ret_rot;
}

ChVector<> Ch_8DOF_vehicle::GetVel() {
  return ChVector<>(veh1_st.m_u, veh1_st.m_v, 0.0);
}

float Ch_8DOF_vehicle::GetStepSize() { return veh1_param.m_step; }

std::shared_ptr<ChBodyAuxRef> Ch_8DOF_vehicle::GetChassisBody() {
  return chassis_body;
}

void Ch_8DOF_vehicle::SetInitPos(ChVector<> init_pos) {
  veh1_st.m_x = init_pos.x();
  veh1_st.m_y = init_pos.y();
}

void Ch_8DOF_vehicle::SetInitRot(float yaw) { veh1_st.m_psi = yaw; }

float Ch_8DOF_vehicle::GetTireRotation(int idx) {
  return prev_tire_rotation[idx];
}

DriverInputs Ch_8DOF_vehicle::GetDriverInputs() { return m_inputs; }