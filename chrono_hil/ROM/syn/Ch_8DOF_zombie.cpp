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
//
// This is a class which encapsulates the 8dof vehicle model
//
// =============================================================================
#include "Ch_8DOF_zombie.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;

Ch_8DOF_zombie::Ch_8DOF_zombie(std::string rom_json, float z_plane, bool vis) {

  rom_z_plane = z_plane;
  enable_vis = vis;

  rapidjson::Document d;
  vehicle::ReadFileJSON(rom_json, d);

  if (d.HasParseError()) {
    std::cout << "Error with 8DOF Json file:" << std::endl
              << d.GetParseError() << std::endl;
  }

  std::string vehicle_dyn_json =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Dynamic_File"].GetString();

  rapidjson::Document d_dyn;
  vehicle::ReadFileJSON(vehicle_dyn_json, d_dyn);

  if (d_dyn.HasParseError()) {
    std::cout << "Error with 8DOF Dyn Json file:" << std::endl
              << d_dyn.GetParseError() << std::endl;
  }

  max_steer_angle = d_dyn["maxSteer"].GetDouble();

  chassis_mesh =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Chassis_Mesh"].GetString();
  wheel_mesh =
      std::string(STRINGIFY(HIL_DATA_DIR)) + d["Wheel_Mesh"].GetString();

  // 1 -> LF
  // 2 -> RF
  // 3 -> LR
  // 4 -> RR
  wheels_offset_pos[0] = vehicle::ReadVectorJSON(d["Wheel_Pos_0"]);
  wheels_offset_pos[1] = vehicle::ReadVectorJSON(d["Wheel_Pos_1"]);
  wheels_offset_pos[2] = vehicle::ReadVectorJSON(d["Wheel_Pos_2"]);
  wheels_offset_pos[3] = vehicle::ReadVectorJSON(d["Wheel_Pos_3"]);

  wheels_offset_rot[0].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_0"]));
  wheels_offset_rot[1].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_1"]));
  wheels_offset_rot[2].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_2"]));
  wheels_offset_rot[3].Q_from_Euler123(
      vehicle::ReadVectorJSON(d["Wheel_Rot_3"]));
}

void Ch_8DOF_zombie::Update(ChVector<> pos, ChVector<> rot, float steering,
                            float tire_rot_0, float tire_rot_1,
                            float tire_rot_2, float tire_rot_3) {
  if (enable_vis) {
    chassis_body->SetPos(pos);

    chassis_body->SetRot(Q_from_Euler123(rot));

    ChFrame<> chassis_body_fr = ChFrame<>(pos, Q_from_Euler123(rot));

    tire_rotation[0] = tire_rot_0;
    tire_rotation[1] = tire_rot_1;
    tire_rotation[2] = tire_rot_2;
    tire_rotation[3] = tire_rot_3;

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
      if (i == 0 || i == 1) {
        ChQuaternion<> temp = ChQuaternion<>(1, 0, 0, 0);
        temp.Q_from_AngZ(steering * max_steer_angle);
        rot_operator = rot_operator * temp;
      }

      // 3 - take into tire rotation
      // apply to all tires
      ChQuaternion<> temp(1, 0, 0, 0);
      temp.Q_from_AngY(tire_rotation[i]);
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

void Ch_8DOF_zombie::Initialize(ChSystem *sys) {
  if (enable_vis) {

    chassis_body = chrono_types::make_shared<ChBodyAuxRef>();

    chassis_body->SetCollide(false);

    chassis_body->SetBodyFixed(true);

    auto chassis_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    chassis_mmesh->LoadWavefrontMesh(chassis_mesh, false, true);

    auto chassis_trimesh_shape =
        chrono_types::make_shared<ChTriangleMeshShape>();
    chassis_trimesh_shape->SetMesh(chassis_mmesh);
    chassis_trimesh_shape->SetMutable(false);
    chassis_body->AddVisualShape(chassis_trimesh_shape);

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

    for (int i = 0; i < 4; i++) {
      wheels_body[i] = chrono_types::make_shared<ChBodyAuxRef>();
      wheels_body[i]->SetCollide(false);

      wheels_body[i]->SetBodyFixed(true);

      if (enable_vis) {
        auto wheel_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        wheel_mmesh->LoadWavefrontMesh(wheel_mesh, false, true);

        // transform all wheel rotations, to the meshes
        wheel_mmesh->Transform(ChVector<>(0.0, 0.0, 0.0), wheels_offset_rot[i]);

        auto wheel_trimesh_shape =
            chrono_types::make_shared<ChTriangleMeshShape>();
        wheel_trimesh_shape->SetMesh(wheel_mmesh);
        wheel_trimesh_shape->SetMutable(false);

        wheels_body[i]->AddVisualShape(wheel_trimesh_shape);
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

ChVector<> Ch_8DOF_zombie::GetPos() { return rom_pos; }

ChQuaternion<> Ch_8DOF_zombie::GetRot() { return rom_rot; }

std::shared_ptr<ChBodyAuxRef> Ch_8DOF_zombie::GetChassisBody() {
  return chassis_body;
}
