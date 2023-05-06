// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) Jason Zhou
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

#ifndef CH_EIGHT_ROM_ZOMBIE_H
#define CH_EIGHT_ROM_ZOMBIE_H

#include "../../ChApiHil.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include <string>

using namespace chrono;
using namespace chrono::vehicle;

class CH_HIL_API Ch_8DOF_zombie {

public:
  Ch_8DOF_zombie(std::string rom_json, float z_plane, bool vis = false);

  void Initialize(ChSystem *sys);

  void Update(ChVector<> pos, ChVector<> rot, float steering, float tire_rot_0,
              float tire_rot_1, float tire_rot_2, float tire_rot_3);

  void SetPos(ChVector<> pos);

  void SetRot(float roll, float yaw);

  ChVector<> GetPos();

  ChQuaternion<> GetRot();

  std::shared_ptr<ChBodyAuxRef> GetChassisBody();

private:
  float rom_z_plane;
  bool enable_vis;

  std::string chassis_mesh;
  std::string wheel_mesh;

  std::shared_ptr<ChBodyAuxRef> chassis_body;
  std::shared_ptr<ChBodyAuxRef> wheels_body[4];

  ChVector<> wheels_offset_pos[4];
  ChQuaternion<> wheels_offset_rot[4];

  ChVector<> rom_pos;
  ChQuaternion<> rom_rot;

  float tire_rotation[4];
  float max_steer_angle;
};

#endif
