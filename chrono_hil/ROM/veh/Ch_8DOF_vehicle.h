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

#ifndef CH_EIGHT_ROM_H
#define CH_EIGHT_ROM_H

#include "../../ChApiHil.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "rom_Eightdof.h"
#include <string>

using namespace chrono;
using namespace chrono::vehicle;

class CH_HIL_API Ch_8DOF_vehicle {

public:
  Ch_8DOF_vehicle(std::string rom_json, float z_plane, float step_size,
                  bool vis = false);

  void Initialize(ChSystem *sys);

  void SetInitPos(ChVector<> init_pos);

  void SetInitRot(float yaw);

  void Advance(float time, DriverInputs inputs);
  ChVector<> GetPos();
  ChQuaternion<> GetRot();
  float GetStepSize();

  ChVector<> GetVel();

  std::shared_ptr<ChBodyAuxRef> GetChassisBody();

  float GetTireRotation(int idx);

  DriverInputs GetDriverInputs();

  int GetGear() { return veh1_st.m_cur_gear; }

  double GetMotorSpeed() { return veh1_st.m_motor_speed; }

private:
  bool enable_vis;

  float rom_z_plane;

  std::string vehicle_dyn_json;
  std::string tire_json;
  std::string engine_json;

  std::string chassis_mesh;
  std::string wheel_mesh;

  VehicleState veh1_st;
  VehicleParam veh1_param;

  // lets define our tires, we have 4 different
  // tires so 4 states
  TMeasyState tirelf_st;
  TMeasyState tirerf_st;
  TMeasyState tirelr_st;
  TMeasyState tirerr_st;

  // but all of them have the same parameters
  // so only one parameter structure
  TMeasyParam tire_param;

  // cached previous input
  DriverInputs m_inputs;

  std::shared_ptr<ChBodyAuxRef> chassis_body;
  std::shared_ptr<ChBodyAuxRef> wheels_body[4];

  ChVector<> wheels_offset_pos[4];
  ChQuaternion<> wheels_offset_rot[4];

  float prev_tire_rotation[4];
};

#endif
