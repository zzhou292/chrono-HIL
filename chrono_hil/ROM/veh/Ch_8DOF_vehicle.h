// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
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
using namespace chrono::geometry;

// Class definition for the 8DOF Reduced-Order Vehicle Model (ROM).
class CH_HIL_API Ch_8DOF_vehicle {

  /// ROM class constructor
public:
  Ch_8DOF_vehicle(std::string rom_json, float z_plane, float step_size,
                  bool vis = false);

  Ch_8DOF_vehicle(std::string rom_json, float z_plane, float step_size,
                  std::shared_ptr<ChTriangleMeshConnected> chassis_mesh,
                  std::shared_ptr<ChTriangleMeshConnected> wheel_mesh_l,
                  std::shared_ptr<ChTriangleMeshConnected> wheel_mesh_r,
                  bool vis = false);

  /// Initialize the 8DOF ROM vehicle instance
  /// The Chrono system in which the 8DOF ROM belongs to
  void Initialize(ChSystem *sys);

  /// Set 8DOF ROM initial position. Note
  /// that the z position sets the plane the 8DOF ROM is moving on
  void SetInitPos(ChVector<> init_pos);

  /// Set 8DOF ROM initial yaw angle
  void SetInitRot(float yaw);

  /// Advance 8DOF ROM dynamics simulation
  void Advance(float time, DriverInputs inputs);

  /// Get the current position of the 8DOF ROM
  ChVector<> GetPos();

  /// Get the current rotation of the 8DOF ROM
  /// Returns A quaternion which describes the orientation of the 8DOF ROM in
  /// the space. Note that the dynamics of the 8DOF ROM is configured without
  /// pitch. (pitch angle is not involved in the calculation).
  ChQuaternion<> GetRot();

  /// Get the simulation step size
  float GetStepSize();

  /// Get the current velocity of the 8DOF ROM
  ChVector<> GetVel();

  /// Obtain the ChBody attached on the chassis
  std::shared_ptr<ChBodyAuxRef> GetChassisBody();

  /// Obtain the rotation angle of a specific tire
  float GetTireRotation(int idx);

  /// Get the last driver input for the current 8DOF ROM
  DriverInputs GetDriverInputs();

  /// Return the current transmission gear
  int GetGear() { return veh1_st.m_cur_gear; }

  /// Return the current engine speed
  double GetMotorSpeed() { return veh1_st.m_motor_speed; }

private:
  bool enable_vis; ///< Whether visualization is enabled. Note that if
                   ///< enable_vis is set to false, no communication will happen
                   ///< between Chrono system and the ROM dynamics solver.

  bool preload_vis_mesh; ///< Whether to preload visualization mesh

  float
      rom_z_plane; ///< The height of the z plane the ROM's motion is limited to

  std::string vehicle_dyn_json; ///< Path to the json file which contains the
                                ///< vehicle dynamics parameters

  std::string
      tire_json; ///< Path to the json file which contains the tire parameters

  std::string engine_json; ///< Path to the json file which contains the engine
                           ///< parameters

  std::string m_chassis_mesh; ///< Path to the mesh file which contains the
                              ///< vehicle chassis trimesh (obj)

  std::string m_wheel_mesh; ///< Path to the mesh file which contains the
                            ///< vehicle wheel+tire trimesh (obj)

  std::shared_ptr<ChTriangleMeshConnected> m_chassis_trimesh;
  std::shared_ptr<ChTriangleMeshConnected> m_wheel_trimesh_l;
  std::shared_ptr<ChTriangleMeshConnected> m_wheel_trimesh_r;

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
