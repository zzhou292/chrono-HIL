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
// Authors: Simone Benatti, Jason Zhou
// =============================================================================
//
// This is an IDM follower driver designed to not to accept a vehicle as an
// argument
// The driver definition is Intelligence Driver Model (IDM)
//
// =============================================================================

#ifndef CH_IDM_FOLLOWER_H
#define CH_IDM_FOLLOWER_H

#include <string>

#include "../ChApiHil.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

using namespace chrono::vehicle;

#define MS_TO_MPH 2.23694
#define MPH_TO_MS 0.44704
#define AUDI_LENGTH 4.86
#define M_TO_MILE 0.000621371
#define MILE_TO_M 1609.34449789
namespace chrono {
namespace hil {

class CH_HIL_API ChIDMFollower : public ChPathFollowerDriver {
public:
  ChIDMFollower(
      ChVehicle &vehicle,                   ///< associated vehicle
      const std::string &steering_filename, ///< JSON file with steering
                                            ///< controller specification
      const std::string
          &speed_filename, ///< JSON file with speed controller specification
      std::shared_ptr<ChBezierCurve> path, ///< Bezier curve with target path
      const std::string &path_name,        ///< name of the path curve
      double target_speed,                 ///< constant target speed
      std::vector<double> params)          ///< JSON file with piecewise params

      : ChPathFollowerDriver(vehicle, steering_filename, speed_filename, path,
                             path_name, target_speed),
        behavior_data(params), cruise_speed(target_speed) {
    previousPos = vehicle.GetChassis()->GetPos();
    dist = 0;
  }

  ~ChIDMFollower() {}

  void Synchronize(double time, double step, double lead_distance,
                   double lead_speed);

  void SetCruiseSpeed(double speed);

  void SetParam(std::vector<double> params);

  double Get_Dist();

  void Set_CruiseSpeed(float idm_cruise_speed);

private:
  // starting pos to compare with to obtain traveled dist
  ChVector<> previousPos;
  // traveldistance
  double dist;
  // theoretical speed
  double thero_speed = 0;
  // vector of vectors containing the instruction for target speed
  std::vector<double> behavior_data;
  // Cruise speed between sinusoidal stretches
  double cruise_speed;

  void Set_TheroSpeed(float target_thero_speed);
};

} // namespace hil
} // namespace chrono

#endif
