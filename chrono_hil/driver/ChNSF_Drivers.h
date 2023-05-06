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
// Custom drivers for the NSF project.
// Both are specialization of ChPathFollowerDriver
// The leader will adjust its behavior depending on the traveled distance
// The follower will adjust the speed to reach a target gap with the leader
//
// =============================================================================

#ifndef CH_NSF_DRIVER_H
#define CH_NSF_DRIVER_H

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

// Driver for the leader vehicle, it adjusts its target speed according to a
// piecewise sinusoidal function In the buffer-areas between pieces it keeps the
// target speed specified in target_speed
class CH_HIL_API ChNSFLeaderDriver : public ChPathFollowerDriver {
public:
  /// Construct an interactive driver.
  ChNSFLeaderDriver(
      ChVehicle &vehicle,                   ///< associated vehicle
      const std::string &steering_filename, ///< JSON file with steering
                                            ///< controller specification
      const std::string
          &speed_filename, ///< JSON file with speed controller specification
      std::shared_ptr<ChBezierCurve> path, ///< Bezier curve with target path
      const std::string &path_name,        ///< name of the path curve
      double target_speed,                 ///< constant target speed
      std::vector<std::vector<double>>
          behavior) ///< JSON file with piecewise directives

      : ChPathFollowerDriver(vehicle, steering_filename, speed_filename, path,
                             path_name, target_speed),
        behavior_data(behavior), cruise_speed(target_speed) {
    previousPos = vehicle.GetChassis()->GetPos();
    dist = 0;
  }

  ~ChNSFLeaderDriver() {}

  void Synchronize(double time);

  void SetCruiseSpeed(double speed);

  double Get_Dist();

private:
  // starting pos to compare with to obtain traveled dist
  ChVector<> previousPos;
  // traveldistance
  double dist;
  // vector of vectors containing the instruction for target speed
  std::vector<std::vector<double>> behavior_data;
  // Cruise speed between sinusoidal stretches
  double cruise_speed;
};

// Driver for the follower vehicle, it adjust its speed
class CH_HIL_API ChNSFFollowerDriver : public ChPathFollowerDriver {
public:
  /// Construct an interactive driver.
  ChNSFFollowerDriver(
      ChVehicle &vehicle,                   ///< associated vehicle
      const std::string &steering_filename, ///< JSON file with steering
                                            ///< controller specification
      const std::string
          &speed_filename, ///< JSON file with speed controller specification
      std::shared_ptr<ChBezierCurve> path, ///< Bezier curve with target path
      const std::string &path_name,        ///< name of the path curve
      double target_speed,                 ///< constant target speed
      std::shared_ptr<ChVehicle> lead_vehicle, ///< followed_vehicle
      std::vector<double> params) ///< JSON file with piecewise params

      : ChPathFollowerDriver(vehicle, steering_filename, speed_filename, path,
                             path_name, target_speed),
        behavior_data(params), cruise_speed(target_speed),
        leader(lead_vehicle) {
    previousPos = vehicle.GetChassis()->GetPos();
    dist = 0;
    m_no_lead = false;
  }

  ChNSFFollowerDriver(
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
    m_no_lead = true;
  }

  ~ChNSFFollowerDriver() {}

  void Synchronize(double time, double step);

  void SetCruiseSpeed(double speed);

  double Get_Dist();

  void Set_TheroSpeed(float target_thero_speed);

private:
  // starting pos to compare with to obtain traveled dist
  ChVector<> previousPos;
  // traveldistance
  double dist;
  // theoretical speed
  double thero_speed = 0;
  // no lead indicator (if no lead, then PID should be set seperately than IDM)
  bool m_no_lead = false;
  // vector of vectors containing the instruction for target speed
  std::vector<double> behavior_data;
  // Cruise speed between sinusoidal stretches
  double cruise_speed;
  // leader vehicle to follow
  std::shared_ptr<ChVehicle> leader;
};

} // namespace hil
} // namespace chrono

#endif
