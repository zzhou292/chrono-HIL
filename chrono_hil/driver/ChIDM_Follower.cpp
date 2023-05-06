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
// This is an IDM follower driver designed to not to accept a vehicle as an
// argument
// The driver definition is Intelligence Driver Model (IDM)
//
// =============================================================================

#include "ChIDM_Follower.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
namespace chrono {
namespace hil {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIDMFollower::Synchronize(double time, double step, double lead_distance,
                                double lead_speed) {
  // In this portion we adjust the target speed according to custom piece-wise
  // sinusoidal defined in behavior_data. We use the driver model explained
  // here, using a desired speed instead:
  // https://traffic-simulation.de/info/info_IDM.html the parameters are
  // data[0] - velocity desired [m/s]
  // data[1] - time gap [s]
  // data[2] - spacing standstill [m]
  // data[3] - acceleration rate [m/s^2]
  // data[4] - deceleration [m/s^2]
  // data[5] - acceleration exponent [-]
  // data[6] - avergae length between lead and ego vehicles [m]
  dist += (m_vehicle.GetChassis()->GetPos() - previousPos).Length();
  previousPos = m_vehicle.GetChassis()->GetPos();

  double s = lead_distance - behavior_data[6];
  double v = m_vehicle.GetChassis()->GetSpeed();
  double delta_v = v - lead_speed;
  double s_star =
      behavior_data[2] +
      ChMax(0.0, v * behavior_data[1] +
                     (v * delta_v) /
                         (2 * sqrt(behavior_data[3] * behavior_data[4])));
  double dv_dt =
      behavior_data[3] *
      (1 - pow(v / behavior_data[0], behavior_data[5]) - pow(s_star / s, 2));

  // integrate intended acceleration into theoretical soeed
  thero_speed = thero_speed + dv_dt * step;
  double v_ms = ChMax(0.0, thero_speed);

  // to avoid large negative value during self drive
  if (thero_speed < 0) {
    thero_speed = 0;
  }

  SetDesiredSpeed(v_ms);

  ChPathFollowerDriver::Synchronize(time);
}

double ChIDMFollower::Get_Dist() { return dist; }

void ChIDMFollower::Set_TheroSpeed(float target_thero_speed) {
  thero_speed = target_thero_speed;
}

void ChIDMFollower::Set_CruiseSpeed(float idm_cruise_speed) {
  behavior_data[0] = idm_cruise_speed;
}

void ChIDMFollower::SetParam(std::vector<double> params) {
  if (params.size() != behavior_data.size()) {
    std::cout << "behavior data invalid!" << std::endl;
    return;
  }
  behavior_data.assign(params.begin(), params.end());
}

} // end namespace hil
} // end namespace chrono
