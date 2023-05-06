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
// An IDM wrapped around path follower
//
// =============================================================================
#include "ChROM_IDMFollower.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
namespace chrono {
namespace hil {

void ChROM_IDMFollower::Synchronize(double time, double step,
                                    double lead_distance, double lead_speed) {
  // In this portion we adjust the target speed according to custom piece-wise
  // sinusoidal defined in behavior_data. We use the driver model explained
  // here, using a desired speed instead:
  // https://traffic-simulation.de/info/info_IDM.html the parameters are: start
  // [miles], end [miles], v0 desired v [m/s], T desired time headway [s],
  // desired space headway [m], a: accel reate a [m/s^2], b: comfort decel
  // [m/s^2], delta: accel exponent

  // update IDM param if disturbed
  std::vector<double> temp_params;
  if (m_enable_sto == true) {
    // cruise speed
    if (int(time / step) % 2000 == 0) {
      temp_params.push_back(m_d1(m_gen));
      temp_params.push_back(m_params[1]);
      temp_params.push_back(m_d2(m_gen));
      temp_params.push_back(m_d3(m_gen));
      temp_params.push_back(m_d4(m_gen));
      temp_params.push_back(m_params[5]);
      temp_params.push_back(m_params[6]);
    } else {
      temp_params = m_params;
    }

  } else {
    temp_params = m_params;
  }

  dist += (m_rom->GetPos() - previousPos).Length();
  previousPos = m_rom->GetPos();

  double s = lead_distance - temp_params[6];
  double v = (m_rom->GetVel()).Length();
  double delta_v = v - lead_speed;

  double s_star =
      temp_params[2] +
      ChMax(0.0,
            v * temp_params[1] +
                (v * delta_v) / (2 * sqrt(temp_params[3] * temp_params[4])));
  double dv_dt = temp_params[3] * (1 - pow(v / temp_params[0], temp_params[5]) -
                                   pow(s_star / s, 2));

  // integrate intended acceleration into theoretical soeed
  thero_speed = thero_speed + dv_dt * step;
  double v_ms = ChMax(0.0, thero_speed);

  // to avoid large negative value during self drive
  if (thero_speed < 0) {
    thero_speed = 0;
  }

  m_path_follower->SetCruiseSpeed(v_ms);

  m_path_follower->Advance(step);
}

} // end namespace hil
} // end namespace chrono
