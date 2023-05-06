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
// An IDM wrapped around path follower
//
// =============================================================================
#ifndef CH_IDM_FOLLOWER_H
#define CH_IDM_FOLLOWER_H

#include "../../ChApiHil.h"
#include "../veh/Ch_8DOF_vehicle.h"
#include "ChROM_PathFollowerDriver.h"
#include <cmath>
#include <random>
#include <string>
#define MS_TO_MPH 2.23694
#define MPH_TO_MS 0.44704
#define AUDI_LENGTH 4.86
#define SUV_LENGTH 5.56
#define M_TO_MILE 0.000621371
#define MILE_TO_M 1609.34449789
namespace chrono {
namespace hil {

class CH_HIL_API ChROM_IDMFollower {
public:
  ChROM_IDMFollower(
      std::shared_ptr<Ch_8DOF_vehicle> rom, ///< associated vehicle
      std::shared_ptr<ChROM_PathFollowerDriver> path_follower,
      std::vector<double> params) ///< JSON file with piecewise params
  {
    m_rom = rom;
    m_path_follower = path_follower;
    m_params = params;
  }

  void SetSto(bool enable_sto, double sd_cruise_speed, double sd_spacing,
              double sd_acc, double sd_dec) {
    m_enable_sto = enable_sto;
    m_sd_cruise_speed = sd_cruise_speed;
    m_sd_spacing = sd_spacing;
    m_sd_acc = sd_acc;
    m_sd_dec = sd_dec;

    std::normal_distribution<double> d1{m_params[0], m_sd_cruise_speed};
    std::normal_distribution<double> d2{m_params[2], m_sd_spacing};
    std::normal_distribution<double> d3{m_params[3], m_sd_acc};
    std::normal_distribution<double> d4{m_params[4], m_sd_dec};

    m_d1 = d1;
    m_d2 = d2;
    m_d3 = d3;
    m_d4 = d4;
  }

  void SetBehaviorParams(std::vector<double> new_params) {
    m_params = new_params;
  }

  void Synchronize(double time, double step, double lead_distance,
                   double lead_speed);

private:
  std::shared_ptr<ChROM_PathFollowerDriver> m_path_follower;
  std::shared_ptr<Ch_8DOF_vehicle> m_rom;
  std::vector<double> m_params;

  // stochasticity
  bool m_enable_sto = false;
  std::random_device m_rd{};
  std::mt19937 m_gen{m_rd()};
  double m_sd_spacing = 1.0;      // spacing standard deviation
  double m_sd_acc = 0.8;          // acceleration standard deviation
  double m_sd_dec = 0.5;          // deceleration standard deviation
  double m_sd_cruise_speed = 1.0; // cruise speed standard deviation
  std::normal_distribution<double> m_d1;
  std::normal_distribution<double> m_d2;
  std::normal_distribution<double> m_d3;
  std::normal_distribution<double> m_d4;
  int update_step_resolution; // IDM parameter disturb frequencies (update each
                              // x steps)

  ChVector<> previousPos;

  // traveldistance
  double dist;
  // theoretical speed
  double thero_speed = 0;
};

} // namespace hil
} // namespace chrono

#endif
