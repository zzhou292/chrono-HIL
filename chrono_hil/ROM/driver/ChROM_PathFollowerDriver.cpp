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
// A path follower driver for ROM
//
// =============================================================================
#include "ChROM_PathFollowerDriver.h"
namespace chrono {
namespace hil {
ChROM_PathFollowerDriver::ChROM_PathFollowerDriver(
    std::shared_ptr<Ch_8DOF_vehicle> rom, std::shared_ptr<ChBezierCurve> curve,
    double target_speed, double look_ahead_dist, double PID_st_kp,
    double PID_st_ki, double PID_st_kd, double PID_sp_kp, double PID_sp_ki,
    double PID_sp_kd) {
  m_dist = look_ahead_dist;
  m_target_speed = target_speed;

  // steering PID controller parameter
  m_st_kp = PID_st_kp;
  m_st_ki = PID_st_ki;
  m_st_kd = PID_st_kd;

  // speed PID controller parameter
  m_sp_kp = PID_sp_kp;
  m_sp_ki = PID_sp_ki;
  m_sp_kd = PID_sp_kd;

  m_curve = curve;
  m_rom = rom;

  m_tracker = chrono_types::make_shared<ChBezierCurveTracker>(m_curve);
}

DriverInputs ChROM_PathFollowerDriver::GetDriverInput() { return m_inputs; }

void ChROM_PathFollowerDriver::Advance(double time_step) {
  ChVector<> cur_pos = m_rom->GetPos(); // current vehicle position
  ChVector<> cur_vel = m_rom->GetVel(); // current vehicle velocity

  // control steering of the vehicle
  ChVector<> sentinel =
      m_rom->GetChassisBody()
          ->GetFrame_REF_to_abs()
          .TransformPointLocalToParent(m_dist * ChWorldFrame::Forward());

  ChVector<> target;

  m_tracker->calcClosestPoint(sentinel, target);

  ChVector<> sentinel_vec = sentinel - cur_pos;
  ChWorldFrame::Project(sentinel_vec);

  ChVector<> target_vec = target - cur_pos;
  ChWorldFrame::Project(target_vec);

  ChVector<> err_vec = target - sentinel;
  // to do
  err_vec.z() = 0.0;

  double temp =
      Vdot(Vcross(sentinel_vec, target_vec), ChWorldFrame::Vertical());

  double st_err = ChSignum(temp) * err_vec.Length();

  m_st_err_d = (st_err - m_st_err) / time_step;
  m_st_err_i += (st_err + m_st_err) * time_step / 2;
  m_st_err = st_err;

  m_inputs.m_steering =
      m_st_kp * m_st_err + m_st_ki * m_st_err_i + m_st_kd * m_st_err_d;
  ChClampValue(m_inputs.m_steering, -1.0, 1.0);

  // control speed of the vehicle
  double cur_speed = cur_vel.Length();

  double sp_err = m_target_speed - cur_speed;
  // std::cout << "sp_err:" << sp_err << std::endl;

  m_sp_err_d = (sp_err - m_sp_err) / time_step;

  m_sp_err_i += (sp_err + m_sp_err) * time_step / 2;

  m_sp_err = sp_err; // cache new speed error

  double sp_res =
      m_sp_kp * m_sp_err + m_sp_ki * m_sp_err_i + m_sp_kd * m_sp_err_d;
  // std::cout << "sp_res:" << sp_res << std::endl;

  if (sp_res < 0) {
    double temp = abs(sp_res);
    ChClampValue(temp, 0.0, 1.0);
    m_inputs.m_braking = temp;
    m_inputs.m_throttle = 0.0;
  } else {
    double temp = abs(sp_res);
    ChClampValue(temp, 0.0, 1.0);
    m_inputs.m_braking = 0.0;
    m_inputs.m_throttle = temp;
  }
}

void ChROM_PathFollowerDriver::SetCruiseSpeed(double target_speed) {
  m_target_speed = target_speed;
}

} // namespace hil
} // namespace chrono