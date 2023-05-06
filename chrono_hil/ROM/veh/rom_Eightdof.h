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
// Authors: Jason Zhou, Huzaifa Mustafa Unjhawala
// =============================================================================
//
// The 8dof vehicle model base class
// The class includes 8dof vehicle dynamics computation
//
// =============================================================================

#include "../../ChApiHil.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "rom_TMeasy.h"
#include "rom_utils.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#ifndef EIGHTDOF_H
#define EIGHTDOF_H

using namespace chrono;

/*
Header file for the 8 DOF model implemented in cpp
*/

// forward declaration
struct TMeasyState;
struct TMeasyParam;

// vehicle Parameters structure
struct VehicleParam {

  // default constructor with pre tuned values from HMMVW calibration
  VehicleParam()
      : m_a(1.6889), m_b(1.6889), m_h(0.713), m_m(2097.85), m_jz(4519.),
        m_jx(1289.), m_jxz(3.265), m_cf(1.82), m_cr(1.82), m_muf(127.866),
        m_mur(129.98), m_hrcf(0.379), m_hrcr(0.327), m_krof(31000),
        m_kror(31000), m_brof(3300), m_bror(3300), m_maxSteer(0.6525249),
        m_diffRatio(0.06), m_maxBrakeTorque(4000.), m_step(1e-2) {}

  // constructor
  VehicleParam(double a, double b, double h, double m, double Jz, double Jx,
               double Jxz, double cf, double cr, double muf, double mur,
               double hrcf, double hrcr, double krof, double kror, double brof,
               double bror, double maxSteer, double gearRatio, double maxTorque,
               double brakeTorque, double maxSpeed, double c1, double c0,
               double step)
      : m_a(a), m_b(b), m_h(h), m_m(m), m_jz(Jz), m_jx(Jx), m_jxz(Jxz),
        m_cf(cf), m_cr(cr), m_muf(muf), m_mur(mur), m_hrcf(hrcf), m_hrcr(hrcr),
        m_krof(krof), m_kror(kror), m_brof(bror), m_bror(bror),
        m_maxSteer(maxSteer), m_diffRatio(gearRatio),
        m_maxBrakeTorque(brakeTorque), m_step(step) {}

  double m_a,
      m_b;      ///< Distance c.g. - front axle & distance c.g. - rear axle (m)
  double m_h;   ///< height of c.g
  double m_m;   ///< total vehicle mass (kg)
  double m_jz;  ///< yaw moment inertia (kg.m^2)
  double m_jx;  ///< roll inertia
  double m_jxz; ///< XZ inertia
  double m_cf, m_cr;     ///< front and rear track width
  double m_muf, m_mur;   ///< front and rear unsprung mass
  double m_hrcf, m_hrcr; ///< front and rear roll centre height below C.g
  double m_krof, m_kror, m_brof,
      m_bror;              ///< front and rear roll stiffness and damping
  double m_maxSteer;       ///< max steer angle parameters of the vehicle
  double m_diffRatio;      ///< end drive shaft gear ratio
  double m_maxBrakeTorque; ///< max brake torque

  /// engine and transmission parameters
  double m_max_rpm; ///< maximum engine RPM

  std::vector<std::pair<double, double>> m_shift_points; ///< shift pair
  std::vector<double> m_fwd_gear_ratio;                  ///< forward gear ratio
  double m_rev_gear_ratio;                               ///< reverse gear ratio
  ChFunction_Recorder map_0;                             ///< 0 throttle map
  ChFunction_Recorder map_f;                             ///< full throttle map

  double m_step; ///< vehicle integration time step
};

/// vehicle states structure
struct VehicleState {

  /// default constructor just assigns zero to all members
  VehicleState()
      : m_x(0.), m_y(0.), m_u(0.), m_v(0.), m_psi(0.), m_wz(0.), m_phi(0.),
        m_wx(0.), m_udot(0.), m_vdot(0.), m_wxdot(0.), m_wzdot(0.), m_fzlf(0.),
        m_fzrf(0.), m_fzlr(0.), m_fzrr(0.), m_cur_gear(0.), m_motor_speed(0.) {}

  /// special constructor in case need to start simulation
  /// from some other state
  double m_x, m_y;    ///< x and y position
  double m_u, m_v;    ///< x and y velocity
  double m_psi, m_wz; ///< yaw angle and yaw rate
  double m_phi, m_wx; ///< roll angle and roll rate

  /// acceleration 'states'
  double m_udot, m_vdot;
  double m_wxdot, m_wzdot;

  /// vertical forces on each tire
  double m_fzlf, m_fzrf, m_fzlr, m_fzrr;

  /// rotational speed of each tire
  double m_tire_w[4];

  /// transmission states
  int m_cur_gear;       /// current gear
  double m_motor_speed; /// engine RPM
};

/// sets the vertical forces based on the vehicle weight
void vehInit(VehicleState &v_state, VehicleParam &v_params, float step_size);

double driveTorque(const VehicleParam &v_params, VehicleState &v_state,
                   const double throttle, const double omega, int tire_idx);

inline double brakeTorque(const VehicleParam &v_params, const double brake) {
  return v_params.m_maxBrakeTorque * brake;
}

/// function to advance the time step of the vehicle
void vehAdv(VehicleState &v_states, const VehicleParam &v_params,
            const std::vector<double> &fx, const std::vector<double> &fy,
            const double huf, const double hur);

/// setting vehicle parameters using a JSON file
void setVehParamsJSON(VehicleParam &v_params, rapidjson::Document &d);

/// setting engine parameters using a JSON file
void setEngParamsJSON(VehicleParam &v_params, rapidjson::Document &d);

void vehToTireTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls);

void tireToVehTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls);
#endif