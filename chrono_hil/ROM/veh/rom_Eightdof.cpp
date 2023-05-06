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
// Authors: Jason Zhou, Huzaifa Mustafa Unjhawala
// =============================================================================
//
// The 8dof vehicle model base class
// The class includes 8dof vehicle dynamics computation
//
// =============================================================================

#include "rom_Eightdof.h"

using namespace chrono;
using namespace chrono::vehicle;

const double rpm2rads = CH_C_PI / 30;

/*
Code for the Eight dof model implemented in cpp
*/

// sets the vertical forces based on the vehicle weight
void vehInit(VehicleState &v_state, VehicleParam &v_param, float time_step) {
  double weight_split =
      ((v_param.m_m * G * v_param.m_a) / (2 * (v_param.m_a + v_param.m_b)) +
       v_param.m_muf * G);
  v_state.m_fzlf = v_state.m_fzrf = weight_split;

  weight_split =
      ((v_param.m_m * G * v_param.m_b) / (2 * (v_param.m_a + v_param.m_b)) +
       v_param.m_mur * G);

  v_state.m_fzlr = v_state.m_fzrr = weight_split;
  v_param.m_step = time_step;
}

// returns drive toruqe at a given omega
double driveTorque(const VehicleParam &v_params, VehicleState &v_state,
                   const double throttle, const double omega, int tire_idx) {

  v_state.m_tire_w[tire_idx] =
      omega; // update tire rotational speed info back to vehicle

  double motor_speed = v_state.m_motor_speed;

  double motor_torque = 0.0;
  if (throttle == 0) {
    motor_torque = v_params.map_0.Get_y(motor_speed);
  } else {
    motor_torque = v_params.map_f.Get_y(motor_speed);
  }

  motor_torque = motor_torque * throttle;

  // share the torque output between two wheels on the same axle
  return motor_torque / v_params.m_fwd_gear_ratio[v_state.m_cur_gear] /
         v_params.m_diffRatio / 2;
}

/*
function to advance the time step of the 8DOF vehicle
along with the vehicle state that will be updated, we pass the
vehicle paramters , a vector containing the longitudinal forces,
and a vector containing the lateral forces by reference, the front
and rear unspring mass positions (loaded tire radius) and the controls
--- There is no point passing all the tire states and paramters
since none of the are used here - however, there is thus an additional
copy in main for getting these states into a vector. So thus, do not know
if this way is actually faster, but I think it is much cleaner than passing
4 tire states and paramaters
*/

void vehAdv(VehicleState &v_states, const VehicleParam &v_params,
            const std::vector<double> &fx, const std::vector<double> &fy,
            const double huf, const double hur) {

  // get the total mass of the vehicle and the vertical distance from the sprung
  // mass C.M. to the vehicle
  double mt = v_params.m_m + 2 * (v_params.m_muf + v_params.m_mur);
  double hrc =
      (v_params.m_hrcf * v_params.m_b + v_params.m_hrcr * v_params.m_a) /
      (v_params.m_a + v_params.m_b);

  // a bunch of varaibles to simplify the formula
  double E1 =
      -mt * v_states.m_wz * v_states.m_u + (fy[0] + fy[1] + fy[2] + fy[3]);

  double E2 = (fy[0] + fy[1]) * v_params.m_a - (fy[2] + fy[3]) * v_params.m_b +
              (fx[1] - fx[0]) * v_params.m_cf / 2 +
              (fx[3] - fx[2]) * v_params.m_cr / 2 +
              (-v_params.m_muf * v_params.m_a + v_params.m_mur * v_params.m_b) *
                  v_states.m_wz * v_states.m_u;

  double E3 = v_params.m_m * G * hrc * v_states.m_phi -
              (v_params.m_krof + v_params.m_kror) * v_states.m_phi -
              (v_params.m_brof + v_params.m_bror) * v_states.m_wx +
              hrc * v_params.m_m * v_states.m_wz * v_states.m_u;

  double A1 = v_params.m_mur * v_params.m_b - v_params.m_muf * v_params.m_a;

  double A2 = v_params.m_jx + v_params.m_m * std::pow(hrc, 2);

  double A3 = hrc * v_params.m_m;

  // Integration using half implicit - level 2 variables found first in next
  // time step

  // update the acceleration states - level 2 variables

  v_states.m_udot =
      v_states.m_wz * v_states.m_v +
      (1 / mt) *
          ((fx[0] + fx[1] + fx[2] + fx[3]) +
           (-v_params.m_mur * v_params.m_b + v_params.m_muf * v_params.m_a) *
               std::pow(v_states.m_wz, 2) -
           2. * hrc * v_params.m_m * v_states.m_wz * v_states.m_wx);

  // common denominator
  double denom = (A2 * std::pow(A1, 2) - 2. * A1 * A3 * v_params.m_jxz +
                  v_params.m_jz * std::pow(A3, 2) +
                  mt * std::pow(v_params.m_jxz, 2) - A2 * v_params.m_jz * mt);

  v_states.m_vdot = (E1 * std::pow(v_params.m_jxz, 2) - A1 * A2 * E2 +
                     A1 * E3 * v_params.m_jxz + A3 * E2 * v_params.m_jxz -
                     A2 * E1 * v_params.m_jz - A3 * E3 * v_params.m_jz) /
                    denom;

  v_states.m_wxdot = (std::pow(A1, 2) * E3 - A1 * A3 * E2 +
                      A1 * E1 * v_params.m_jxz - A3 * E1 * v_params.m_jz +
                      E2 * v_params.m_jxz * mt - E3 * v_params.m_jz * mt) /
                     denom;

  v_states.m_wzdot =
      (std::pow(A3, 2) * E2 - A1 * A2 * E1 - A1 * A3 * E3 +
       A3 * E1 * v_params.m_jxz - A2 * E2 * mt + E3 * v_params.m_jxz * mt) /
      denom;

  // update the level 1 varaibles using the next time step level 2 variable
  v_states.m_u = v_states.m_u + v_params.m_step * v_states.m_udot;
  v_states.m_v = v_states.m_v + v_params.m_step * v_states.m_vdot;
  v_states.m_wx = v_states.m_wx + v_params.m_step * v_states.m_wxdot;
  v_states.m_wz = v_states.m_wz + v_params.m_step * v_states.m_wzdot;

  // update the level 0 varaibles using the next time step level 1 varibales
  // over here still using the old psi and phi.. should we update psi and phi
  // first and then use those????

  v_states.m_x = v_states.m_x +
                 v_params.m_step * (v_states.m_u * std::cos(v_states.m_psi) -
                                    v_states.m_v * std::sin(v_states.m_psi));

  v_states.m_y = v_states.m_y +
                 v_params.m_step * (v_states.m_u * std::sin(v_states.m_psi) +
                                    v_states.m_v * std::cos(v_states.m_psi));

  v_states.m_psi = v_states.m_psi + v_params.m_step * v_states.m_wz;
  v_states.m_phi = v_states.m_phi + v_params.m_step * v_states.m_wx;

  // update the vertical forces
  // sketchy load transfer technique

  double Z1 =
      (v_params.m_m * G * v_params.m_b) / (2. * (v_params.m_a + v_params.m_b)) +
      (v_params.m_muf * G) / 2.;

  double Z2 = ((v_params.m_muf * huf) / v_params.m_cf +
               v_params.m_m * v_params.m_b * (v_params.m_h - v_params.m_hrcf) /
                   (v_params.m_cf * (v_params.m_a + v_params.m_b))) *
              (v_states.m_vdot + v_states.m_wz * v_states.m_u);

  double Z3 =
      (v_params.m_krof * v_states.m_phi + v_params.m_brof * v_states.m_wx) /
      v_params.m_cf;

  double Z4 = ((v_params.m_m * v_params.m_h + v_params.m_muf * huf +
                v_params.m_mur * hur) *
               (v_states.m_udot - v_states.m_wz * v_states.m_v)) /
              (2. * (v_params.m_a + v_params.m_b));

  // evaluate the vertical forces for front
  v_states.m_fzlf = (Z1 - Z2 - Z3 - Z4) > 0. ? (Z1 - Z2 - Z3 - Z4) : 0.;
  v_states.m_fzrf = (Z1 + Z2 + Z3 - Z4) > 0. ? (Z1 + Z2 + Z3 - Z4) : 0.;

  Z1 =
      (v_params.m_m * G * v_params.m_a) / (2. * (v_params.m_a + v_params.m_b)) +
      (v_params.m_mur * G) / 2.;

  Z2 = ((v_params.m_mur * hur) / v_params.m_cr +
        v_params.m_m * v_params.m_a * (v_params.m_h - v_params.m_hrcr) /
            (v_params.m_cr * (v_params.m_a + v_params.m_b))) *
       (v_states.m_vdot + v_states.m_wz * v_states.m_u);

  Z3 = (v_params.m_kror * v_states.m_phi + v_params.m_bror * v_states.m_wx) /
       v_params.m_cr;

  // evaluate vertical forces for the rear
  v_states.m_fzlr = (Z1 - Z2 - Z3 + Z4) > 0. ? (Z1 - Z2 - Z3 + Z4) : 0.;
  v_states.m_fzrr = (Z1 + Z2 + Z3 + Z4) > 0. ? (Z1 + Z2 + Z3 + Z4) : 0.;

  // update vehicle transmission information
  // compute the average omega
  v_states.m_motor_speed = (v_states.m_tire_w[0] + v_states.m_tire_w[1] +
                            v_states.m_tire_w[2] + v_states.m_tire_w[3]) /
                           4.0 /
                           v_params.m_fwd_gear_ratio[v_states.m_cur_gear] /
                           v_params.m_diffRatio;

  // upshift or downshift gear
  if (v_states.m_motor_speed <
      v_params.m_shift_points[v_states.m_cur_gear].first) {
    if (v_states.m_cur_gear > 0) {
      v_states.m_cur_gear = v_states.m_cur_gear - 1;
    }
  } else if (v_states.m_motor_speed >
             v_params.m_shift_points[v_states.m_cur_gear].second) {
    if (v_states.m_cur_gear < v_params.m_shift_points.size() - 1) {
      v_states.m_cur_gear = v_states.m_cur_gear + 1;
    }
  }
}

void vehToTireTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params.m_maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  // left front
  tirelf_st.m_fz = v_states.m_fzlf;
  tirelf_st.m_vsy = v_states.m_v + v_states.m_wz * v_params.m_a;
  tirelf_st.m_vsx =
      (v_states.m_u - (v_states.m_wz * v_params.m_cf) / 2.) * std::cos(delta) +
      tirelf_st.m_vsy * std::sin(delta);

  // right front
  tirerf_st.m_fz = v_states.m_fzrf;
  tirerf_st.m_vsy = v_states.m_v + v_states.m_wz * v_params.m_a;
  tirerf_st.m_vsx =
      (v_states.m_u + (v_states.m_wz * v_params.m_cf) / 2.) * std::cos(delta) +
      tirerf_st.m_vsy * std::sin(delta);

  // left rear - No steer
  tirelr_st.m_fz = v_states.m_fzlr;
  tirelr_st.m_vsy = v_states.m_v - v_states.m_wz * v_params.m_b;
  tirelr_st.m_vsx = v_states.m_u - (v_states.m_wz * v_params.m_cr) / 2.;

  // rigth rear - No steer
  tirerr_st.m_fz = v_states.m_fzrr;
  tirerr_st.m_vsy = v_states.m_v - v_states.m_wz * v_params.m_b;
  tirerr_st.m_vsx = v_states.m_u + (v_states.m_wz * v_params.m_cr) / 2.;
}

void tireToVehTransform(TMeasyState &tirelf_st, TMeasyState &tirerf_st,
                        TMeasyState &tirelr_st, TMeasyState &tirerr_st,
                        const VehicleState &v_states,
                        const VehicleParam &v_params,
                        const std::vector<double> &controls) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params.m_maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  double m_fx, m_fy;

  // left front
  m_fx = tirelf_st.m_fx * std::cos(delta) - tirelf_st.m_fy * std::sin(delta);
  m_fy = tirelf_st.m_fx * std::sin(delta) + tirelf_st.m_fy * std::cos(delta);
  tirelf_st.m_fx = m_fx;
  tirelf_st.m_fy = m_fy;

  // right front
  m_fx = tirerf_st.m_fx * std::cos(delta) - tirerf_st.m_fy * std::sin(delta);
  m_fy = tirerf_st.m_fx * std::sin(delta) + tirerf_st.m_fy * std::cos(delta);
  tirerf_st.m_fx = m_fx;
  tirerf_st.m_fy = m_fy;

  // rear tires - No steer so no need to transform
}

// setting Vehicle parameters using a JSON file
void setVehParamsJSON(VehicleParam &v_params, rapidjson::Document &d) {
  // the file should have all these parameters defined
  v_params.m_a = d["a"].GetDouble();
  v_params.m_b = d["b"].GetDouble();
  v_params.m_m = d["m"].GetDouble();
  v_params.m_h = d["h"].GetDouble();
  v_params.m_jz = d["jz"].GetDouble();
  v_params.m_jx = d["jx"].GetDouble();
  v_params.m_jxz = d["jxz"].GetDouble();
  v_params.m_cf = d["cf"].GetDouble();
  v_params.m_cr = d["cr"].GetDouble();
  v_params.m_muf = d["muf"].GetDouble();
  v_params.m_mur = d["mur"].GetDouble();
  v_params.m_hrcf = d["hrcf"].GetDouble();
  v_params.m_hrcr = d["hrcr"].GetDouble();
  v_params.m_krof = d["krof"].GetDouble();
  v_params.m_kror = d["kror"].GetDouble();
  v_params.m_brof = d["brof"].GetDouble();
  v_params.m_bror = d["bror"].GetDouble();
  v_params.m_maxSteer = d["maxSteer"].GetDouble();
  v_params.m_diffRatio = d["diffRatio"].GetDouble();
  v_params.m_maxBrakeTorque = d["maxBrakeTorque"].GetDouble();
}

// setting vehicle's engine parameters using a JSON file
void setEngParamsJSON(VehicleParam &v_params, rapidjson::Document &d) {

  // assigning full throttle map data
  assert(d["Map Full Throttle"].IsArray());
  for (unsigned int i = 0; i < d["Map Full Throttle"].Size(); i++) {
    v_params.map_f.AddPoint(d["Map Full Throttle"][i][0u].GetDouble() *
                                rpm2rads,
                            d["Map Full Throttle"][i][1u].GetDouble());
  }

  // assigning zero throttle map data
  assert(d["Map Zero Throttle"].IsArray());
  for (unsigned int i = 0; i < d["Map Zero Throttle"].Size(); i++) {
    v_params.map_0.AddPoint(d["Map Zero Throttle"][i][0u].GetDouble() *
                                rpm2rads,
                            d["Map Zero Throttle"][i][1u].GetDouble());
  }

  // assigning reverse gear ratio
  assert(d["Reverse Gear Ratio"].IsFloat());
  v_params.m_rev_gear_ratio = d["Reverse Gear Ratio"].GetFloat();

  // assigning max rpm
  assert(d["Maximal Engine Speed RPM"].IsFloat());
  v_params.m_max_rpm = d["Maximal Engine Speed RPM"].IsFloat();

  // assigning forward gear ratio
  assert(d["Forward Gear Ratios"].IsArray());
  for (unsigned int i = 0; i < d["Forward Gear Ratios"].Size(); i++) {
    v_params.m_fwd_gear_ratio.push_back(d["Forward Gear Ratios"][i].GetFloat());
  }

  assert(d["Shift Points Map RPM"].IsArray());
  for (unsigned int i = 0; i < d["Shift Points Map RPM"].Size(); i++) {
    v_params.m_shift_points.push_back(std::make_pair<double, double>(
        rpm2rads * d["Shift Points Map RPM"][i][0u].GetDouble(),
        rpm2rads * d["Shift Points Map RPM"][i][1u].GetDouble()));
  }
}