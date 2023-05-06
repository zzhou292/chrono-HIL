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
// The TMeasy for the 8dof vehicle model
// This class includes the computation of tire dynamics for 8dof vehicle model
//
// =============================================================================

#ifndef TMEASY_H
#define TMEASY_H

#include "../../ChApiHil.h"
#include "rom_Eightdof.h"
#include <stdint.h>

/// TMeasy parameter structure
struct TMeasyParam {

  /// constructor that takes default values of HMMWV tire
  TMeasyParam()
      : m_jw(6.69), m_rr(0.015), m_mu(0.8), m_r0(0.4699), m_pn(8562.8266),
        m_pnmax(29969.893), m_cx(185004.42), m_cy(164448.37), m_kt(411121.0),
        m_dx(3700.), m_dy(3488.), m_rdyncoPn(0.375), m_rdyncoP2n(0.75),
        m_fzRdynco(0), m_rdyncoCrit(0), m_dfx0Pn(151447.29),
        m_dfx0P2n(236412.79), m_fxmPn(7575.3614), m_fxmP2n(12808.276),
        m_fxsPn(4657.9208), m_fxsP2n(8625.3352), m_sxmPn(0.12), m_sxmP2n(0.15),
        m_sxsPn(0.9), m_sxsP2n(0.95), m_dfy0Pn(50931.693), m_dfy0P2n(94293.847),
        m_fymPn(6615.0404), m_fymP2n(12509.947), m_fysPn(6091.5092),
        m_fysP2n(11443.875), m_symPn(0.38786), m_symP2n(0.38786),
        m_sysPn(0.82534), m_sysP2n(0.91309) {}

  /// constructor that takes given values
  TMeasyParam(double jw, double rr, double mu, double r0, double pn,
              double pnmax, double cx, double cy, double dx, double dy,
              double kt, double rdyncoPn, double rdyncoP2n, double fzRdynco,
              double rdyncoCrit, double dfx0Pn, double dfx0P2n, double fxmPn,
              double fxmP2n, double fxsPn, double fxsP2n, double sxmPn,
              double sxmP2n, double sxsPn, double sxsP2n, double dfy0Pn,
              double dfy0P2n, double fymPn, double fymP2n, double fysPn,
              double fysP2n, double symPn, double symP2n, double sysPn,
              double sysP2n)
      : m_jw(jw), m_rr(rr), m_mu(mu), m_r0(r0), m_pn(pn), m_pnmax(pnmax),
        m_cx(cx), m_cy(cy), m_kt(kt), m_dx(dx), m_dy(dy), m_rdyncoPn(rdyncoPn),
        m_rdyncoP2n(rdyncoP2n), m_fzRdynco(fzRdynco), m_rdyncoCrit(rdyncoCrit),
        m_dfx0Pn(dfx0Pn), m_dfx0P2n(dfx0P2n), m_fxmPn(fxmPn), m_fxmP2n(fxmP2n),
        m_fxsPn(fxsPn), m_fxsP2n(fxsP2n), m_sxmPn(sxmPn), m_sxmP2n(sxmP2n),
        m_sxsPn(sxsPn), m_sxsP2n(sxsP2n), m_dfy0Pn(dfy0Pn), m_dfy0P2n(dfy0P2n),
        m_fymPn(fymPn), m_fymP2n(fymP2n), m_fysPn(fysPn), m_fysP2n(fysP2n),
        m_symPn(symPn), m_symP2n(symP2n), m_sysPn(sysPn), m_sysP2n(sysP2n) {}

  /// basic tire parameters
  double m_jw; ///< wheel inertia
  double m_rr; ///< rolling resistance of tire
  double m_mu; ///< friction constant
  double m_r0; ///< unloaded tire radius

  /// TM easy specific tire params
  double m_pn, m_pnmax;    ///< nominal and max vertical force
  double m_cx, m_cy, m_kt; ///< longitudinal, lateral and vertical stiffness
  double m_dx,             ///< longitudinal damping coefficient
      m_dy;                ///< lateral damping coefficient

  /// TMeasy force characteristic params
  /// 2 values - one at nominal load and one at max load

  /// dynamic radius weighting coefficient and a critical value for the vertical
  /// force
  double m_rdyncoPn, m_rdyncoP2n, m_fzRdynco, m_rdyncoCrit;

  /// Longitudinal
  double m_dfx0Pn, m_dfx0P2n; ///< intial longitudinal slopes dFx/dsx [N]
  double m_fxmPn, m_fxmP2n;   ///< maximum longituidnal force [N]
  double m_fxsPn, m_fxsP2n;   ///< Longitudinal load at sliding [N]
  double m_sxmPn, m_sxmP2n;   ///< slip sx at maximum longitudinal load Fx
  double m_sxsPn, m_sxsP2n;   ///< slip sx where sliding begins

  /// Lateral
  double m_dfy0Pn, m_dfy0P2n; ///< intial lateral slopes dFx/dsx [N]
  double m_fymPn, m_fymP2n;   ///< maximum lateral force [N]
  double m_fysPn, m_fysP2n;   ///< Lateral load at sliding [N]
  double m_symPn, m_symP2n;   ///< slip sx at maximum lateral load Fx
  double m_sysPn, m_sysP2n;   ///< slip sx where sliding begins

  double m_step; ///< integration time step
};

/// Tmeasy state structure - actual states + tracking variables
struct TMeasyState {
  TMeasyState()
      : m_xe(0.), m_ye(0.), m_xedot(0.), m_yedot(0.), m_omega(0.), m_xt(0.),
        m_rStat(0.), m_fx(0.), m_fy(0.), m_fz(0.), m_vsx(0.), m_vsy(0.) {}

  /// special constructor in case we want to start the simualtion at
  /// some other time step
  TMeasyState(double xe, double ye, double xedot, double yedot, double omega,
              double xt, double rStat, double fx, double fy, double fz,
              double vsx, double vsy)
      : m_xe(xe), m_ye(ye), m_xedot(xedot), m_yedot(yedot), m_omega(omega),
        m_xt(xt), m_rStat(rStat), m_fx(fx), m_fy(fy), m_fz(fz), m_vsx(vsx),
        m_vsy(vsy) {}

  /// the actual state that are intgrated
  double m_xe, m_ye;       ///< long and lat tire deflection
  double m_xedot, m_yedot; ///< long and lat tire deflection velocity
  double m_omega;          ///< angular velocity of wheel

  /// other "states" that we need to keep track of
  double m_xt;             ///< vertical tire compression
  double m_rStat;          ///< loaded tire radius
  double m_fx, m_fy, m_fz; ///< long, lateral and vertical force in tire frame

  /// velocities in tire frame
  double m_vsx, m_vsy;
};

// sets the vertical tire deflection based on the vehicle weight
// template based on which tire
void tireInit(TMeasyParam &t_params, double step_size);

// function to calculate the force from the force charactristics
// used by tireSync
void tmxy_combined(double &f, double &fos, double s, double df0, double sm,
                   double fm, double ss, double fs);

// Advances the time step
// updates the tire forces which is then used by the vehicle model
void tireAdv(TMeasyState &t_states, const TMeasyParam &t_params,
             VehicleState &v_states, const VehicleParam &v_params,
             const std::vector<double> &controls, int tire_idx);

// setting tire parameters using a JSON file
void setTireParamsJSON(TMeasyParam &t_params, rapidjson::Document &d);

#endif