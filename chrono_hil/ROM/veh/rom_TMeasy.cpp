// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou, Huzaifa Mustafa Unjhawala
// =============================================================================
//
// The TMeasy for the 8dof vehicle model
// This class includes the computation of tire dynamics for 8dof vehicle model
//
// =============================================================================

#include "rom_TMeasy.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "rom_utils.h"
#include <cmath>
#include <iostream>
#include <stdint.h>

using namespace chrono;
using namespace chrono::vehicle;

/*
Code for the TM easy tire model implemented with the 8DOF model
*/

// initialize the tire deflection based on the vehicle weight
// tempated based on which tire we get
// 0 - LF
// 1 - RF
// 2 - LR
// 3 - RR
void tireInit(TMeasyParam &t_params, double step_size) {

  // calculates some critical values that are needed
  t_params.m_fzRdynco = (t_params.m_pn * (t_params.m_rdyncoP2n -
                                          2.0 * t_params.m_rdyncoPn + 1.)) /
                        (2. * (t_params.m_rdyncoP2n - t_params.m_rdyncoPn));

  t_params.m_rdyncoCrit = InterpL(t_params.m_fzRdynco, t_params.m_rdyncoPn,
                                  t_params.m_rdyncoP2n, t_params.m_pn);

  t_params.m_step = step_size;
}

void tmxy_combined(double &f, double &fos, double s, double df0, double sm,
                   double fm, double ss, double fs) {

  double df0loc = 0.0;
  if (sm > 0.0) {
    df0loc = std::max(2.0 * fm / sm, df0);
  }

  if (s > 0.0 && df0loc > 0.0) { // normal operating conditions
    if (s > ss) {                // full sliding
      f = fs;
      fos = f / s;
    } else {
      if (s < sm) { // adhesion
        double p = df0loc * sm / fm - 2.0;
        double sn = s / sm;
        double dn = 1.0 + (sn + p) * sn;
        f = df0loc * sm * sn / dn;
        fos = df0loc / dn;
      } else {
        double a = std::pow(fm / sm, 2.0) /
                   (df0loc * sm); // parameter from 2. deriv. of f @ s=sm
        double sstar = sm + (fm - fs) / (a * (ss - sm)); // connecting point
        if (sstar <= ss) {                               // 2 parabolas
          if (s <= sstar) {
            // 1. parabola sm < s < sstar
            f = fm - a * (s - sm) * (s - sm);
          } else {
            // 2. parabola sstar < s < ss
            double b = a * (sstar - sm) / (ss - sstar);
            f = fs + b * (ss - s) * (ss - s);
          }
        } else {
          // cubic fallback function
          double sn = (s - sm) / (ss - sm);
          f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn);
        }
        fos = f / s;
      }
    }
  } else {
    f = 0.0;
    fos = 0.0;
  }
}

// Advance the tire to the next time step
// update the tire forces which will be used by the vehicle
// whichTire specifies
// 0 - LF
// 1 - RF
// 2 - LR
// 3 - RR
void tireAdv(TMeasyState &t_states, const TMeasyParam &t_params,
             VehicleState &v_states, const VehicleParam &v_params,
             const std::vector<double> &controls, int tire_idx) {

  // get the controls and time out
  double t = controls[0];
  double delta = controls[1] * v_params.m_maxSteer;
  double throttle = controls[2];
  double brake = controls[3];

  // Get the whichTire based variables out of the way
  double fz = t_states.m_fz;   // vertical force
  double vsy = t_states.m_vsy; // y slip velocity
  double vsx = t_states.m_vsx; // x slip velocity

  // get our tire deflections so that we can get the loaded radius
  t_states.m_xt = fz / t_params.m_kt;
  t_states.m_rStat = t_params.m_r0 - t_states.m_xt;

  double r_eff;
  if (fz <= t_params.m_fzRdynco) {
    double rdynco =
        InterpL(fz, t_params.m_rdyncoPn, t_params.m_rdyncoP2n, t_params.m_pn);
    r_eff = rdynco * t_params.m_r0 + (1. - rdynco) * t_states.m_rStat;
  } else {
    double rdynco = t_params.m_rdyncoCrit;
    r_eff = rdynco * t_params.m_r0 + (1. - rdynco) * t_states.m_rStat;
  }

  // with this r_eff, we can finalize the x slip velocity
  vsx = vsx - (t_states.m_omega * r_eff);

  // get the transport velocity - 0.01 here is to prevent singularity
  double vta = r_eff * std::abs(t_states.m_omega) + 0.01;

  // evaluate the slips
  double sx = -vsx / vta;
  double alpha;
  // only front wheel steering
  alpha = std::atan2(vsy, vta) - delta;
  double sy = -std::tan(alpha);

  // limit fz
  if (fz > t_params.m_pnmax) {
    fz = t_params.m_pnmax;
  }

  // calculate all curve parameters through interpolation
  double dfx0 =
      InterpQ(fz, t_params.m_dfx0Pn, t_params.m_dfx0P2n, t_params.m_pn);
  double dfy0 =
      InterpQ(fz, t_params.m_dfy0Pn, t_params.m_dfy0P2n, t_params.m_pn);

  double fxm = InterpQ(fz, t_params.m_fxmPn, t_params.m_fxmP2n, t_params.m_pn);
  double fym = InterpQ(fz, t_params.m_fymPn, t_params.m_fymP2n, t_params.m_pn);

  double fxs = InterpQ(fz, t_params.m_fxsPn, t_params.m_fxsP2n, t_params.m_pn);
  double fys = InterpQ(fz, t_params.m_fysPn, t_params.m_fysP2n, t_params.m_pn);

  double sxm = InterpL(fz, t_params.m_sxmPn, t_params.m_sxmP2n, t_params.m_pn);
  double sym = InterpL(fz, t_params.m_symPn, t_params.m_symP2n, t_params.m_pn);

  double sxs = InterpL(fz, t_params.m_sxsPn, t_params.m_sxsP2n, t_params.m_pn);
  double sys = InterpL(fz, t_params.m_sysPn, t_params.m_sysP2n, t_params.m_pn);

  // slip normalizing factors
  double hsxn = sxm / (sxm + sym) + (fxm / dfx0) / (fxm / dfx0 + fym / dfy0);
  double hsyn = sym / (sxm + sym) + (fym / dfy0) / (fxm / dfx0 + fym / dfy0);

  // normalized slip
  double sxn = sx / hsxn;
  double syn = sy / hsyn;

  // combined slip
  double sc = std::hypot(sxn, syn);

  // cos and sine alphs
  double calpha;
  double salpha;
  if (sc > 0) {
    calpha = sxn / sc;
    salpha = syn / sc;
  } else {
    calpha = std::sqrt(2.) / 2.;
    salpha = std::sqrt(2.) / 2.;
  }

  // resultant curve parameters in both directions
  double df0 = std::hypot(dfx0 * calpha * hsxn, dfy0 * salpha * hsyn);
  double fm = std::hypot(fxm * calpha, fym * salpha);
  double sm = std::hypot(sxm * calpha / hsxn, sym * salpha / hsyn);
  double fs = std::hypot(fxs * calpha, fys * salpha);
  double ss = std::hypot(sxs * calpha / hsxn, sys * salpha / hsyn);

  // calculate force and force /slip from the curve characteritics
  double f, fos;
  tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);

  // static or "structural" force
  double Fx, Fy;
  if (sc > 0.) {
    Fx = f * sx / sc;
    Fy = f * sy / sc;
  } else {
    Fx = 0.;
    Fy = 0.;
  }

  // rolling resistance with smoothing
  double vx_min = 0;
  double vx_max = 0;

  double My = -sineStep(vta, vx_min, 0., vx_max, 1.) * t_params.m_rr * fz *
              t_states.m_rStat * sgn(t_states.m_omega);

  double h;
  double dOmega;

  // some normalised slip velocities
  double vtxs = vta * hsxn;
  double vtys = vta * hsyn;

  // some varables needed in the loop
  double fxdyn, fydyn;
  double fxstr, fystr;
  double v_step = v_params.m_step;
  double tire_step = t_params.m_step;
  // now we integrate to the next vehicle time step
  double tEnd = t + v_step;
  while (t < tEnd) {

    // ensure that we integrate exactly to step
    h = std::min(tire_step, tEnd - t);

    // always integrate using half implicit
    // just a placeholder to simplify the forumlae
    double dFx = -vtxs * t_params.m_cx / (vtxs * t_params.m_dx + fos);

    t_states.m_xedot = 1. / (1. - h * dFx) *
                       (-vtxs * t_params.m_cx * t_states.m_xe - fos * vsx) /
                       (vtxs * t_params.m_dx + fos);

    t_states.m_xe = t_states.m_xe + h * t_states.m_xedot;

    double dFy = -vtys * t_params.m_cy / (vtys * t_params.m_dy + fos);
    t_states.m_yedot =
        (1. / (1. - h * dFy)) *
        (-vtys * t_params.m_cy * t_states.m_ye - fos * (-sy * vta)) /
        (vtys * t_params.m_dy + fos);

    t_states.m_ye = t_states.m_ye + h * t_states.m_yedot;

    // update the force since we need to update the force to get the omegas
    // some wierd stuff happens between the dynamic and structural force
    fxdyn = t_params.m_dx *
                (-vtxs * t_params.m_cx * t_states.m_xe - fos * vsx) /
                (vtxs * t_params.m_dx + fos) +
            t_params.m_cx * t_states.m_xe;

    fydyn = t_params.m_dy *
                ((-vtys * t_params.m_cy * t_states.m_ye - fos * (-sy * vta)) /
                 (vtys * t_params.m_dy + fos)) +
            (t_params.m_cy * t_states.m_ye);

    fxstr =
        clamp(t_states.m_xe * t_params.m_cx + t_states.m_xedot * t_params.m_dx,
              -t_params.m_fxmP2n, t_params.m_fxmP2n);
    fystr =
        clamp(t_states.m_ye * t_params.m_cy + t_states.m_yedot * t_params.m_dy,
              -t_params.m_fymP2n, t_params.m_fymP2n);

    double weightx = sineStep(std::abs(vsx), 1., 1., 1.5, 0.);
    double weighty = sineStep(std::abs(-sy * vta), 1., 1., 1.5, 0.);

    // now finally get the resultant force
    t_states.m_fx = weightx * fxstr + (1. - weightx) * fxdyn;
    t_states.m_fy = weighty * fystr + (1. - weighty) * fydyn;

    // now use this force for our omegas
    dOmega =
        (1 / t_params.m_jw) *
        (driveTorque(v_params, v_states, throttle, t_states.m_omega, tire_idx) /
             4. +
         My - sgn(t_states.m_omega) * brakeTorque(v_params, brake) -
         t_states.m_fx * t_states.m_rStat);

    // integrate omega using the latest dOmega
    t_states.m_omega = t_states.m_omega + h * dOmega;

    t += h;
  }
}

// setting Tire parameters using a JSON file
void setTireParamsJSON(TMeasyParam &t_params, rapidjson::Document &d) {

  // pray to what ever you believe in and hope that the json file has all these
  t_params.m_jw = d["jw"].GetDouble();
  t_params.m_rr = d["rr"].GetDouble();
  t_params.m_r0 = d["r0"].GetDouble();
  t_params.m_pn = d["pn"].GetDouble();
  t_params.m_pnmax = d["pnmax"].GetDouble();
  t_params.m_cx = d["cx"].GetDouble();
  t_params.m_cy = d["cy"].GetDouble();
  t_params.m_kt = d["kt"].GetDouble();
  t_params.m_dx = d["dx"].GetDouble();
  t_params.m_dy = d["dy"].GetDouble();
  t_params.m_rdyncoPn = d["rdyncoPn"].GetDouble();
  t_params.m_rdyncoP2n = d["rdyncoP2n"].GetDouble();
  t_params.m_fzRdynco = d["fzRdynco"].GetDouble();
  t_params.m_rdyncoCrit = d["rdyncoCrit"].GetDouble();

  t_params.m_dfx0Pn = d["dfx0Pn"].GetDouble();
  t_params.m_dfx0P2n = d["dfx0P2n"].GetDouble();
  t_params.m_fxmPn = d["fxmPn"].GetDouble();
  t_params.m_fxmP2n = d["fxmP2n"].GetDouble();
  t_params.m_fxsPn = d["fxsPn"].GetDouble();
  t_params.m_fxsP2n = d["fxsP2n"].GetDouble();
  t_params.m_sxmPn = d["sxmPn"].GetDouble();
  t_params.m_sxmP2n = d["sxmP2n"].GetDouble();
  t_params.m_sxsPn = d["sxsPn"].GetDouble();
  t_params.m_sxsP2n = d["sxsP2n"].GetDouble();

  t_params.m_dfy0Pn = d["dfy0Pn"].GetDouble();
  t_params.m_dfy0P2n = d["dfy0P2n"].GetDouble();
  t_params.m_fymPn = d["fymPn"].GetDouble();
  t_params.m_fymP2n = d["fymP2n"].GetDouble();
  t_params.m_fysPn = d["fysPn"].GetDouble();
  t_params.m_fysP2n = d["fysP2n"].GetDouble();
  t_params.m_symPn = d["symPn"].GetDouble();
  t_params.m_symP2n = d["symP2n"].GetDouble();
  t_params.m_sysPn = d["sysPn"].GetDouble();
  t_params.m_sysP2n = d["sysP2n"].GetDouble();
}
