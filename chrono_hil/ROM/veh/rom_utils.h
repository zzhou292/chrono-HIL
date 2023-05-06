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
// 8dof utility class
//
// =============================================================================

#ifndef UTILS_H
#define UTILS_H

#include "../../ChApiHil.h"
#include <fstream>
#include <sstream>
#include <vector>

static const double C_PI = 3.141592653589793238462643383279;
static const double C_2PI = 6.283185307179586476925286766559;
static const double G = 9.81; // gravity constant

/// Structure for storing driver input - Similar to Chrono
struct Entry {
  Entry() {}
  /// constructor
  Entry(double time, double steering, double throttle, double braking)
      : m_time(time), m_steering(steering), m_throttle(throttle),
        m_braking(braking) {}

  double m_time;
  double m_steering;
  double m_throttle;
  double m_braking;
};

/// Driver inputs from data file.
void driverInput(std::vector<Entry> &m_data, const std::string &filename);

/// function needed to compare times for driver input
bool compareTime(const Entry &a, const Entry &b);

/// Function to get the vehicle controls at a given time
/// need to pass the data as well
void getControls(std::vector<double> &controls, std::vector<Entry> &m_data,
                 const double time);

/// linear interpolation function
inline double InterpL(double fz, double w1, double w2, double pn) {
  return w1 + (w2 - w1) * (fz / pn - 1.);
}

/// quadratic interpolation function
inline double InterpQ(double fz, double w1, double w2, double pn) {
  return (fz / pn) * (2. * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / pn));
}

/// temlate safe signum function
template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

double sineStep(double x, double x1, double y1, double x2, double y2);

/// clamp function from chrono
template <typename T> T clamp(T value, T limitMin, T limitMax) {
  if (value < limitMin)
    return limitMin;
  if (value > limitMax)
    return limitMax;

  return value;
};

#endif