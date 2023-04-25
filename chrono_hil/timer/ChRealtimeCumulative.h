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
// Authors: Simone Benatti, Jason Zhou
// =============================================================================
//
// This is a global soft real-time synchronization function
//
// =============================================================================

#ifndef CHREALTIMECUM_H
#define CHREALTIMECUM_H

#include <limits>

#include "../ChApiHil.h"
#include "chrono/core/ChTimer.h"

namespace chrono {
namespace hil {

/// Class for a timer which attempts to enforce soft real-time.
class ChRealtimeCumulative : public ChTimer<double> {
public:
  /// Create the timer (outside the simulation loop, preferably just before
  /// beginning the loop)
  ChRealtimeCumulative() { start(); }

  /// Call this function INSIDE the simulation loop, just ONCE per loop
  /// (preferably as the last call in the loop), passing it the integration step
  /// size used at this step. If the time elapsed over the last step (i.e., from
  /// the last call to Spin) is small than the integration step size, this
  /// function will spin in place until real time catches up with the simulation
  /// time, thus providing soft real-time capabilities.
  void Spin(double sim_time) {
    while (GetTimeSecondsIntermediate() < sim_time) {
    }
  }

  void Reset() {
    reset();
    start();
  }
};

} // end namespace hil
} // end namespace chrono

#endif
