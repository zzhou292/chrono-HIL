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
// Path processing utilities for waypoint-based navigation
// =============================================================================

#ifndef PATH_UTILS_H
#define PATH_UTILS_H

#include <vector>
#include <string>

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"

namespace chrono {
namespace hil {

// =============================================================================
// Path Processing Functions
// =============================================================================

/// Build a resampled set of points from input samples with uniform spacing
/// @param samples Original waypoint samples
/// @param spacing Desired spacing between points (meters)
/// @return Resampled points with approximately uniform spacing
std::vector<ChVector3d> BuildResampledPoints(const std::vector<ChVector3d>& samples, double spacing);

/// Apply moving average smoothing to path points
/// @param points Input path points
/// @param spacing Point spacing (used to calculate window size)
/// @param smoothing_window Smoothing window size in meters
/// @return Smoothed path points (endpoints preserved)
std::vector<ChVector3d> SmoothPathPoints(const std::vector<ChVector3d>& points, 
                                         double spacing, 
                                         double smoothing_window);

/// Estimate initial rotation quaternion from first path segment
/// @param points Path points (need at least 2)
/// @return Quaternion representing heading from first to second point
ChQuaterniond EstimateInitialRotation(const std::vector<ChVector3d>& points);

/// Load waypoints from a CSV file
/// @param filename Path to CSV file (x,y,z format)
/// @param out_points Output vector to store loaded points
/// @return true if loading succeeded (at least 2 points)
bool LoadWaypointCSV(const std::string& filename, std::vector<ChVector3d>& out_points);

}  // namespace hil
}  // namespace chrono

#endif  // PATH_UTILS_H
