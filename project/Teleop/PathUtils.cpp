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
// Path processing utilities implementation
// =============================================================================

#include "PathUtils.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace chrono {
namespace hil {

std::vector<ChVector3d> BuildResampledPoints(const std::vector<ChVector3d>& samples, double spacing) {
    std::vector<ChVector3d> points;
    if (samples.empty())
        return points;

    spacing = std::max(1e-3, spacing);
    points.push_back(samples.front());

    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        ChVector3d start = samples[i];
        ChVector3d end = samples[i + 1];
        ChVector3d delta = end - start;
        double seg_len = delta.Length();
        if (seg_len < 1e-6)
            continue;

        ChVector3d dir = delta / seg_len;
        double dist = spacing;
        while (dist < seg_len) {
            points.push_back(start + dir * dist);
            dist += spacing;
        }
        points.push_back(end);
    }
    return points;
}

std::vector<ChVector3d> SmoothPathPoints(const std::vector<ChVector3d>& points,
                                         double spacing,
                                         double smoothing_window) {
    if (points.size() <= 2 || smoothing_window <= 0.0)
        return points;

    int window_half = static_cast<int>(std::round(std::max(smoothing_window / spacing, 1.0)));
    window_half = std::max(1, window_half);
    std::vector<ChVector3d> smoothed(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        ChVector3d accum(0, 0, 0);
        int count = 0;
        int start = static_cast<int>(std::max<int>(0, static_cast<int>(i) - window_half));
        int end = static_cast<int>(std::min<int>(static_cast<int>(points.size()) - 1, static_cast<int>(i) + window_half));
        for (int j = start; j <= end; ++j) {
            accum += points[j];
            ++count;
        }
        smoothed[i] = accum / static_cast<double>(count);
    }

    // Preserve endpoints exactly to avoid drift
    smoothed.front() = points.front();
    smoothed.back() = points.back();
    return smoothed;
}

ChQuaterniond EstimateInitialRotation(const std::vector<ChVector3d>& points) {
    if (points.size() < 2)
        return QUNIT;

    ChVector3d dir = points[1] - points[0];
    dir.z() = 0.0;
    if (dir.Length2() < 1e-8)
        return QUNIT;
    dir.Normalize();
    double yaw = std::atan2(dir.y(), dir.x());
    ChQuaterniond rot;
    rot.SetFromAngleZ(yaw);
    return rot;
}

bool LoadWaypointCSV(const std::string& filename, std::vector<ChVector3d>& out_points) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Unable to open waypoint CSV: " << filename << std::endl;
        return false;
    }

    out_points.clear();
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty())
            continue;
        if (line[0] == '#')
            continue;
        std::stringstream ss(line);
        double x, y, z;
        char delim;
        if (!(ss >> x)) {
            continue;  // skip header or invalid lines
        }
        if (ss.peek() == ',' || ss.peek() == ';')
            ss >> delim;
        if (!(ss >> y))
            continue;
        if (ss.peek() == ',' || ss.peek() == ';')
            ss >> delim;
        if (!(ss >> z))
            continue;
        out_points.emplace_back(x, y, z);
    }

    if (out_points.size() < 2) {
        std::cerr << "Waypoint CSV must contain at least two points: " << filename << std::endl;
        return false;
    }

    return true;
}

}  // namespace hil
}  // namespace chrono
