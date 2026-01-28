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
// Actor playback system for path-following vehicles with speed profiles
// =============================================================================

#ifndef ACTOR_PLAYBACK_H
#define ACTOR_PLAYBACK_H

#include <memory>
#include <vector>
#include <limits>
#include <string>

#include "chrono/core/ChVector3.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace hil {

// =============================================================================
// Speed Profile Types
// =============================================================================

enum class SpeedProfileType {
    VELOCITY,      // Direct velocity targets
    ACCELERATION   // Acceleration-based profile
};

// =============================================================================
// Speed Profile Segment
// =============================================================================

struct SpeedProfileSegment {
    double start_time = 0.0;
    double end_time = std::numeric_limits<double>::infinity();
    bool until_end = false;
    bool has_explicit_end = false;
    double value = 0.0;  // velocity (m/s) or acceleration (m/s^2)
};

// =============================================================================
// Actor Playback State
// =============================================================================

struct ActorPlayback {
    // Vehicle components (nullptr for distributed actors)
    std::shared_ptr<vehicle::WheeledVehicle> vehicle;
    std::shared_ptr<vehicle::ChPathFollowerDriver> path_driver;
    std::shared_ptr<vehicle::ChPowertrainAssembly> powertrain;
    
    // Path data
    std::vector<ChVector3d> waypoints;
    
    // Timing
    double start_time = 0.0;
    
    // Path following parameters
    double look_ahead_distance = -1.0;
    double path_spacing = 0.5;
    double smoothing_window = 0.0;
    
    // Steering controller gains
    double steering_kp = -1.0;
    double steering_ki = 0.0;
    double steering_kd = 0.0;
    
    // State
    bool active = false;
    bool goal_reached = false;
    
    // Speed profile
    SpeedProfileType profile_type = SpeedProfileType::VELOCITY;
    std::vector<SpeedProfileSegment> profile_segments;
    double initial_speed = 0.0;
    double current_speed = 0.0;
    double last_profile_time = 0.0;
    bool profile_defined = false;
    double max_decel = 3.0;
};

// =============================================================================
// Speed Profile Functions
// =============================================================================

/// Parse a speed profile from JSON
/// @param profile_json The JSON value containing the speed profile
/// @param actor The ActorPlayback to populate with profile data
/// @return true if parsing succeeded
bool ParseSpeedProfileJSON(const rapidjson::Value& profile_json, ActorPlayback& actor);

/// Get the currently active segment for a given local time
/// @param actor The actor with the speed profile
/// @param local_time Time since actor activation
/// @return Pointer to active segment, or nullptr if none
const SpeedProfileSegment* GetActiveSegment(const ActorPlayback& actor, double local_time);

/// Evaluate the desired speed for an actor at the current time
/// @param actor The actor (will be modified to update current_speed)
/// @param local_time Time since actor activation
/// @param step Simulation step size
/// @param within_stop_zone Whether the actor is near the end of its path
/// @return Desired speed in m/s
double EvaluateDesiredSpeed(ActorPlayback& actor, double local_time, double step, bool within_stop_zone);

}  // namespace hil
}  // namespace chrono

#endif  // ACTOR_PLAYBACK_H
