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
// Actor configuration loader for distributed and local playback actors
// =============================================================================

#ifndef ACTOR_CONFIG_LOADER_H
#define ACTOR_CONFIG_LOADER_H

#include <string>
#include <vector>

#include "ActorPlayback.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

namespace chrono {
namespace hil {

// =============================================================================
// Actor Configuration Loading
// =============================================================================

/// Initialize playback actors from a JSON configuration file (local mode)
/// Creates new vehicles in the same system as the reference vehicle
/// @param config_path Path to the actor configuration JSON file
/// @param reference_vehicle Reference vehicle (used to get the system)
/// @param vehicle_filename Vehicle JSON file path
/// @param engine_filename Engine JSON file path  
/// @param transmission_filename Transmission JSON file path
/// @param tire_filename Tire JSON file path
/// @param steering_file Steering controller JSON file path
/// @param speed_file Speed controller JSON file path
/// @param tire_step_size Tire simulation step size
/// @param playback_actors Output vector to store initialized actors
/// @return true if at least one actor was successfully initialized
bool InitializePlaybackActors(const std::string& config_path,
                              vehicle::WheeledVehicle& reference_vehicle,
                              const std::string& vehicle_filename,
                              const std::string& engine_filename,
                              const std::string& transmission_filename,
                              const std::string& tire_filename,
                              const std::string& steering_file,
                              const std::string& speed_file,
                              double tire_step_size,
                              std::vector<ActorPlayback>& playback_actors);

/// Get the starting position and rotation for a distributed actor from config
/// @param config_path Path to the actor configuration JSON file
/// @param actor_index 0-based index into the actors array
/// @param out_pos Output: starting position
/// @param out_rot Output: starting rotation  
/// @return true if pose was successfully retrieved
bool GetActorStartPose(const std::string& config_path,
                       int actor_index,
                       ChVector3d& out_pos,
                       ChQuaterniond& out_rot);

/// Parse actor configuration for a distributed actor (doesn't create vehicle)
/// @param config_path Path to the actor configuration JSON file
/// @param actor_index 0-based index into the actors array
/// @param my_vehicle The vehicle to attach the path driver to
/// @param steering_file Steering controller JSON file path
/// @param speed_file Speed controller JSON file path
/// @param out_actor Output: populated ActorPlayback (vehicle will be nullptr)
/// @param node_id Node ID for logging
/// @param cruise_speed Default cruise speed if not specified
/// @return true if configuration was successfully parsed
bool ParseDistributedActorConfig(const std::string& config_path,
                                  int actor_index,
                                  vehicle::WheeledVehicle& my_vehicle,
                                  const std::string& steering_file,
                                  const std::string& speed_file,
                                  ActorPlayback& out_actor,
                                  int node_id,
                                  double cruise_speed);

}  // namespace hil
}  // namespace chrono

#endif  // ACTOR_CONFIG_LOADER_H
