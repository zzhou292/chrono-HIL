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
// Actor configuration loader implementation
// =============================================================================

#include "ActorConfigLoader.h"
#include "PathUtils.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChBodyGeometry.h"

namespace chrono {
namespace hil {

bool InitializePlaybackActors(const std::string& config_path,
                              vehicle::WheeledVehicle& reference_vehicle,
                              const std::string& vehicle_filename,
                              const std::string& engine_filename,
                              const std::string& transmission_filename,
                              const std::string& tire_filename,
                              const std::string& steering_file,
                              const std::string& speed_file,
                              double tire_step_size,
                              std::vector<ActorPlayback>& playback_actors) {
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open actor configuration file: " << config_path << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    rapidjson::Document d;
    d.Parse(buffer.str().c_str());
    if (d.HasParseError()) {
        std::cerr << "Failed to parse actor configuration file: " << config_path << std::endl;
        return false;
    }
    if (!d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray()) {
        std::cerr << "Actor configuration missing 'actors' array: " << config_path << std::endl;
        return false;
    }

    const auto& actors_array = d["actors"].GetArray();
    for (const auto& actor_entry : actors_array) {
        if (!actor_entry.IsObject() || !actor_entry.HasMember("path_file"))
            continue;

        ActorPlayback actor;
        actor.start_time = actor_entry.HasMember("start_time") ? actor_entry["start_time"].GetDouble() : 0.0;
        
        if (actor_entry.HasMember("look_ahead")) {
            actor.look_ahead_distance = actor_entry["look_ahead"].GetDouble();
        }
        if (actor_entry.HasMember("path_spacing")) {
            actor.path_spacing = std::max(0.05, actor_entry["path_spacing"].GetDouble());
        }
        if (actor_entry.HasMember("smooth_window")) {
            actor.smoothing_window = std::max(0.0, actor_entry["smooth_window"].GetDouble());
        }
        if (actor_entry.HasMember("steering_kp")) {
            actor.steering_kp = actor_entry["steering_kp"].GetDouble();
        }
        if (actor_entry.HasMember("steering_ki")) {
            actor.steering_ki = actor_entry["steering_ki"].GetDouble();
        }
        if (actor_entry.HasMember("steering_kd")) {
            actor.steering_kd = actor_entry["steering_kd"].GetDouble();
        }
        
        std::string path_file = actor_entry["path_file"].GetString();
        if (!LoadWaypointCSV(path_file, actor.waypoints)) {
            std::cerr << "Skipping actor due to failed path load: " << path_file << std::endl;
            continue;
        }
        
        if (!actor_entry.HasMember("speed_profile")) {
            std::cerr << "Actor entry missing speed_profile; skipping.\n";
            continue;
        }
        if (!ParseSpeedProfileJSON(actor_entry["speed_profile"], actor)) {
            std::cerr << "Failed to parse speed profile for actor path " << path_file << std::endl;
            continue;
        }

        std::vector<ChVector3d> path_points = BuildResampledPoints(actor.waypoints, actor.path_spacing);
        if (actor.smoothing_window > 0.0) {
            path_points = SmoothPathPoints(path_points, actor.path_spacing, actor.smoothing_window);
        }
        auto path_curve = chrono_types::make_shared<ChBezierCurve>(path_points, false);
        actor.waypoints = path_points;

        auto actor_vehicle = chrono_types::make_shared<vehicle::WheeledVehicle>(
            reference_vehicle.GetSystem(), vehicle_filename);
        actor_vehicle->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        ChQuaterniond start_rot = EstimateInitialRotation(path_points);
        actor_vehicle->Initialize(ChCoordsys<>(path_points.front(), start_rot));
        actor_vehicle->GetChassis()->SetFixed(false);
        actor_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        actor_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        actor_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        actor_vehicle->SetWheelVisualizationType(VisualizationType::MESH);

        auto actor_engine = vehicle::ReadEngineJSON(engine_filename);
        auto actor_transmission = vehicle::ReadTransmissionJSON(transmission_filename);
        actor.powertrain = chrono_types::make_shared<vehicle::ChPowertrainAssembly>(actor_engine, actor_transmission);
        actor_vehicle->InitializePowertrain(actor.powertrain);

        for (auto& axle : actor_vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = vehicle::ReadTireJSON(tire_filename);
                tire->SetStepsize(tire_step_size);
                actor_vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
        
        auto driver = chrono_types::make_shared<vehicle::ChPathFollowerDriver>(
            *actor_vehicle, steering_file, speed_file, path_curve, "actor_path", 0.0);
        if (actor.look_ahead_distance > 0.0) {
            driver->GetSteeringController().SetLookAheadDistance(actor.look_ahead_distance);
        }
        driver->GetSteeringController().SetGains(actor.steering_kp, actor.steering_ki, actor.steering_kd);
        driver->Initialize();
        actor.path_driver = driver;

        actor.vehicle = actor_vehicle;
        actor.active = (actor.start_time <= 0.0);

        playback_actors.push_back(std::move(actor));
    }

    if (playback_actors.empty()) {
        std::cerr << "No valid actors were initialized from " << config_path << std::endl;
        return false;
    }

    std::cout << "Loaded " << playback_actors.size() << " playback actor(s) from " << config_path << std::endl;
    return true;
}

bool GetActorStartPose(const std::string& config_path,
                       int actor_index,
                       ChVector3d& out_pos,
                       ChQuaterniond& out_rot) {
    std::ifstream ifs(config_path);
    if (!ifs.is_open())
        return false;

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    rapidjson::Document d;
    d.Parse(buffer.str().c_str());
    if (d.HasParseError() || !d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray())
        return false;

    const auto& actors_array = d["actors"].GetArray();
    if (actor_index < 0 || actor_index >= static_cast<int>(actors_array.Size()))
        return false;

    const auto& actor_entry = actors_array[actor_index];
    if (!actor_entry.IsObject() || !actor_entry.HasMember("path_file"))
        return false;

    // Load waypoints to get starting position
    std::vector<ChVector3d> waypoints;
    std::string path_file = actor_entry["path_file"].GetString();
    if (!LoadWaypointCSV(path_file, waypoints) || waypoints.empty())
        return false;

    double path_spacing = actor_entry.HasMember("path_spacing") 
        ? std::max(0.05, actor_entry["path_spacing"].GetDouble()) : 0.5;
    double smoothing = actor_entry.HasMember("smooth_window")
        ? std::max(0.0, actor_entry["smooth_window"].GetDouble()) : 0.0;

    std::vector<ChVector3d> path_points = BuildResampledPoints(waypoints, path_spacing);
    if (smoothing > 0.0) {
        path_points = SmoothPathPoints(path_points, path_spacing, smoothing);
    }

    if (path_points.empty())
        return false;

    out_pos = path_points.front();
    out_rot = EstimateInitialRotation(path_points);
    return true;
}

bool ParseDistributedActorConfig(const std::string& config_path,
                                  int actor_index,
                                  vehicle::WheeledVehicle& my_vehicle,
                                  const std::string& steering_file,
                                  const std::string& speed_file,
                                  ActorPlayback& out_actor,
                                  int node_id,
                                  double cruise_speed) {
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open actor configuration file: " << config_path << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    rapidjson::Document d;
    d.Parse(buffer.str().c_str());
    if (d.HasParseError()) {
        std::cerr << "Failed to parse actor configuration file: " << config_path << std::endl;
        return false;
    }
    if (!d.IsObject() || !d.HasMember("actors") || !d["actors"].IsArray()) {
        std::cerr << "Actor configuration missing 'actors' array: " << config_path << std::endl;
        return false;
    }

    const auto& actors_array = d["actors"].GetArray();
    if (actor_index < 0 || actor_index >= static_cast<int>(actors_array.Size())) {
        std::cerr << "Actor index " << actor_index << " out of range (only " 
                  << actors_array.Size() << " actors in config)" << std::endl;
        return false;
    }

    const auto& actor_cfg = actors_array[actor_index];
    
    // Get path file
    std::string actor_path_file;
    if (actor_cfg.HasMember("path_file") && actor_cfg["path_file"].IsString()) {
        actor_path_file = actor_cfg["path_file"].GetString();
    } else {
        std::cerr << "ERROR: Actor " << actor_index << " missing 'path_file'!" << std::endl;
        return false;
    }

    // Get start_time
    out_actor.start_time = actor_cfg.HasMember("start_time") ? actor_cfg["start_time"].GetDouble() : 0.0;
    out_actor.active = false;

    // Parse speed profile (object format with type and entries)
    double actor_target_speed = cruise_speed;
    if (actor_cfg.HasMember("speed_profile") && actor_cfg["speed_profile"].IsObject()) {
        if (!ParseSpeedProfileJSON(actor_cfg["speed_profile"], out_actor)) {
            std::cerr << "WARNING: Failed to parse speed_profile for actor " << actor_index 
                      << ", using default speed" << std::endl;
        } else {
            actor_target_speed = out_actor.initial_speed;
            std::cout << "  Speed profile type: " 
                      << (out_actor.profile_type == SpeedProfileType::VELOCITY ? "velocity" : "acceleration") 
                      << std::endl;
            std::cout << "  Initial speed: " << out_actor.initial_speed << " m/s" << std::endl;
            std::cout << "  Max decel: " << out_actor.max_decel << " m/s^2" << std::endl;
        }
    }

    // Get driver parameters
    double look_ahead = 5.0;
    double steering_p = 0.8, steering_i = 0.0, steering_d = 0.0;
    double speed_p = 0.6, speed_i = 0.05, speed_d = 0.0;

    if (actor_cfg.HasMember("look_ahead") && actor_cfg["look_ahead"].IsNumber()) {
        look_ahead = actor_cfg["look_ahead"].GetDouble();
    }
    // Support individual steering gain fields
    if (actor_cfg.HasMember("steering_kp") && actor_cfg["steering_kp"].IsNumber()) {
        steering_p = actor_cfg["steering_kp"].GetDouble();
    }
    if (actor_cfg.HasMember("steering_ki") && actor_cfg["steering_ki"].IsNumber()) {
        steering_i = actor_cfg["steering_ki"].GetDouble();
    }
    if (actor_cfg.HasMember("steering_kd") && actor_cfg["steering_kd"].IsNumber()) {
        steering_d = actor_cfg["steering_kd"].GetDouble();
    }
    // Also support array format for backwards compatibility
    if (actor_cfg.HasMember("steering_gains") && actor_cfg["steering_gains"].IsArray()) {
        const auto& sg = actor_cfg["steering_gains"].GetArray();
        if (sg.Size() >= 3) {
            steering_p = sg[0].GetDouble();
            steering_i = sg[1].GetDouble();
            steering_d = sg[2].GetDouble();
        }
    }
    if (actor_cfg.HasMember("speed_gains") && actor_cfg["speed_gains"].IsArray()) {
        const auto& spg = actor_cfg["speed_gains"].GetArray();
        if (spg.Size() >= 3) {
            speed_p = spg[0].GetDouble();
            speed_i = spg[1].GetDouble();
            speed_d = spg[2].GetDouble();
        }
    }

    // Get path processing parameters
    double path_spacing = 0.5;
    double smoothing_window = 0.0;
    if (actor_cfg.HasMember("path_spacing") && actor_cfg["path_spacing"].IsNumber()) {
        path_spacing = std::max(0.05, actor_cfg["path_spacing"].GetDouble());
    }
    if (actor_cfg.HasMember("smooth_window") && actor_cfg["smooth_window"].IsNumber()) {
        smoothing_window = std::max(0.0, actor_cfg["smooth_window"].GetDouble());
    }

    // Load waypoints from CSV and build bezier curve
    std::vector<ChVector3d> waypoints;
    if (!LoadWaypointCSV(actor_path_file, waypoints) || waypoints.empty()) {
        std::cerr << "ERROR: Failed to load waypoints from " << actor_path_file << std::endl;
        return false;
    }

    std::vector<ChVector3d> path_points = BuildResampledPoints(waypoints, path_spacing);
    if (smoothing_window > 0.0) {
        path_points = SmoothPathPoints(path_points, path_spacing, smoothing_window);
    }

    if (path_points.empty()) {
        std::cerr << "ERROR: No path points after processing for actor " << actor_index << std::endl;
        return false;
    }

    auto actor_path = chrono_types::make_shared<ChBezierCurve>(path_points, false);

    auto actor_path_driver = chrono_types::make_shared<vehicle::ChPathFollowerDriver>(
        my_vehicle, actor_path, "actor_path", actor_target_speed);
    actor_path_driver->GetSteeringController().SetLookAheadDistance(look_ahead);
    actor_path_driver->GetSteeringController().SetGains(steering_p, steering_i, steering_d);
    actor_path_driver->GetSpeedController().SetGains(speed_p, speed_i, speed_d);
    actor_path_driver->Initialize();

    // Store in out_actor
    out_actor.path_driver = actor_path_driver;
    out_actor.waypoints = path_points;
    out_actor.look_ahead_distance = look_ahead;
    out_actor.steering_kp = steering_p;
    out_actor.steering_ki = steering_i;
    out_actor.steering_kd = steering_d;
    out_actor.path_spacing = path_spacing;
    out_actor.smoothing_window = smoothing_window;
    out_actor.vehicle = nullptr;  // Distributed actor uses my_vehicle directly

    std::cout << "Actor node " << node_id << " initialized path follower driver:" << std::endl;
    std::cout << "  Path file: " << actor_path_file << std::endl;
    std::cout << "  Start time: " << out_actor.start_time << " s" << std::endl;
    std::cout << "  Initial target speed: " << actor_target_speed << " m/s" << std::endl;
    std::cout << "  Look ahead: " << look_ahead << " m" << std::endl;
    std::cout << "  Steering gains (P/I/D): " << steering_p << "/" << steering_i << "/" << steering_d << std::endl;

    return true;
}

}  // namespace hil
}  // namespace chrono
