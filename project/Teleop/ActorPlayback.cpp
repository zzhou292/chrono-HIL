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
// Actor playback system implementation
// =============================================================================

#include "ActorPlayback.h"

#include <algorithm>
#include <iostream>
#include <iomanip>

namespace chrono {
namespace hil {

bool ParseSpeedProfileJSON(const rapidjson::Value& profile_json, ActorPlayback& actor) {
    if (!profile_json.IsObject() || !profile_json.HasMember("type") || !profile_json.HasMember("entries")) {
        std::cerr << "Speed profile must contain 'type' and 'entries'.\n";
        return false;
    }

    std::string type = profile_json["type"].GetString();
    if (type == "velocity") {
        actor.profile_type = SpeedProfileType::VELOCITY;
        actor.initial_speed = 0.0;
        actor.max_decel = profile_json.HasMember("max_decel") ? profile_json["max_decel"].GetDouble() : 3.0;
        if (actor.max_decel < 0.1)
            actor.max_decel = 3.0;
    } else if (type == "acceleration") {
        actor.profile_type = SpeedProfileType::ACCELERATION;
        actor.initial_speed = profile_json.HasMember("initial_speed") ? profile_json["initial_speed"].GetDouble() : 0.0;
        actor.current_speed = actor.initial_speed;
        actor.max_decel = profile_json.HasMember("max_decel") ? std::max(0.1, profile_json["max_decel"].GetDouble()) : 3.0;
    } else {
        std::cerr << "Unknown speed profile type: " << type << "\n";
        return false;
    }

    const auto& entries = profile_json["entries"];
    if (!entries.IsArray() || entries.Empty()) {
        std::cerr << "Speed profile entries must be a non-empty array.\n";
        return false;
    }

    actor.profile_segments.clear();
    for (const auto& entry : entries.GetArray()) {
        if (!entry.HasMember("start_time") || !entry.HasMember("value")) {
            std::cerr << "Each speed profile entry must have 'start_time' and 'value'.\n";
            return false;
        }

        SpeedProfileSegment seg;
        seg.start_time = entry["start_time"].GetDouble();
        seg.value = entry["value"].GetDouble();
        seg.until_end = entry.HasMember("until_end") && entry["until_end"].GetBool();
        
        if (entry.HasMember("end_time")) {
            seg.end_time = entry["end_time"].GetDouble();
            seg.has_explicit_end = true;
        } else if (entry.HasMember("duration")) {
            seg.end_time = seg.start_time + entry["duration"].GetDouble();
            seg.has_explicit_end = true;
        } else if (seg.until_end) {
            seg.end_time = std::numeric_limits<double>::infinity();
        }
        actor.profile_segments.push_back(seg);
    }

    // Sort segments by start time
    std::sort(actor.profile_segments.begin(), actor.profile_segments.end(),
              [](const SpeedProfileSegment& a, const SpeedProfileSegment& b) { 
                  return a.start_time < b.start_time; 
              });

    // Fill in implicit end times
    for (size_t i = 0; i + 1 < actor.profile_segments.size(); ++i) {
        auto& seg = actor.profile_segments[i];
        auto& next = actor.profile_segments[i + 1];
        if (!seg.until_end && !seg.has_explicit_end) {
            seg.end_time = next.start_time;
        } else if (!seg.until_end && seg.end_time > next.start_time) {
            seg.end_time = next.start_time;
        }
    }

    actor.profile_defined = true;
    actor.last_profile_time = 0.0;
    
    if (actor.profile_type == SpeedProfileType::VELOCITY) {
        actor.initial_speed = actor.profile_segments.front().value;
        actor.current_speed = actor.initial_speed;
    } else {
        actor.current_speed = actor.initial_speed;
    }
    
    // Debug: Print parsed profile
    std::cout << "[SPEED PROFILE] Parsed " << actor.profile_segments.size() << " segment(s), type: " 
              << (actor.profile_type == SpeedProfileType::VELOCITY ? "VELOCITY" : "ACCELERATION")
              << ", initial_speed: " << actor.initial_speed << " m/s" << std::endl;
    for (size_t i = 0; i < actor.profile_segments.size(); ++i) {
        const auto& seg = actor.profile_segments[i];
        std::cout << "  [Segment " << i << "] start_time=" << seg.start_time 
                  << "s, end_time=" << (seg.end_time == std::numeric_limits<double>::infinity() ? "INF" : std::to_string(seg.end_time))
                  << "s, value=" << seg.value 
                  << (actor.profile_type == SpeedProfileType::VELOCITY ? " m/s" : " m/s^2")
                  << (seg.until_end ? " (until_end)" : "") << std::endl;
    }
    
    return true;
}

const SpeedProfileSegment* GetActiveSegment(const ActorPlayback& actor, double local_time) {
    const SpeedProfileSegment* active = nullptr;
    for (const auto& segment : actor.profile_segments) {
        if (local_time < segment.start_time)
            break;
        if (segment.until_end || local_time < segment.end_time)
            active = &segment;
    }
    return active;
}

double EvaluateDesiredSpeed(ActorPlayback& actor, double local_time, double step, bool within_stop_zone) {
    if (!actor.profile_defined || actor.profile_segments.empty())
        return 0.0;

    const SpeedProfileSegment* segment = GetActiveSegment(actor, local_time);
    
    // Debug logging (only log periodically to avoid spam)
    static double last_debug_time = -1.0;
    bool should_log = (local_time - last_debug_time >= 1.0) || (local_time < 0.1 && last_debug_time < 0.0);
    
    if (actor.profile_type == SpeedProfileType::VELOCITY) {
        double target = segment ? std::max(0.0, segment->value) : 0.0;
        double dt = step;
        if (within_stop_zone) {
            actor.current_speed = std::max(0.0, actor.current_speed - actor.max_decel * dt);
            target = std::min(target, actor.current_speed);
        } else {
            actor.current_speed = target;
        }
        // Debug output for velocity profile
        if (should_log) {
            last_debug_time = local_time;
            std::cout << "[VEL PROFILE] local_time=" << std::fixed << std::setprecision(2) << local_time 
                      << "s, segment=" << (segment ? "FOUND" : "NONE");
            if (!segment && !actor.profile_segments.empty()) {
                std::cout << " (first segment starts at " << actor.profile_segments.front().start_time << "s)";
            }
            std::cout << ", target=" << target << " m/s" << std::endl;
        }
        return target;
    }

    // Acceleration profile
    const double configured_accel = segment ? segment->value : 0.0;
    double accel = configured_accel;
    double dt = local_time - actor.last_profile_time;
    
    // Use step size if dt is invalid or very small (first frame after activation)
    if (dt < step * 0.5 || dt > 1.0)
        dt = step;
        
    if (within_stop_zone && accel > 0) {
        accel = -actor.max_decel;
    }
    
    double old_speed = actor.current_speed;
    actor.current_speed = std::max(0.0, actor.current_speed + accel * dt);
    actor.last_profile_time = local_time;
    
    // Debug output
    if (should_log) {
        last_debug_time = local_time;
        std::cout << "[ACCEL PROFILE] "
                  << "local_time=" << std::fixed << std::setprecision(2) << local_time << "s"
                  << ", segment=" << (segment ? "ACTIVE" : "NONE");
        if (!segment && !actor.profile_segments.empty()) {
            std::cout << " (first segment starts at " << actor.profile_segments.front().start_time << "s)";
        }
        std::cout << ", prev_speed=" << std::setprecision(2) << old_speed << " m/s"
                  << ", configured_accel=" << std::setprecision(3) << configured_accel << " m/s^2"
                  << ", applied_accel=" << accel << " m/s^2"
                  << ", dt=" << std::setprecision(4) << dt << " s"
                  << ", desired_vel=" << std::setprecision(2) << actor.current_speed << " m/s"
                  << ", current_speed=" << std::setprecision(2) << actor.current_speed << " m/s"
                  << std::endl;
    }
    
    return actor.current_speed;
}

}  // namespace hil
}  // namespace chrono
