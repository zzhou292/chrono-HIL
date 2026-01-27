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
    
    if (actor.profile_type == SpeedProfileType::VELOCITY) {
        double target = segment ? std::max(0.0, segment->value) : 0.0;
        double dt = step;
        if (within_stop_zone) {
            actor.current_speed = std::max(0.0, actor.current_speed - actor.max_decel * dt);
            target = std::min(target, actor.current_speed);
        } else {
            actor.current_speed = target;
        }
        return target;
    }

    // Acceleration profile
    double accel = segment ? segment->value : 0.0;
    double dt = local_time - actor.last_profile_time;
    if (dt < 0.0 || dt > 1.0)
        dt = step;
    if (within_stop_zone && accel > 0) {
        accel = -actor.max_decel;
    }
    actor.current_speed = std::max(0.0, actor.current_speed + accel * dt);
    actor.last_profile_time = local_time;
    return actor.current_speed;
}

}  // namespace hil
}  // namespace chrono
