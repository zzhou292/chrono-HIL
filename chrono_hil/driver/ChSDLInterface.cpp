// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// Jason Zhou
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// This is a class which intends to replace ChIrrGuiDriver with SDL backed
// joystick input reading
// This class is not currently inheriting ChDriver, as it's also intended to
// support ROM vehicle model control
//
// =============================================================================

#include "ChSDLInterface.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

namespace chrono {
namespace hil {

ChSDLInterface::ChSDLInterface() {}
ChSDLInterface::~ChSDLInterface() {
  SDL_JoystickClose(m_joystick);
  SDL_Quit();
}
void ChSDLInterface::Initialize() {

  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        return;
    }

    int num_joy = SDL_NumJoysticks();
    if (num_joy <= 0) {
        printf("There are no joysticks connected. Quitting now...\n");
        SDL_Quit();
        return;
    }

    printf("Found %d joystick(s):\n", num_joy);
    for (int i = 0; i < num_joy; ++i) {
        const char* name = SDL_JoystickNameForIndex(i);
        printf("  [%d] %s\n", i, name ? name : "(unknown)");
    }

    // Optional: env var to force a particular device by substring
    const char* preferred_substr = std::getenv("CHRONO_JOYSTICK_NAME");

    int chosen_index = -1;
    for (int i = 0; i < num_joy; ++i) {
        const char* name = SDL_JoystickNameForIndex(i);
        if (!name)
            continue;

        // If user specified a preferred name substring, use that
        if (preferred_substr && std::strstr(name, preferred_substr)) {
            chosen_index = i;
            break;
        }

        // Otherwise, skip obvious non-gamepad devices (tweak these as needed)
        if (!preferred_substr) {
            if (std::strstr(name, "Keyboard") || std::strstr(name, "RDMCTMZT") || std::strstr(name, "KVM")) {
                continue;
            }

            // First non-keyboard/KVM device wins
            if (chosen_index == -1)
                chosen_index = i;
        }
    }

    // If nothing matched, just fall back to 0
    if (chosen_index == -1)
        chosen_index = 0;

    printf("Opening joystick index %d\n", chosen_index);
    m_joystick = SDL_JoystickOpen(chosen_index);

    if (m_joystick != NULL) {
        const char *name = SDL_JoystickName(m_joystick);
        const int num_axes    = SDL_JoystickNumAxes(m_joystick);
        const int num_buttons = SDL_JoystickNumButtons(m_joystick);
        const int num_hats    = SDL_JoystickNumHats(m_joystick);

        printf("Using joystick '%s' with:\n"
               "  %d axes\n"
               "  %d buttons\n"
               "  %d hats\n\n",
               name ? name : "(unknown)", num_axes, num_buttons, num_hats);
    } else {
        printf("Couldn't open the joystick: %s\n", SDL_GetError());
    }
}

void ChSDLInterface::SetJoystickConfigFile(std::string config_filename) {
  // read from joystick json file, this process is mimicing ChIrrGuiDriver
  // currently, only read from steering, throttle, and braking
  rapidjson::Document d;
  chrono::vehicle::ReadFileJSON(config_filename, d);

  if (d.HasMember("steering")) {
    m_steering_axis.axis = d["steering"]["axis"].GetInt();
    m_steering_axis.min = d["steering"]["min"].GetDouble();
    m_steering_axis.max = d["steering"]["max"].GetDouble();
    m_steering_axis.scaled_min = d["steering"]["scaled_min"].GetDouble();
    m_steering_axis.scaled_max = d["steering"]["scaled_max"].GetDouble();
  }

  if (d.HasMember("throttle")) {
    m_throttle_axis.axis = d["throttle"]["axis"].GetInt();
    m_throttle_axis.min = d["throttle"]["min"].GetDouble();
    m_throttle_axis.max = d["throttle"]["max"].GetDouble();
    m_throttle_axis.scaled_min = d["throttle"]["scaled_min"].GetDouble();
    m_throttle_axis.scaled_max = d["throttle"]["scaled_max"].GetDouble();
  }

  if (d.HasMember("brake")) {
    m_braking_axis.axis = d["brake"]["axis"].GetInt();
    m_braking_axis.min = d["brake"]["min"].GetDouble();
    m_braking_axis.max = d["brake"]["max"].GetDouble();
    m_braking_axis.scaled_min = d["brake"]["scaled_min"].GetDouble();
    m_braking_axis.scaled_max = d["brake"]["scaled_max"].GetDouble();
  }
}

float ChSDLInterface::GetThrottle() {
  // NOTE: SDL_QuitRequested() has to be called to make program run properly
  SDL_JoystickUpdate();
  float sdl_raw = SDL_JoystickGetAxis(m_joystick, m_throttle_axis.axis);
  return (sdl_raw - m_throttle_axis.max) *
             (m_throttle_axis.scaled_max - m_throttle_axis.scaled_min) /
             (m_throttle_axis.max - m_throttle_axis.min) +
         m_throttle_axis.scaled_max;
}

float ChSDLInterface::GetSteering() {
  // NOTE: SDL_QuitRequested() has to be called to make program run properly
  SDL_JoystickUpdate();
  float sdl_raw = SDL_JoystickGetAxis(m_joystick, m_steering_axis.axis);
  return (sdl_raw - m_steering_axis.max) *
             (m_steering_axis.scaled_max - m_steering_axis.scaled_min) /
             (m_steering_axis.max - m_steering_axis.min) +
         m_steering_axis.scaled_max;
}

float ChSDLInterface::GetBraking() {
  // NOTE: SDL_QuitRequested() has to be called to make program run properly
  SDL_JoystickUpdate();
  float sdl_raw = SDL_JoystickGetAxis(m_joystick, m_braking_axis.axis);
  return (sdl_raw - m_braking_axis.max) *
             (m_braking_axis.scaled_max - m_braking_axis.scaled_min) /
             (m_braking_axis.max - m_braking_axis.min) +
         m_braking_axis.scaled_max;
}

void ChSDLInterface::AddCallbackButtons(int button) {
  m_active_buttons_idx.push_back(button);
  m_active_buttons_val.push_back(false);
}

void ChSDLInterface::GetButtonStatus(std::vector<int> &ref_idx,
                                     std::vector<int> &ref_val) {
  for (int i = 0; i < m_active_buttons_idx.size(); i++) {
    m_active_buttons_val[i] =
        SDL_JoystickGetButton(m_joystick, m_active_buttons_idx[i]);
  }
  ref_idx = m_active_buttons_idx;
  ref_val = m_active_buttons_val;
}

int ChSDLInterface::Synchronize() {
  if (SDL_QuitRequested()) {
    return 1;
  } else {
    return 0;
  }
}

} // namespace hil
} // namespace chrono