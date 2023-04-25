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

  // Initialize the joystick subsystem
  SDL_Init(SDL_INIT_JOYSTICK);

  // If there are no joysticks connected, quit the program
  if (SDL_NumJoysticks() <= 0) {
    printf("There are no joysticks connected. Quitting now...\n");
    SDL_Quit();
  }

  // Open the joystick for reading and store its handle in the joy variable
  m_joystick = SDL_JoystickOpen(0);

  // If the joy variable is NULL, there was an error opening it.
  if (m_joystick != NULL) {
    // Get information about the joystick
    const char *name = SDL_JoystickName(m_joystick);
    const int num_axes = SDL_JoystickNumAxes(m_joystick);
    const int num_buttons = SDL_JoystickNumButtons(m_joystick);
    const int num_hats = SDL_JoystickNumHats(m_joystick);

    printf("Detected joystick '%s' with:\n"
           "%d axes\n"
           "%d buttons\n"
           "%d hats\n\n",
           name, num_axes, num_buttons, num_hats);

  } else {
    printf("Couldn't open the joystick. Quitting now...\n");
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