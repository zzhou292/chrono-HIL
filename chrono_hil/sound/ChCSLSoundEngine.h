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
// Interactive driver for a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
//
// =============================================================================

#ifndef CH_SOUND_ENGINE_H
#define CH_SOUND_ENGINE_H
#ifdef CHRONO_IRRKLANG
#include <string>

#include "../ChApiHil.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"

#include <irrKlang.h>

using namespace chrono::vehicle;

namespace chrono {
namespace hil {
/// Sound effect tools for the CSL simulator
///
class CH_HIL_API ChCSLSoundEngine {
public:
  /// Construct the sound reproduction engine
  ChCSLSoundEngine(ChVehicle *vehicle) {
    thisvehicle = vehicle;
    sound_engine = irrklang::createIrrKlangDevice();
    car_sound = sound_engine->play2D((std::string(STRINGIFY(HIL_DATA_DIR)) +
                                      "/Environments/Iowa/Sounds/audi_dr.ogg")
                                         .c_str(),
                                     true, false, true);
    car_sound->setIsPaused(true);
  }

  ~ChCSLSoundEngine() { delete sound_engine; };

  /// Updates sound engine
  void Synchronize(double time) {
    // update every 0.01 sec
    if (time - last_time_played > 0.2) {
      last_time_played = time;
      int cur_gear = thisvehicle->GetTransmission()->GetCurrentGear();
      double rpm = thisvehicle->GetEngine()->GetMotorSpeed() * 60 / CH_C_2PI;
      if (cur_gear == 1) {
        double soundspeed = rpm / (10000.); // denominator: to guess
        if (soundspeed < 0.1)
          soundspeed = 0.1;
        if (car_sound->getIsPaused())
          car_sound->setIsPaused(false);
        car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
      } else if (cur_gear == 2) {
        double soundspeed = rpm / (8000.); // denominator: to guess
        if (soundspeed < 0.1)
          soundspeed = 0.1;
        if (car_sound->getIsPaused())
          car_sound->setIsPaused(false);
        car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
      } else if (cur_gear == 3) {
        double soundspeed = rpm / (8000.); // denominator: to guess
        if (soundspeed < 0.1)
          soundspeed = 0.1;
        if (car_sound->getIsPaused())
          car_sound->setIsPaused(false);
        car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
      } else {
        double soundspeed = rpm / (6000.); // denominator: to guess
        if (soundspeed < 0.1)
          soundspeed = 0.1;
        if (car_sound->getIsPaused())
          car_sound->setIsPaused(false);
        car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
      }
    }
  }

private:
  ChVehicle *thisvehicle;
  irrklang::ISound *car_sound;
  // irrklang::ISound* motor_sound;
  irrklang::ISoundEngine *sound_engine;
  // std::vector<std::string> motor_soundfiles;
  // std::vector<irrklang::ISound*> motor_sounds;
  double last_time_played = 0;
  int last_threshold = 0;
};
} // end namespace hil
} // end namespace chrono
#endif
#endif
