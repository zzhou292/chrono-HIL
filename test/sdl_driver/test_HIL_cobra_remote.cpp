#include <chrono>
#include <iostream>
#include <stdint.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_hil/driver/ChSDLInterface.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <unistd.h>

using namespace chrono::vehicle;
using namespace chrono::hil;

#define PORT_OUT 1209
#define IP_OUT "10.140.224.134"

// This program shows how to work with joysticks using SDL2.
// This example shows how to do it by manually polling the joystick
// rather than using the sdl event queue.
int main() {
  ChSDLInterface SDLDriver;
  // Set the time response for steering and throttle keyboard inputs.

  SDLDriver.Initialize();

  std::string joystick_file =
      (STRINGIFY(HIL_DATA_DIR)) + std::string("/joystick/controller_G27.json");
  SDLDriver.SetJoystickConfigFile(joystick_file);

  // create boost data streaming interface
  ChBoostOutStreamer boost_streamer(IP_OUT, PORT_OUT);

  while (true) {

    // get the controls for this time step
    // Driver inputs
    DriverInputs driver_inputs;
    driver_inputs.m_steering = SDLDriver.GetSteering();
    driver_inputs.m_throttle = SDLDriver.GetThrottle();
    driver_inputs.m_braking = SDLDriver.GetBraking();

    std::cout << "steer:" << driver_inputs.m_steering << std::endl;
    std::cout << "throttle:" << driver_inputs.m_throttle << std::endl;
    std::cout << "brake:" << driver_inputs.m_braking << std::endl;

    boost_streamer.AddData(driver_inputs.m_steering);
    boost_streamer.AddData(driver_inputs.m_throttle);
    boost_streamer.AddData(driver_inputs.m_braking);

    boost_streamer.Synchronize();

    usleep(10000);

    if (SDLDriver.Synchronize() == 1) {
      break;
    }
  }
  return 0;
}