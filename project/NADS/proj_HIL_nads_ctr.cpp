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

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_hil/driver/ChSDLInterface.h"

#include "chrono_hil/network/udp/ChBoostInStreamer.h"
#include "chrono_hil/network/udp/ChBoostOutStreamer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace chrono;
using namespace chrono::hil;

using boost::asio::ip::address;
using boost::asio::ip::udp;

#define PORT_IN 1204
#define PORT_OUT 1209
#define IP_OUT "127.0.0.1"

int main(int argc, char *argv[]) {
  ChSDLInterface SDLDriver;

  SDLDriver.Initialize();

  SDLDriver.SetJoystickConfigFile(std::string(STRINGIFY(HIL_DATA_DIR)) +
                                  std::string("/joystick/controller_G27.json"));
  SDLDriver.AddCallbackButtons(6);
  float gear =
      0.f; // 0.0 for park, 1.0 for forward, 2.0 for backward, 3.0 for neutral

  std::vector<int> check_button_idx;
  std::vector<int> check_button_val;

  ChBoostOutStreamer out_streamer(IP_OUT, PORT_OUT);
  ChBoostInStreamer in_streamer(PORT_IN, 19);

  std::vector<float> recv_data;

  while (true) {
    // data out
    float throttle = SDLDriver.GetThrottle();
    float steering = SDLDriver.GetSteering();
    float braking = SDLDriver.GetBraking();

    out_streamer.AddData(throttle); // out - 0 - throttle
    out_streamer.AddData(steering); // out - 1 - steering
    out_streamer.AddData(braking);  // out - 2 - braking

    SDLDriver.GetButtonStatus(check_button_idx, check_button_val);
    if (check_button_val[0] == 1) {
      static auto last_invoked_1 =
          std::chrono::system_clock::now().time_since_epoch();
      auto current_invoke_1 =
          std::chrono::system_clock::now().time_since_epoch();

      if (std::chrono::duration_cast<std::chrono::seconds>(current_invoke_1 -
                                                           last_invoked_1)
              .count() >= 1.0) {
        gear = ((int)(gear + 1.0)) % 4;

        last_invoked_1 = current_invoke_1;
      }
    }
    out_streamer.AddData(gear);

    out_streamer.Synchronize();

    // data in
    in_streamer.Synchronize();

    // recv_data = in_streamer.GetRecvData();
    //  for (int i = 0; i < recv_data.size(); i++) {
    //    std::cout << recv_data[i] << ",";
    //  }
    //  std::cout << std::endl;

    if (SDLDriver.Synchronize() == 1) {
      break;
    }
  }

  return 0;
}
