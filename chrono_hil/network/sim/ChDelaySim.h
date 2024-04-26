// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
// This is a stream-based input driver interface based on boost TCP networking
// =============================================================================
#ifndef CH_DELAY_SIM_H
#define CH_DELAY_SIM_H

#include <queue>
#include <vector>
#include <mutex>
#include <chrono>
#include <iostream>

#include "../../ChApiHil.h"

namespace chrono
{
    namespace hil
    {

        struct Packet
        {
            std::vector<char> data;
            std::chrono::steady_clock::time_point timestamp;
        };

        class ChDelaySim
        {
        public:
            ChDelaySim(float delayInterval);
            ~ChDelaySim();

            void addPacket(const std::vector<char> &data);
            std::vector<char> getDelayedPacket();

        private:
            std::queue<std::pair<std::chrono::steady_clock::time_point, std::vector<char>>> packetQueue;
            std::mutex queueMutex;
            float delayInterval; // Delay in milliseconds

            static void displayData(const std::vector<char> &data);
        };
    }
}

#endif // CH_DELAY_SIM_H