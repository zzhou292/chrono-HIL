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

#include "ChDelaySim.h"

namespace chrono
{
    namespace hil
    {

        ChDelaySim::ChDelaySim(float delayInterval)
            : delayInterval(delayInterval)
        {
        }

        ChDelaySim::~ChDelaySim()
        {
            // Destructor if any cleanup needed
        }

        void ChDelaySim::addPacket(const std::vector<char> &data)
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            packetQueue.push({std::chrono::steady_clock::now(), data});
        }

        std::vector<char> ChDelaySim::getDelayedPacket()
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (!packetQueue.empty())
            {
                auto &front = packetQueue.front();
                auto currentTime = std::chrono::steady_clock::now();
                auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - front.first).count();
                if (delay >= delayInterval)
                {
                    std::vector<char> data = front.second;
                    packetQueue.pop();
                    return data;
                }
            }
            return std::vector<char>(); // Return empty vector if no packet is ready
        }

        void ChDelaySim::displayData(const std::vector<char> &data)
        {
            std::cout << "Packet data: ";
            for (auto byte : data)
            {
                std::cout << static_cast<int>(byte) << " ";
            }
            std::cout << std::endl;
        }

    }
}