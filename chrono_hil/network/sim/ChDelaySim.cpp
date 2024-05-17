#include "ChDelaySim.h"

namespace chrono
{
    namespace hil
    {

        ChDelaySim::ChDelaySim(std::shared_ptr<DelayDistribution> distribution, float bandwidthLimit)
            : delayDistribution(distribution), bandwidthLimit(bandwidthLimit)
        {
            std::random_device rd;
            generator.seed(rd());
        }

        ChDelaySim::~ChDelaySim()
        {
            // Destructor if any cleanup needed
        }

        void ChDelaySim::addPacket(const std::vector<char> &data)
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            auto currentTime = std::chrono::steady_clock::now();
            float currentBandwidth = packetQueue.size() * 8 / (expectedDelay * 1e-3);

            if (currentBandwidth < bandwidthLimit)
            {
                packetQueue.push({currentTime, data});
            }
            else
            {
                if (enableLogging)
                {
                    std::cout << "Packet dropped due to bandwidth limit. Current bandwidth: " << currentBandwidth << " packets/s" << std::endl;
                    packetDropCount_buffer++;
                }
            }
        }

        std::vector<char> ChDelaySim::getDelayedPacket()
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            auto currentTime = std::chrono::steady_clock::now();
            expectedDelay = delayDistribution->sample(generator);

            while (!packetQueue.empty())
            {
                auto &front = packetQueue.front();
                auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - front.first).count();
                if (enableLogging)
                {
                    delay_buffer.push_back(delay);
                }

                if (delay >= expectedDelay)
                {
                    latestData = front.second;
                    packetQueue.pop();
                }
                else
                {
                    break; // Stop processing if the next packet is not ready yet
                }
            }
            return latestData;
        }

        void ChDelaySim::changeDistribution(std::shared_ptr<DelayDistribution> newDistribution)
        {
            delayDistribution = newDistribution;
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

        int ChDelaySim::getPacketDropCount()
        {
            int res = packetDropCount_buffer;
            packetDropCount_buffer = 0;
            return res;
        }

        std::vector<float> ChDelaySim::getDelayBuffer()
        {
            std::vector<float> res;
            res = delay_buffer;
            delay_buffer.clear();
            delay_buffer.resize(0);
            return res;
        }

    } // namespace hil
} // namespace chrono
