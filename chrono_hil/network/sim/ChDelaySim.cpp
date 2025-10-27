#include "ChDelaySim.h"
#include <fstream>
#include "chrono_vehicle/utils/ChUtilsJSON.h"

namespace chrono
{
    namespace hil
    {

        ChDelaySim::ChDelaySim(std::shared_ptr<DelayDistribution> distribution, float bandwidthLimit)
            : delayDistribution(distribution), bandwidthLimit(bandwidthLimit), currentSimTime(0.0)
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
            
            // Calculate bandwidth, avoiding division by zero or very small delays
            float currentBandwidth = 0.0f;
            if (expectedDelay > 0.01f) {  // Only check bandwidth if delay is meaningful (> 0.01ms)
                currentBandwidth = packetQueue.size() * 8 / (expectedDelay * 1e-3);
            }

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

        bool ChDelaySim::loadDelayConfig(const std::string& jsonFilePath)
        {
            rapidjson::Document d;
            chrono::vehicle::ReadFileJSON(jsonFilePath, d);

            if (!d.IsObject()) {
                std::cerr << "Error: Invalid JSON format in delay config file: " << jsonFilePath << std::endl;
                return false;
            }

            delayPeriods.clear();

            // Load bandwidth limit if specified
            if (d.HasMember("bandwidth_limit")) {
                bandwidthLimit = d["bandwidth_limit"].GetFloat();
                if (enableLogging) {
                    std::cout << "Set bandwidth limit to: " << bandwidthLimit << " packets/s" << std::endl;
                }
            }

            if (d.HasMember("delay_periods") && d["delay_periods"].IsArray()) {
                const rapidjson::Value& periods = d["delay_periods"];
                
                for (rapidjson::SizeType i = 0; i < periods.Size(); i++) {
                    const rapidjson::Value& period = periods[i];
                    
                    DelayPeriod dp;
                    dp.startTime = period["start_time"].GetDouble();
                    dp.endTime = period["end_time"].GetDouble();
                    dp.delayMean = period["delay_mean"].GetFloat();
                    dp.delayStddev = period.HasMember("delay_stddev") ? period["delay_stddev"].GetFloat() : 0.001f;
                    
                    delayPeriods.push_back(dp);
                    
                    if (enableLogging) {
                        std::cout << "Loaded delay period: [" << dp.startTime << ", " << dp.endTime 
                                  << "] with mean=" << dp.delayMean << "ms, stddev=" << dp.delayStddev << "ms" << std::endl;
                    }
                }
            }

            if (enableLogging) {
                std::cout << "Successfully loaded " << delayPeriods.size() << " delay periods from " << jsonFilePath << std::endl;
            }

            return true;
        }

        void ChDelaySim::updateDelayForTime(double currentSimTime)
        {
            this->currentSimTime = currentSimTime;
            
            // Check if current time falls within any delay period
            for (const auto& period : delayPeriods) {
                if (currentSimTime >= period.startTime && currentSimTime <= period.endTime) {
                    // We're in a delay period, update the distribution
                    auto newDist = std::make_shared<NormalDistribution>(period.delayMean, period.delayStddev);
                    changeDistribution(newDist);
                    
                    if (enableLogging) {
                        static double lastLogTime = -1.0;
                        // Log only when entering a new second to reduce spam
                        if (std::floor(currentSimTime) != std::floor(lastLogTime)) {
                            std::cout << "Time " << currentSimTime << "s: Applying delay (mean=" 
                                      << period.delayMean << "ms, stddev=" << period.delayStddev << "ms)" << std::endl;
                            lastLogTime = currentSimTime;
                        }
                    }
                    return;
                }
            }
            
            // Not in any delay period, use zero delay
            auto zeroDist = std::make_shared<NormalDistribution>(0.0f, 0.001f);
            changeDistribution(zeroDist);
        }

    } // namespace hil
} // namespace chrono
