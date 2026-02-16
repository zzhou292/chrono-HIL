#include "ChDelaySim.h"
#include <fstream>
#include <cmath>
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
            
            // Sample delay at SEND time (this is the key fix!)
            float sampledDelay = delayDistribution->sample(generator);
            
            // Quantize delay to nearest step if quantization is enabled
            if (quantizationStep > 0.0f) {
                sampledDelay = std::round(sampledDelay / quantizationStep) * quantizationStep;
            }
            if (sampledDelay < 0.0f) sampledDelay = 0.0f;
            
            lastSampledDelayMs = sampledDelay;
            
            // Calculate apply time = current wall time + sampled delay
            auto applyTime = currentTime + std::chrono::milliseconds(static_cast<long long>(sampledDelay));
            
            // Calculate bandwidth, avoiding division by zero or very small delays
            float currentBandwidth = 0.0f;
            if (sampledDelay > 0.01f) {  // Only check bandwidth if delay is meaningful (> 0.01ms)
                currentBandwidth = packetQueue.size() * 8 / (sampledDelay * 1e-3);
            }

            if (currentBandwidth < bandwidthLimit)
            {
                DelayedPacket pkt;
                pkt.applyTime = applyTime;
                pkt.sourceTime = currentTime;
                pkt.sampledDelayMs = sampledDelay;
                pkt.data = data;
                
                packetQueue.push_back(pkt);
                // Maintain heap property (min-heap by apply time)
                std::push_heap(packetQueue.begin(), packetQueue.end(), 
                    [](const DelayedPacket& a, const DelayedPacket& b) {
                        return a.applyTime > b.applyTime;  // Min-heap: earliest apply time first
                    });
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

            // Process all packets whose apply time has arrived
            while (!packetQueue.empty())
            {
                // Peek at the earliest packet (min-heap: front has smallest apply time)
                const DelayedPacket& front = packetQueue.front();
                
                // Check if this packet's apply time has arrived
                if (front.applyTime > currentTime) {
                    break; // No more packets ready yet
                }
                
                // Pop the packet from the heap
                std::pop_heap(packetQueue.begin(), packetQueue.end(),
                    [](const DelayedPacket& a, const DelayedPacket& b) {
                        return a.applyTime > b.applyTime;
                    });
                DelayedPacket pkt = packetQueue.back();
                packetQueue.pop_back();
                
                // Anti-rewind check: prevent old commands from overwriting newer ones
                // This handles cases where a packet with large latency arrives after
                // a packet with small latency that was sent later
                if (enableAntiRewind && hasAppliedAnyPacket) {
                    if (pkt.sourceTime < lastAppliedSourceTime) {
                        antiRewindDiscardCount_buffer++;
                        continue; // Skip this stale packet
                    }
                }
                
                // Log the actual delay experienced
                if (enableLogging) {
                    auto actualDelay = std::chrono::duration_cast<std::chrono::milliseconds>(
                        currentTime - pkt.sourceTime).count();
                    delay_buffer.push_back(static_cast<float>(actualDelay));
                }
                
                // Apply this packet
                latestData = pkt.data;
                lastAppliedSourceTime = pkt.sourceTime;
                hasAppliedAnyPacket = true;
                lastSampledDelayMs = pkt.sampledDelayMs;
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

        int ChDelaySim::getAntiRewindDiscardCount()
        {
            int res = antiRewindDiscardCount_buffer;
            antiRewindDiscardCount_buffer = 0;
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

        float ChDelaySim::getExpectedDelayMs()
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            return lastSampledDelayMs;
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
