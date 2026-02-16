#ifndef CH_DELAY_SIM_H
#define CH_DELAY_SIM_H

#include <queue>
#include <vector>
#include <mutex>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <boost/math/distributions/normal.hpp> // Include Boost normal distribution

#include "../../ChApiHil.h"

namespace chrono
{
    namespace hil
    {

        // Interface for delay distributions
        class DelayDistribution
        {
        public:
            virtual ~DelayDistribution() {}
            virtual float sample(std::default_random_engine &generator) = 0;
        };

        // Normal distribution using Boost
        class NormalDistribution : public DelayDistribution
        {
        private:
            boost::math::normal dist; // Boost normal distribution
        public:
            NormalDistribution(float mean, float stddev) : dist(mean, stddev) {}
            float sample(std::default_random_engine &generator) override
            {
                std::uniform_real_distribution<> dist(0, 1); // Use a uniform distribution to generate a quantile
                double quantile = dist(generator);
                return boost::math::quantile(this->dist, quantile);
            }
        };

        // Modify the ChDelaySim class to use DelayDistribution
        class ChDelaySim
        {
        public:
            // Structure to hold delay configuration for a specific time period
            struct DelayPeriod {
                double startTime;  // Simulation time when this delay period starts (seconds)
                double endTime;    // Simulation time when this delay period ends (seconds)
                float delayMean;   // Mean delay in milliseconds
                float delayStddev; // Standard deviation of delay in milliseconds
            };
            
            // Structure for a delayed packet with apply-time semantics
            struct DelayedPacket {
                std::chrono::steady_clock::time_point applyTime;  // Wall time when packet should be released
                std::chrono::steady_clock::time_point sourceTime; // Wall time when packet was sent
                float sampledDelayMs;  // The delay that was sampled for this packet
                std::vector<char> data;
            };

            ChDelaySim(std::shared_ptr<DelayDistribution> distribution, float bandwidthLimit);
            ~ChDelaySim();

            void addPacket(const std::vector<char> &data);
            std::vector<char> getDelayedPacket();
            void changeDistribution(std::shared_ptr<DelayDistribution> newDistribution);

            // New methods for JSON-based configuration
            bool loadDelayConfig(const std::string& jsonFilePath);
            void updateDelayForTime(double currentSimTime);
            
            // Get/Set bandwidth limit
            void setBandwidthLimit(float limit) { bandwidthLimit = limit; }
            float getBandwidthLimit() const { return bandwidthLimit; }

            void setLogging(bool enable) { enableLogging = enable; }

            // Set delay quantization step (ms). If > 0, delays are rounded to nearest multiple.
            // E.g., quantizationStep=10 means delays like 217.8 become 220.
            void setQuantizationStep(float stepMs) { quantizationStep = stepMs; }
            float getQuantizationStep() const { return quantizationStep; }
            
            // Enable/disable anti-rewind logic (prevents old packets from overwriting newer ones)
            void setAntiRewind(bool enable) { enableAntiRewind = enable; }
            bool getAntiRewind() const { return enableAntiRewind; }

            int getPacketDropCount();
            int getAntiRewindDiscardCount();
            std::vector<float> getDelayBuffer();
            float getExpectedDelayMs();

        private:
            // Priority queue ordered by apply time (earliest first)
            // Using vector + make_heap for priority queue with custom comparator
            std::vector<DelayedPacket> packetQueue;
            std::mutex queueMutex;
            std::vector<char> latestData; // Store the latest released packet data
            float lastSampledDelayMs = 0.0f;   // Last delay that was sampled (for logging)
            
            // Anti-rewind: track the newest source time that has been applied
            std::chrono::steady_clock::time_point lastAppliedSourceTime;
            bool hasAppliedAnyPacket = false;
            bool enableAntiRewind = true;  // Enabled by default
            float quantizationStep = 0.0f; // Delay quantization step in ms (0 = disabled)

            std::default_random_engine generator;
            std::shared_ptr<DelayDistribution> delayDistribution; // Use shared_ptr for flexibility

            float bandwidthLimit; // Maximum bandwidth limit in packets per millisecond

            // Delay configuration
            std::vector<DelayPeriod> delayPeriods;
            double currentSimTime;

            static void displayData(const std::vector<char> &data);

            // logging related variables
            bool enableLogging = false;
            int packetDropCount_buffer = 0; // Count of dropped packets
            int antiRewindDiscardCount_buffer = 0; // Count of anti-rewind discards
            std::vector<float> delay_buffer;
        };
    }
}

#endif // CH_DELAY_SIM_H
