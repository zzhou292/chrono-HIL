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

            int getPacketDropCount();
            std::vector<float> getDelayBuffer();

        private:
            std::queue<std::pair<std::chrono::steady_clock::time_point, std::vector<char>>> packetQueue;
            std::mutex queueMutex;
            std::vector<char> latestData; // Store the latest packet data
            float expectedDelay;          // Expected delay in milliseconds

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
            std::vector<float> delay_buffer;
        };
    }
}

#endif // CH_DELAY_SIM_H
