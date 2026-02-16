#ifndef CH_CAMERA_DELAY_SIM_H
#define CH_CAMERA_DELAY_SIM_H

#include <queue>
#include <vector>
#include <mutex>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <functional>
#include <boost/math/distributions/normal.hpp>

#include "../../ChApiHil.h"
#include "ChDelaySim.h"

// Forward declare GLFW types to avoid including GLFW header here
struct GLFWwindow;

namespace chrono
{
    namespace hil
    {
        /**
         * @brief Camera frame delay simulator
         * 
         * Buffers camera frames and releases them with variable delays,
         * similar to ChDelaySim for control inputs. Supports the same
         * JSON-based delay configuration for time-varying latency.
         * 
         * Usage:
         * 1. Create ChCameraDelaySim with a delay distribution
         * 2. Call addFrame() each time a new camera frame is available
         * 3. Call getDelayedFrame() to get the appropriately delayed frame
         * 4. Optionally call displayDelayedFrame() to show in a GLFW window
         */
        class ChCameraDelaySim
        {
        public:
            // Structure to hold a delayed camera frame
            struct DelayedFrame {
                std::chrono::steady_clock::time_point applyTime;  // Wall time when frame should be displayed
                std::chrono::steady_clock::time_point sourceTime; // Wall time when frame was captured
                float sampledDelayMs;  // The delay that was sampled for this frame
                std::vector<unsigned char> data;  // RGBA pixel data
                unsigned int width;
                unsigned int height;
            };

            /**
             * @brief Construct a new ChCameraDelaySim
             * @param distribution Delay distribution (e.g., NormalDistribution)
             * @param maxBufferFrames Maximum number of frames to buffer (prevents memory issues)
             */
            ChCameraDelaySim(std::shared_ptr<DelayDistribution> distribution, 
                             size_t maxBufferFrames = 100);
            ~ChCameraDelaySim();

            /**
             * @brief Add a new frame to the delay buffer
             * @param data RGBA pixel data
             * @param width Frame width in pixels
             * @param height Frame height in pixels
             */
            void addFrame(const unsigned char* data, unsigned int width, unsigned int height);

            /**
             * @brief Add a frame using a Chrono sensor buffer pointer
             * @param buffer UserRGBA8BufferPtr from ChCameraSensor
             */
            template<typename BufferPtr>
            void addFrame(BufferPtr buffer) {
                if (buffer && buffer->Buffer) {
                    addFrame(reinterpret_cast<const unsigned char*>(buffer->Buffer.get()),
                             buffer->Width, buffer->Height);
                }
            }

            /**
             * @brief Get the current delayed frame
             * @return Pointer to the delayed frame, or nullptr if no frame ready
             * 
             * This processes the frame queue and returns the most recent frame
             * whose apply time has passed. Implements anti-rewind logic to prevent
             * old frames from appearing after newer ones.
             */
            const DelayedFrame* getDelayedFrame();

            /**
             * @brief Initialize the display window
             * @param windowTitle Title for the display window
             * @param windowWidth Display window width (can differ from frame size)
             * @param windowHeight Display window height
             * @return true if window created successfully
             */
            bool initDisplay(const std::string& windowTitle, int windowWidth, int windowHeight);

            /**
             * @brief Display the current delayed frame in the window
             * @return true if displayed successfully, false if window was closed
             * 
             * Must call initDisplay() first. This handles all OpenGL rendering.
             */
            bool displayDelayedFrame();

            /**
             * @brief Check if the display window is still open
             */
            bool isDisplayOpen() const;

            /**
             * @brief Close the display window
             */
            void closeDisplay();

            /**
             * @brief Change the delay distribution
             * @param newDistribution New delay distribution to use
             */
            void changeDistribution(std::shared_ptr<DelayDistribution> newDistribution);

            /**
             * @brief Load delay configuration from JSON file
             * @param jsonFilePath Path to JSON configuration file
             * @return true if loaded successfully
             * 
             * Uses same format as ChDelaySim delay_config.json
             */
            bool loadDelayConfig(const std::string& jsonFilePath);

            /**
             * @brief Update delay distribution based on simulation time
             * @param currentSimTime Current simulation time in seconds
             * 
             * Call this each frame to enable time-varying delays
             */
            void updateDelayForTime(double currentSimTime);

            // Logging and stats
            void setLogging(bool enable) { enableLogging = enable; }
            float getExpectedDelayMs();
            size_t getBufferedFrameCount() const { return frameQueue.size(); }
            
            // Configuration
            void setQuantizationStep(float stepMs) { quantizationStep = stepMs; }
            float getQuantizationStep() const { return quantizationStep; }
            void setAntiRewind(bool enable) { enableAntiRewind = enable; }
            bool getAntiRewind() const { return enableAntiRewind; }

        private:
            // Frame queue (ordered by apply time)
            std::vector<DelayedFrame> frameQueue;
            std::mutex queueMutex;
            DelayedFrame latestFrame;  // Most recently released frame
            bool hasLatestFrame = false;
            float lastSampledDelayMs = 0.0f;

            // Anti-rewind tracking
            std::chrono::steady_clock::time_point lastAppliedSourceTime;
            bool hasAppliedAnyFrame = false;
            bool enableAntiRewind = true;

            // Configuration
            float quantizationStep = 0.0f;
            size_t maxBufferFrames;
            
            // Random generator and distribution
            std::default_random_engine generator;
            std::shared_ptr<DelayDistribution> delayDistribution;

            // Delay periods from JSON config
            struct DelayPeriod {
                double startTime;
                double endTime;
                float delayMean;
                float delayStddev;
            };
            std::vector<DelayPeriod> delayPeriods;
            double currentSimTime = 0.0;

            // Logging
            bool enableLogging = false;

            // Display (GLFW/OpenGL)
            GLFWwindow* window = nullptr;
            unsigned int textureId = 0;
            int displayWidth = 0;
            int displayHeight = 0;
            bool glfwInitialized = false;
            
            // Texture cache for efficient updates
            unsigned int lastTextureWidth = 0;
            unsigned int lastTextureHeight = 0;

            // OpenGL helpers
            void updateTexture(const DelayedFrame& frame);
            void renderFrame();
        };

    } // namespace hil
} // namespace chrono

#endif // CH_CAMERA_DELAY_SIM_H
