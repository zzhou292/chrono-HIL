#ifndef CH_CAMERA_DELAY_SIM_THREADED_H
#define CH_CAMERA_DELAY_SIM_THREADED_H

#include <queue>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
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
         * @brief Threaded camera frame delay simulator
         * 
         * Similar to ChCameraDelaySim but runs display/rendering on a separate thread
         * to avoid blocking the main simulation loop. Uses a producer-consumer pattern:
         * 
         * Main Thread (Producer):
         *   - Calls addFrame() to submit frames (fast - just copies to staging buffer)
         *   - Calls updateDelayForTime() to update delay distribution
         * 
         * Display Thread (Consumer):
         *   - Processes frame queue and applies delays
         *   - Handles all OpenGL rendering
         *   - Runs independently at target display rate
         * 
         * This design minimizes main thread blocking since:
         *   - Frame memcpy happens on display thread (from staging area)
         *   - OpenGL calls happen on display thread
         *   - Main thread just does a quick pointer swap
         */
        class ChCameraDelaySimThreaded
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

            // Staging frame for quick handoff from main thread
            struct StagingFrame {
                std::vector<unsigned char> data;
                unsigned int width;
                unsigned int height;
                std::chrono::steady_clock::time_point captureTime;
                bool ready;
            };

            /**
             * @brief Construct a new ChCameraDelaySimThreaded
             * @param distribution Delay distribution (e.g., NormalDistribution)
             * @param maxBufferFrames Maximum number of frames to buffer (prevents memory issues)
             * @param targetDisplayRateHz Target display refresh rate (default 60Hz)
             */
            ChCameraDelaySimThreaded(std::shared_ptr<DelayDistribution> distribution, 
                                      size_t maxBufferFrames = 100,
                                      double targetDisplayRateHz = 60.0);
            ~ChCameraDelaySimThreaded();

            /**
             * @brief Add a new frame to the delay buffer (called from main thread)
             * @param data RGBA pixel data
             * @param width Frame width in pixels
             * @param height Frame height in pixels
             * 
             * This is optimized to be fast - it copies to a staging buffer and signals
             * the display thread. The heavy processing happens on the display thread.
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
             * @brief Initialize the display window and start display thread
             * @param windowTitle Title for the display window
             * @param windowWidth Display window width (can differ from frame size)
             * @param windowHeight Display window height
             * @return true if window created successfully
             */
            bool initDisplay(const std::string& windowTitle, int windowWidth, int windowHeight);

            /**
             * @brief Check if the display window is still open
             */
            bool isDisplayOpen() const;

            /**
             * @brief Close the display window and stop the display thread
             */
            void closeDisplay();

            /**
             * @brief Change the delay distribution (thread-safe)
             * @param newDistribution New delay distribution to use
             */
            void changeDistribution(std::shared_ptr<DelayDistribution> newDistribution);

            /**
             * @brief Load delay configuration from JSON file
             * @param jsonFilePath Path to JSON configuration file
             * @return true if loaded successfully
             */
            bool loadDelayConfig(const std::string& jsonFilePath);

            /**
             * @brief Update delay distribution based on simulation time (thread-safe)
             * @param currentSimTime Current simulation time in seconds
             */
            void updateDelayForTime(double currentSimTime);

            // Logging and stats
            void setLogging(bool enable) { enableLogging = enable; }
            float getExpectedDelayMs();
            size_t getBufferedFrameCount() const;
            
            // Configuration
            void setQuantizationStep(float stepMs);
            float getQuantizationStep() const { return quantizationStep; }
            void setAntiRewind(bool enable);
            bool getAntiRewind() const { return enableAntiRewind; }

        private:
            // === Main thread -> Display thread communication ===
            // Double-buffered staging area for minimal main thread blocking
            static constexpr int NUM_STAGING_BUFFERS = 3;  // Triple buffer for smooth handoff
            std::array<StagingFrame, NUM_STAGING_BUFFERS> stagingBuffers;
            std::atomic<int> writeBufferIdx{0};     // Buffer being written by main thread
            std::atomic<int> readBufferIdx{-1};     // Buffer ready for display thread (-1 = none ready)
            std::mutex stagingMutex;
            std::condition_variable frameAvailableCV;
            
            // === Display thread state ===
            std::thread displayThread;
            std::atomic<bool> shouldStop{false};
            std::atomic<bool> displayInitialized{false};
            double targetDisplayRateHz;
            
            // Display thread initialization parameters (set before thread starts)
            std::string pendingWindowTitle;
            int pendingWindowWidth = 0;
            int pendingWindowHeight = 0;
            std::atomic<bool> initRequested{false};
            std::atomic<bool> initComplete{false};
            std::atomic<bool> initSuccess{false};
            
            // === Frame queue (owned by display thread) ===
            std::vector<DelayedFrame> frameQueue;
            DelayedFrame latestFrame;
            bool hasLatestFrame = false;
            float lastSampledDelayMs = 0.0f;

            // Anti-rewind tracking
            std::chrono::steady_clock::time_point lastAppliedSourceTime;
            bool hasAppliedAnyFrame = false;
            std::atomic<bool> enableAntiRewind{true};

            // Configuration (atomic for thread-safe access)
            std::atomic<float> quantizationStep{0.0f};
            size_t maxBufferFrames;
            
            // Random generator and distribution (protected by distributionMutex)
            std::default_random_engine generator;
            std::shared_ptr<DelayDistribution> delayDistribution;
            std::mutex distributionMutex;

            // Delay periods from JSON config
            struct DelayPeriod {
                double startTime;
                double endTime;
                float delayMean;
                float delayStddev;
            };
            std::vector<DelayPeriod> delayPeriods;
            std::mutex delayPeriodsMutex;
            std::atomic<double> currentSimTime{0.0};

            // Logging
            std::atomic<bool> enableLogging{false};

            // Display (GLFW/OpenGL) - only accessed by display thread
            GLFWwindow* window = nullptr;
            unsigned int textureId = 0;
            int displayWidth = 0;
            int displayHeight = 0;
            bool glfwInitialized = false;
            unsigned int lastTextureWidth = 0;
            unsigned int lastTextureHeight = 0;

            // === Display thread methods ===
            void displayThreadFunc();
            bool initDisplayInternal(const std::string& windowTitle, int windowWidth, int windowHeight);
            void processIncomingFrames();
            const DelayedFrame* getDelayedFrame();
            void updateTexture(const DelayedFrame& frame);
            void renderFrame();
        };

    } // namespace hil
} // namespace chrono

#endif // CH_CAMERA_DELAY_SIM_THREADED_H
