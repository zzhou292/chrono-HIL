#include "ChCameraDelaySimThreaded.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "chrono_vehicle/utils/ChUtilsJSON.h"

// GLFW and OpenGL for display
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace chrono
{
    namespace hil
    {

        ChCameraDelaySimThreaded::ChCameraDelaySimThreaded(std::shared_ptr<DelayDistribution> distribution,
                                                           size_t maxBufferFrames,
                                                           double targetDisplayRateHz)
            : delayDistribution(distribution), 
              maxBufferFrames(maxBufferFrames),
              targetDisplayRateHz(targetDisplayRateHz)
        {
            std::random_device rd;
            generator.seed(rd());
            
            // Initialize staging buffers
            for (auto& buf : stagingBuffers) {
                buf.ready = false;
                buf.width = 0;
                buf.height = 0;
            }
        }

        ChCameraDelaySimThreaded::~ChCameraDelaySimThreaded()
        {
            stopCaptureThread();
            closeDisplay();
        }

        // =====================================================================
        // Async Capture Thread Implementation
        // =====================================================================

        void ChCameraDelaySimThreaded::startCaptureThread()
        {
            if (captureThreadRunning.load()) return;
            
            captureThreadRunning = true;
            captureThread = std::thread(&ChCameraDelaySimThreaded::captureThreadFunc, this);
            
            if (enableLogging.load()) {
                std::cout << "[CameraDelaySimThreaded] Capture thread started" << std::endl;
            }
        }

        void ChCameraDelaySimThreaded::stopCaptureThread()
        {
            if (!captureThreadRunning.load()) return;
            
            captureThreadRunning = false;
            captureQueueCV.notify_all();
            
            if (captureThread.joinable()) {
                captureThread.join();
            }
            
            if (enableLogging.load()) {
                std::cout << "[CameraDelaySimThreaded] Capture thread stopped" << std::endl;
            }
        }

        void ChCameraDelaySimThreaded::captureThreadFunc()
        {
            if (enableLogging.load()) {
                std::ostringstream oss;
                oss << std::this_thread::get_id();
                std::cout << "[CameraDelaySimThreaded] Capture thread running, ID: " << oss.str() << std::endl;
            }
            
            // Stats tracking
            uint64_t totalFrames = 0;
            double totalCopyTimeMs = 0.0;
            double lastCopyTimeMs = 0.0;
            auto lastStatsTime = std::chrono::steady_clock::now();
            
            while (captureThreadRunning.load())
            {
                PendingCapture capture;
                bool hasWork = false;
                
                {
                    std::unique_lock<std::mutex> lock(captureQueueMutex);
                    
                    // Wait for work or stop signal
                    captureQueueCV.wait_for(lock, std::chrono::milliseconds(100), [this] {
                        return !pendingCaptures.empty() || !captureThreadRunning.load();
                    });
                    
                    if (!pendingCaptures.empty()) {
                        capture = std::move(pendingCaptures.front());
                        pendingCaptures.erase(pendingCaptures.begin());
                        hasWork = true;
                    }
                }
                
                if (hasWork && capture.holder) {
                    // Time the memcpy
                    auto copyStart = std::chrono::steady_clock::now();
                    
                    // Do the heavy memcpy here on the capture thread
                    const unsigned char* data = capture.holder->getData();
                    
                    {
                        std::lock_guard<std::mutex> lock(stagingMutex);
                        
                        int writeIdx = writeBufferIdx.load();
                        StagingFrame& staging = stagingBuffers[writeIdx];
                        
                        size_t dataSize = capture.width * capture.height * 4;
                        if (staging.data.size() != dataSize) {
                            staging.data.resize(dataSize);
                        }
                        
                        std::memcpy(staging.data.data(), data, dataSize);
                        staging.width = capture.width;
                        staging.height = capture.height;
                        staging.captureTime = capture.captureTime;
                        staging.ready = true;
                        
                        // Rotate to next buffer
                        readBufferIdx.store(writeIdx);
                        writeBufferIdx.store((writeIdx + 1) % NUM_STAGING_BUFFERS);
                    }
                    
                    frameAvailableCV.notify_one();
                    
                    auto copyEnd = std::chrono::steady_clock::now();
                    lastCopyTimeMs = std::chrono::duration<double, std::milli>(copyEnd - copyStart).count();
                    totalCopyTimeMs += lastCopyTimeMs;
                    totalFrames++;
                    
                    // Periodic stats logging
                    auto now = std::chrono::steady_clock::now();
                    if (enableLogging.load() && 
                        std::chrono::duration<double>(now - lastStatsTime).count() >= 1.0) {
                        size_t queueDepth;
                        {
                            std::lock_guard<std::mutex> lock(captureQueueMutex);
                            queueDepth = pendingCaptures.size();
                        }
                        std::cout << "[CameraDelaySimThreaded] Async capture stats: count=" << totalFrames
                                  << ", avg copy time=" << std::fixed << std::setprecision(2) << (totalCopyTimeMs / totalFrames) << "ms"
                                  << ", last copy=" << lastCopyTimeMs << "ms"
                                  << ", queue depth=" << queueDepth << std::endl;
                        lastStatsTime = now;
                    }
                }
            }
        }

        void ChCameraDelaySimThreaded::addFrame(const unsigned char* data, unsigned int width, unsigned int height)
        {
            // Quick path: copy to staging buffer and signal display thread
            // This should be fast since we're just doing a memcpy and signal
            
            auto captureTime = std::chrono::steady_clock::now();
            
            // Log main thread ID on first call
            static bool loggedMainThread = false;
            if (!loggedMainThread && enableLogging.load()) {
                std::ostringstream oss;
                oss << std::this_thread::get_id();
                std::cout << "[CameraDelaySimThreaded] Main thread ID (addFrame): " << oss.str() << std::endl;
                loggedMainThread = true;
            }
            
            // Track timing for performance stats
            auto copyStart = std::chrono::steady_clock::now();
            
            {
                std::lock_guard<std::mutex> lock(stagingMutex);
                
                // Get the write buffer
                int writeIdx = writeBufferIdx.load();
                StagingFrame& staging = stagingBuffers[writeIdx];
                
                // Resize if needed (should only happen on first frame or resolution change)
                size_t dataSize = width * height * 4;
                if (staging.data.size() != dataSize) {
                    staging.data.resize(dataSize);
                }
                
                // Copy frame data
                std::memcpy(staging.data.data(), data, dataSize);
                staging.width = width;
                staging.height = height;
                staging.captureTime = captureTime;
                staging.ready = true;
                
                // Rotate to next write buffer
                int nextWriteIdx = (writeIdx + 1) % NUM_STAGING_BUFFERS;
                writeBufferIdx.store(nextWriteIdx);
                
                // Mark this buffer as ready for reading
                readBufferIdx.store(writeIdx);
            }
            
            // Signal the display thread that a frame is available
            frameAvailableCV.notify_one();
            
            // Log copy time periodically
            auto copyEnd = std::chrono::steady_clock::now();
            double copyTimeMs = std::chrono::duration<double, std::milli>(copyEnd - copyStart).count();
            
            static int addFrameCount = 0;
            static double totalCopyTime = 0.0;
            addFrameCount++;
            totalCopyTime += copyTimeMs;
            
            if (enableLogging.load() && addFrameCount % 35 == 0) {
                std::cout << "[CameraDelaySimThreaded] addFrame stats: "
                          << "count=" << addFrameCount 
                          << ", avg copy time=" << (totalCopyTime / addFrameCount) << "ms"
                          << ", last copy=" << copyTimeMs << "ms" << std::endl;
            }
        }

        bool ChCameraDelaySimThreaded::initDisplay(const std::string& windowTitle, 
                                                    int windowWidth, int windowHeight)
        {
            // Store parameters for the display thread to use
            pendingWindowTitle = windowTitle;
            pendingWindowWidth = windowWidth;
            pendingWindowHeight = windowHeight;
            
            // Start the display thread if not already running
            if (!displayThread.joinable()) {
                shouldStop.store(false);
                initRequested.store(true);
                initComplete.store(false);
                
                displayThread = std::thread(&ChCameraDelaySimThreaded::displayThreadFunc, this);
                
                // Wait for initialization to complete
                while (!initComplete.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                
                return initSuccess.load();
            }
            
            return displayInitialized.load();
        }

        bool ChCameraDelaySimThreaded::isDisplayOpen() const
        {
            return displayInitialized.load() && !shouldStop.load();
        }

        void ChCameraDelaySimThreaded::closeDisplay()
        {
            // Signal thread to stop
            shouldStop.store(true);
            frameAvailableCV.notify_all();
            
            // Wait for thread to finish
            if (displayThread.joinable()) {
                displayThread.join();
            }
            
            displayInitialized.store(false);
        }

        void ChCameraDelaySimThreaded::changeDistribution(std::shared_ptr<DelayDistribution> newDistribution)
        {
            std::lock_guard<std::mutex> lock(distributionMutex);
            delayDistribution = newDistribution;
        }

        float ChCameraDelaySimThreaded::getExpectedDelayMs()
        {
            return lastSampledDelayMs;
        }

        size_t ChCameraDelaySimThreaded::getBufferedFrameCount() const
        {
            // Note: this is approximate since frameQueue is owned by display thread
            return frameQueue.size();
        }

        void ChCameraDelaySimThreaded::setQuantizationStep(float stepMs)
        {
            quantizationStep.store(stepMs);
        }

        void ChCameraDelaySimThreaded::setAntiRewind(bool enable)
        {
            enableAntiRewind.store(enable);
        }

        bool ChCameraDelaySimThreaded::loadDelayConfig(const std::string& jsonFilePath)
        {
            rapidjson::Document d;
            chrono::vehicle::ReadFileJSON(jsonFilePath, d);

            if (!d.IsObject()) {
                std::cerr << "[CameraDelaySimThreaded] Error: Invalid JSON format in delay config: " 
                          << jsonFilePath << std::endl;
                return false;
            }

            std::lock_guard<std::mutex> lock(delayPeriodsMutex);
            delayPeriods.clear();
            std::string usedSection = "none";

            if (d.HasMember("camera_delay_periods") && d["camera_delay_periods"].IsArray()) {
                usedSection = "camera_delay_periods";
                const rapidjson::Value& periods = d["camera_delay_periods"];
                for (rapidjson::SizeType i = 0; i < periods.Size(); i++) {
                    const rapidjson::Value& period = periods[i];
                    DelayPeriod dp;
                    dp.startTime = period["start_time"].GetDouble();
                    dp.endTime = period["end_time"].GetDouble();
                    dp.delayMean = period["delay_mean"].GetFloat();
                    dp.delayStddev = period.HasMember("delay_stddev") ? 
                                     period["delay_stddev"].GetFloat() : 0.001f;
                    delayPeriods.push_back(dp);
                    
                    if (enableLogging.load()) {
                        std::cout << "[CameraDelaySimThreaded] Period " << i << ": t=[" 
                                  << dp.startTime << "," << dp.endTime << "]s, delay=" 
                                  << dp.delayMean << "ms (stddev=" << dp.delayStddev << "ms)" << std::endl;
                    }
                }
            } else if (d.HasMember("delay_periods") && d["delay_periods"].IsArray()) {
                usedSection = "delay_periods (fallback)";
                const rapidjson::Value& periods = d["delay_periods"];
                for (rapidjson::SizeType i = 0; i < periods.Size(); i++) {
                    const rapidjson::Value& period = periods[i];
                    DelayPeriod dp;
                    dp.startTime = period["start_time"].GetDouble();
                    dp.endTime = period["end_time"].GetDouble();
                    dp.delayMean = period["delay_mean"].GetFloat();
                    dp.delayStddev = period.HasMember("delay_stddev") ? 
                                     period["delay_stddev"].GetFloat() : 0.001f;
                    delayPeriods.push_back(dp);
                    
                    if (enableLogging.load()) {
                        std::cout << "[CameraDelaySimThreaded] Period " << i << ": t=[" 
                                  << dp.startTime << "," << dp.endTime << "]s, delay=" 
                                  << dp.delayMean << "ms (stddev=" << dp.delayStddev << "ms)" << std::endl;
                    }
                }
            }

            if (enableLogging.load()) {
                std::cout << "[CameraDelaySimThreaded] Loaded " << delayPeriods.size() 
                          << " delay periods from section '" << usedSection << "'" << std::endl;
            }

            return !delayPeriods.empty();
        }

        void ChCameraDelaySimThreaded::updateDelayForTime(double simTime)
        {
            currentSimTime.store(simTime);
            
            // Check if current time falls within any delay period
            std::lock_guard<std::mutex> lock(delayPeriodsMutex);
            for (const auto& period : delayPeriods) {
                if (simTime >= period.startTime && simTime <= period.endTime) {
                    auto newDist = std::make_shared<NormalDistribution>(
                        period.delayMean, period.delayStddev);
                    
                    {
                        std::lock_guard<std::mutex> distLock(distributionMutex);
                        delayDistribution = newDist;
                    }

                    if (enableLogging.load()) {
                        static double lastLogTime = -1.0;
                        if (std::floor(simTime) != std::floor(lastLogTime)) {
                            std::cout << "[CameraDelaySimThreaded] Time " << simTime 
                                      << "s: delay=" << period.delayMean << "ms" << std::endl;
                            lastLogTime = simTime;
                        }
                    }
                    return;
                }
            }

            // Not in any delay period, use zero delay
            auto zeroDist = std::make_shared<NormalDistribution>(0.0f, 0.001f);
            {
                std::lock_guard<std::mutex> distLock(distributionMutex);
                delayDistribution = zeroDist;
            }
        }

        // =====================================================================
        // Display Thread Implementation
        // =====================================================================

        void ChCameraDelaySimThreaded::displayThreadFunc()
        {
            // Log display thread ID
            if (enableLogging.load()) {
                std::ostringstream oss;
                oss << std::this_thread::get_id();
                std::cout << "[CameraDelaySimThreaded] Display thread started, ID: " << oss.str() << std::endl;
                std::cout << "[CameraDelaySimThreaded] Target display rate: " << targetDisplayRateHz << " Hz" << std::endl;
            }
            
            // Performance tracking
            int frameCount = 0;
            double totalProcessTime = 0.0;
            double totalRenderTime = 0.0;
            auto statsStartTime = std::chrono::steady_clock::now();
            
            // Calculate frame time for target display rate
            auto frameTime = std::chrono::microseconds(
                static_cast<long long>(1000000.0 / targetDisplayRateHz));
            
            // Handle initialization request
            if (initRequested.load()) {
                bool success = initDisplayInternal(pendingWindowTitle, 
                                                   pendingWindowWidth, 
                                                   pendingWindowHeight);
                initSuccess.store(success);
                displayInitialized.store(success);
                initComplete.store(true);
                initRequested.store(false);
                
                if (!success) {
                    return;  // Exit thread on init failure
                }
            }
            
            auto lastFrameTime = std::chrono::steady_clock::now();
            
            while (!shouldStop.load())
            {
                auto frameStart = std::chrono::steady_clock::now();
                
                // Check for window close
                if (window && glfwWindowShouldClose(window)) {
                    break;
                }
                
                // Process any incoming frames from the main thread
                processIncomingFrames();
                
                // Get the current delayed frame to display
                const DelayedFrame* frame = getDelayedFrame();
                
                // Update texture if we have a new frame
                if (frame) {
                    updateTexture(*frame);
                }
                
                // Always render (even if no new frame) to keep display responsive
                if (hasLatestFrame && window) {
                    auto renderStart = std::chrono::steady_clock::now();
                    
                    glViewport(0, 0, displayWidth, displayHeight);
                    glMatrixMode(GL_PROJECTION);
                    glLoadIdentity();
                    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
                    glMatrixMode(GL_MODELVIEW);
                    glLoadIdentity();
                    
                    renderFrame();
                    glfwSwapBuffers(window);
                    
                    auto renderEnd = std::chrono::steady_clock::now();
                    totalRenderTime += std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
                }
                
                glfwPollEvents();
                
                // Rate limiting - sleep to maintain target frame rate
                auto frameEnd = std::chrono::steady_clock::now();
                auto elapsed = frameEnd - frameStart;
                if (elapsed < frameTime) {
                    std::this_thread::sleep_for(frameTime - elapsed);
                }
                
                // Performance stats logging every ~2 seconds (120 frames at 60Hz)
                frameCount++;
                if (enableLogging.load() && frameCount % 120 == 0) {
                    auto now = std::chrono::steady_clock::now();
                    double elapsedSec = std::chrono::duration<double>(now - statsStartTime).count();
                    double actualFps = frameCount / elapsedSec;
                    
                    std::cout << "[CameraDelaySimThreaded] Display thread stats: "
                              << "frames=" << frameCount
                              << ", actual FPS=" << std::fixed << std::setprecision(1) << actualFps
                              << ", avg render=" << std::setprecision(2) << (totalRenderTime / frameCount) << "ms"
                              << ", queue size=" << frameQueue.size() << std::endl;
                }
            }
            
            // Log thread exit
            if (enableLogging.load()) {
                std::cout << "[CameraDelaySimThreaded] Display thread exiting after " << frameCount << " frames" << std::endl;
            }
            
            // Cleanup OpenGL resources
            if (textureId != 0) {
                glDeleteTextures(1, &textureId);
                textureId = 0;
            }
            if (window) {
                glfwDestroyWindow(window);
                window = nullptr;
            }
            
            displayInitialized.store(false);
        }

        void ChCameraDelaySimThreaded::processIncomingFrames()
        {
            // Check if there's a new frame in the staging area
            int readIdx = readBufferIdx.exchange(-1);  // Atomically get and clear
            
            if (readIdx < 0) {
                return;  // No new frame
            }
            
            StagingFrame* staging = nullptr;
            {
                std::lock_guard<std::mutex> lock(stagingMutex);
                staging = &stagingBuffers[readIdx];
                if (!staging->ready) {
                    return;
                }
            }
            
            // Sample delay for this frame
            float sampledDelay;
            {
                std::lock_guard<std::mutex> lock(distributionMutex);
                sampledDelay = delayDistribution->sample(generator);
            }
            
            // Apply quantization if enabled
            float qStep = quantizationStep.load();
            if (qStep > 0.0f) {
                sampledDelay = std::round(sampledDelay / qStep) * qStep;
            }
            if (sampledDelay < 0.0f) sampledDelay = 0.0f;
            
            lastSampledDelayMs = sampledDelay;
            
            // Calculate apply time
            auto applyTime = staging->captureTime + 
                std::chrono::milliseconds(static_cast<long long>(sampledDelay));
            
            // Log periodically
            static int totalFrameCount = 0;
            totalFrameCount++;
            if (enableLogging.load() && (totalFrameCount <= 5 || totalFrameCount % 35 == 0)) {
                std::cout << "[CameraDelaySimThreaded] Frame " << totalFrameCount 
                          << ": sampled delay = " << sampledDelay << "ms" << std::endl;
            }
            
            // Check buffer size limit
            int droppedCount = 0;
            while (frameQueue.size() >= maxBufferFrames && !frameQueue.empty()) {
                std::pop_heap(frameQueue.begin(), frameQueue.end(),
                    [](const DelayedFrame& a, const DelayedFrame& b) {
                        return a.applyTime > b.applyTime;
                    });
                frameQueue.pop_back();
                droppedCount++;
            }
            
            if (enableLogging.load() && droppedCount > 0) {
                static int lastDropLogFrame = 0;
                if (totalFrameCount - lastDropLogFrame >= 35) {
                    std::cout << "[CameraDelaySimThreaded] Dropped " << droppedCount 
                              << " frame(s), queue: " << frameQueue.size() << std::endl;
                    lastDropLogFrame = totalFrameCount;
                }
            }
            
            // Create delayed frame - the heavy memcpy happens here, on the display thread
            DelayedFrame frame;
            frame.applyTime = applyTime;
            frame.sourceTime = staging->captureTime;
            frame.sampledDelayMs = sampledDelay;
            frame.width = staging->width;
            frame.height = staging->height;
            
            {
                std::lock_guard<std::mutex> lock(stagingMutex);
                // Move data from staging to frame
                frame.data = std::move(staging->data);
                staging->ready = false;
                // Re-allocate staging buffer for next frame
                staging->data.resize(staging->width * staging->height * 4);
            }
            
            // Add to queue
            frameQueue.push_back(std::move(frame));
            std::push_heap(frameQueue.begin(), frameQueue.end(),
                [](const DelayedFrame& a, const DelayedFrame& b) {
                    return a.applyTime > b.applyTime;
                });
        }

        const ChCameraDelaySimThreaded::DelayedFrame* ChCameraDelaySimThreaded::getDelayedFrame()
        {
            auto currentTime = std::chrono::steady_clock::now();
            bool antiRewind = enableAntiRewind.load();

            while (!frameQueue.empty())
            {
                const DelayedFrame& front = frameQueue.front();

                if (front.applyTime > currentTime) {
                    break;
                }

                std::pop_heap(frameQueue.begin(), frameQueue.end(),
                    [](const DelayedFrame& a, const DelayedFrame& b) {
                        return a.applyTime > b.applyTime;
                    });
                DelayedFrame frame = std::move(frameQueue.back());
                frameQueue.pop_back();

                if (antiRewind && hasAppliedAnyFrame) {
                    if (frame.sourceTime < lastAppliedSourceTime) {
                        continue;
                    }
                }

                latestFrame = std::move(frame);
                lastAppliedSourceTime = latestFrame.sourceTime;
                hasAppliedAnyFrame = true;
                hasLatestFrame = true;
                lastSampledDelayMs = latestFrame.sampledDelayMs;
            }

            return hasLatestFrame ? &latestFrame : nullptr;
        }

        bool ChCameraDelaySimThreaded::initDisplayInternal(const std::string& windowTitle, 
                                                            int windowWidth, int windowHeight)
        {
            displayWidth = windowWidth;
            displayHeight = windowHeight;

            if (!glfwInit()) {
                std::cerr << "[CameraDelaySimThreaded] Failed to initialize GLFW" << std::endl;
                return false;
            }
            glfwInitialized = true;

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
            glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

            window = glfwCreateWindow(windowWidth, windowHeight, 
                                       windowTitle.c_str(), nullptr, nullptr);
            if (!window) {
                std::cerr << "[CameraDelaySimThreaded] Failed to create GLFW window" << std::endl;
                return false;
            }

            glfwMakeContextCurrent(window);

            glewExperimental = GL_TRUE;
            if (glewInit() != GLEW_OK) {
                std::cerr << "[CameraDelaySimThreaded] Failed to initialize GLEW" << std::endl;
                glfwDestroyWindow(window);
                window = nullptr;
                return false;
            }

            glGenTextures(1, &textureId);
            glBindTexture(GL_TEXTURE_2D, textureId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            
            lastTextureWidth = 0;
            lastTextureHeight = 0;

            glfwSwapInterval(0);  // Disable vsync for low latency
            
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_LIGHTING);
            
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            
            glViewport(0, 0, windowWidth, windowHeight);
            
            glClear(GL_COLOR_BUFFER_BIT);
            glfwSwapBuffers(window);

            if (enableLogging.load()) {
                std::cout << "[CameraDelaySimThreaded] Display initialized: " 
                          << windowWidth << "x" << windowHeight << std::endl;
            }

            return true;
        }

        void ChCameraDelaySimThreaded::updateTexture(const DelayedFrame& frame)
        {
            glBindTexture(GL_TEXTURE_2D, textureId);
            
            if (lastTextureWidth == frame.width && lastTextureHeight == frame.height) {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, frame.width, frame.height,
                               GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
            } else {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, frame.width, frame.height,
                             0, GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
                lastTextureWidth = frame.width;
                lastTextureHeight = frame.height;
            }
        }

        void ChCameraDelaySimThreaded::renderFrame()
        {
            glClear(GL_COLOR_BUFFER_BIT);

            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, textureId);

            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex2f( 1.0f, -1.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex2f( 1.0f,  1.0f);
            glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f,  1.0f);
            glEnd();

            glDisable(GL_TEXTURE_2D);
        }

    } // namespace hil
} // namespace chrono
