#include "ChCameraDelaySim.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include "chrono_vehicle/utils/ChUtilsJSON.h"

// GLFW and OpenGL for display
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace chrono
{
    namespace hil
    {

        ChCameraDelaySim::ChCameraDelaySim(std::shared_ptr<DelayDistribution> distribution,
                                           size_t maxBufferFrames)
            : delayDistribution(distribution), maxBufferFrames(maxBufferFrames)
        {
            std::random_device rd;
            generator.seed(rd());
        }

        ChCameraDelaySim::~ChCameraDelaySim()
        {
            closeDisplay();
        }

        void ChCameraDelaySim::addFrame(const unsigned char* data, unsigned int width, unsigned int height)
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            auto currentTime = std::chrono::steady_clock::now();

            // Sample delay at capture time
            float sampledDelay = delayDistribution->sample(generator);

            // Quantize delay to nearest step if enabled
            if (quantizationStep > 0.0f) {
                sampledDelay = std::round(sampledDelay / quantizationStep) * quantizationStep;
            }
            if (sampledDelay < 0.0f) sampledDelay = 0.0f;

            lastSampledDelayMs = sampledDelay;

            // Calculate apply time = current wall time + sampled delay
            auto applyTime = currentTime + std::chrono::milliseconds(static_cast<long long>(sampledDelay));

            // Log delay being applied (first few frames and periodically)
            static int totalFrameCount = 0;
            totalFrameCount++;
            if (enableLogging && (totalFrameCount <= 5 || totalFrameCount % 35 == 0)) {
                std::cout << "[CameraDelaySim] Frame " << totalFrameCount 
                          << ": sampled delay = " << sampledDelay << "ms" << std::endl;
            }

            // Check buffer size limit - drop oldest frames if necessary
            int droppedCount = 0;
            while (frameQueue.size() >= maxBufferFrames && !frameQueue.empty()) {
                std::pop_heap(frameQueue.begin(), frameQueue.end(),
                    [](const DelayedFrame& a, const DelayedFrame& b) {
                        return a.applyTime > b.applyTime;
                    });
                frameQueue.pop_back();
                droppedCount++;
            }
            if (enableLogging && droppedCount > 0) {
                static int lastDropLogFrame = 0;
                // Only log drops once per second (every 35 frames at 35Hz)
                if (totalFrameCount - lastDropLogFrame >= 35) {
                    std::cout << "[CameraDelaySim] Dropped " << droppedCount 
                              << " frame(s), queue: " << frameQueue.size() << std::endl;
                    lastDropLogFrame = totalFrameCount;
                }
            }

            // Create the delayed frame at full resolution
            // Performance is fine now since we capture at ~35Hz (camera rate) not 1000Hz
            DelayedFrame frame;
            frame.applyTime = applyTime;
            frame.sourceTime = currentTime;
            frame.sampledDelayMs = sampledDelay;
            frame.width = width;
            frame.height = height;
            
            // Copy full resolution pixel data
            size_t dataSize = width * height * 4;
            frame.data.resize(dataSize);
            std::memcpy(frame.data.data(), data, dataSize);

            // Add to queue and maintain heap property
            frameQueue.push_back(std::move(frame));
            std::push_heap(frameQueue.begin(), frameQueue.end(),
                [](const DelayedFrame& a, const DelayedFrame& b) {
                    return a.applyTime > b.applyTime;
                });
        }

        const ChCameraDelaySim::DelayedFrame* ChCameraDelaySim::getDelayedFrame()
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            auto currentTime = std::chrono::steady_clock::now();

            // Process all frames whose apply time has arrived
            while (!frameQueue.empty())
            {
                const DelayedFrame& front = frameQueue.front();

                // Check if this frame's apply time has arrived
                if (front.applyTime > currentTime) {
                    break; // No more frames ready yet
                }

                // Pop the frame from the heap
                std::pop_heap(frameQueue.begin(), frameQueue.end(),
                    [](const DelayedFrame& a, const DelayedFrame& b) {
                        return a.applyTime > b.applyTime;
                    });
                DelayedFrame frame = std::move(frameQueue.back());
                frameQueue.pop_back();

                // Anti-rewind check: prevent old frames from displaying after newer ones
                if (enableAntiRewind && hasAppliedAnyFrame) {
                    if (frame.sourceTime < lastAppliedSourceTime) {
                        continue; // Skip this stale frame
                    }
                }

                // Apply this frame
                latestFrame = std::move(frame);
                lastAppliedSourceTime = latestFrame.sourceTime;
                hasAppliedAnyFrame = true;
                hasLatestFrame = true;
                lastSampledDelayMs = latestFrame.sampledDelayMs;
            }

            return hasLatestFrame ? &latestFrame : nullptr;
        }

        void ChCameraDelaySim::changeDistribution(std::shared_ptr<DelayDistribution> newDistribution)
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            delayDistribution = newDistribution;
        }

        float ChCameraDelaySim::getExpectedDelayMs()
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            return lastSampledDelayMs;
        }

        bool ChCameraDelaySim::loadDelayConfig(const std::string& jsonFilePath)
        {
            rapidjson::Document d;
            chrono::vehicle::ReadFileJSON(jsonFilePath, d);

            if (!d.IsObject()) {
                std::cerr << "[CameraDelaySim] Error: Invalid JSON format in delay config: " 
                          << jsonFilePath << std::endl;
                return false;
            }

            delayPeriods.clear();
            std::string usedSection = "none";

            if (d.HasMember("camera_delay_periods") && d["camera_delay_periods"].IsArray()) {
                // Use camera-specific delay periods if available
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
                    
                    if (enableLogging) {
                        std::cout << "[CameraDelaySim] Period " << i << ": t=[" 
                                  << dp.startTime << "," << dp.endTime << "]s, delay=" 
                                  << dp.delayMean << "ms (stddev=" << dp.delayStddev << "ms)" << std::endl;
                    }
                }
            } else if (d.HasMember("delay_periods") && d["delay_periods"].IsArray()) {
                // Fall back to regular delay periods (same as input delay)
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
                    
                    if (enableLogging) {
                        std::cout << "[CameraDelaySim] Period " << i << ": t=[" 
                                  << dp.startTime << "," << dp.endTime << "]s, delay=" 
                                  << dp.delayMean << "ms (stddev=" << dp.delayStddev << "ms)" << std::endl;
                    }
                }
            }

            if (enableLogging) {
                std::cout << "[CameraDelaySim] Loaded " << delayPeriods.size() 
                          << " delay periods from section '" << usedSection << "'" << std::endl;
            }

            return !delayPeriods.empty();
        }

        void ChCameraDelaySim::updateDelayForTime(double simTime)
        {
            currentSimTime = simTime;

            // Debug: log first few calls
            static int callCount = 0;
            callCount++;
            if (enableLogging && callCount <= 5) {
                std::cout << "[CameraDelaySim] updateDelayForTime called: simTime=" << simTime 
                          << ", delayPeriods.size()=" << delayPeriods.size() << std::endl;
            }

            // Check if current time falls within any delay period
            for (const auto& period : delayPeriods) {
                if (simTime >= period.startTime && simTime <= period.endTime) {
                    auto newDist = std::make_shared<NormalDistribution>(
                        period.delayMean, period.delayStddev);
                    changeDistribution(newDist);

                    if (enableLogging) {
                        static double lastLogTime = -1.0;
                        if (std::floor(simTime) != std::floor(lastLogTime)) {
                            std::cout << "[CameraDelaySim] Time " << simTime 
                                      << "s: delay=" << period.delayMean << "ms" << std::endl;
                            lastLogTime = simTime;
                        }
                    }
                    return;
                }
            }

            // Not in any delay period, use zero delay
            auto zeroDist = std::make_shared<NormalDistribution>(0.0f, 0.001f);
            changeDistribution(zeroDist);
        }

        // =====================================================================
        // GLFW/OpenGL Display Methods
        // =====================================================================

        bool ChCameraDelaySim::initDisplay(const std::string& windowTitle, 
                                            int windowWidth, int windowHeight)
        {
            displayWidth = windowWidth;
            displayHeight = windowHeight;

            // Initialize GLFW if not already done
            // Note: Chrono sensor may have already initialized GLFW
            if (!glfwInit()) {
                std::cerr << "[CameraDelaySim] Failed to initialize GLFW" << std::endl;
                return false;
            }
            glfwInitialized = true;

            // Set window hints
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
            glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

            // Create window
            window = glfwCreateWindow(windowWidth, windowHeight, 
                                       windowTitle.c_str(), nullptr, nullptr);
            if (!window) {
                std::cerr << "[CameraDelaySim] Failed to create GLFW window" << std::endl;
                return false;
            }

            // Make context current for this thread
            glfwMakeContextCurrent(window);

            // Initialize GLEW
            glewExperimental = GL_TRUE;
            if (glewInit() != GLEW_OK) {
                std::cerr << "[CameraDelaySim] Failed to initialize GLEW" << std::endl;
                glfwDestroyWindow(window);
                window = nullptr;
                return false;
            }

            // Create texture for frame display
            glGenTextures(1, &textureId);
            glBindTexture(GL_TEXTURE_2D, textureId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            
            // Texture will be allocated on first frame upload
            lastTextureWidth = 0;
            lastTextureHeight = 0;

            // Disable vsync for lowest latency (allows GPU to run freely)
            glfwSwapInterval(0);
            
            // Set up OpenGL state
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_LIGHTING);
            
            // Set up orthographic projection
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            
            // Set viewport
            glViewport(0, 0, windowWidth, windowHeight);
            
            // Clear to black initially
            glClear(GL_COLOR_BUFFER_BIT);
            glfwSwapBuffers(window);

            if (enableLogging) {
                std::cout << "[CameraDelaySim] Display initialized: " 
                          << windowWidth << "x" << windowHeight << std::endl;
            }

            return true;
        }

        bool ChCameraDelaySim::isDisplayOpen() const
        {
            return window && !glfwWindowShouldClose(window);
        }

        void ChCameraDelaySim::closeDisplay()
        {
            if (textureId != 0) {
                glDeleteTextures(1, &textureId);
                textureId = 0;
            }
            if (window) {
                glfwDestroyWindow(window);
                window = nullptr;
            }
            // Don't terminate GLFW as Chrono sensor may still be using it
        }

        void ChCameraDelaySim::updateTexture(const DelayedFrame& frame)
        {
            glBindTexture(GL_TEXTURE_2D, textureId);
            
            // Use glTexSubImage2D if texture size matches, otherwise reallocate
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

        void ChCameraDelaySim::renderFrame()
        {
            glClear(GL_COLOR_BUFFER_BIT);

            // Enable texturing
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, textureId);

            // Draw fullscreen quad with texture
            // Chrono sensor outputs images with origin at top-left, OpenGL expects bottom-left
            // So we DON'T flip - use standard texture coordinates
            glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
            glTexCoord2f(1.0f, 0.0f); glVertex2f( 1.0f, -1.0f);
            glTexCoord2f(1.0f, 1.0f); glVertex2f( 1.0f,  1.0f);
            glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f,  1.0f);
            glEnd();

            glDisable(GL_TEXTURE_2D);
        }

        bool ChCameraDelaySim::displayDelayedFrame()
        {
            if (!window || glfwWindowShouldClose(window)) {
                return false;
            }

            // Check if any delayed frame is ready
            const DelayedFrame* frame = getDelayedFrame();
            
            // Save current context
            GLFWwindow* previousContext = glfwGetCurrentContext();
            glfwMakeContextCurrent(window);
            
            // Update texture only if we have a new frame
            if (frame) {
                updateTexture(*frame);
            }
            
            // Always redraw - even if no new frame, this keeps display smooth
            // and prevents tearing/flicker when called at high rate
            if (hasLatestFrame) {
                glViewport(0, 0, displayWidth, displayHeight);
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                
                renderFrame();
                glfwSwapBuffers(window);
            }
            
            // Restore previous context 
            if (previousContext && previousContext != window) {
                glfwMakeContextCurrent(previousContext);
            }
            
            glfwPollEvents();
            return true;
        }

    } // namespace hil
} // namespace chrono
