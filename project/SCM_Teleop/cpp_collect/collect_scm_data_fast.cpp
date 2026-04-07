// =============================================================================
// FAST SCM Data Collection for NN Tire Model Training
// Optimized version with:
//   - Larger timestep (5e-4 instead of 2e-4)
//   - Shorter simulation time (2.0s instead of 4.0s)
//   - Coarser SCM mesh option
//   - OpenMP parallelization
// =============================================================================

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <mutex>
#include <atomic>
#include <csignal>
#include <algorithm>

// Linux-specific: process control and memory management
#ifdef __linux__
#include <malloc.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/functions/ChFunctionPoly.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#ifdef CHRONO_OPENMP
#include <omp.h>
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Simulation parameters - TUNED FOR SPEED
// =============================================================================

// Timestep: Larger = faster but less accurate. 5e-4 is a good balance.
constexpr double STEP_SIZE_FAST = 5e-4;      // 5x faster than 2e-4
constexpr double STEP_SIZE_ACCURATE = 2e-4;  // Original

// Simulation duration per sample
constexpr double T_DELAY_FAST = 0.5;         // Settling time (was 1.0)
constexpr double T_SWEEP_FAST = 1.5;         // Sweep duration (was 3.0)
constexpr double T_END_FAST = T_DELAY_FAST + T_SWEEP_FAST;  // 2.0s total

constexpr double T_DELAY_ACCURATE = 1.0;
constexpr double T_SWEEP_ACCURATE = 3.0;
constexpr double T_END_ACCURATE = 4.0;

// SCM mesh spacing
constexpr double MESH_SPACING_FAST = 0.10;     // Coarser mesh (was 0.05)
constexpr double MESH_SPACING_ACCURATE = 0.05; // Original fine mesh

// Solver iterations
constexpr int SOLVER_ITERS_FAST = 50;       // Reduced (was 100)
constexpr int SOLVER_ITERS_ACCURATE = 100;

// =============================================================================
// Parameter ranges for Latin Hypercube Sampling
// MATCHES Dallas et al. Table I "Terrain Adaptive Trajectory Planning"
// =============================================================================
struct ParameterRanges {
    // Operating conditions - EXACTLY matching Dallas et al. Table I
    double slip_angle_min = -34.38;  // degrees (= -0.6 rad, Dallas et al.)
    double slip_angle_max = 34.38;   // degrees (= 0.6 rad, Dallas et al.)
    double slip_ratio_min = -1.0;    // Dallas et al. Table I: -1 to 1
    double slip_ratio_max = 1.0;
    double velocity_min = 2.0;       // m/s - Dallas et al. Table I
    double velocity_max = 10.0;
    // Note: Dallas uses 500-5500 N for a lighter vehicle. HMMWV is heavier,
    // so we use 1500-7500 N to avoid "load too small" warnings from tire rig.
    double vertical_load_min = 2500;  // N — HMMWV tire needs ≥2500 to settle on soft SCM
    double vertical_load_max = 7500;
    double steering_rate_min = -0.56; // rad/s - Dallas et al. Table I (CRITICAL!)
    double steering_rate_max = 0.56;
    
    // Bekker parameters - ranges from Dallas et al. Table I
    // Note: Dallas uses k* = kc/b + k_phi as aggregate param, but we keep separate
    // for flexibility. k* range 43000-2080000 N/m^(n+1)
    double bekker_Kphi_min = 0.5e6;  // Pa (0.5 MPa) - covers clay 692k
    double bekker_Kphi_max = 4.0e6;  // Pa (4 MPa)
    double bekker_Kc_min = 0.0;      // Pa
    double bekker_Kc_max = 20000.0;  // Pa (20 kPa) - covers clay 13.2k
    double bekker_n_min = 0.3;       // Dallas et al.: 0.3 to 1.3
    double bekker_n_max = 1.3;
    
    // Mohr-Coulomb parameters - Dallas et al. Table I
    double mohr_cohesion_min = 650.0;   // Pa - Dallas et al.: 650 to 20700 Pa
    double mohr_cohesion_max = 20700.0;
    double mohr_friction_min = 6.0;     // degrees - Dallas et al.: 0.105 to 0.66 rad = 6° to 37.8°
    double mohr_friction_max = 37.8;
    
    // Janosi shear coefficient (shear deformation modulus)
    // Dallas et al. Table I: 0.01 to 0.024 m
    double janosi_shear_min = 0.01;
    double janosi_shear_max = 0.024;
    
    // SCM mesh spacing (node density) - larger = coarser/faster, smaller = finer/slower
    double mesh_spacing_min = 0.05;   // Fine mesh (original accurate)
    double mesh_spacing_max = 0.15;   // Coarser mesh (still reasonable detail)
};

// =============================================================================
// Sample parameters structure - matches Dallas et al. Table I
// =============================================================================
struct SampleParams {
    // Operating conditions (5 params matching Dallas et al.)
    double slip_angle;     // degrees (stored), output as radians
    double slip_ratio;     // longitudinal slip ratio
    double velocity;       // m/s
    double vertical_load;  // N
    double steering_rate;  // rad/s - CRITICAL: Dallas et al. Table I, impacts force generation
    
    // Terrain parameters (5 params matching Dallas et al.)
    double bekker_Kphi;    // N/m^(n+2) - frictional modulus
    double bekker_Kc;      // N/m^(n+1) - cohesive modulus
    double bekker_n;       // sinkage exponent
    double mohr_cohesion;  // Pa
    double mohr_friction;  // degrees (stored), output as radians
    double janosi_shear;   // m - shear deformation modulus
    
    // Simulation parameter (not NN input)
    double mesh_spacing;   // SCM grid spacing (node density)
};

// =============================================================================
// Latin Hypercube Sampling
// =============================================================================
std::vector<SampleParams> GenerateLHSSamples(int n_samples, const ParameterRanges& ranges, unsigned int seed = 42) {
    std::mt19937 rng(seed);
    std::vector<SampleParams> samples(n_samples);
    
    // Generate permutations for each parameter
    auto generate_lhs_values = [&](double min_val, double max_val) {
        std::vector<double> values(n_samples);
        std::vector<int> perm(n_samples);
        std::iota(perm.begin(), perm.end(), 0);
        std::shuffle(perm.begin(), perm.end(), rng);
        
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        for (int i = 0; i < n_samples; i++) {
            double u = (perm[i] + dist(rng)) / n_samples;
            values[i] = min_val + u * (max_val - min_val);
        }
        return values;
    };
    
    // Generate LHS values for each parameter (11 params: 5 operating + 5 terrain + 1 mesh)
    // Operating: slip_angle, slip_ratio, velocity, vertical_load, steering_rate
    // Terrain: bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear
    auto slip_angles = generate_lhs_values(ranges.slip_angle_min, ranges.slip_angle_max);
    auto slip_ratios = generate_lhs_values(ranges.slip_ratio_min, ranges.slip_ratio_max);
    auto velocities = generate_lhs_values(ranges.velocity_min, ranges.velocity_max);
    auto vert_loads = generate_lhs_values(ranges.vertical_load_min, ranges.vertical_load_max);
    auto steering_rates = generate_lhs_values(ranges.steering_rate_min, ranges.steering_rate_max);
    auto bekker_Kphis = generate_lhs_values(ranges.bekker_Kphi_min, ranges.bekker_Kphi_max);
    auto bekker_Kcs = generate_lhs_values(ranges.bekker_Kc_min, ranges.bekker_Kc_max);
    auto bekker_ns = generate_lhs_values(ranges.bekker_n_min, ranges.bekker_n_max);
    auto mohr_cohesions = generate_lhs_values(ranges.mohr_cohesion_min, ranges.mohr_cohesion_max);
    auto mohr_frictions = generate_lhs_values(ranges.mohr_friction_min, ranges.mohr_friction_max);
    auto janosi_shears = generate_lhs_values(ranges.janosi_shear_min, ranges.janosi_shear_max);
    auto mesh_spacings = generate_lhs_values(ranges.mesh_spacing_min, ranges.mesh_spacing_max);
    
    for (int i = 0; i < n_samples; i++) {
        samples[i].slip_angle = slip_angles[i];
        samples[i].slip_ratio = slip_ratios[i];
        samples[i].velocity = velocities[i];
        samples[i].vertical_load = vert_loads[i];
        samples[i].steering_rate = steering_rates[i];
        samples[i].bekker_Kphi = bekker_Kphis[i];
        samples[i].bekker_Kc = bekker_Kcs[i];
        samples[i].bekker_n = bekker_ns[i];
        samples[i].mohr_cohesion = mohr_cohesions[i];
        samples[i].mohr_friction = mohr_frictions[i];
        samples[i].janosi_shear = janosi_shears[i];
        samples[i].mesh_spacing = mesh_spacings[i];
    }
    
    return samples;
}

// =============================================================================
// Single sample data collection (thread-safe)
// =============================================================================
struct SampleResult {
    SampleParams params;
    double Fz;
    double Fx;
    double Fy;
    bool success;
};

SampleResult CollectSingleSample(const SampleParams& params, bool fast_mode) {
    SampleResult result;
    result.params = params;
    result.success = false;
    
    // Select parameters based on mode
    double step_size = fast_mode ? STEP_SIZE_FAST : STEP_SIZE_ACCURATE;
    double t_delay = fast_mode ? T_DELAY_FAST : T_DELAY_ACCURATE;
    double t_end = fast_mode ? T_END_FAST : T_END_ACCURATE;
    // Use per-sample mesh spacing from LHS
    double mesh_spacing = params.mesh_spacing;
    int solver_iters = fast_mode ? SOLVER_ITERS_FAST : SOLVER_ITERS_ACCURATE;
    
    // Tire radius for HMMWV
    constexpr double tire_radius = 0.47;
    
    try {
        // Use unique_ptr for explicit lifetime control and guaranteed cleanup
        // This prevents memory leaks from SCM terrain and collision system
        auto sys = std::make_unique<ChSystemNSC>();
        sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        sys->GetSolver()->AsIterative()->SetMaxIterations(solver_iters);
        
        // Create wheel and tire from JSON files (RigidTire required for SCM terrain)
        std::string data_path = GetChronoDataPath();
        auto wheel = ReadWheelJSON(data_path + "vehicle/hmmwv/wheel/HMMWV_Wheel.json");
        auto tire = ReadTireJSON(data_path + "vehicle/hmmwv/tire/HMMWV_RigidTire.json");
        tire->SetStepsize(step_size);
        
        // Create tire test rig - use raw pointer since rig doesn't own the system
        ChTireTestRig rig(wheel, tire, sys.get());
        
        // Set basic parameters
        rig.SetGravitationalAcceleration(9.8);
        rig.SetNormalLoad(params.vertical_load);
        // Note: Dallas paper does not use camber angle, so we set it to 0
        rig.SetCamberAngle(0.0);
        
        // Set tire parameters
        rig.SetTireStepsize(step_size);
        rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
        rig.SetTireVisualizationType(VisualizationType::PRIMITIVES);
        
        // Set up SCM terrain with sample-specific parameters
        ChTireTestRig::TerrainParamsSCM scm_params;
        scm_params.length = 200.0;
        scm_params.width = 1.0;
        scm_params.Bekker_Kphi = params.bekker_Kphi;
        scm_params.Bekker_Kc = params.bekker_Kc;
        scm_params.Bekker_n = params.bekker_n;
        scm_params.Mohr_cohesion = params.mohr_cohesion;
        scm_params.Mohr_friction = params.mohr_friction;
        scm_params.Janosi_shear = params.janosi_shear;
        scm_params.grid_spacing = mesh_spacing;
        rig.SetTerrainSCM(scm_params);
        
        // Set motion functions
        double base_speed = params.velocity;
        // Angular velocity to achieve target slip ratio: omega = v/r * (1 + slip)
        double ang_speed = (base_speed / tire_radius) * (1.0 + params.slip_ratio);
        
        // CRITICAL: Use a linear slip angle function to independently control
        // slip_angle AND steering_rate (d(slip_angle)/dt) per Dallas et al.
        // slip(t) = target_slip_angle + steering_rate * (t - t_measure)
        // At t = t_measure: slip = target_slip_angle, d(slip)/dt = steering_rate
        double target_slip_rad = params.slip_angle * CH_DEG_TO_RAD;
        double t_measure = t_delay + 0.5;  // Measure 0.5s after settling
        
        // ChFunctionPoly: y = coeff[0] + coeff[1]*t + coeff[2]*t^2 + ...
        // We want: y(t) = target_slip + steering_rate * (t - t_measure)
        //                = (target_slip - steering_rate * t_measure) + steering_rate * t
        auto slip_func = chrono_types::make_shared<ChFunctionPoly>();
        slip_func->SetCoefficients({
            target_slip_rad - params.steering_rate * t_measure,  // a0
            params.steering_rate                                  // a1
        });
        
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(base_speed));
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(ang_speed));
        rig.SetSlipAngleFunction(slip_func);
        rig.SetTimeDelay(t_delay);
        rig.Initialize(ChTireTestRig::Mode::TEST);
        
        // Simulation loop - measure forces at t_measure (already defined above)
        // At this time: slip_angle = target, d(slip_angle)/dt = steering_rate
        TerrainForce measured_force;
        double t = 0;
        while (t < t_end) {
            rig.Advance(step_size);
            t += step_size;
            
            // Capture forces near measurement time
            if (std::abs(t - t_measure) < step_size) {
                measured_force = rig.ReportTireForce();
            }
        }
        
        result.Fz = measured_force.force.z();
        result.Fx = measured_force.force.x();
        result.Fy = measured_force.force.y();
        result.success = true;
        
        // CRITICAL: Explicit cleanup to prevent memory leaks from SCM terrain
        // Clear all bodies/links before system destruction to break circular refs
        sys->Clear();
        
        // Force the system unique_ptr to destruct NOW, before malloc_trim
        sys.reset();
        
        // Force glibc to return freed memory to the OS
        // Without this, heap grows unbounded due to fragmentation
#ifdef __linux__
        malloc_trim(0);
#endif
        
    } catch (const std::exception& e) {
        std::cerr << "Sample failed: " << e.what() << std::endl;
        result.Fz = 0;
        result.Fx = 0;
        result.Fy = 0;
        result.success = false;
    }
    
    return result;
}

// =============================================================================
// Global signal handling for graceful Ctrl+C termination
// =============================================================================
volatile sig_atomic_t g_stop_requested = 0;

void signal_handler(int signum) {
    g_stop_requested = 1;
    std::cout << "\n[SIGNAL] Ctrl+C received, finishing current samples and stopping...\n";
}

// =============================================================================
// Generate a single LHS sample (for continuous mode)
// =============================================================================
SampleParams GenerateSingleLHSSample(const ParameterRanges& ranges, std::mt19937& rng) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    SampleParams sample;
    
    sample.slip_angle = ranges.slip_angle_min + dist(rng) * (ranges.slip_angle_max - ranges.slip_angle_min);
    sample.slip_ratio = ranges.slip_ratio_min + dist(rng) * (ranges.slip_ratio_max - ranges.slip_ratio_min);
    sample.velocity = ranges.velocity_min + dist(rng) * (ranges.velocity_max - ranges.velocity_min);
    sample.vertical_load = ranges.vertical_load_min + dist(rng) * (ranges.vertical_load_max - ranges.vertical_load_min);
    sample.steering_rate = ranges.steering_rate_min + dist(rng) * (ranges.steering_rate_max - ranges.steering_rate_min);
    sample.bekker_Kphi = ranges.bekker_Kphi_min + dist(rng) * (ranges.bekker_Kphi_max - ranges.bekker_Kphi_min);
    sample.bekker_Kc = ranges.bekker_Kc_min + dist(rng) * (ranges.bekker_Kc_max - ranges.bekker_Kc_min);
    sample.bekker_n = ranges.bekker_n_min + dist(rng) * (ranges.bekker_n_max - ranges.bekker_n_min);
    sample.mohr_cohesion = ranges.mohr_cohesion_min + dist(rng) * (ranges.mohr_cohesion_max - ranges.mohr_cohesion_min);
    sample.mohr_friction = ranges.mohr_friction_min + dist(rng) * (ranges.mohr_friction_max - ranges.mohr_friction_min);
    sample.janosi_shear = ranges.janosi_shear_min + dist(rng) * (ranges.janosi_shear_max - ranges.janosi_shear_min);
    sample.mesh_spacing = ranges.mesh_spacing_min + dist(rng) * (ranges.mesh_spacing_max - ranges.mesh_spacing_min);
    
    return sample;
}

// =============================================================================
// Write a single result to CSV (thread-safe)
// Dallas et al. format: 10 NN inputs + measured forces
// =============================================================================
void WriteResultToCSV(std::ofstream& csv_file, std::mutex& csv_mutex, const SampleResult& result) {
    if (!result.success) return;
    
    // Convert degrees to radians for output (Dallas et al. format)
    double slip_angle_rad = result.params.slip_angle * CH_DEG_TO_RAD;
    double mohr_friction_rad = result.params.mohr_friction * CH_DEG_TO_RAD;
    
    // Dallas et al. order: operating params (5), then terrain params (5), then forces
    std::ostringstream line;
    line << std::fixed << std::setprecision(6)
         // Operating conditions (5 params per Dallas et al. Table I)
         << result.params.slip_ratio << ","
         << slip_angle_rad << ","
         << result.params.velocity << ","
         << result.params.vertical_load << ","
         << result.params.steering_rate << ","
         // Terrain parameters (5 params per Dallas et al. Table I)
         << result.params.bekker_Kphi << ","
         << result.params.bekker_Kc << ","
         << result.params.bekker_n << ","
         << result.params.mohr_cohesion << ","
         << mohr_friction_rad << ","
         << result.params.janosi_shear << ","
         // Simulation meta-parameter (not NN input)
         << result.params.mesh_spacing << ","
         // Forces (outputs)
         << result.Fz << ","
         << result.Fx << ","
         << result.Fy << "\n";
    
    std::lock_guard<std::mutex> lock(csv_mutex);
    csv_file << line.str();
    csv_file.flush();  // Ensure data is written immediately
}

// =============================================================================
// Process a batch of samples (used by subprocess mode)
// Returns number of successful samples
// =============================================================================
int ProcessBatch(const std::vector<SampleParams>& samples, const std::string& output_file,
                 bool fast_mode, int num_threads, bool use_parallel, bool append_mode) {
    
#ifdef CHRONO_OPENMP
    if (use_parallel && num_threads > 0) {
        omp_set_num_threads(num_threads);
    }
#endif
    
    // Open CSV file (append if not first batch)
    std::ofstream csv_file;
    if (append_mode) {
        csv_file.open(output_file, std::ios::app);
    } else {
        csv_file.open(output_file);
        // CSV header matches Dallas et al. Table I order:
        // Operating params (5), Terrain params (6), mesh_spacing, Forces (3)
        csv_file << "slip_ratio,slip_angle,velocity,vertical_load,steering_rate,"
                 << "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,mesh_spacing,"
                 << "Fz,Fx,Fy\n";
    }
    
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return 0;
    }
    
    std::mutex csv_mutex;
    std::atomic<int> success_count{0};
    int n_samples = static_cast<int>(samples.size());
    
    // Run warm-up sample single-threaded
    if (n_samples > 0) {
        SampleResult warmup = CollectSingleSample(samples[0], fast_mode);
        if (warmup.success) {
            WriteResultToCSV(csv_file, csv_mutex, warmup);
            success_count++;
        }
    }
    
#ifdef CHRONO_OPENMP
    #pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 1; i < n_samples; i++) {
        if (g_stop_requested) continue;
        
        SampleResult result = CollectSingleSample(samples[i], fast_mode);
        
        if (result.success) {
            WriteResultToCSV(csv_file, csv_mutex, result);
            success_count++;
        }
    }
    
    csv_file.close();
    return success_count.load();
}

// =============================================================================
// Run collection in subprocess batches to prevent memory accumulation
// Each subprocess handles a batch and exits, fully releasing memory
// =============================================================================
#ifdef __linux__
void CollectWithSubprocessBatching(int n_samples, const std::string& output_file,
                                   bool fast_mode, int num_threads, bool use_parallel,
                                   int batch_size) {
    
    std::cout << "\n=== Subprocess-Batched SCM Data Collection ===" << std::endl;
    std::cout << "Total samples: " << n_samples << std::endl;
    std::cout << "Batch size: " << batch_size << " (memory released after each batch)" << std::endl;
    std::cout << "Mode: " << (fast_mode ? "FAST" : "ACCURATE") << std::endl;
    
    // Write CSV header
    {
        std::ofstream csv_file(output_file);
        // CSV header matches Dallas et al. Table I order
        csv_file << "slip_ratio,slip_angle,velocity,vertical_load,steering_rate,"
                 << "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,mesh_spacing,"
                 << "Fz,Fx,Fy\n";
        csv_file.close();
    }
    
    // Generate all LHS samples upfront (small memory footprint)
    ParameterRanges ranges;
    auto all_samples = GenerateLHSSamples(n_samples, ranges);
    
    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int total_success = 0;
    int total_processed = 0;
    
    for (int batch_start = 0; batch_start < n_samples && !g_stop_requested; batch_start += batch_size) {
        int batch_end = std::min(batch_start + batch_size, n_samples);
        int current_batch_size = batch_end - batch_start;
        
        std::cout << "\n--- Batch " << (batch_start / batch_size + 1) 
                  << ": samples " << batch_start << "-" << (batch_end - 1) << " ---" << std::endl;
        
        pid_t pid = fork();
        
        if (pid == 0) {
            // Child process: process this batch and exit
            SetChronoDataPath(CHRONO_DATA_DIR);
            
            // Extract batch samples
            std::vector<SampleParams> batch_samples(
                all_samples.begin() + batch_start,
                all_samples.begin() + batch_end
            );
            
            int success = ProcessBatch(batch_samples, output_file, fast_mode, 
                                       num_threads, use_parallel, true);
            
            // Exit with success count encoded (capped at 255)
            _exit(std::min(success, 255));
            
        } else if (pid > 0) {
            // Parent process: wait for child
            int status;
            waitpid(pid, &status, 0);
            
            if (WIFEXITED(status)) {
                int batch_success = WEXITSTATUS(status);
                // If batch was larger than 255, assume all succeeded if exit code is 255
                if (current_batch_size > 255 && batch_success == 255) {
                    batch_success = current_batch_size;  // Assume all succeeded
                }
                total_success += batch_success;
                total_processed += current_batch_size;
                
                auto now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                double rate = total_processed / elapsed;
                double remaining = (n_samples - total_processed) / rate;
                
                std::cout << "Batch complete: " << batch_success << "/" << current_batch_size << " succeeded" << std::endl;
                std::cout << "Progress: " << total_processed << "/" << n_samples 
                          << " (" << std::fixed << std::setprecision(1) 
                          << (100.0 * total_processed / n_samples) << "%)"
                          << " - " << std::setprecision(2) << rate << " samples/s"
                          << " - ETA: " << std::setprecision(0) << remaining << "s" << std::endl;
            } else {
                std::cerr << "Batch process crashed!" << std::endl;
                total_processed += current_batch_size;
            }
        } else {
            std::cerr << "Fork failed!" << std::endl;
            break;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();
    
    std::cout << "\n=== Collection Complete ===" << std::endl;
    std::cout << "Total time: " << std::fixed << std::setprecision(1) << total_time << "s" << std::endl;
    std::cout << "Total samples: " << total_processed << std::endl;
    std::cout << "Successful samples: " << total_success << std::endl;
    if (total_time > 0) {
        std::cout << "Average rate: " << std::setprecision(2) << (total_processed / total_time) << " samples/s" << std::endl;
    }
    std::cout << "Output: " << output_file << std::endl;
}
#endif

// =============================================================================
// Main data collection function with optional parallelization
// Supports both fixed-count and continuous (Ctrl+C) modes
// =============================================================================
void CollectSCMDataFast(int n_samples, const std::string& output_file, 
                        bool fast_mode = true, int num_threads = 0, bool use_parallel = true) {
    
    bool continuous_mode = (n_samples <= 0);
    
    std::cout << "\n=== Fast SCM Data Collection ===" << std::endl;
    std::cout << "Mode: " << (fast_mode ? "FAST" : "ACCURATE") << std::endl;
    if (continuous_mode) {
        std::cout << "Samples: CONTINUOUS (press Ctrl+C to stop)" << std::endl;
    } else {
        std::cout << "Samples: " << n_samples << std::endl;
    }
    std::cout << "Step size: " << (fast_mode ? STEP_SIZE_FAST : STEP_SIZE_ACCURATE) << std::endl;
    std::cout << "Sim duration: " << (fast_mode ? T_END_FAST : T_END_ACCURATE) << "s per sample" << std::endl;
    std::cout << "Mesh spacing: LHS varied [0.05, 0.15]m" << std::endl;
    
#ifdef CHRONO_OPENMP
    if (use_parallel) {
        if (num_threads <= 0) {
            // IMPORTANT: SCM terrain uses ~50-200MB per instance. Cap threads to avoid OOM.
            // With 30 threads at ~100MB each = 3GB concurrent memory + fragmentation = OOM
            // Safe default: min(max_threads - 2, 8) to stay under ~1GB concurrent
            int max_available = omp_get_max_threads() - 2;
            num_threads = std::min(max_available, 8);  // Cap at 8 threads for SCM
        }
        omp_set_num_threads(num_threads);
        std::cout << "OpenMP threads: " << num_threads << " (capped for SCM memory)" << std::endl;
    } else {
        num_threads = 1;
        omp_set_num_threads(1);
        std::cout << "OpenMP: Disabled (single-threaded)" << std::endl;
    }
#else
    num_threads = 1;
    std::cout << "OpenMP: Not available (single-threaded)" << std::endl;
#endif
    
    // Setup signal handler for graceful termination
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Open CSV file and write header
    std::ofstream csv_file(output_file);
    if (!csv_file.is_open()) {
        throw std::runtime_error("Failed to open output file: " + output_file);
    }
    // CSV header matches Dallas et al. Table I order
    csv_file << "slip_ratio,slip_angle,velocity,vertical_load,steering_rate,"
             << "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,mesh_spacing,"
             << "Fz,Fx,Fy\n";
    csv_file.flush();
    
    std::mutex csv_mutex;
    ParameterRanges ranges;
    std::atomic<int> completed{0};
    std::atomic<int> success_count{0};
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (continuous_mode) {
        // Continuous mode: generate and process samples until Ctrl+C
        std::cout << "\nStarting continuous data collection..." << std::endl;
        std::cout << "Press Ctrl+C to stop and save data." << std::endl;
        
        // CRITICAL: Run one warm-up sample single-threaded first
        std::cout << "Running warm-up sample to initialize Chrono..." << std::endl;
        {
            std::mt19937 warmup_rng(42);
            SampleParams warmup_params = GenerateSingleLHSSample(ranges, warmup_rng);
            SampleResult warmup = CollectSingleSample(warmup_params, fast_mode);
            if (warmup.success) {
                WriteResultToCSV(csv_file, csv_mutex, warmup);
                success_count++;
            }
            completed++;
        }
        std::cout << "Warm-up complete, starting parallel collection..." << std::endl;
        
        // Each thread gets its own RNG seeded differently
        const int batch_size = num_threads;  // Process in batches of thread count
        
        while (!g_stop_requested) {
            // Generate a batch of samples
            std::vector<SampleParams> batch(batch_size);
            for (int i = 0; i < batch_size; i++) {
                unsigned int seed = static_cast<unsigned int>(
                    std::chrono::steady_clock::now().time_since_epoch().count() + i);
                std::mt19937 rng(seed);
                batch[i] = GenerateSingleLHSSample(ranges, rng);
            }
            
#ifdef CHRONO_OPENMP
            #pragma omp parallel for schedule(dynamic)
#endif
            for (int i = 0; i < batch_size; i++) {
                if (g_stop_requested) continue;
                
                SampleResult result = CollectSingleSample(batch[i], fast_mode);
                
                if (result.success) {
                    WriteResultToCSV(csv_file, csv_mutex, result);
                    success_count++;
                }
                
                int done = ++completed;
                if (done % 10 == 0) {
#ifdef CHRONO_OPENMP
                    #pragma omp critical
#endif
                    {
                        auto now = std::chrono::high_resolution_clock::now();
                        double elapsed = std::chrono::duration<double>(now - start_time).count();
                        double rate = done / elapsed;
                        std::cout << "Samples: " << done 
                                  << " (success: " << success_count.load() << ")"
                                  << " - " << std::fixed << std::setprecision(2) << rate << " samples/s"
                                  << std::endl;
                    }
                }
            }
        }
    } else {
        // Fixed-count mode: generate all LHS samples upfront
        std::cout << "\nGenerating " << n_samples << " LHS samples..." << std::endl;
        auto samples = GenerateLHSSamples(n_samples, ranges);
        
        // CRITICAL: Run one warm-up sample single-threaded to initialize all global/static state
        // (Chrono data paths, JSON parser, BULLET collision system, etc.)
        // This prevents race conditions during lazy initialization with OpenMP
        std::cout << "Running warm-up sample to initialize Chrono..." << std::endl;
        {
            SampleResult warmup = CollectSingleSample(samples[0], fast_mode);
            if (warmup.success) {
                WriteResultToCSV(csv_file, csv_mutex, warmup);
                success_count++;
            }
            completed++;
        }
        std::cout << "Warm-up complete, starting parallel collection..." << std::endl;
        
#ifdef CHRONO_OPENMP
        #pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 1; i < n_samples; i++) {  // Start from 1, sample 0 already done
            if (g_stop_requested) continue;
            
            SampleResult result = CollectSingleSample(samples[i], fast_mode);
            
            // Write result immediately (thread-safe)
            if (result.success) {
                WriteResultToCSV(csv_file, csv_mutex, result);
                success_count++;
            }
            
            int done = ++completed;
            if (done % 10 == 0 || done == n_samples) {
#ifdef CHRONO_OPENMP
                #pragma omp critical
#endif
                {
                    auto now = std::chrono::high_resolution_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_time).count();
                    double rate = done / elapsed;
                    double remaining = (n_samples - done) / rate;
                    std::cout << "Progress: " << done << "/" << n_samples 
                              << " (" << std::fixed << std::setprecision(1) 
                              << (100.0 * done / n_samples) << "%)"
                              << " - " << std::setprecision(2) << rate << " samples/s"
                              << " - ETA: " << std::setprecision(0) << remaining << "s" << std::endl;
                }
            }
        }
    }
    
    csv_file.close();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();
    
    std::cout << "\n=== Collection Complete ===" << std::endl;
    std::cout << "Total time: " << std::fixed << std::setprecision(1) << total_time << "s" << std::endl;
    std::cout << "Total samples: " << completed.load() << std::endl;
    std::cout << "Successful samples: " << success_count.load() << std::endl;
    if (total_time > 0) {
        std::cout << "Average rate: " << std::setprecision(2) << (completed.load() / total_time) << " samples/s" << std::endl;
    }
    std::cout << "Output: " << output_file << std::endl;
}

// =============================================================================
int main(int argc, char* argv[]) {
    int n_samples = 0;  // 0 = continuous mode (run until Ctrl+C)
    std::string output_file = "scm_training_data_fast.csv";
    bool fast_mode = true;
    bool use_parallel = true;
    int num_threads = 0;  // 0 = auto-detect
    int batch_size = 0;   // 0 = no subprocess batching, use single process
    bool samples_specified = false;
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--accurate" || arg == "-a") {
            fast_mode = false;
            use_parallel = false;  // Single-threaded
        } else if (arg == "--parallel-only" || arg == "-p") {
            // Use accurate settings but still parallelize
            fast_mode = false;
            use_parallel = true;
        } else if (arg == "--threads" || arg == "-t") {
            if (i + 1 < argc) {
                num_threads = std::atoi(argv[++i]);
            }
        } else if (arg == "--batch-size" || arg == "-b") {
            // Subprocess batching: process N samples per subprocess to prevent memory accumulation
            if (i + 1 < argc) {
                batch_size = std::atoi(argv[++i]);
            }
        } else if (arg == "--continuous" || arg == "-c") {
            n_samples = 0;  // Continuous mode
            samples_specified = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [num_samples] [output.csv] [options]\n"
                      << "\nSample count:\n"
                      << "  [no number]         Continuous mode (run until Ctrl+C)\n"
                      << "  N                   Run exactly N samples then exit\n"
                      << "  --continuous, -c    Explicitly set continuous mode\n"
                      << "\nOptions:\n"
                      << "  --accurate, -a      Use accurate (slow) settings, single-threaded\n"
                      << "  --parallel-only, -p Use accurate settings WITH parallelization\n"
                      << "  --threads N, -t N   Number of OpenMP threads (0=auto)\n"
                      << "  --batch-size N, -b N  Process N samples per subprocess (prevents memory leaks)\n"
                      << "                        Recommended: 500-2000 for large runs (>5000 samples)\n"
                      << "  --help, -h          Show this help\n"
                      << "\nLHS Parameters (11 NN inputs + 1 sim param) per Dallas et al. Table I:\n"
                      << "  Operating (5): slip_ratio, slip_angle, velocity, vertical_load, steering_rate\n"
                      << "  Terrain (6): bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction, janosi_shear\n"
                      << "  Sim only: mesh_spacing [0.05, 0.15]m (SCM node density, not NN input)\n"
                      << "\nFast mode (default):\n"
                      << "  - Step size: " << STEP_SIZE_FAST << "s\n"
                      << "  - Sim time: " << T_END_FAST << "s per sample\n"
                      << "  - Parallelized with OpenMP\n"
                      << "\nAccurate mode (--accurate):\n"
                      << "  - Step size: " << STEP_SIZE_ACCURATE << "s\n"
                      << "  - Sim time: " << T_END_ACCURATE << "s per sample\n"
                      << "  - Single-threaded\n"
                      << "\nParallel-only mode (--parallel-only):\n"
                      << "  - Same accuracy as --accurate\n"
                      << "  - Parallelized with OpenMP (Nx speedup)\n"
                      << "\nMemory management (--batch-size):\n"
                      << "  For large sample counts (>5000), use --batch-size to prevent\n"
                      << "  memory accumulation. Each batch runs in a subprocess that exits\n"
                      << "  after completion, fully releasing all memory.\n"
                      << "  Example: " << argv[0] << " 50000 --batch-size 1000\n"
                      << "\nData is written incrementally to CSV after each completed sample.\n"
                      << "Use Ctrl+C to stop at any time - all data up to that point is saved.\n";
            return 0;
        } else if (arg[0] >= '0' && arg[0] <= '9') {
            n_samples = std::atoi(argv[i]);
            samples_specified = true;
        } else if (arg.find(".csv") != std::string::npos) {
            output_file = arg;
        }
    }
    
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    try {
#ifdef __linux__
        // Use subprocess batching for large fixed-count runs
        if (batch_size > 0 && n_samples > 0) {
            CollectWithSubprocessBatching(n_samples, output_file, fast_mode, 
                                         num_threads, use_parallel, batch_size);
            return 0;
        }
#else
        if (batch_size > 0) {
            std::cerr << "Warning: --batch-size requires Linux (fork). Using single-process mode." << std::endl;
        }
#endif
        CollectSCMDataFast(n_samples, output_file, fast_mode, num_threads, use_parallel);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
