// =============================================================================
// Full Vehicle SCM Data Collection for NN Tire Model Training
// =============================================================================
//
// Unlike the tire test rig, this collects data from actual vehicle simulation:
// - Real tire loads from vehicle weight and load transfer
// - Natural slip angles from cornering maneuvers  
// - Forces that match what the estimator sees
//
// This should produce more accurate training data for the terrain estimator.
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>

#ifdef __linux__
#include <malloc.h>
#endif

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_OPENMP
#include <omp.h>
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Terrain parameter ranges (matching cpp_collect)
// =============================================================================
struct TerrainRanges {
    double bekker_Kphi_min = 0.5e6;
    double bekker_Kphi_max = 4.0e6;
    double bekker_Kc_min = 0;
    double bekker_Kc_max = 20000;
    double bekker_n_min = 0.3;
    double bekker_n_max = 1.5;
    double mohr_cohesion_min = 0;
    double mohr_cohesion_max = 10000;
    double mohr_friction_min = 10;   // degrees
    double mohr_friction_max = 45;   // degrees
    double janosi_shear_min = 0.005;
    double janosi_shear_max = 0.05;
};

// =============================================================================
// Sample parameters for a single run
// =============================================================================
struct TerrainParams {
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;  // degrees
    double janosi_shear;
};

// =============================================================================
// Single tire measurement
// =============================================================================
struct TireMeasurement {
    double time;
    int axle;           // 0 = front, 1 = rear
    int side;           // 0 = left, 1 = right
    double slip_angle;  // radians
    double slip_ratio;
    double camber;      // radians
    double velocity;    // forward velocity m/s
    double Fz;          // vertical force (positive = compression)
    double Fx;          // longitudinal force
    double Fy;          // lateral force
    // Terrain params for this sample
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;  // degrees
    double janosi_shear;
};

// =============================================================================
// Generate random terrain parameters
// =============================================================================
TerrainParams GenerateRandomTerrain(const TerrainRanges& ranges, std::mt19937& rng) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    
    TerrainParams params;
    params.bekker_Kphi = ranges.bekker_Kphi_min + dist(rng) * (ranges.bekker_Kphi_max - ranges.bekker_Kphi_min);
    params.bekker_Kc = ranges.bekker_Kc_min + dist(rng) * (ranges.bekker_Kc_max - ranges.bekker_Kc_min);
    params.bekker_n = ranges.bekker_n_min + dist(rng) * (ranges.bekker_n_max - ranges.bekker_n_min);
    params.mohr_cohesion = ranges.mohr_cohesion_min + dist(rng) * (ranges.mohr_cohesion_max - ranges.mohr_cohesion_min);
    params.mohr_friction = ranges.mohr_friction_min + dist(rng) * (ranges.mohr_friction_max - ranges.mohr_friction_min);
    params.janosi_shear = ranges.janosi_shear_min + dist(rng) * (ranges.janosi_shear_max - ranges.janosi_shear_min);
    
    return params;
}

// =============================================================================
// Run a single vehicle simulation and collect tire data
// =============================================================================
std::vector<TireMeasurement> RunVehicleSimulation(
    const TerrainParams& terrain,
    int sample_idx,
    double sim_duration = 15.0,
    double step_size = 3e-3,
    double mesh_spacing = 0.1,
    bool verbose = false
) {
    std::vector<TireMeasurement> measurements;
    
    // Create system
    auto sys = std::make_unique<ChSystemSMC>();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys->GetSolver()->AsIterative()->SetMaxIterations(100);
    
    // Create HMMWV vehicle
    HMMWV_Full hmmwv(sys.get());
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.6), ChQuaterniond(1, 0, 0, 0)));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);  // RIGID for SCM interaction
    hmmwv.Initialize();
    
    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::NONE);
    hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv.SetTireVisualizationType(VisualizationType::NONE);
    
    // Create SCM terrain
    SCMTerrain scm_terrain(sys.get());
    scm_terrain.SetSoilParameters(
        terrain.bekker_Kphi,
        terrain.bekker_Kc,
        terrain.bekker_n,
        terrain.mohr_cohesion,
        terrain.mohr_friction * CH_DEG_TO_RAD,  // Convert to radians
        terrain.janosi_shear,
        2e8,   // elastic stiffness
        3e4    // damping
    );
    
    // Add moving patch for efficiency
    scm_terrain.AddMovingPatch(
        hmmwv.GetChassisBody(),
        ChVector3d(0, 0, 0),
        ChVector3d(6, 3, 1)
    );
    
    scm_terrain.SetPlotType(SCMTerrain::PLOT_NONE, 0, 0);
    scm_terrain.Initialize(100.0, 30.0, mesh_spacing);
    
    // Vehicle parameters
    const double Lf = 1.689;  // front axle to CG
    const double Lr = 1.689;  // rear axle to CG
    
    // Simulation loop
    double t = 0.0;
    int sample_count = 0;
    const int sample_interval = 100;  // Every 100 steps = 0.3s
    
    while (t < sim_duration) {
        // Generate driving inputs: acceleration then cornering
        double steer = 0.0;
        double throttle = 0.0;
        
        if (t < 2.0) {
            // Accelerate straight
            steer = 0.0;
            throttle = 0.5;
        } else if (t < 5.0) {
            // Left turn ramp up
            steer = 0.3 * (t - 2.0) / 3.0;
            throttle = 0.35;
        } else if (t < 8.0) {
            // Hold left turn
            steer = 0.3;
            throttle = 0.35;
        } else if (t < 11.0) {
            // Transition to right
            steer = 0.3 - 0.6 * (t - 8.0) / 3.0;
            throttle = 0.35;
        } else {
            // Hold right turn
            steer = -0.3;
            throttle = 0.35;
        }
        
        // Apply inputs
        DriverInputs inputs;
        inputs.m_steering = steer;
        inputs.m_throttle = throttle;
        inputs.m_braking = 0.0;
        
        // Synchronize and advance
        scm_terrain.Synchronize(t);
        hmmwv.Synchronize(t, inputs, scm_terrain);
        scm_terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        
        // Sample tire data periodically after initial settling
        if (t > 3.0 && (sample_count % sample_interval) == 0) {
            // Get chassis state
            auto chassis = hmmwv.GetChassisBody();
            auto vel_global = chassis->GetPosDt();
            auto rot = chassis->GetRot();
            auto rot_inv = rot.GetInverse();
            auto vel_body = rot_inv.Rotate(vel_global);
            auto omega_vec = chassis->GetAngVelLocal();
            
            double u = vel_body.x();  // forward velocity
            double v = vel_body.y();  // lateral velocity
            double omega = omega_vec.z();  // yaw rate
            double delta = steer * 0.5;  // steering angle (approximate ratio)
            
            // Skip if vehicle too slow
            if (u < 2.0) {
                sample_count++;
                t += step_size;
                continue;
            }
            
            // Get tire forces for all 4 wheels
            for (int axle = 0; axle < 2; axle++) {
                for (int side = 0; side < 2; side++) {
                    auto side_enum = (side == 0) ? VehicleSide::LEFT : VehicleSide::RIGHT;
                    auto tire = hmmwv.GetVehicle().GetTire(axle, side_enum);
                    
                    // Get tire force in global frame, then transform
                    auto tire_force = tire->ReportTireForce(&scm_terrain);
                    auto force_local = rot_inv.Rotate(tire_force.force);
                    
                    // Compute slip angle for this wheel
                    double L = (axle == 0) ? Lf : -Lr;  // + for front, - for rear
                    double wheel_delta = (axle == 0) ? delta : 0.0;  // only front steers
                    
                    // Slip angle: alpha = atan2(v + L*omega, u) - delta
                    double eps = 0.5;
                    double u_safe = std::max(u, eps);
                    double alpha = std::atan2(v + L * omega, u_safe) - wheel_delta;
                    
                    // Store measurement
                    TireMeasurement m;
                    m.time = t;
                    m.axle = axle;
                    m.side = side;
                    m.slip_angle = alpha;
                    m.slip_ratio = 0.0;  // Not computing this accurately for now
                    m.camber = 0.0;
                    m.velocity = u;
                    m.Fz = force_local.z();  // Positive = compression
                    m.Fx = force_local.x();
                    m.Fy = force_local.y();
                    m.bekker_Kphi = terrain.bekker_Kphi;
                    m.bekker_Kc = terrain.bekker_Kc;
                    m.bekker_n = terrain.bekker_n;
                    m.mohr_cohesion = terrain.mohr_cohesion;
                    m.mohr_friction = terrain.mohr_friction;
                    m.janosi_shear = terrain.janosi_shear;
                    
                    // Only keep samples with reasonable forces
                    if (m.Fz > 1000 && std::abs(m.Fy) < 20000 && std::abs(alpha) > 0.01) {
                        measurements.push_back(m);
                    }
                }
            }
        }
        
        sample_count++;
        t += step_size;
    }
    
    // Cleanup
    sys->Clear();
    sys.reset();
    
#ifdef __linux__
    malloc_trim(0);
#endif
    
    if (verbose) {
        std::cout << "Sample " << sample_idx << ": collected " << measurements.size() 
                  << " tire measurements (n=" << terrain.bekker_n 
                  << ", phi=" << terrain.mohr_friction << "°)" << std::endl;
    }
    
    return measurements;
}

// =============================================================================
// Write measurements to CSV
// =============================================================================
void WriteMeasurementsToCSV(std::ofstream& csv_file, std::mutex& csv_mutex,
                            const std::vector<TireMeasurement>& measurements) {
    std::ostringstream buffer;
    buffer << std::fixed << std::setprecision(6);
    
    for (const auto& m : measurements) {
        buffer << m.Fz << ","                 // vertical_load
               << m.slip_angle << ","         // slip_angle (radians)
               << m.slip_ratio << ","         // longitudinal_slip
               << m.camber << ","             // camber_angle
               << m.velocity << ","           // velocity
               << m.bekker_Kphi << ","
               << m.bekker_Kc << ","
               << m.bekker_n << ","
               << m.mohr_cohesion << ","
               << m.mohr_friction << ","      // degrees
               << m.janosi_shear << ","
               << 0.1 << ","                  // mesh_spacing (fixed)
               << m.Fz << ","                 // Fz (same as vertical_load for vehicle)
               << m.Fx << ","
               << m.Fy << "\n";
    }
    
    std::lock_guard<std::mutex> lock(csv_mutex);
    csv_file << buffer.str();
    csv_file.flush();
}

// =============================================================================
// Main function
// =============================================================================
int main(int argc, char* argv[]) {
    // Parse command line
    ChCLI cli(argv[0]);
    cli.AddOption<int>("", "n", "Number of vehicle simulations to run", "100");
    cli.AddOption<std::string>("", "o", "Output CSV file", "vehicle_tire_data.csv");
    cli.AddOption<int>("", "j", "Number of parallel threads", "4");
    cli.AddOption<double>("", "t", "Simulation duration per run (seconds)", "15");
    cli.AddOption<double>("", "mesh", "SCM mesh spacing", "0.1");
    cli.AddOption<bool>("", "v", "Verbose output", "true");
    cli.AddOption<int>("", "seed", "Random seed", "12345");
    
    if (!cli.Parse(argc, argv, true)) {
        cli.Help();
        return 1;
    }
    
    int num_sims = cli.GetAsType<int>("n");
    std::string output_file = cli.GetAsType<std::string>("o");
    int num_threads = cli.GetAsType<int>("j");
    double sim_duration = cli.GetAsType<double>("t");
    double mesh_spacing = cli.GetAsType<double>("mesh");
    bool verbose = cli.GetAsType<bool>("v");
    int seed = cli.GetAsType<int>("seed");
    
    std::cout << "==================================================================" << std::endl;
    std::cout << "Full Vehicle SCM Data Collection" << std::endl;
    std::cout << "==================================================================" << std::endl;
    std::cout << "Simulations: " << num_sims << std::endl;
    std::cout << "Threads: " << num_threads << std::endl;
    std::cout << "Duration each: " << sim_duration << "s" << std::endl;
    std::cout << "Output: " << output_file << std::endl;
    std::cout << "==================================================================" << std::endl;
    
    // Open output CSV
    std::ofstream csv_file(output_file);
    csv_file << "vertical_load,slip_angle,longitudinal_slip,camber_angle,velocity,"
             << "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,mesh_spacing,"
             << "Fz,Fx,Fy\n";
    std::mutex csv_mutex;
    
    // Setup terrain ranges
    TerrainRanges ranges;
    
    // Progress tracking
    std::atomic<int> completed(0);
    std::atomic<int> total_samples(0);
    auto start_time = std::chrono::steady_clock::now();
    
#ifdef CHRONO_OPENMP
    omp_set_num_threads(num_threads);
    
    #pragma omp parallel
    {
        // Each thread gets its own RNG
        int thread_id = omp_get_thread_num();
        std::mt19937 rng(seed + thread_id * 1000);
        
        #pragma omp for schedule(dynamic)
        for (int i = 0; i < num_sims; i++) {
            // Generate random terrain
            TerrainParams terrain = GenerateRandomTerrain(ranges, rng);
            
            // Run simulation
            auto measurements = RunVehicleSimulation(
                terrain, i, sim_duration, 3e-3, mesh_spacing, false
            );
            
            // Write results
            if (!measurements.empty()) {
                WriteMeasurementsToCSV(csv_file, csv_mutex, measurements);
                total_samples += measurements.size();
            }
            
            // Progress
            int done = ++completed;
            if (done % 10 == 0 || done == num_sims) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(now - start_time).count();
                double rate = done / elapsed;
                double remaining = (num_sims - done) / rate;
                
                #pragma omp critical
                {
                    std::cout << "\rProgress: " << done << "/" << num_sims 
                              << " (" << int(100.0 * done / num_sims) << "%)"
                              << " | Samples: " << total_samples.load()
                              << " | ETA: " << int(remaining) << "s" << std::flush;
                }
            }
        }
    }
#else
    // Serial fallback
    std::mt19937 rng(seed);
    
    for (int i = 0; i < num_sims; i++) {
        TerrainParams terrain = GenerateRandomTerrain(ranges, rng);
        auto measurements = RunVehicleSimulation(terrain, i, sim_duration, 3e-3, mesh_spacing, verbose);
        
        if (!measurements.empty()) {
            WriteMeasurementsToCSV(csv_file, csv_mutex, measurements);
            total_samples += measurements.size();
        }
        
        completed++;
        if (verbose && (i % 10 == 0)) {
            std::cout << "Progress: " << i << "/" << num_sims << std::endl;
        }
    }
#endif
    
    csv_file.close();
    
    auto end_time = std::chrono::steady_clock::now();
    auto total_time = std::chrono::duration<double>(end_time - start_time).count();
    
    std::cout << std::endl;
    std::cout << "==================================================================" << std::endl;
    std::cout << "Collection complete!" << std::endl;
    std::cout << "  Total simulations: " << num_sims << std::endl;
    std::cout << "  Total tire samples: " << total_samples.load() << std::endl;
    std::cout << "  Time: " << total_time << "s" << std::endl;
    std::cout << "  Rate: " << num_sims / total_time << " sims/s" << std::endl;
    std::cout << "  Output: " << output_file << std::endl;
    std::cout << "==================================================================" << std::endl;
    
    return 0;
}
