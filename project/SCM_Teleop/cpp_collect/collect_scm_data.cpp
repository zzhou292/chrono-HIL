// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// SCM Terrain Data Collection for Neural Network Training
// Collects tire force data from SCM terrain using ChTireTestRig
// Based on Dallas et al. "Terrain Adaptive Trajectory Planning"
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/functions/ChFunctionSine.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Latin Hypercube Sampling implementation
// =============================================================================
class LatinHypercubeSampler {
public:
    LatinHypercubeSampler(int n_samples, int n_dims, unsigned seed = 42)
        : n_samples(n_samples), n_dims(n_dims), gen(seed) {}

    std::vector<std::vector<double>> Sample() {
        std::vector<std::vector<double>> samples(n_samples, std::vector<double>(n_dims));
        
        // Generate permutations for each dimension
        for (int dim = 0; dim < n_dims; dim++) {
            std::vector<int> perm(n_samples);
            for (int i = 0; i < n_samples; i++) perm[i] = i;
            std::shuffle(perm.begin(), perm.end(), gen);
            
            // Generate random samples within each cell
            std::uniform_real_distribution<> dis(0.0, 1.0);
            for (int i = 0; i < n_samples; i++) {
                samples[i][dim] = (perm[i] + dis(gen)) / n_samples;
            }
        }
        
        return samples;
    }

private:
    int n_samples;
    int n_dims;
    std::mt19937 gen;
};

// Scale LHS samples from [0,1] to parameter ranges
struct WheelTestParams {
    double vertical_load;      // N
    double slip_angle;         // rad
    double longitudinal_slip;  // ratio
    double camber_angle;       // rad
    double velocity;           // m/s
    
    // Soil parameters
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;
    double janosi_shear;
};

WheelTestParams ScaleSample(const std::vector<double>& lhs_sample) {
    WheelTestParams params;
    
    // Full LHS range for ALL parameters (same for visualization and data collection)
    params.vertical_load = 2500.0 + lhs_sample[0] * 5000.0;    // 2500-7500 N
    params.slip_angle = -0.6 + lhs_sample[1] * 1.2;            // -0.6 to 0.6 rad (Dallas et al.)
    params.longitudinal_slip = -0.12 + lhs_sample[2] * 0.24;   // -0.12 to 0.12
    params.camber_angle = -0.087 + lhs_sample[3] * 0.174;      // -5 to 5 deg
    params.velocity = 0.5 + lhs_sample[4] * 10;               // 0.5-10.5 m/s (limited to test stability)
    
    // Soil parameters - EXPANDED ranges for clay/sand/dirt coverage
    // Clay: Kphi=692k, Kc=13.2k, n=0.5, phi=13, c=4140
    // Sand: Kphi=2.1M, Kc=500, n=1.38, phi=30
    // Dirt: Kphi=3.5M, Kc=2k, n=1.1, phi=32
    params.bekker_Kphi = 0.5e6 + lhs_sample[5] * 3.5e6;        // 0.5-4 MPa (covers clay 692k)
    params.bekker_Kc = 0.0 + lhs_sample[6] * 20000.0;          // 0-20 kPa (covers clay 13.2k)
    params.bekker_n = 0.3 + lhs_sample[7] * 1.2;               // 0.3-1.5 (covers sand 1.38)
    params.mohr_cohesion = 0.0 + lhs_sample[8] * 10000.0;      // 0-10 kPa (covers clay 4140)
    params.mohr_friction = 10.0 + lhs_sample[9] * 35.0;        // 10-45 deg (covers clay 13 deg)
    params.janosi_shear = 0.005 + lhs_sample[10] * 0.055;      // 0.005-0.06 m

    return params;
}

// =============================================================================
// Main data collection function
// =============================================================================
void CollectSCMData(int n_samples, const std::string& output_file, bool visualize = false) {
    std::cout << "\n=== SCM Tire Force Data Collection ===" << std::endl;
    std::cout << "Samples: " << n_samples << std::endl;
    std::cout << "Output: " << output_file << std::endl;
    std::cout << "Visualization: " << (visualize ? "Enabled (RIGID terrain demo)" : "Disabled (SCM terrain)") << std::endl;
    
    // Generate LHS samples (11 parameters)
    LatinHypercubeSampler sampler(n_samples, 11);
    auto lhs_samples = sampler.Sample();
    
    // Open output CSV file
    std::ofstream csv_file(output_file);
    // NN training uses column Fz (measured) as load input, not vertical_load (commanded). See train_terrain_nn.py.
    csv_file << "vertical_load,slip_angle,longitudinal_slip,camber_angle,velocity,"
             << "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,mohr_friction,janosi_shear,"
             << "Fz,Fx,Fy\n";  // Fz = measured normal force (ReportTireForce); use this for NN input
    csv_file << std::fixed << std::setprecision(6);
    
    // Process each sample
    for (int sample_idx = 0; sample_idx < n_samples; sample_idx++) {
        // LHS sampled params (same for visualization and data collection)
        auto params = ScaleSample(lhs_samples[sample_idx]);
        
        // Create Chrono system for this sample (like demo)
        ChSystemNSC sys;
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        
        // Set solver settings (like demo)
        double step_size = 2e-4;
        sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        sys.GetSolver()->AsIterative()->SetMaxIterations(100);
        
        // Create wheel and tire from JSON files
        std::string data_path = GetChronoDataPath();
        std::shared_ptr<ChWheel> wheel;
        std::shared_ptr<ChTire> tire;
        
        // Use RigidTire for SCM terrain (required - TMeasy doesn't work with SCM)
        wheel = ReadWheelJSON(data_path + "vehicle/hmmwv/wheel/HMMWV_Wheel.json");
        tire = ReadTireJSON(data_path + "vehicle/hmmwv/tire/HMMWV_RigidTire.json");
        
        // Set tire stepsize BEFORE creating rig (like demo)
        tire->SetStepsize(step_size);
        
        // Create tire test rig
        ChTireTestRig rig(wheel, tire, &sys);
        
        // Set basic parameters
        rig.SetGravitationalAcceleration(9.8);
        rig.SetNormalLoad(params.vertical_load);
        rig.SetCamberAngle(params.camber_angle);
        
        // Set tire parameters
        rig.SetTireStepsize(step_size);
        rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
        rig.SetTireVisualizationType(VisualizationType::PRIMITIVES);  // Use PRIMITIVES to avoid mesh loading issues
        
        // Set up terrain and motion - SAME for both visualization and data collection
        ChTireTestRig::TerrainParamsSCM scm_params;
        scm_params.length = 200.0;
        scm_params.width = 1.0;
        scm_params.Bekker_Kphi = params.bekker_Kphi;
        scm_params.Bekker_Kc = params.bekker_Kc;
        scm_params.Bekker_n = params.bekker_n;
        scm_params.Mohr_cohesion = params.mohr_cohesion;
        scm_params.Mohr_friction = params.mohr_friction;
        scm_params.Janosi_shear = params.janosi_shear;
        scm_params.grid_spacing = visualize ? 0.04 : 0.05;  // finer grid for visualization
        rig.SetTerrainSCM(scm_params);
        
        // LHS motion parameters: velocity and angular speed
        double base_speed = params.velocity;
        double tire_radius = 0.47;  // HMMWV wheel radius (must match HMMWV_Wheel.json and demo vehicle)
        // Angular velocity to achieve target slip ratio: omega = v/r * (1 + slip)
        double ang_speed = (base_speed / tire_radius) * (1.0 + params.longitudinal_slip);
        
        // Sinusoidal slip angle: starts at 0, reaches |target| at t=T/4 
        // Frequency chosen so we measure at t=2s (after 1s time delay)
        double slip_freq = 0.25;  // 0.25 Hz = 4s period, so T/4 = 1s after delay
        double slip_amp = std::abs(params.slip_angle);  // amplitude = |target|
        // doing something fun and stupid just once:
        // double slip_amp = std::abs(40.0 * (3.14159 / 180.0));  // 30 deg slip angle for visualization demo
        auto slip_func = chrono_types::make_shared<ChFunctionSine>(slip_amp, slip_freq);
        
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(base_speed));
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(ang_speed));
        rig.SetSlipAngleFunction(slip_func);
        rig.SetTimeDelay(1.0);
        rig.Initialize(ChTireTestRig::Mode::TEST);
        
#ifdef CHRONO_IRRLICHT
        // Create visualization system if requested
        std::shared_ptr<ChVisualSystemIrrlicht> vis;
        if (visualize) {
            vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis->AttachSystem(&sys);
            vis->SetCameraVertical(CameraVerticalDir::Z);
            vis->SetWindowSize(1200, 600);
            vis->SetWindowTitle("SCM Data Collection - Sample " + std::to_string(sample_idx + 1));
            vis->Initialize();
            vis->AddLogo();
            vis->AddSkyBox();
            vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis->AddLightDirectional();
        }
#endif
        
        // Run simulation to steady state
        // For data collection with sinusoidal slip: measure at peak
        // - freq = 0.25 Hz, T = 4s, so T/4 = 1s
        // - With 1s delay: positive peak at t=2s, negative peak at t=4s
        double t_end = visualize ? 10.0 : 4.0;  // Need 4s for possible negative slip peak
        
        // Determine when to measure based on slip angle sign
        // t_delay = 1.0, T/4 = 1.0s, so positive peak at t=2.0, negative at t=4.0
        double t_measure = 2.0;  // default: positive peak
        if (!visualize && params.slip_angle < 0) {
            t_measure = 4.0;  // negative peak for negative slip targets
        }
        
        double t = 0;
        double render_step = 1.0 / 60.0;  // 60 FPS
        double next_render = 0;
        
        // Variables to store forces at measurement time
        TerrainForce measured_force;
        bool force_measured = false;
        
        while (t < t_end) {
#ifdef CHRONO_IRRLICHT
            if (visualize && vis && t >= next_render) {
                auto& loc = rig.GetPos();
                // Camera follows the tire rig position
                vis->UpdateCamera(loc + ChVector3d(1.5, 3.0, 1.0), loc);
                if (!vis->Run())
                    break;
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                next_render += render_step;
            }
#endif
            rig.Advance(step_size);
            t += step_size;
            
            // Capture forces at measurement time (when slip angle = target)
            if (!visualize && !force_measured && t >= t_measure) {
                measured_force = rig.ReportTireForce();
                force_measured = true;
            }
        }
        
        // Get tire forces - use measured_force for data collection, or final force for viz
        TerrainForce tire_force;
        if (!visualize && force_measured) {
            tire_force = measured_force;
        } else {
            tire_force = rig.ReportTireForce();
        }
        double Fz = tire_force.force.z();  // MEASURED normal force (use this as NN input, not commanded load)
        double Fx = tire_force.force.x();  // Longitudinal force
        double Fy = tire_force.force.y();  // Lateral force
        
        // Debug output for first few samples
        if (sample_idx < 3) {
            std::cout << "  Sample " << sample_idx << ": Fz=" << Fz 
                      << " N (cmd=" << params.vertical_load << "), Fx=" << Fx << " N, Fy=" << Fy << " N" << std::endl;
        }
        
        // Write to CSV (vertical_load is commanded, Fz is measured - NN should use Fz)
        csv_file << params.vertical_load << ","
                 << params.slip_angle << ","
                 << params.longitudinal_slip << ","
                 << params.camber_angle << ","
                 << params.velocity << ","
                 << params.bekker_Kphi << ","
                 << params.bekker_Kc << ","
                 << params.bekker_n << ","
                 << params.mohr_cohesion << ","
                 << params.mohr_friction << ","
                 << params.janosi_shear << ","
                 << Fz << ","
                 << Fx << ","
                 << Fy << "\n";
        
        // Progress update
        if ((sample_idx + 1) % 10 == 0 || sample_idx == n_samples - 1) {
            std::cout << "Progress: " << (sample_idx + 1) << "/" << n_samples 
                      << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (sample_idx + 1) / n_samples) << "%)" << std::endl;
        }
    }
    
    csv_file.close();
    std::cout << "\nData collection complete!" << std::endl;
    std::cout << "Output saved to: " << output_file << std::endl;
}

// =============================================================================
int main(int argc, char* argv[]) {
    // Parse command line arguments
    int n_samples = 100;
    std::string output_file = "scm_training_data.csv";
    bool visualize = false;
    
    // Simple argument parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--visualize" || arg == "-v") {
            visualize = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [num_samples] [output.csv] [--visualize]" << std::endl;
            std::cout << "  num_samples: Number of samples to collect (default: 100)" << std::endl;
            std::cout << "  output.csv:  Output CSV filename (default: scm_training_data.csv)" << std::endl;
            std::cout << "  --visualize: Enable Irrlicht visualization" << std::endl;
            return 0;
        } else if (i == 1 && arg[0] >= '0' && arg[0] <= '9') {
            n_samples = std::atoi(argv[i]);
        } else if (i == 2 || (i == 1 && arg.find(".csv") != std::string::npos)) {
            output_file = arg;
        }
    }
    
    // Set Chrono data path
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    std::cout << "=== SCM Data Collection with ChTireTestRig ===" << std::endl;
    std::cout << "Collection " << n_samples << " samples..." << std::endl;
    
#ifndef CHRONO_IRRLICHT
    if (visualize) {
        std::cerr << "Warning: Irrlicht not available. Visualization disabled." << std::endl;
        visualize = false;
    }
#endif
    
    try {
        CollectSCMData(n_samples, output_file, visualize);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
