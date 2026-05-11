// =============================================================================
// collect_static_data.cpp
// Static SCM tire force data collection for neural network training.
//
// Key difference from collect_temporal_data:
//   ALL parameters (terrain + operating conditions) are independently sampled
//   via Latin Hypercube Sampling. Each sample runs a brief run with constant
//   speed/load/slip-ratio and linear slip-angle profile alpha(t), then records
//   average forces near the instant alpha reaches the target slip angle.
//
// This produces datasets where every terrain config is paired with diverse
// operating conditions, preventing the model from learning terrain-only lookup
// tables.
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionPoly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_OPENMP
#include <omp.h>
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>
#include <atomic>
#include <mutex>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <string>

#ifdef __linux__
#include <sys/wait.h>
#include <malloc.h>
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Simulation parameters
// =============================================================================
constexpr double STEP_SIZE = 5e-4;
constexpr double T_MEASURE = 0.05;        // averaging window duration
constexpr double T_SETTLE_MIN = 1.0;      // enforce measurement starts after initial 0.5s settling
constexpr double T_RAMP_MIN = 0.5;        // minimum effective ramp time after delay
constexpr double T_TARGET_MIN = T_SETTLE_MIN + T_RAMP_MIN + 0.5 * T_MEASURE;
constexpr double T_TARGET_MAX = 3.0;      // cap per-sample target time to keep runtime bounded
constexpr double T_TAIL = 0.05;           // short post-measurement tail
constexpr double T_END_MIN = T_TARGET_MIN + 0.5 * T_MEASURE + T_TAIL;
constexpr double T_END_MAX = T_TARGET_MAX + 0.5 * T_MEASURE + T_TAIL;
constexpr double MEASURE_DT = 0.005;      // force sample interval during measurement
constexpr double ALPHA_LIMIT = 0.6;       // diagnostic warning threshold in linear-only mode
constexpr int SOLVER_ITERS = 50;

// =============================================================================
// Parameter ranges for LHS sampling
// =============================================================================
struct ParameterRanges {
    // Operating conditions — matches TRAINING_RANGES_V6 in param_consistency.py
    double slip_ratio_min = -1.0, slip_ratio_max = 1.0;
    double slip_angle_min = -0.6, slip_angle_max = 0.6;  // radians
    double velocity_min = 2.0, velocity_max = 10.0;
    double vertical_load_min = 2500.0, vertical_load_max = 7500.0;
    double steering_rate_min = -0.56, steering_rate_max = 0.56;  // rad/s

    // Terrain — matches TRAINING_RANGES_V6
    double bekker_Kphi_min = 0.5e6, bekker_Kphi_max = 4.0e6;
    double bekker_Kc_min = 0.0, bekker_Kc_max = 20000.0;
    double bekker_n_min = 0.3, bekker_n_max = 1.3;
    double mohr_cohesion_min = 650.0, mohr_cohesion_max = 20700.0;
    double mohr_friction_min = 6.0, mohr_friction_max = 37.8;   // degrees (V6: 0.105-0.66 rad)
    double janosi_shear_min = 0.01, janosi_shear_max = 0.025;
    double mesh_spacing_min = 0.08, mesh_spacing_max = 0.12;
};

// =============================================================================
// Sample parameters (one per simulation)
// =============================================================================
struct StaticSampleParams {
    // Operating conditions
    double slip_ratio;
    double slip_angle;
    double velocity;
    double vertical_load;
    double steering_rate;   // rad/s (applied as d(alpha)/dt in slip-angle profile)
    double target_time;     // s (internal): time where alpha(t) reaches slip_angle
    // Terrain
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;   // degrees
    double janosi_shear;
    double mesh_spacing;
};

struct OperatingPointSample {
    double slip_ratio;
    double slip_angle;
    double velocity;
    double vertical_load;
    double target_time;
};

struct TerrainPointSample {
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;
    double janosi_shear;
    double mesh_spacing;
};

// =============================================================================
// Result from one simulation
// =============================================================================
struct StaticResult {
    double Fz_avg = 0, Fx_avg = 0, Fy_avg = 0;
    double slip_angle_eff = 0;     // commanded alpha at measurement time (from delay-aware analytical model)
    double steering_rate_eff = 0;  // commanded d(alpha)/dt at measurement time
    double max_abs_alpha = 0;      // max |alpha(t)| over [0, t_end]
    double max_abs_rate = 0;       // max |dalpha/dt| over [0, t_end]
    double scale_factor = 1.0;     // reserved for compatibility (always 1.0 in linear-only mode)
    double target_time = 0;        // per-sample target crossing time
    int n_samples = 0;
    bool valid = false;
    // Actual rig-measured state (for verification and corrected CSV output)
    double actual_kappa = 0;       // tire-computed longitudinal slip
    double actual_alpha = 0;       // tire-computed slip angle
    double actual_velocity = 0;    // carrier longitudinal speed
    double actual_steering_rate = 0; // finite-diff of actual alpha across window
};

// =============================================================================
// Latin Hypercube Sampling over all independently sampled parameters
// =============================================================================
std::vector<double> GenerateLHSDimension(int n, double lo, double hi, std::mt19937& rng) {
    std::vector<double> v(n);
    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);
    std::shuffle(perm.begin(), perm.end(), rng);
    std::uniform_real_distribution<> u(0.0, 1.0);
    for (int i = 0; i < n; i++) {
        v[i] = lo + ((perm[i] + u(rng)) / n) * (hi - lo);
    }
    return v;
}

void FinalizeSampleKinematics(StaticSampleParams& s, const ParameterRanges& r) {
    double t_target = std::clamp(s.target_time, T_TARGET_MIN, T_TARGET_MAX);
    if (r.steering_rate_max > 1e-8) {
        double t_ramp_min = std::abs(s.slip_angle) / r.steering_rate_max;
        double t_min_sr = t_ramp_min + T_SETTLE_MIN;
        t_target = std::max(t_target, t_min_sr);
    }
    t_target = std::clamp(t_target, T_TARGET_MIN, T_TARGET_MAX);
    s.target_time = t_target;
    double t_ramp = s.target_time - T_SETTLE_MIN;
    s.steering_rate = (std::abs(s.slip_angle) < 1e-8 || t_ramp < 1e-8) ? 0.0 : (s.slip_angle / t_ramp);
}

std::vector<StaticSampleParams> GenerateLHSSamples(int n, const ParameterRanges& r) {
    constexpr int NDIM = 12;  // steering_rate is derived from (slip_angle, target_time)
    std::mt19937 rng(42);

    // Create LHS grid: n intervals per dimension, shuffle each dimension
    std::vector<std::vector<double>> lhs(NDIM, std::vector<double>(n));
    for (int d = 0; d < NDIM; d++) {
        std::vector<int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);
        std::shuffle(perm.begin(), perm.end(), rng);
        std::uniform_real_distribution<> u(0.0, 1.0);
        for (int i = 0; i < n; i++) {
            lhs[d][i] = (perm[i] + u(rng)) / n;  // stratified uniform in [0,1]
        }
    }

    // Map to parameter ranges
    auto lerp = [](double t, double a, double b) { return a + t * (b - a); };

    std::vector<StaticSampleParams> samples(n);
    for (int i = 0; i < n; i++) {
        auto& s = samples[i];
        s.slip_ratio    = lerp(lhs[0][i], r.slip_ratio_min, r.slip_ratio_max);
        s.slip_angle    = lerp(lhs[1][i], r.slip_angle_min, r.slip_angle_max);
        s.velocity      = lerp(lhs[2][i], r.velocity_min, r.velocity_max);
        s.vertical_load = lerp(lhs[3][i], r.vertical_load_min, r.vertical_load_max);
        // Linear-only consistency: sample target time, derive steering rate.
        // NOTE: ChTireTestRig wraps all motion functions with DelayedFun, which
        // evaluates f(t - delay) instead of f(t). The polynomial f(x) = c1*x
        // becomes c1*(t - T_SETTLE_MIN) at sim time t. The effective ramp time
        // is therefore (t_target - T_SETTLE_MIN), not t_target.
        s.target_time = lerp(lhs[4][i], T_TARGET_MIN, T_TARGET_MAX);
        s.bekker_Kphi   = lerp(lhs[5][i], r.bekker_Kphi_min, r.bekker_Kphi_max);
        s.bekker_Kc     = lerp(lhs[6][i], r.bekker_Kc_min, r.bekker_Kc_max);
        s.bekker_n      = lerp(lhs[7][i], r.bekker_n_min, r.bekker_n_max);
        s.mohr_cohesion = lerp(lhs[8][i], r.mohr_cohesion_min, r.mohr_cohesion_max);
        s.mohr_friction = lerp(lhs[9][i], r.mohr_friction_min, r.mohr_friction_max);
        s.janosi_shear  = lerp(lhs[10][i], r.janosi_shear_min, r.janosi_shear_max);
        s.mesh_spacing  = lerp(lhs[11][i], r.mesh_spacing_min, r.mesh_spacing_max);
        FinalizeSampleKinematics(s, r);
    }
    return samples;
}

std::vector<StaticSampleParams> GenerateFactoredSamples(int n, const ParameterRanges& r, int terrain_bank_size) {
    std::mt19937 rng(42);
    int n_terrain = std::max(1, std::min(terrain_bank_size, n));
    int n_ops = std::max(1, (n + n_terrain - 1) / n_terrain);

    auto slip_ratios = GenerateLHSDimension(n_ops, r.slip_ratio_min, r.slip_ratio_max, rng);
    auto slip_angles = GenerateLHSDimension(n_ops, r.slip_angle_min, r.slip_angle_max, rng);
    auto velocities = GenerateLHSDimension(n_ops, r.velocity_min, r.velocity_max, rng);
    auto vertical_loads = GenerateLHSDimension(n_ops, r.vertical_load_min, r.vertical_load_max, rng);
    auto target_times = GenerateLHSDimension(n_ops, T_TARGET_MIN, T_TARGET_MAX, rng);

    auto bk_Kphis = GenerateLHSDimension(n_terrain, r.bekker_Kphi_min, r.bekker_Kphi_max, rng);
    auto bk_Kcs = GenerateLHSDimension(n_terrain, r.bekker_Kc_min, r.bekker_Kc_max, rng);
    auto bk_ns = GenerateLHSDimension(n_terrain, r.bekker_n_min, r.bekker_n_max, rng);
    auto mc_cohesions = GenerateLHSDimension(n_terrain, r.mohr_cohesion_min, r.mohr_cohesion_max, rng);
    auto mc_frictions = GenerateLHSDimension(n_terrain, r.mohr_friction_min, r.mohr_friction_max, rng);
    auto j_shears = GenerateLHSDimension(n_terrain, r.janosi_shear_min, r.janosi_shear_max, rng);
    auto meshes = GenerateLHSDimension(n_terrain, r.mesh_spacing_min, r.mesh_spacing_max, rng);

    std::vector<OperatingPointSample> ops(n_ops);
    for (int i = 0; i < n_ops; i++) {
        ops[i] = OperatingPointSample{
            slip_ratios[i], slip_angles[i], velocities[i], vertical_loads[i], target_times[i],
        };
    }
    std::vector<TerrainPointSample> terrains(n_terrain);
    for (int i = 0; i < n_terrain; i++) {
        terrains[i] = TerrainPointSample{
            bk_Kphis[i], bk_Kcs[i], bk_ns[i], mc_cohesions[i],
            mc_frictions[i], j_shears[i], meshes[i],
        };
    }

    std::vector<int> op_perm(n_ops);
    std::vector<int> terrain_perm(n_terrain);
    std::iota(op_perm.begin(), op_perm.end(), 0);
    std::iota(terrain_perm.begin(), terrain_perm.end(), 0);
    std::shuffle(op_perm.begin(), op_perm.end(), rng);
    std::shuffle(terrain_perm.begin(), terrain_perm.end(), rng);

    std::vector<StaticSampleParams> samples;
    samples.reserve(n);
    for (int o = 0; o < n_ops && static_cast<int>(samples.size()) < n; o++) {
        int terrain_offset = (o * 7) % n_terrain;
        const auto& op = ops[op_perm[o]];
        for (int j = 0; j < n_terrain && static_cast<int>(samples.size()) < n; j++) {
            const auto& terrain = terrains[terrain_perm[(j + terrain_offset) % n_terrain]];
            StaticSampleParams s{};
            s.slip_ratio = op.slip_ratio;
            s.slip_angle = op.slip_angle;
            s.velocity = op.velocity;
            s.vertical_load = op.vertical_load;
            s.target_time = op.target_time;
            s.bekker_Kphi = terrain.bekker_Kphi;
            s.bekker_Kc = terrain.bekker_Kc;
            s.bekker_n = terrain.bekker_n;
            s.mohr_cohesion = terrain.mohr_cohesion;
            s.mohr_friction = terrain.mohr_friction;
            s.janosi_shear = terrain.janosi_shear;
            s.mesh_spacing = terrain.mesh_spacing;
            FinalizeSampleKinematics(s, r);
            samples.push_back(s);
        }
    }

    return samples;
}

// =============================================================================
// Run one static simulation, return averaged forces
// =============================================================================
StaticResult CollectStaticSample(const StaticSampleParams& params, int sample_id, bool visualize = false) {
    StaticResult result;
    constexpr double tire_radius = 0.47;

    try {
        auto sys = std::make_unique<ChSystemNSC>();
        sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        sys->GetSolver()->AsIterative()->SetMaxIterations(SOLVER_ITERS);

        std::string data_path = GetChronoDataPath();
        auto wheel = ReadWheelJSON(data_path + "vehicle/hmmwv/wheel/HMMWV_Wheel.json");
        auto tire  = ReadTireJSON(data_path + "vehicle/hmmwv/tire/HMMWV_RigidTire.json");
        tire->SetStepsize(STEP_SIZE);

        ChTireTestRig rig(wheel, tire, sys.get());
        rig.SetGravitationalAcceleration(9.8);
        rig.SetNormalLoad(params.vertical_load);
        rig.SetCamberAngle(0.0);
        rig.SetTireStepsize(STEP_SIZE);
        rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
        rig.SetTireVisualizationType(VisualizationType::PRIMITIVES);

        // SCM terrain
        ChTireTestRig::TerrainParamsSCM scm;
        scm.length        = 200.0;
        scm.width         = 1.0;
        scm.Bekker_Kphi   = params.bekker_Kphi;
        scm.Bekker_Kc     = params.bekker_Kc;
        scm.Bekker_n      = params.bekker_n;
        scm.Mohr_cohesion = params.mohr_cohesion;
        scm.Mohr_friction = params.mohr_friction;
        scm.Janosi_shear  = params.janosi_shear;
        scm.grid_spacing  = params.mesh_spacing;
        rig.SetTerrainSCM(scm);

        // Constant operating conditions
        double ang_speed = (params.velocity / tire_radius) * (1.0 + params.slip_ratio);

        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(params.velocity));
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(ang_speed));
        // Linear-only protocol:
        //   The polynomial we pass is f(x) = sr_cmd * x.
        //   ChTireTestRig wraps this with DelayedFun, so the tire actually sees:
        //     alpha(t) = sr_cmd * max(0, t - T_SETTLE_MIN)
        //   To get alpha(t_target) == requested slip_angle, we need:
        //     sr_cmd = slip_angle / (t_target - T_SETTLE_MIN)
        double t_target = std::clamp(params.target_time, T_TARGET_MIN, T_TARGET_MAX);
        double t_ramp = t_target - T_SETTLE_MIN;  // effective ramp time after delay
        double sr_cmd = (std::abs(params.slip_angle) < 1e-8 || t_ramp < 1e-8) ? 0.0 : (params.slip_angle / t_ramp);

        double c0 = 0.0;
        double c1 = sr_cmd;
        // alpha_eval models what the tire actually sees (delay-shifted)
        auto alpha_eval = [&](double t_s) { return (t_s < T_SETTLE_MIN) ? 0.0 : c1 * (t_s - T_SETTLE_MIN); };
        auto rate_eval = [&](double t_s) { return (t_s < T_SETTLE_MIN) ? 0.0 : c1; };
        const double t_measure_start = t_target - 0.5 * T_MEASURE;
        const double t_measure_end = t_target + 0.5 * T_MEASURE;
        const double t_end = t_measure_end + T_TAIL;

        double max_alpha = 0.0;
        double max_rate = 0.0;
        for (int k = 0; k <= 40; k++) {
            double t_s = k * t_end / 40.0;
            max_alpha = std::max(max_alpha, std::abs(alpha_eval(t_s)));
            max_rate = std::max(max_rate, std::abs(rate_eval(t_s)));
        }
        auto slip_func = chrono_types::make_shared<ChFunctionPoly>();
        std::vector<double> slip_coeffs = {c0, c1};
        slip_func->SetCoefficients(slip_coeffs);
        rig.SetSlipAngleFunction(slip_func);
        rig.SetTimeDelay(T_SETTLE_MIN);
        rig.Initialize(ChTireTestRig::Mode::TEST);

#ifdef CHRONO_IRRLICHT
        std::shared_ptr<ChVisualSystemIrrlicht> vis;
        if (visualize) {
            vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis->AttachSystem(sys.get());
            vis->SetCameraVertical(CameraVerticalDir::Z);
            vis->SetWindowSize(1200, 600);
            vis->SetWindowTitle("Static Data - Sample " + std::to_string(sample_id));
            vis->Initialize();
            vis->AddLogo();
            vis->AddSkyBox();
            vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis->AddLightDirectional();
        }
#endif

        // Simulate
        double t = 0;
        double t_next_measure = t_measure_start;
        double sum_Fx = 0, sum_Fy = 0, sum_Fz = 0;
        double sum_kappa = 0, sum_alpha = 0, sum_v = 0;
        double first_alpha = 0, last_alpha = 0;
        double first_alpha_t = 0, last_alpha_t = 0;
        int count = 0;
        double render_step = 1.0 / 60.0;   // 60 FPS
        double next_render = 0.0;
        double t_vis_end = t_end;  // do not extend visual run; linear alpha(t) would keep growing

#ifndef CHRONO_IRRLICHT
        (void)visualize;
#endif

        while (t < t_vis_end) {
#ifdef CHRONO_IRRLICHT
            if (visualize && vis && t >= next_render) {
                auto& loc = rig.GetPos();
                vis->UpdateCamera(loc + ChVector3d(1.5, 3.0, 1.0), loc);
                if (!vis->Run())
                    break;
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                next_render += render_step;
            }
#endif
            rig.Advance(STEP_SIZE);
            t += STEP_SIZE;

            // Record forces and actual state in a short window around the target-angle crossing.
            if (t <= t_end && t >= t_next_measure && t >= t_measure_start && t <= t_measure_end) {
                auto force = rig.ReportTireForce();
                sum_Fx += force.force.x();
                sum_Fy += force.force.y();
                sum_Fz += force.force.z();
                double cur_kappa = rig.GetLongitudinalSlip();
                double cur_alpha = rig.GetSlipAngle();
                double cur_v     = rig.GetLongSpeed();
                sum_kappa += cur_kappa;
                sum_alpha += cur_alpha;
                sum_v     += cur_v;
                if (count == 0) { first_alpha = cur_alpha; first_alpha_t = t; }
                last_alpha = cur_alpha; last_alpha_t = t;
                count++;
                t_next_measure += MEASURE_DT;
            }
        }

        if (count > 0) {
            result.Fx_avg = sum_Fx / count;
            result.Fy_avg = sum_Fy / count;
            result.Fz_avg = sum_Fz / count;
            result.slip_angle_eff = alpha_eval(t_target);
            result.steering_rate_eff = rate_eval(t_target);
            result.max_abs_alpha = max_alpha;
            result.max_abs_rate = max_rate;
            result.scale_factor = 1.0;
            result.target_time = t_target;
            result.n_samples = count;
            result.actual_kappa = sum_kappa / count;
            result.actual_alpha = sum_alpha / count;
            result.actual_velocity = sum_v / count;
            double dt_window = last_alpha_t - first_alpha_t;
            result.actual_steering_rate = (dt_window > 1e-8) ? (last_alpha - first_alpha) / dt_window : 0.0;
            result.valid = true;
        }

        sys->Clear();
        sys.reset();
#ifdef __linux__
        malloc_trim(0);
#endif
    } catch (const std::exception& e) {
        std::cerr << "Sample " << sample_id << " failed: " << e.what() << std::endl;
    }
    return result;
}

// =============================================================================
// Write one result row to CSV (thread-safe)
// =============================================================================
void WriteStaticRecord(std::ofstream& csv, std::mutex& mtx,
                       const StaticSampleParams& params,
                       const StaticResult& result) {
    if (!result.valid) return;

    double mohr_friction_rad = params.mohr_friction * CH_DEG_TO_RAD;

    std::ostringstream buf;
    buf << std::fixed << std::setprecision(6);

    buf << result.actual_kappa << ","
        << result.actual_alpha << ","
        << result.actual_velocity << ","
        << params.vertical_load << ","
        << result.actual_steering_rate << ","
        << params.bekker_Kphi << ","
        << params.bekker_Kc << ","
        << params.bekker_n << ","
        << params.mohr_cohesion << ","
        << mohr_friction_rad << ","
        << params.janosi_shear << ","
        << params.mesh_spacing << ","
        << result.Fz_avg << ","
        << result.Fx_avg << ","
        << result.Fy_avg << "\n";

    std::lock_guard<std::mutex> lock(mtx);
    csv << buf.str();
    csv.flush();
}

// =============================================================================
static const std::string CSV_HEADER =
    "slip_ratio,slip_angle,velocity,vertical_load,"
    "steering_rate,bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,"
    "mohr_friction,janosi_shear,mesh_spacing,Fz,Fx,Fy\n";

// =============================================================================
// Process a batch of samples (for subprocess batching)
// =============================================================================
int ProcessBatch(const std::vector<StaticSampleParams>& samples,
                 int base_id,
                 const std::string& output_file,
                 int num_threads, bool use_parallel, bool append_mode)
{
#ifdef CHRONO_OPENMP
    if (use_parallel && num_threads > 0) omp_set_num_threads(num_threads);
#endif

    std::ofstream csv;
    if (append_mode) {
        csv.open(output_file, std::ios::app);
    } else {
        csv.open(output_file);
        csv << CSV_HEADER;
    }
    if (!csv.is_open()) {
        std::cerr << "Failed to open: " << output_file << std::endl;
        return 0;
    }

    std::mutex csv_mtx;
    std::atomic<int> success{0};
    int n = static_cast<int>(samples.size());

    // Warm-up (single-threaded first sample)
    if (n > 0) {
        auto res = CollectStaticSample(samples[0], base_id);
        if (res.valid) {
            WriteStaticRecord(csv, csv_mtx, samples[0], res);
            success++;
        }
    }

#ifdef CHRONO_OPENMP
    #pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 1; i < n; i++) {
        auto res = CollectStaticSample(samples[i], base_id + i);
        if (res.valid) {
            WriteStaticRecord(csv, csv_mtx, samples[i], res);
            success++;
        }
    }

    csv.close();
    return success.load();
}

// =============================================================================
volatile sig_atomic_t g_stop = 0;
void signal_handler(int) { g_stop = 1; }

// =============================================================================
// Subprocess-batched collection (Linux only)
// =============================================================================
#ifdef __linux__
void CollectWithSubprocessBatching(int n_samples, const std::string& output_file,
                                   int num_threads, bool use_parallel, int batch_size,
                                   bool use_factored_sampling, int terrain_bank_size)
{
    std::cout << "\n=== Subprocess-Batched Static Data Collection ===\n"
              << "Total samples: " << n_samples << "\n"
              << "Batch size: " << batch_size << "\n"
              << "Linear-only profile alpha(t)=sr*t with alpha(0)=0\n"
              << "Target crossing time (per sample): [" << T_TARGET_MIN << ", " << T_TARGET_MAX << "] s\n"
              << "Measurement window: " << T_MEASURE << "s | sim/sample in ["
              << T_END_MIN << ", " << T_END_MAX << "] s → 1 output row\n";

    // Write header
    {
        std::ofstream hdr(output_file);
        hdr << CSV_HEADER;
    }
    ParameterRanges ranges;
    auto all_samples = use_factored_sampling
        ? GenerateFactoredSamples(n_samples, ranges, terrain_bank_size)
        : GenerateLHSSamples(n_samples, ranges);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto t0 = std::chrono::high_resolution_clock::now();
    int total_ok = 0, total_done = 0;

    for (int bs = 0; bs < n_samples && !g_stop; bs += batch_size) {
        int be = std::min(bs + batch_size, n_samples);
        int cur = be - bs;

        std::cout << "\n--- Batch: samples " << bs << "-" << (be-1) << " ---\n";

        pid_t pid = fork();
        if (pid == 0) {
            SetChronoDataPath(CHRONO_DATA_DIR);
            std::vector<StaticSampleParams> batch(
                all_samples.begin() + bs, all_samples.begin() + be);
            int ok = ProcessBatch(batch, bs, output_file, num_threads, use_parallel, true);
            _exit(std::min(ok, 255));
        } else if (pid > 0) {
            int status;
            waitpid(pid, &status, 0);
            int ok = WIFEXITED(status) ? WEXITSTATUS(status) : 0;
            if (cur > 255 && ok == 255) ok = cur;
            total_ok += ok;
            total_done += cur;

            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - t0).count();
            double rate = total_done / elapsed;
            std::cout << "Batch: " << ok << "/" << cur << " ok | Progress: "
                      << total_done << "/" << n_samples
                      << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * total_done / n_samples) << "%) "
                      << std::setprecision(2) << rate << " samples/s\n";
        } else {
            std::cerr << "Fork failed!\n";
            break;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double total_t = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "\n=== Collection Complete ===\n"
              << "Time: " << std::fixed << std::setprecision(1) << total_t << "s\n"
              << "Samples: " << total_ok << "/" << total_done << " succeeded\n"
              << "Output: " << output_file << "\n";
}
#endif

// =============================================================================
// Single-process collection
// =============================================================================
void CollectStaticData(int n_samples, const std::string& output_file,
                       int num_threads, bool use_parallel, bool visualize,
                       bool use_factored_sampling, int terrain_bank_size)
{
    std::cout << "\n=== Static SCM Data Collection ===\n"
              << "Samples: " << n_samples << "\n"
              << "Step size: " << STEP_SIZE << "\n"
              << "Linear-only profile alpha(t)=sr*t with alpha(0)=0\n"
              << "Target crossing time (per sample): [" << T_TARGET_MIN << ", " << T_TARGET_MAX << "] s\n"
              << "Measurement window: " << T_MEASURE << "s\n"
              << "Visualization: " << (visualize ? "ON" : "OFF") << "\n"
              << (use_factored_sampling
                  ? "Factored sampling: separate terrain bank and operating-point bank\n"
                  : "LHS over 11 independent parameters (+ steering_rate derived from slip/time)\n");

#ifdef CHRONO_OPENMP
    if (use_parallel) {
        if (num_threads <= 0) num_threads = std::min(omp_get_max_threads() - 2, 8);
        omp_set_num_threads(num_threads);
        std::cout << "OpenMP threads: " << num_threads << "\n";
    } else {
        num_threads = 1;
        omp_set_num_threads(1);
    }
#else
    num_threads = 1;
#endif

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::ofstream csv(output_file);
    csv << CSV_HEADER;
    csv.flush();

    std::mutex csv_mtx;
    ParameterRanges ranges;
    auto samples = use_factored_sampling
        ? GenerateFactoredSamples(n_samples, ranges, terrain_bank_size)
        : GenerateLHSSamples(n_samples, ranges);
    std::atomic<int> completed{0};
    std::atomic<int> success{0};
    std::atomic<int> n_alpha_warn{0};

    auto t_start = std::chrono::high_resolution_clock::now();

    auto print_verify = [&](int sid, const StaticSampleParams& p, const StaticResult& r) {
        if (!visualize || !r.valid)
            return;
        bool alpha_warn = (r.max_abs_alpha > ALPHA_LIMIT + 1e-9);
        if (alpha_warn)
            n_alpha_warn++;
        std::cout << "[verify] sample " << sid << "\n"
                  << "  prescribed:  kappa=" << std::fixed << std::setprecision(3)
                  << p.slip_ratio << "  alpha=" << r.slip_angle_eff
                  << "  v=" << std::setprecision(1) << p.velocity
                  << "  sr=" << std::setprecision(3) << r.steering_rate_eff << "\n"
                  << "  actual(rig): kappa=" << r.actual_kappa
                  << "  alpha=" << r.actual_alpha
                  << "  v=" << std::setprecision(1) << r.actual_velocity
                  << "  sr=" << std::setprecision(3) << r.actual_steering_rate << "\n"
                  << "  error:       kappa=" << std::setprecision(4)
                  << (r.actual_kappa - p.slip_ratio)
                  << "  alpha=" << (r.actual_alpha - r.slip_angle_eff)
                  << "  v=" << std::setprecision(3) << (r.actual_velocity - p.velocity) << "\n"
                  << "  forces: Fz=" << std::setprecision(0) << r.Fz_avg
                  << "  Fx=" << r.Fx_avg << "  Fy=" << r.Fy_avg
                  << " (" << r.n_samples << " avg)"
                  << (alpha_warn ? " [alpha>limit]" : "")
                  << "\n";
    };

    // Warm-up (single-threaded)
    {
        auto res = CollectStaticSample(samples[0], 0, visualize);
        if (res.valid) {
            WriteStaticRecord(csv, csv_mtx, samples[0], res);
            success++;
            print_verify(0, samples[0], res);
        }
        completed++;
    }

#ifdef CHRONO_OPENMP
    #pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 1; i < n_samples; i++) {
        if (g_stop) continue;
        auto res = CollectStaticSample(samples[i], i, visualize);
        if (res.valid) {
            WriteStaticRecord(csv, csv_mtx, samples[i], res);
            success++;
            print_verify(i, samples[i], res);
        }
        int done = ++completed;
        if (done % 50 == 0 || done == n_samples) {
#ifdef CHRONO_OPENMP
            #pragma omp critical
#endif
            {
                auto now = std::chrono::high_resolution_clock::now();
                double el = std::chrono::duration<double>(now - t_start).count();
                double rate = done / el;
                std::cout << "Progress: " << done << "/" << n_samples
                          << " (" << std::fixed << std::setprecision(1)
                          << (100.0 * done / n_samples) << "%)"
                          << " " << std::setprecision(2) << rate << " samples/s"
                          << " ETA: " << std::setprecision(0) << ((n_samples - done) / rate) << "s\n";
            }
        }
    }

    csv.close();
    auto t_end = std::chrono::high_resolution_clock::now();
    double total = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "\n=== Collection Complete ===\n"
              << "Time: " << std::fixed << std::setprecision(1) << total << "s\n"
              << "Samples: " << success.load() << "/" << completed.load() << "\n"
              << "Samples with max|alpha| > " << ALPHA_LIMIT << " rad: " << n_alpha_warn.load() << "\n"
              << "Output: " << output_file << "\n";
}

// =============================================================================
int main(int argc, char* argv[]) {
    int n_samples = 10000;  // default: 10k diverse samples
    std::string output_file = "scm_static_data.csv";
    bool use_parallel = true;
    bool visualize = false;
    bool use_factored_sampling = false;
    int num_threads = 0;
    int batch_size = 0;
    int terrain_bank_size = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--threads" || arg == "-t") {
            if (i + 1 < argc) num_threads = std::atoi(argv[++i]);
        } else if (arg == "--batch-size" || arg == "-b") {
            if (i + 1 < argc) batch_size = std::atoi(argv[++i]);
        } else if (arg == "--sequential" || arg == "-s") {
            use_parallel = false;
        } else if (arg == "--visualize" || arg == "-v") {
            visualize = true;
        } else if (arg == "--factored") {
            use_factored_sampling = true;
        } else if (arg == "--terrain-bank-size") {
            if (i + 1 < argc) terrain_bank_size = std::atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [num_samples] [output.csv] [options]\n\n"
                      << "Collects static steady-state tire force data for NN training.\n"
                      << "Linear-only mode with alpha(0)=0.\n"
                      << "11 parameters are independently sampled via LHS; steering_rate is\n"
                      << "derived from (slip_angle / target_time) for kinematic consistency.\n\n"
                      << "Factored mode separates terrain sampling from operating-point sampling,\n"
                      << "then pairs them systematically so each terrain appears under multiple\n"
                      << "operating conditions without biasing toward any named preset.\n\n"
                      << "Linear-only profile: alpha(t)=sr*t with alpha(0)=0.\n"
                      << "Per-sample target crossing time is clamped to ["
                      << T_TARGET_MIN << ", " << T_TARGET_MAX << "] s.\n"
                      << "Measurement window: " << T_MEASURE
                      << "s. Sim/sample in [" << T_END_MIN << ", " << T_END_MAX
                      << "] s → 1 output row.\n\n"
                      << "Options:\n"
                      << "  N                   Number of samples (default: 10000)\n"
                      << "  --visualize, -v     Enable Irrlicht visualization (single-thread only)\n"
                      << "  --sequential, -s    Single-threaded\n"
                      << "  --factored          Use separate terrain/op sampling banks\n"
                      << "  --terrain-bank-size N  Terrain bank size for --factored (default: sqrt(N))\n"
                      << "  --threads N, -t N   OpenMP threads (0=auto)\n"
                      << "  --batch-size N, -b N  Subprocess batch size (prevents OOM)\n"
                      << "  --help, -h          Show help\n";
            return 0;
        } else if (arg[0] >= '0' && arg[0] <= '9') {
            n_samples = std::atoi(argv[i]);
        } else if (arg.find(".csv") != std::string::npos) {
            output_file = arg;
        }
    }

    SetChronoDataPath(CHRONO_DATA_DIR);

    try {
        if (use_factored_sampling && terrain_bank_size <= 0) {
            terrain_bank_size = std::max(16, static_cast<int>(std::sqrt(std::max(1, n_samples))));
        }
        terrain_bank_size = std::max(1, std::min(terrain_bank_size, n_samples));

        if (visualize) {
#ifndef CHRONO_IRRLICHT
            std::cerr << "Error: built without Irrlicht support. "
                      << "Rebuild with CHRONO_IRRLICHT enabled.\n";
            return 1;
#else
            // Visualization is only safe in single-process, single-thread mode.
            use_parallel = false;
            num_threads = 1;
            if (batch_size > 0) {
                std::cout << "Visualization enabled: ignoring --batch-size and running in single process.\n";
                batch_size = 0;
            }
#endif
        }

#ifdef __linux__
        if (batch_size > 0) {
            CollectWithSubprocessBatching(
                n_samples, output_file, num_threads, use_parallel, batch_size,
                use_factored_sampling, terrain_bank_size
            );
            return 0;
        }
#endif
        CollectStaticData(
            n_samples, output_file, num_threads, use_parallel, visualize,
            use_factored_sampling, terrain_bank_size
        );
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
