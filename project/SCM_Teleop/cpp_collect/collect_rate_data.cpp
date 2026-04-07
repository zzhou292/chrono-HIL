// =============================================================================
// Rate-Aware SCM Data Collection for NN Tire Model Training
//
// Like collect_temporal_data.cpp, but ALL operating conditions vary within each
// scenario (not just slip angle).  This produces non-zero finite-difference
// rate features (d_slip_ratio/dt, d_slip_angle/dt, d_velocity/dt) so a
// rate-augmented NN can learn transient tire dynamics.
//
// Time-varying profiles (all are polynomials evaluated by ChTireTestRig):
//   velocity(t)   = v0 + dv*t                   (linear ramp)
//   slip_ratio(t)  = kappa0 + dk*t              (linear ramp)
//   slip_angle(t)  = a0 + a1*t + a2*t^2 + a3*t^3  (poly deg 1-3)
//   ang_speed(t)   = v(t)/R * (1 + kappa(t))     (quadratic polynomial)
//
// Recording rate: configurable (default 5ms = 200Hz).
//
// Output CSV columns:
//   scenario_id, timestep, slip_ratio, slip_angle, velocity, vertical_load,
//   steering_rate, bekker_Kphi, ..., janosi_shear, mesh_spacing, Fz, Fx, Fy
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
#include <numeric>

#ifdef __linux__
#include <malloc.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
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
// Simulation parameters
// =============================================================================
constexpr double STEP_SIZE = 5e-4;
constexpr double T_DELAY = 0.5;          // Settling time before recording
constexpr double T_RECORD_DURATION = 2.0; // Duration of force recording
constexpr double T_END = T_DELAY + T_RECORD_DURATION;
constexpr double RECORD_DT = 0.005;      // 5ms recording interval (200 Hz)
constexpr double MESH_SPACING_FAST = 0.10;
constexpr int SOLVER_ITERS = 50;

// =============================================================================
// Parameter ranges (extends Dallas et al. Table I with ramp rates)
// =============================================================================
struct ParameterRanges {
    // --- Slip angle polynomial profile ---
    double slip_angle_base_min = -0.40;  // rad
    double slip_angle_base_max = 0.40;
    double sr_base_min = -0.56;          // rad/s steering rate
    double sr_base_max = 0.56;
    double sr_accel_min = -1.5;          // rad/s^2
    double sr_accel_max = 1.5;
    double sr_jerk_min = -3.0;           // rad/s^3
    double sr_jerk_max = 3.0;

    // --- Slip ratio (now time-varying) ---
    double slip_ratio_min = -1.0;
    double slip_ratio_max = 1.0;
    double d_slip_ratio_min = -0.4;      // /s  ramp rate
    double d_slip_ratio_max = 0.4;

    // --- Velocity (now time-varying) ---
    double velocity_min = 2.0;           // m/s
    double velocity_max = 10.0;
    double d_velocity_min = -2.0;        // m/s^2  acceleration/braking
    double d_velocity_max = 2.0;

    // --- Vertical load (constant per scenario) ---
    double vertical_load_min = 2500;     // N
    double vertical_load_max = 7500;

    // --- Terrain ---
    double bekker_Kphi_min = 0.5e6;
    double bekker_Kphi_max = 4.0e6;
    double bekker_Kc_min = 0.0;
    double bekker_Kc_max = 20000.0;
    double bekker_n_min = 0.3;
    double bekker_n_max = 1.3;
    double mohr_cohesion_min = 650.0;
    double mohr_cohesion_max = 20700.0;
    double mohr_friction_min = 6.0;      // degrees
    double mohr_friction_max = 37.8;
    double janosi_shear_min = 0.01;
    double janosi_shear_max = 0.024;
    double mesh_spacing_min = 0.05;
    double mesh_spacing_max = 0.15;
};

// =============================================================================
// Sample parameters
// =============================================================================
struct RateSampleParams {
    // --- Velocity ramp: v(t) = v0 + dv*t ---
    double v0;        // initial velocity (m/s)
    double dv;        // velocity ramp rate (m/s^2)

    // --- Slip ratio ramp: kappa(t) = kappa0 + dk*t ---
    double kappa0;    // initial slip ratio
    double dk;        // slip ratio ramp rate (/s)

    // --- Slip angle polynomial: alpha(t) = a0 + a1*t + a2*t^2 + a3*t^3 ---
    int    profile_degree;   // 1, 2, or 3
    double poly_a0;
    double poly_a1;
    double poly_a2;
    double poly_a3;

    // --- Vertical load (constant) ---
    double vertical_load;

    // --- Terrain (constant per scenario) ---
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_friction;   // degrees (internal)
    double janosi_shear;
    double mesh_spacing;

    // Evaluate operating conditions at time t
    double velocity(double t) const { return v0 + dv * t; }
    double kappa(double t)    const { return kappa0 + dk * t; }
    double alpha(double t)    const {
        return poly_a0 + poly_a1 * t + poly_a2 * t * t + poly_a3 * t * t * t;
    }
    double steering_rate(double t) const {
        return poly_a1 + 2.0 * poly_a2 * t + 3.0 * poly_a3 * t * t;
    }
};

// =============================================================================
// Single timestep record
// =============================================================================
struct TimestepRecord {
    double time;
    double slip_ratio;     // instantaneous kappa(t)
    double slip_angle;     // instantaneous alpha(t)  (rad)
    double velocity;       // instantaneous v(t) (m/s)
    double steering_rate;  // d(alpha)/dt (rad/s)
    double Fz, Fx, Fy;    // measured forces (N)
};

// =============================================================================
// LHS sampling
// =============================================================================
std::vector<RateSampleParams> GenerateLHSSamples(int n, const ParameterRanges& r, unsigned int seed = 42) {
    std::mt19937 rng(seed);
    std::vector<RateSampleParams> samples(n);

    auto lhs = [&](double lo, double hi) {
        std::vector<double> v(n);
        std::vector<int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);
        std::shuffle(perm.begin(), perm.end(), rng);
        std::uniform_real_distribution<> d(0.0, 1.0);
        for (int i = 0; i < n; i++)
            v[i] = lo + ((perm[i] + d(rng)) / n) * (hi - lo);
        return v;
    };

    // Operating condition base values + ramp rates
    auto v0s       = lhs(r.velocity_min, r.velocity_max);
    auto dvs       = lhs(r.d_velocity_min, r.d_velocity_max);
    auto kappa0s   = lhs(r.slip_ratio_min, r.slip_ratio_max);
    auto dks       = lhs(r.d_slip_ratio_min, r.d_slip_ratio_max);
    auto vert_loads = lhs(r.vertical_load_min, r.vertical_load_max);

    // Slip angle polynomial
    auto alpha_bases = lhs(r.slip_angle_base_min, r.slip_angle_base_max);
    auto sr_bases    = lhs(r.sr_base_min, r.sr_base_max);
    auto sr_accels   = lhs(r.sr_accel_min, r.sr_accel_max);
    auto sr_jerks    = lhs(r.sr_jerk_min, r.sr_jerk_max);

    // Terrain
    auto bk_Kphis     = lhs(r.bekker_Kphi_min, r.bekker_Kphi_max);
    auto bk_Kcs       = lhs(r.bekker_Kc_min, r.bekker_Kc_max);
    auto bk_ns        = lhs(r.bekker_n_min, r.bekker_n_max);
    auto mc_cohesions = lhs(r.mohr_cohesion_min, r.mohr_cohesion_max);
    auto mc_frictions = lhs(r.mohr_friction_min, r.mohr_friction_max);
    auto j_shears     = lhs(r.janosi_shear_min, r.janosi_shear_max);
    auto meshes       = lhs(r.mesh_spacing_min, r.mesh_spacing_max);

    for (int i = 0; i < n; i++) {
        auto& s = samples[i];

        s.v0     = v0s[i];
        s.dv     = dvs[i];
        s.kappa0 = kappa0s[i];
        s.dk     = dks[i];
        s.vertical_load = vert_loads[i];

        s.bekker_Kphi   = bk_Kphis[i];
        s.bekker_Kc     = bk_Kcs[i];
        s.bekker_n      = bk_ns[i];
        s.mohr_cohesion = mc_cohesions[i];
        s.mohr_friction = mc_frictions[i];
        s.janosi_shear  = j_shears[i];
        s.mesh_spacing  = meshes[i];

        // Slip angle profile degree: cycle through 1, 2, 3
        s.profile_degree = 1 + (i % 3);
        s.poly_a0 = alpha_bases[i];
        s.poly_a1 = sr_bases[i];
        s.poly_a2 = (s.profile_degree >= 2) ? sr_accels[i] / 2.0 : 0.0;
        s.poly_a3 = (s.profile_degree >= 3) ? sr_jerks[i] / 6.0  : 0.0;

        // ---- Clamp velocity to stay in [1.5, 12] over [0, T_END] ----
        {
            constexpr double V_MIN = 1.5, V_MAX = 12.0;
            double v_end = s.velocity(T_END);
            if (v_end < V_MIN) {
                // reduce deceleration so v(T_END) >= V_MIN
                s.dv = (V_MIN - s.v0) / T_END;
            } else if (v_end > V_MAX) {
                s.dv = (V_MAX - s.v0) / T_END;
            }
            // Also check v(0) is fine (it equals v0 which is already in [2,10])
        }

        // ---- Clamp slip ratio to stay in [-1.0, 1.0] over [0, T_END] ----
        {
            constexpr double K_LIMIT = 1.0;
            double k_end = s.kappa(T_END);
            if (std::abs(k_end) > K_LIMIT) {
                double target = (k_end > 0) ? K_LIMIT * 0.95 : -K_LIMIT * 0.95;
                s.dk = (target - s.kappa0) / T_END;
            }
        }

        // ---- Clamp slip angle polynomial to [-0.6, 0.6] ----
        {
            constexpr double ALPHA_LIMIT = 0.6;
            double max_alpha = 0.0;
            for (int k = 0; k <= 20; k++) {
                double t = T_DELAY + k * T_RECORD_DURATION / 20.0;
                max_alpha = std::max(max_alpha, std::abs(s.alpha(t)));
            }
            if (max_alpha > ALPHA_LIMIT) {
                double scale = ALPHA_LIMIT / max_alpha * 0.95;
                s.poly_a0 *= scale;
                s.poly_a1 *= scale;
                s.poly_a2 *= scale;
                s.poly_a3 *= scale;
            }
        }
    }
    return samples;
}

// =============================================================================
// Build angular-speed polynomial:  omega(t) = v(t)/R * (1 + kappa(t))
//
//   v(t) = v0 + dv*t
//   1 + kappa(t) = (1 + kappa0) + dk*t
//
//   omega(t) = [v0*(1+kappa0)]/R
//            + [v0*dk + dv*(1+kappa0)]/R * t
//            + [dv*dk]/R * t^2
// =============================================================================
std::vector<double> BuildAngSpeedCoeffs(const RateSampleParams& p, double R) {
    double one_plus_k0 = 1.0 + p.kappa0;
    double c0 = (p.v0 * one_plus_k0) / R;
    double c1 = (p.v0 * p.dk + p.dv * one_plus_k0) / R;
    double c2 = (p.dv * p.dk) / R;

    std::vector<double> coeffs = {c0, c1};
    if (std::abs(c2) > 1e-12)
        coeffs.push_back(c2);
    return coeffs;
}

// =============================================================================
// Run one scenario, return time-series of force measurements
// =============================================================================
std::vector<TimestepRecord> CollectRateSample(
    const RateSampleParams& params, int /*scenario_id*/)
{
    std::vector<TimestepRecord> records;
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

        // --- Time-varying velocity function: v(t) = v0 + dv*t ---
        auto vel_func = chrono_types::make_shared<ChFunctionPoly>();
        vel_func->SetCoefficients({params.v0, params.dv});

        // --- Time-varying angular speed: omega(t) = v(t)/R * (1 + kappa(t)) ---
        auto ang_func = chrono_types::make_shared<ChFunctionPoly>();
        ang_func->SetCoefficients(BuildAngSpeedCoeffs(params, tire_radius));

        // --- Slip angle polynomial ---
        auto slip_func = chrono_types::make_shared<ChFunctionPoly>();
        std::vector<double> alpha_coeffs = {params.poly_a0, params.poly_a1};
        if (params.profile_degree >= 2)
            alpha_coeffs.push_back(params.poly_a2);
        if (params.profile_degree >= 3)
            alpha_coeffs.push_back(params.poly_a3);
        slip_func->SetCoefficients(alpha_coeffs);

        rig.SetLongSpeedFunction(vel_func);
        rig.SetAngSpeedFunction(ang_func);
        rig.SetSlipAngleFunction(slip_func);
        rig.SetTimeDelay(T_DELAY);
        rig.Initialize(ChTireTestRig::Mode::TEST);

        // Simulate and record
        double t = 0;
        double t_next_record = T_DELAY;
        int timestep_idx = 0;

        while (t < T_END) {
            rig.Advance(STEP_SIZE);
            t += STEP_SIZE;

            if (t >= t_next_record) {
                auto force = rig.ReportTireForce();
                TimestepRecord rec;
                rec.time          = t;
                rec.slip_ratio    = params.kappa(t);
                rec.slip_angle    = params.alpha(t);
                rec.velocity      = params.velocity(t);
                rec.steering_rate = params.steering_rate(t);
                rec.Fz = force.force.z();
                rec.Fx = force.force.x();
                rec.Fy = force.force.y();
                records.push_back(rec);
                timestep_idx++;
                t_next_record += RECORD_DT;
            }
        }

        sys->Clear();
        sys.reset();
#ifdef __linux__
        malloc_trim(0);
#endif
    } catch (const std::exception& e) {
        std::cerr << "Scenario failed: " << e.what() << std::endl;
        records.clear();
    }
    return records;
}

// =============================================================================
// Write one scenario's records to CSV (thread-safe)
// =============================================================================
void WriteRateRecords(std::ofstream& csv, std::mutex& mtx,
                      int scenario_id,
                      const RateSampleParams& params,
                      const std::vector<TimestepRecord>& records)
{
    if (records.empty()) return;

    double mohr_friction_rad = params.mohr_friction * CH_DEG_TO_RAD;

    std::ostringstream buf;
    buf << std::fixed << std::setprecision(6);

    for (int i = 0; i < static_cast<int>(records.size()); i++) {
        const auto& r = records[i];
        buf << scenario_id << ","
            << i << ","
            << r.slip_ratio << ","
            << r.slip_angle << ","
            << r.velocity << ","
            << params.vertical_load << ","
            << r.steering_rate << ","
            << params.bekker_Kphi << ","
            << params.bekker_Kc << ","
            << params.bekker_n << ","
            << params.mohr_cohesion << ","
            << mohr_friction_rad << ","
            << params.janosi_shear << ","
            << params.mesh_spacing << ","
            << r.Fz << ","
            << r.Fx << ","
            << r.Fy << "\n";
    }

    std::lock_guard<std::mutex> lock(mtx);
    csv << buf.str();
    csv.flush();
}

// =============================================================================
// CSV header (same columns as temporal data — training script is compatible)
// =============================================================================
static const char* CSV_HEADER =
    "scenario_id,timestep,slip_ratio,slip_angle,velocity,vertical_load,"
    "steering_rate,bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,"
    "mohr_friction,janosi_shear,mesh_spacing,Fz,Fx,Fy\n";

// =============================================================================
// Process a batch of scenarios
// =============================================================================
int ProcessBatch(const std::vector<RateSampleParams>& samples,
                 int base_scenario_id,
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

    // Warm-up sample (single-threaded)
    if (n > 0) {
        auto recs = CollectRateSample(samples[0], base_scenario_id);
        if (!recs.empty()) {
            WriteRateRecords(csv, csv_mtx, base_scenario_id, samples[0], recs);
            success++;
        }
    }

#ifdef CHRONO_OPENMP
    #pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 1; i < n; i++) {
        auto recs = CollectRateSample(samples[i], base_scenario_id + i);
        if (!recs.empty()) {
            WriteRateRecords(csv, csv_mtx, base_scenario_id + i, samples[i], recs);
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
                                   int num_threads, bool use_parallel, int batch_size)
{
    std::cout << "\n=== Subprocess-Batched Rate Data Collection ===\n"
              << "Total scenarios: " << n_samples << "\n"
              << "Batch size: " << batch_size << "\n"
              << "Recording: every " << RECORD_DT*1000 << "ms over "
              << T_RECORD_DURATION << "s = ~"
              << static_cast<int>(T_RECORD_DURATION / RECORD_DT) << " timesteps/scenario\n"
              << "Operating conditions: velocity, slip_ratio, slip_angle ALL time-varying\n";

    // Write header
    {
        std::ofstream hdr(output_file);
        hdr << CSV_HEADER;
    }

    ParameterRanges ranges;
    auto all_samples = GenerateLHSSamples(n_samples, ranges);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto t0 = std::chrono::high_resolution_clock::now();
    int total_ok = 0, total_done = 0;

    for (int bs = 0; bs < n_samples && !g_stop; bs += batch_size) {
        int be = std::min(bs + batch_size, n_samples);
        int cur = be - bs;

        std::cout << "\n--- Batch: scenarios " << bs << "-" << (be-1) << " ---\n";

        pid_t pid = fork();
        if (pid == 0) {
            SetChronoDataPath(CHRONO_DATA_DIR);
            std::vector<RateSampleParams> batch(
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
                      << std::setprecision(2) << rate << " scen/s\n";
        } else {
            std::cerr << "Fork failed!\n";
            break;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double total_t = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "\n=== Collection Complete ===\n"
              << "Time: " << std::fixed << std::setprecision(1) << total_t << "s\n"
              << "Scenarios: " << total_ok << "/" << total_done << " succeeded\n"
              << "Output: " << output_file << "\n";
}
#endif

// =============================================================================
// Single-process collection (fixed count or continuous)
// =============================================================================
void CollectRateData(int n_samples, const std::string& output_file,
                     int num_threads, bool use_parallel)
{
    bool continuous = (n_samples <= 0);

    std::cout << "\n=== Rate-Aware SCM Data Collection ===\n"
              << "Scenarios: " << (continuous ? "CONTINUOUS (Ctrl+C to stop)" : std::to_string(n_samples)) << "\n"
              << "Step size: " << STEP_SIZE << "\n"
              << "Recording: every " << RECORD_DT*1000 << "ms over "
              << T_RECORD_DURATION << "s\n"
              << "Time-varying: velocity (linear), slip_ratio (linear), slip_angle (poly deg 1-3)\n";

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
    std::atomic<int> completed{0};
    std::atomic<int> success{0};

    auto t_start = std::chrono::high_resolution_clock::now();

    if (continuous) {
        // Warm-up
        {
            RateSampleParams warm;
            warm.v0 = 5; warm.dv = 0;
            warm.kappa0 = 0; warm.dk = 0;
            warm.profile_degree = 1; warm.poly_a0 = 0; warm.poly_a1 = 0.1;
            warm.poly_a2 = 0; warm.poly_a3 = 0;
            warm.vertical_load = 4000;
            warm.bekker_Kphi = 2e6; warm.bekker_Kc = 5000; warm.bekker_n = 0.8;
            warm.mohr_cohesion = 5000; warm.mohr_friction = 20; warm.janosi_shear = 0.015;
            warm.mesh_spacing = 0.10;
            auto recs = CollectRateSample(warm, 0);
            if (!recs.empty()) { WriteRateRecords(csv, csv_mtx, 0, warm, recs); success++; }
            completed++;
        }

        while (!g_stop) {
            int batch_sz = num_threads;
            std::vector<RateSampleParams> batch(batch_sz);
            for (int i = 0; i < batch_sz; i++) {
                unsigned int s = static_cast<unsigned int>(
                    std::chrono::steady_clock::now().time_since_epoch().count() + i);
                std::mt19937 rng(s);
                std::uniform_real_distribution<> d(0.0, 1.0);
                auto& p = batch[i];

                p.v0     = ranges.velocity_min + d(rng) * (ranges.velocity_max - ranges.velocity_min);
                p.dv     = ranges.d_velocity_min + d(rng) * (ranges.d_velocity_max - ranges.d_velocity_min);
                p.kappa0 = ranges.slip_ratio_min + d(rng) * (ranges.slip_ratio_max - ranges.slip_ratio_min);
                p.dk     = ranges.d_slip_ratio_min + d(rng) * (ranges.d_slip_ratio_max - ranges.d_slip_ratio_min);
                p.vertical_load = ranges.vertical_load_min + d(rng) * (ranges.vertical_load_max - ranges.vertical_load_min);

                p.poly_a0 = ranges.slip_angle_base_min + d(rng) * (ranges.slip_angle_base_max - ranges.slip_angle_base_min);
                p.poly_a1 = ranges.sr_base_min + d(rng) * (ranges.sr_base_max - ranges.sr_base_min);
                p.profile_degree = 1 + static_cast<int>(d(rng) * 3) % 3;
                p.poly_a2 = (p.profile_degree >= 2) ? (ranges.sr_accel_min + d(rng) * (ranges.sr_accel_max - ranges.sr_accel_min)) / 2.0 : 0;
                p.poly_a3 = (p.profile_degree >= 3) ? (ranges.sr_jerk_min + d(rng) * (ranges.sr_jerk_max - ranges.sr_jerk_min)) / 6.0 : 0;

                p.bekker_Kphi   = ranges.bekker_Kphi_min + d(rng) * (ranges.bekker_Kphi_max - ranges.bekker_Kphi_min);
                p.bekker_Kc     = ranges.bekker_Kc_min + d(rng) * (ranges.bekker_Kc_max - ranges.bekker_Kc_min);
                p.bekker_n      = ranges.bekker_n_min + d(rng) * (ranges.bekker_n_max - ranges.bekker_n_min);
                p.mohr_cohesion = ranges.mohr_cohesion_min + d(rng) * (ranges.mohr_cohesion_max - ranges.mohr_cohesion_min);
                p.mohr_friction = ranges.mohr_friction_min + d(rng) * (ranges.mohr_friction_max - ranges.mohr_friction_min);
                p.janosi_shear  = ranges.janosi_shear_min + d(rng) * (ranges.janosi_shear_max - ranges.janosi_shear_min);
                p.mesh_spacing  = ranges.mesh_spacing_min + d(rng) * (ranges.mesh_spacing_max - ranges.mesh_spacing_min);

                // Clamp velocity
                double v_end = p.velocity(T_END);
                if (v_end < 1.5) p.dv = (1.5 - p.v0) / T_END;
                else if (v_end > 12.0) p.dv = (12.0 - p.v0) / T_END;

                // Clamp slip ratio
                double k_end = p.kappa(T_END);
                if (std::abs(k_end) > 1.0) {
                    double target = (k_end > 0) ? 0.95 : -0.95;
                    p.dk = (target - p.kappa0) / T_END;
                }

                // Clamp slip angle
                double max_a = 0;
                for (int k = 0; k <= 20; k++) {
                    double t = T_DELAY + k * T_RECORD_DURATION / 20.0;
                    max_a = std::max(max_a, std::abs(p.alpha(t)));
                }
                if (max_a > 0.6) {
                    double sc = 0.57 / max_a;
                    p.poly_a0 *= sc; p.poly_a1 *= sc; p.poly_a2 *= sc; p.poly_a3 *= sc;
                }
            }

#ifdef CHRONO_OPENMP
            #pragma omp parallel for schedule(dynamic)
#endif
            for (int i = 0; i < batch_sz; i++) {
                if (g_stop) continue;
                int sid = completed.load() + i;
                auto recs = CollectRateSample(batch[i], sid);
                if (!recs.empty()) {
                    WriteRateRecords(csv, csv_mtx, sid, batch[i], recs);
                    success++;
                }
                int done = ++completed;
                if (done % 10 == 0) {
#ifdef CHRONO_OPENMP
                    #pragma omp critical
#endif
                    {
                        auto now = std::chrono::high_resolution_clock::now();
                        double el = std::chrono::duration<double>(now - t_start).count();
                        std::cout << "Scenarios: " << done
                                  << " (ok: " << success.load() << ") "
                                  << std::fixed << std::setprecision(2) << (done/el) << " scen/s\n";
                    }
                }
            }
        }
    } else {
        // Fixed-count mode with LHS
        auto samples = GenerateLHSSamples(n_samples, ranges);

        // Warm-up
        {
            auto recs = CollectRateSample(samples[0], 0);
            if (!recs.empty()) { WriteRateRecords(csv, csv_mtx, 0, samples[0], recs); success++; }
            completed++;
        }

#ifdef CHRONO_OPENMP
        #pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 1; i < n_samples; i++) {
            if (g_stop) continue;
            auto recs = CollectRateSample(samples[i], i);
            if (!recs.empty()) {
                WriteRateRecords(csv, csv_mtx, i, samples[i], recs);
                success++;
            }
            int done = ++completed;
            if (done % 10 == 0 || done == n_samples) {
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
                              << " " << std::setprecision(2) << rate << " scen/s"
                              << " ETA: " << std::setprecision(0) << ((n_samples - done) / rate) << "s\n";
                }
            }
        }
    }

    csv.close();
    auto t_end = std::chrono::high_resolution_clock::now();
    double total = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "\n=== Collection Complete ===\n"
              << "Time: " << std::fixed << std::setprecision(1) << total << "s\n"
              << "Scenarios: " << success.load() << "/" << completed.load() << "\n"
              << "Output: " << output_file << "\n";
}

// =============================================================================
int main(int argc, char* argv[]) {
    int n_samples = 0;
    std::string output_file = "rate_aware_timeseries.csv";
    bool use_parallel = true;
    int num_threads = 0;
    int batch_size = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--threads" || arg == "-t") {
            if (i + 1 < argc) num_threads = std::atoi(argv[++i]);
        } else if (arg == "--batch-size" || arg == "-b") {
            if (i + 1 < argc) batch_size = std::atoi(argv[++i]);
        } else if (arg == "--sequential" || arg == "-s") {
            use_parallel = false;
        } else if (arg == "--continuous" || arg == "-c") {
            n_samples = 0;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [num_scenarios] [output.csv] [options]\n\n"
                      << "Collects time-series tire force data with ALL operating conditions\n"
                      << "varying within each scenario (velocity, slip_ratio, slip_angle).\n"
                      << "Produces non-zero rate features for rate-augmented NN training.\n\n"
                      << "Each scenario records ~" << static_cast<int>(T_RECORD_DURATION/RECORD_DT)
                      << " timesteps at " << RECORD_DT*1000 << "ms intervals.\n\n"
                      << "Time-varying profiles:\n"
                      << "  velocity(t)    = v0 + dv*t         (linear ramp)\n"
                      << "  slip_ratio(t)  = kappa0 + dk*t     (linear ramp)\n"
                      << "  slip_angle(t)  = polynomial deg 1-3\n\n"
                      << "Options:\n"
                      << "  [no number]         Continuous mode (Ctrl+C to stop)\n"
                      << "  N                   Run N scenarios\n"
                      << "  --sequential, -s    Single-threaded\n"
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
#ifdef __linux__
        if (batch_size > 0 && n_samples > 0) {
            CollectWithSubprocessBatching(n_samples, output_file, num_threads, use_parallel, batch_size);
            return 0;
        }
#endif
        CollectRateData(n_samples, output_file, num_threads, use_parallel);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
