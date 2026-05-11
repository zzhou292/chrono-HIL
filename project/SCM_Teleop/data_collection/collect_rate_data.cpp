// =============================================================================
// collect_rate_data.cpp
// Rate-aware SCM tire force data collection for neural network training.
//
// Like collect_static_data, this produces ONE measurement per scenario.
// The key difference: it independently controls both the VALUE and RATE of
// each operating condition (κ, α, v) at the measurement time.
//
// Approach:
//   - LHS sample target values (κ*, α*, v*) AND rates (dκ/dt, dα/dt, dv/dt)
//   - Build quadratic polynomials that start at 0 and reach (value, rate)
//     at the target time:  f(t) = a·t + b·t²
//   - Velocity uses a linear ramp: v(t) = v_start + dv·t
//   - Derived: ω(t) = v(t)/R · (1+κ(t))
//   - Average forces over a short measurement window at t_target
//
// Output CSV (one row per scenario):
//   slip_ratio, slip_angle, velocity, vertical_load, steering_rate,
//   d_slip_ratio, d_slip_angle, d_velocity,
//   bekker_Kphi, ..., janosi_shear, mesh_spacing, Fz, Fx, Fy
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
// Simulation parameters (mirroring collect_static_data)
// =============================================================================
constexpr double STEP_SIZE    = 5e-4;
constexpr double T_SETTLE_MIN = 1.0;      // settling before polynomial ramp starts
constexpr double T_RAMP_MIN   = 1.5;      // minimum ramp time to reach target
constexpr double T_RAMP_MAX   = 4.0;      // maximum ramp time
constexpr double T_MEASURE    = 0.1;      // averaging window (short: everything changing)
constexpr double MEASURE_DT   = 0.001;    // force sample interval during measurement
constexpr double T_TAIL       = 0.1;      // buffer after measurement
constexpr double TIRE_RADIUS  = 0.47;
constexpr int    SOLVER_ITERS = 50;

// Bounds
constexpr double V_MIN = 2.0, V_MAX = 10.0;
constexpr double KAPPA_LIMIT = 1.0;
constexpr double ALPHA_LIMIT = 0.6;

// =============================================================================
// Parameter ranges for LHS sampling
// =============================================================================
struct ParameterRanges {
    // Target values at measurement time
    double kappa_target_min = -0.8, kappa_target_max = 0.8;
    double alpha_target_min = -0.40, alpha_target_max = 0.40;
    double velocity_target_min = 2.5, velocity_target_max = 9.5;

    // Target rates at measurement time
    double dkappa_min = -0.4, dkappa_max = 0.4;     // dκ/dt (1/s)
    double dalpha_min = -0.56, dalpha_max = 0.56;    // dα/dt (rad/s) = steering rate
    double dvelocity_min = -1.5, dvelocity_max = 1.5; // dv/dt (m/s²)

    // Ramp timing (time from delay end to measurement)
    double ramp_time_min = T_RAMP_MIN, ramp_time_max = T_RAMP_MAX;

    // Vertical load (constant per scenario)
    double vertical_load_min = 2500.0, vertical_load_max = 7500.0;

    // Terrain (same as static collector)
    double bekker_Kphi_min = 0.5e6, bekker_Kphi_max = 4.0e6;
    double bekker_Kc_min = 0.0, bekker_Kc_max = 20000.0;
    double bekker_n_min = 0.3, bekker_n_max = 1.3;
    double mohr_cohesion_min = 650.0, mohr_cohesion_max = 20700.0;
    double mohr_friction_min = 6.0, mohr_friction_max = 37.8;  // degrees
    double janosi_shear_min = 0.01, janosi_shear_max = 0.025;
    double mesh_spacing_min = 0.08, mesh_spacing_max = 0.12;
};

// =============================================================================
// Sample parameters — target value + rate for each operating condition
// =============================================================================
struct RateSampleParams {
    // Target values at measurement time
    double kappa_target;      // slip ratio
    double alpha_target;      // slip angle (rad)
    double velocity_target;   // longitudinal speed (m/s)

    // Target rates at measurement time
    double dkappa;            // dκ/dt (1/s)
    double dalpha;            // dα/dt (rad/s) = steering rate
    double dvelocity;         // dv/dt (m/s²)

    // Ramp timing
    double ramp_time;         // time from delay end to measurement (s)

    // Vertical load (constant)
    double vertical_load;

    // Terrain (constant)
    double bekker_Kphi, bekker_Kc, bekker_n;
    double mohr_cohesion, mohr_friction, janosi_shear;
    double mesh_spacing;
};

// =============================================================================
// Result from a single scenario
// =============================================================================
struct RateResult {
    bool valid = false;
    double slip_ratio;      // κ at measurement (prescribed)
    double slip_angle;      // α at measurement (prescribed)
    double velocity;        // v at measurement (prescribed)
    double steering_rate;   // dα/dt at measurement
    double d_slip_ratio;    // dκ/dt at measurement
    double d_slip_angle;    // d(actual_alpha)/dt at measurement
    double d_velocity;      // dv/dt at measurement
    double Fz_avg, Fx_avg, Fy_avg;
    int n_samples;
    // Actual state from rig (for verification)
    double actual_kappa;    // tire-computed longitudinal slip
    double actual_alpha;    // tire-computed slip angle
    double actual_velocity; // carrier longitudinal speed
    double actual_omega;    // wheel angular speed
    // Actual rates from rig (finite-diff across measurement window)
    double actual_dkappa;   // d(actual_kappa)/dt
    double actual_dalpha;   // d(actual_alpha)/dt
    double actual_dvelocity;// d(actual_velocity)/dt
};

// =============================================================================
// Multiply two polynomials: c = a * b
// =============================================================================
std::vector<double> PolyMultiply(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return {};
    int n = static_cast<int>(a.size() + b.size()) - 1;
    std::vector<double> c(n, 0.0);
    for (size_t i = 0; i < a.size(); i++)
        for (size_t j = 0; j < b.size(); j++)
            c[i + j] += a[i] * b[j];
    return c;
}

// =============================================================================
// Check if a quadratic profile f(t) = a*t + b*t² stays within [-limit, limit]
// for t in [0, T].
// =============================================================================
bool ProfileInBounds(double a, double b, double T, double limit) {
    // Check endpoints
    if (std::abs(a * T + b * T * T) > limit) return false;
    // Check extremum if it's in (0, T)
    if (std::abs(b) > 1e-12) {
        double t_ext = -a / (2.0 * b);
        if (t_ext > 0 && t_ext < T) {
            double f_ext = a * t_ext + b * t_ext * t_ext;
            if (std::abs(f_ext) > limit) return false;
        }
    }
    return true;
}

// =============================================================================
// LHS sampling with validity checking
// =============================================================================
std::vector<RateSampleParams> GenerateLHSSamples(int n, const ParameterRanges& r, unsigned int seed = 42) {
    std::mt19937 rng(seed);
    std::vector<RateSampleParams> samples;
    samples.reserve(n);

    // LHS helper: returns n stratified values in [lo, hi]
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

    auto kappa_targets = lhs(r.kappa_target_min, r.kappa_target_max);
    auto alpha_targets = lhs(r.alpha_target_min, r.alpha_target_max);
    auto vel_targets   = lhs(r.velocity_target_min, r.velocity_target_max);
    auto dkappas       = lhs(r.dkappa_min, r.dkappa_max);
    auto dalphas       = lhs(r.dalpha_min, r.dalpha_max);
    auto dvelocities   = lhs(r.dvelocity_min, r.dvelocity_max);
    auto ramp_times    = lhs(r.ramp_time_min, r.ramp_time_max);
    auto vert_loads    = lhs(r.vertical_load_min, r.vertical_load_max);
    auto bk_Kphis      = lhs(r.bekker_Kphi_min, r.bekker_Kphi_max);
    auto bk_Kcs        = lhs(r.bekker_Kc_min, r.bekker_Kc_max);
    auto bk_ns         = lhs(r.bekker_n_min, r.bekker_n_max);
    auto mc_cohesions  = lhs(r.mohr_cohesion_min, r.mohr_cohesion_max);
    auto mc_frictions  = lhs(r.mohr_friction_min, r.mohr_friction_max);
    auto j_shears      = lhs(r.janosi_shear_min, r.janosi_shear_max);
    auto meshes        = lhs(r.mesh_spacing_min, r.mesh_spacing_max);

    for (int i = 0; i < n; i++) {
        RateSampleParams s{};
        s.ramp_time = ramp_times[i];
        double T = s.ramp_time;

        // --- κ profile: κ(t) = a_k*t + b_k*t², κ(0)=0, κ(T)=κ*, κ'(T)=dκ ---
        s.kappa_target = kappa_targets[i];
        s.dkappa = dkappas[i];
        double b_k = (s.dkappa * T - s.kappa_target) / (T * T);
        double a_k = 2.0 * s.kappa_target / T - s.dkappa;
        if (!ProfileInBounds(a_k, b_k, T, KAPPA_LIMIT)) {
            // Fallback: zero rate, linear ramp to target
            s.dkappa = s.kappa_target / T;
        }

        // --- α profile: α(t) = a_a*t + b_a*t², α(0)=0, α(T)=α*, α'(T)=dα ---
        s.alpha_target = alpha_targets[i];
        s.dalpha = dalphas[i];
        double b_a = (s.dalpha * T - s.alpha_target) / (T * T);
        double a_a = 2.0 * s.alpha_target / T - s.dalpha;
        if (!ProfileInBounds(a_a, b_a, T, ALPHA_LIMIT)) {
            s.dalpha = s.alpha_target / T;
        }

        // --- v profile: v(t) = v_start + dv*t (linear) ---
        // v(T) = v_target, v'(t) = dv, so v_start = v_target - dv*T
        s.velocity_target = vel_targets[i];
        s.dvelocity = dvelocities[i];
        double v_start = s.velocity_target - s.dvelocity * T;
        // Clamp dv so v_start stays in [V_MIN, V_MAX]
        if (v_start < V_MIN) {
            v_start = V_MIN;
            s.dvelocity = (s.velocity_target - v_start) / T;
        } else if (v_start > V_MAX) {
            v_start = V_MAX;
            s.dvelocity = (s.velocity_target - v_start) / T;
        }

        // Constant parameters
        s.vertical_load = vert_loads[i];
        s.bekker_Kphi   = bk_Kphis[i];
        s.bekker_Kc     = bk_Kcs[i];
        s.bekker_n      = bk_ns[i];
        s.mohr_cohesion = mc_cohesions[i];
        s.mohr_friction = mc_frictions[i];
        s.janosi_shear  = j_shears[i];
        s.mesh_spacing  = meshes[i];

        samples.push_back(s);
    }

    return samples;
}

// =============================================================================
// Run one scenario, return single measurement at target time
// =============================================================================
RateResult CollectRateSample(
    const RateSampleParams& params, int sample_id, bool visualize = false)
{
    RateResult result{};
    double T = params.ramp_time;
    double t_target = T_SETTLE_MIN + T;

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

        // ---------------------------------------------------------------
        // Profile polynomials (in effective time t_eff after delay)
        // ---------------------------------------------------------------
        // κ(t) = a_k*t + b_k*t²   (κ(0)=0, κ(T)=κ*, κ'(T)=dκ)
        double b_k = (params.dkappa * T - params.kappa_target) / (T * T);
        double a_k = 2.0 * params.kappa_target / T - params.dkappa;
        // (1+κ(t)) = 1 + a_k*t + b_k*t²
        std::vector<double> one_plus_kappa = {1.0, a_k, b_k};

        // α(t) = a_a*t + b_a*t²   (α(0)=0, α(T)=α*, α'(T)=dα)
        double b_a = (params.dalpha * T - params.alpha_target) / (T * T);
        double a_a = 2.0 * params.alpha_target / T - params.dalpha;

        // v(t) = v_start + dv*t  (linear: v(T)=v_target, v'=dv)
        double v_start = params.velocity_target - params.dvelocity * T;
        std::vector<double> v_poly = {v_start, params.dvelocity};

        // ω(t) = v(t)/R * (1+κ(t))  (polynomial product, divided by R)
        std::vector<double> omega_poly_raw = PolyMultiply(v_poly, one_plus_kappa);
        std::vector<double> omega_poly(omega_poly_raw.size());
        for (size_t j = 0; j < omega_poly_raw.size(); j++)
            omega_poly[j] = omega_poly_raw[j] / TIRE_RADIUS;

        // Set rig functions
        auto v_func = chrono_types::make_shared<ChFunctionPoly>();
        v_func->SetCoefficients(v_poly);

        auto omega_func = chrono_types::make_shared<ChFunctionPoly>();
        omega_func->SetCoefficients(omega_poly);

        auto slip_func = chrono_types::make_shared<ChFunctionPoly>();
        slip_func->SetCoefficients({0.0, a_a, b_a});

        rig.SetLongSpeedFunction(v_func);
        rig.SetAngSpeedFunction(omega_func);
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
            vis->SetWindowTitle("Rate Data - Sample " + std::to_string(sample_id));
            vis->Initialize();
            vis->AddLogo();
            vis->AddSkyBox();
            vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis->AddLightDirectional();
        }
#endif

#ifndef CHRONO_IRRLICHT
        (void)visualize;
#endif

        // Simulate and measure
        double t = 0;
        double t_measure_start = t_target - 0.5 * T_MEASURE;
        double t_measure_end   = t_target + 0.5 * T_MEASURE;
        double t_end = t_measure_end + T_TAIL;
        double t_next_measure = t_measure_start;

        double sum_Fx = 0, sum_Fy = 0, sum_Fz = 0;
        double sum_kappa = 0, sum_alpha = 0, sum_v = 0, sum_omega = 0;
        double first_kappa = 0, first_alpha = 0, first_v = 0, first_t = 0;
        double last_kappa = 0, last_alpha = 0, last_v = 0, last_t = 0;
        int count = 0;
        double render_step = 1.0 / 60.0;
        double next_render = 0;

        while (t < t_end) {
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

            // Average forces and actual state over measurement window
            if (t >= t_next_measure && t >= t_measure_start && t <= t_measure_end) {
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
                sum_omega += rig.GetAngSpeed();
                if (count == 0) { first_kappa = cur_kappa; first_alpha = cur_alpha; first_v = cur_v; first_t = t; }
                last_kappa = cur_kappa; last_alpha = cur_alpha; last_v = cur_v; last_t = t;
                count++;
                t_next_measure += MEASURE_DT;
            }
        }

        if (count > 0) {
            result.Fx_avg = sum_Fx / count;
            result.Fy_avg = sum_Fy / count;
            result.Fz_avg = sum_Fz / count;
            result.slip_ratio    = params.kappa_target;
            result.slip_angle    = params.alpha_target;
            result.velocity      = params.velocity_target;
            result.steering_rate = params.dalpha;
            result.d_slip_ratio  = 0.0;
            result.d_slip_angle  = 0.0;
            result.d_velocity    = 0.0;
            result.n_samples     = count;
            result.actual_kappa    = sum_kappa / count;
            result.actual_alpha    = sum_alpha / count;
            result.actual_velocity = sum_v / count;
            result.actual_omega    = sum_omega / count;
            double dt_window = last_t - first_t;
            if (dt_window > 1e-8) {
                result.actual_dkappa    = (last_kappa - first_kappa) / dt_window;
                result.actual_dalpha    = (last_alpha - first_alpha) / dt_window;
                result.actual_dvelocity = (last_v - first_v) / dt_window;
            } else {
                result.actual_dkappa = result.actual_dalpha = result.actual_dvelocity = 0;
            }
            // Rate channels in the CSV represent finite differences of the
            // measured states, matching how rate features are formed from
            // time-series data in training.
            result.d_slip_ratio = result.actual_dkappa;
            result.d_slip_angle = result.actual_dalpha;
            result.d_velocity   = result.actual_dvelocity;
            result.valid         = true;
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
// CSV header and write function (one row per scenario)
// =============================================================================
static const std::string CSV_HEADER =
    "slip_ratio,slip_angle,velocity,vertical_load,"
    "steering_rate,d_slip_ratio,d_slip_angle,d_velocity,"
    "bekker_Kphi,bekker_Kc,bekker_n,mohr_cohesion,"
    "mohr_friction,janosi_shear,mesh_spacing,Fz,Fx,Fy\n";

void WriteRateRecord(std::ofstream& csv, std::mutex& mtx,
                     const RateSampleParams& params,
                     const RateResult& result)
{
    if (!result.valid) return;
    auto finite = [](double v) { return std::isfinite(v); };
    // Guard against rare rig glitches producing pathological rows that can
    // dominate scaler statistics during NN training.
    if (!finite(result.actual_kappa) || !finite(result.actual_alpha) ||
        !finite(result.actual_velocity) || !finite(result.actual_dkappa) ||
        !finite(result.actual_dalpha) || !finite(result.actual_dvelocity) ||
        !finite(result.steering_rate) ||
        !finite(result.Fz_avg) || !finite(result.Fx_avg) || !finite(result.Fy_avg)) {
        return;
    }
    if (std::abs(result.actual_kappa) > 1.2 ||
        std::abs(result.actual_alpha) > 0.7 ||
        result.actual_velocity < 0.25 || result.actual_velocity > 20.0 ||
        std::abs(result.actual_dkappa) > 5.0 ||
        std::abs(result.actual_dalpha) > 2.0 ||
        std::abs(result.actual_dvelocity) > 10.0 ||
        std::abs(result.steering_rate) > 2.0 ||
        result.Fz_avg < 1000.0 || result.Fz_avg > 10000.0 ||
        std::abs(result.Fx_avg) > 5.0e4 || std::abs(result.Fy_avg) > 5.0e4) {
        return;
    }

    double mohr_friction_rad = params.mohr_friction * CH_DEG_TO_RAD;

    std::ostringstream buf;
    buf << std::fixed << std::setprecision(6);

    buf << result.actual_kappa << ","
        << result.actual_alpha << ","
        << result.actual_velocity << ","
        << params.vertical_load << ","
        << result.steering_rate << ","
        << result.d_slip_ratio << ","
        << result.d_slip_angle << ","
        << result.d_velocity << ","
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
// Process a batch of samples (for subprocess batching)
// =============================================================================
int ProcessBatch(const std::vector<RateSampleParams>& samples,
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
        auto res = CollectRateSample(samples[0], base_id);
        if (res.valid) {
            WriteRateRecord(csv, csv_mtx, samples[0], res);
            success++;
        }
    }

#ifdef CHRONO_OPENMP
    #pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 1; i < n; i++) {
        auto res = CollectRateSample(samples[i], base_id + i);
        if (res.valid) {
            WriteRateRecord(csv, csv_mtx, samples[i], res);
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
    std::cout << "\n=== Subprocess-Batched Rate-Aware Data Collection ===\n"
              << "Total samples: " << n_samples << "\n"
              << "Batch size: " << batch_size << "\n"
              << "One measurement per scenario (target value + rate)\n";

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

        std::cout << "\n--- Batch: samples " << bs << "-" << (be-1) << " ---\n";

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
                      << std::setprecision(2) << rate << " samp/s\n";
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
void CollectRateData(int n_samples, const std::string& output_file,
                     int num_threads, bool use_parallel, bool visualize)
{
    bool continuous = (n_samples <= 0);

    std::cout << "\n=== Rate-Aware SCM Data Collection ===\n"
              << "Samples: " << (continuous ? "CONTINUOUS (Ctrl+C to stop)" : std::to_string(n_samples)) << "\n"
              << "Step size: " << STEP_SIZE << "\n"
              << "One measurement per scenario at target (kappa*, alpha*, v*) with (dkappa, dalpha, dv)\n"
              << "Profiles: quadratic kappa(t), alpha(t) from 0; linear v(t)\n"
              << "Measurement window: " << T_MEASURE*1000 << "ms\n"
              << "Visualization: " << (visualize ? "ON" : "OFF") << "\n";

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

    // Verification print for visualization / sequential mode
    auto print_verify = [&](int sid, const RateSampleParams& p,
                            const RateResult& res) {
        if (!visualize || !res.valid)
            return;
        std::cout << "[verify] sample " << sid << "\n"
                  << "  prescribed:  kappa=" << std::fixed << std::setprecision(3)
                  << res.slip_ratio << "  alpha=" << res.slip_angle
                  << "  v=" << std::setprecision(1) << res.velocity << "\n"
                  << "  actual(rig): kappa=" << std::setprecision(3) << res.actual_kappa
                  << "  alpha=" << res.actual_alpha
                  << "  v=" << std::setprecision(1) << res.actual_velocity
                  << "  omega=" << std::setprecision(2) << res.actual_omega << "\n"
                  << "  error:       kappa=" << std::setprecision(4)
                  << (res.actual_kappa - res.slip_ratio)
                  << "  alpha=" << (res.actual_alpha - res.slip_angle)
                  << "  v=" << std::setprecision(3)
                  << (res.actual_velocity - res.velocity) << "\n"
                  << "  rates(cmd): dkappa=" << std::setprecision(3) << res.d_slip_ratio
                  << "  dalpha=" << res.d_slip_angle
                  << "  dv=" << res.d_velocity << "\n"
                  << "  rates(rig): dkappa=" << res.actual_dkappa
                  << "  dalpha=" << res.actual_dalpha
                  << "  dv=" << res.actual_dvelocity << "\n"
                  << "  forces: Fz=" << std::setprecision(0) << res.Fz_avg
                  << "  Fx=" << res.Fx_avg
                  << "  Fy=" << res.Fy_avg
                  << " (" << res.n_samples << " avg)\n";
    };

    if (continuous) {
        // --- Continuous mode ---
        // Warm-up
        {
            RateSampleParams warm{};
            warm.kappa_target = 0.1; warm.dkappa = 0.05;
            warm.alpha_target = 0.05; warm.dalpha = 0.1;
            warm.velocity_target = 5.0; warm.dvelocity = 0.5;
            warm.ramp_time = 2.0;
            warm.vertical_load = 4000;
            warm.bekker_Kphi = 2e6; warm.bekker_Kc = 5000; warm.bekker_n = 0.8;
            warm.mohr_cohesion = 5000; warm.mohr_friction = 20; warm.janosi_shear = 0.015;
            warm.mesh_spacing = 0.10;
            auto res = CollectRateSample(warm, 0, visualize);
            if (res.valid) { WriteRateRecord(csv, csv_mtx, warm, res); success++; }
            print_verify(0, warm, res);
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

                p.ramp_time = ranges.ramp_time_min + d(rng) * (ranges.ramp_time_max - ranges.ramp_time_min);
                double T = p.ramp_time;

                p.kappa_target = ranges.kappa_target_min + d(rng) * (ranges.kappa_target_max - ranges.kappa_target_min);
                p.dkappa = ranges.dkappa_min + d(rng) * (ranges.dkappa_max - ranges.dkappa_min);
                double b_k = (p.dkappa * T - p.kappa_target) / (T * T);
                double a_k = 2.0 * p.kappa_target / T - p.dkappa;
                if (!ProfileInBounds(a_k, b_k, T, KAPPA_LIMIT)) {
                    p.dkappa = p.kappa_target / T;
                }

                p.alpha_target = ranges.alpha_target_min + d(rng) * (ranges.alpha_target_max - ranges.alpha_target_min);
                p.dalpha = ranges.dalpha_min + d(rng) * (ranges.dalpha_max - ranges.dalpha_min);
                double b_a = (p.dalpha * T - p.alpha_target) / (T * T);
                double a_a = 2.0 * p.alpha_target / T - p.dalpha;
                if (!ProfileInBounds(a_a, b_a, T, ALPHA_LIMIT)) {
                    p.dalpha = p.alpha_target / T;
                }

                p.velocity_target = ranges.velocity_target_min + d(rng) * (ranges.velocity_target_max - ranges.velocity_target_min);
                p.dvelocity = ranges.dvelocity_min + d(rng) * (ranges.dvelocity_max - ranges.dvelocity_min);
                double v_start = p.velocity_target - p.dvelocity * T;
                if (v_start < V_MIN) {
                    v_start = V_MIN;
                    p.dvelocity = (p.velocity_target - v_start) / T;
                } else if (v_start > V_MAX) {
                    v_start = V_MAX;
                    p.dvelocity = (p.velocity_target - v_start) / T;
                }

                p.vertical_load = ranges.vertical_load_min + d(rng) * (ranges.vertical_load_max - ranges.vertical_load_min);
                p.bekker_Kphi   = ranges.bekker_Kphi_min + d(rng) * (ranges.bekker_Kphi_max - ranges.bekker_Kphi_min);
                p.bekker_Kc     = ranges.bekker_Kc_min + d(rng) * (ranges.bekker_Kc_max - ranges.bekker_Kc_min);
                p.bekker_n      = ranges.bekker_n_min + d(rng) * (ranges.bekker_n_max - ranges.bekker_n_min);
                p.mohr_cohesion = ranges.mohr_cohesion_min + d(rng) * (ranges.mohr_cohesion_max - ranges.mohr_cohesion_min);
                p.mohr_friction = ranges.mohr_friction_min + d(rng) * (ranges.mohr_friction_max - ranges.mohr_friction_min);
                p.janosi_shear  = ranges.janosi_shear_min + d(rng) * (ranges.janosi_shear_max - ranges.janosi_shear_min);
                p.mesh_spacing  = ranges.mesh_spacing_min + d(rng) * (ranges.mesh_spacing_max - ranges.mesh_spacing_min);
            }

#ifdef CHRONO_OPENMP
            #pragma omp parallel for schedule(dynamic)
#endif
            for (int i = 0; i < batch_sz; i++) {
                if (g_stop) continue;
                int sid = completed.load() + i;
                auto res = CollectRateSample(batch[i], sid, visualize);
                if (res.valid) {
                    WriteRateRecord(csv, csv_mtx, batch[i], res);
                    success++;
                    print_verify(sid, batch[i], res);
                }
                int done = ++completed;
                if (done % 10 == 0) {
#ifdef CHRONO_OPENMP
                    #pragma omp critical
#endif
                    {
                        auto now = std::chrono::high_resolution_clock::now();
                        double el = std::chrono::duration<double>(now - t_start).count();
                        std::cout << "Samples: " << done
                                  << " (ok: " << success.load() << ") "
                                  << std::fixed << std::setprecision(2) << (done/el) << " samp/s\n";
                    }
                }
            }
        }
    } else {
        // --- Fixed-count mode with LHS ---
        auto samples = GenerateLHSSamples(n_samples, ranges);

        // Warm-up
        {
            auto res = CollectRateSample(samples[0], 0, visualize);
            if (res.valid) { WriteRateRecord(csv, csv_mtx, samples[0], res); success++; }
            print_verify(0, samples[0], res);
            completed++;
        }

#ifdef CHRONO_OPENMP
        #pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 1; i < n_samples; i++) {
            if (g_stop) continue;
            auto res = CollectRateSample(samples[i], i, visualize);
            if (res.valid) {
                WriteRateRecord(csv, csv_mtx, samples[i], res);
                success++;
                print_verify(i, samples[i], res);
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
                              << " " << std::setprecision(2) << rate << " samp/s"
                              << " ETA: " << std::setprecision(0) << ((n_samples - done) / rate) << "s\n";
                }
            }
        }
    }

    csv.close();
    auto t_end_clk = std::chrono::high_resolution_clock::now();
    double total = std::chrono::duration<double>(t_end_clk - t_start).count();
    std::cout << "\n=== Collection Complete ===\n"
              << "Time: " << std::fixed << std::setprecision(1) << total << "s\n"
              << "Samples: " << success.load() << "/" << completed.load() << "\n"
              << "Output: " << output_file << "\n";
}

// =============================================================================
int main(int argc, char* argv[]) {
    int n_samples = 0;  // default: continuous
    std::string output_file = "scm_rate_data.csv";
    bool use_parallel = true;
    bool visualize = false;
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
        } else if (arg == "--visualize" || arg == "-v") {
            visualize = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [num_samples] [output.csv] [options]\n\n"
                      << "Collects rate-aware tire force data for NN training.\n"
                      << "Each scenario produces ONE measurement at a target operating\n"
                      << "point (kappa*, alpha*, v*) with target rates (dkappa/dt, dalpha/dt, dv/dt).\n\n"
                      << "Profiles (quadratic, starting from 0):\n"
                      << "  kappa(t) = a*t + b*t^2    reaching (kappa*, dkappa) at t_target\n"
                      << "  alpha(t) = a*t + b*t^2    reaching (alpha*, dalpha) at t_target\n"
                      << "  v(t) = v_start + dv*t     (linear, v(T)=v*)\n"
                      << "  omega(t) = v(t)/R*(1+kappa(t))  (derived)\n\n"
                      << "Options:\n"
                      << "  [no number]         Continuous mode (Ctrl+C to stop)\n"
                      << "  N                   Run N samples\n"
                      << "  --visualize, -v     Enable Irrlicht visualization\n"
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
        if (visualize) {
#ifndef CHRONO_IRRLICHT
            std::cerr << "Error: built without Irrlicht support. "
                      << "Rebuild with CHRONO_IRRLICHT enabled.\n";
            return 1;
#else
            use_parallel = false;
            num_threads = 1;
            if (batch_size > 0) {
                std::cout << "Visualization enabled: ignoring --batch-size.\n";
                batch_size = 0;
            }
#endif
        }

#ifdef __linux__
        if (batch_size > 0 && n_samples > 0) {
            CollectWithSubprocessBatching(n_samples, output_file, num_threads, use_parallel, batch_size);
            return 0;
        }
#endif

        CollectRateData(n_samples, output_file, num_threads, use_parallel, visualize);
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
