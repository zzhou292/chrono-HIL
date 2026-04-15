// verify_slip_angle.cpp
// Visual verification: ramp slip angle to target, then hold visualization.
// Usage: verify_slip_angle [degrees...]
// Default: 10 20 30 40 50

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionPoly.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace chrono;
using namespace chrono::vehicle;

constexpr double STEP_SIZE = 5e-4;
constexpr double TIRE_RADIUS = 0.47;
constexpr double T_DELAY = 1.0;
constexpr double RAMP_RATE = 0.3;  // rad/s — slow enough to watch
constexpr double VELOCITY = 5.0;
constexpr double KAPPA = 0.0;       // no longitudinal slip
constexpr double FZ = 5000.0;

void RunAngleTest(double target_deg) {
    double target_rad = target_deg * CH_DEG_TO_RAD;
    double ramp_time = std::abs(target_rad) / RAMP_RATE;
    double t_target = T_DELAY + ramp_time;
    double t_hold = t_target + 3.0;  // hold 3s after reaching target

    std::cout << "\n========================================\n"
              << "Target slip angle: " << target_deg << " deg ("
              << std::fixed << std::setprecision(3) << target_rad << " rad)\n"
              << "Ramp rate: " << RAMP_RATE << " rad/s\n"
              << "Ramp time: " << std::setprecision(1) << ramp_time << " s\n"
              << "Total sim: " << t_hold << " s\n"
              << "========================================\n";

    auto sys = std::make_unique<ChSystemNSC>();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys->GetSolver()->AsIterative()->SetMaxIterations(50);

    std::string data_path = GetChronoDataPath();
    auto wheel = ReadWheelJSON(data_path + "vehicle/hmmwv/wheel/HMMWV_Wheel.json");
    auto tire = ReadTireJSON(data_path + "vehicle/hmmwv/tire/HMMWV_RigidTire.json");
    tire->SetStepsize(STEP_SIZE);

    ChTireTestRig rig(wheel, tire, sys.get());
    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(FZ);
    rig.SetCamberAngle(0.0);
    rig.SetTireStepsize(STEP_SIZE);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    // SCM terrain (matches data collectors)
    ChTireTestRig::TerrainParamsSCM scm;
    scm.length        = 200.0;
    scm.width         = 1.0;
    scm.Bekker_Kphi   = 2e6;
    scm.Bekker_Kc     = 5000;
    scm.Bekker_n      = 0.8;
    scm.Mohr_cohesion = 5000;
    scm.Mohr_friction = 20;
    scm.Janosi_shear  = 0.015;
    scm.grid_spacing  = 0.10;
    rig.SetTerrainSCM(scm);

    // Constant speed, zero longitudinal slip
    double omega = (VELOCITY / TIRE_RADIUS) * (1.0 + KAPPA);
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(VELOCITY));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(omega));

    // Ramp-and-hold: linear ramp to target_rad over ramp_time, then hold constant.
    // Note: the rig wraps this with DelayedFun, evaluating f(max(0, t - T_DELAY)).
    // So we define f(x) where x = t - T_DELAY (effective time after delay).
    auto slip_func = chrono_types::make_shared<ChFunctionInterp>();
    slip_func->AddPoint(0.0, 0.0);                  // start at 0
    slip_func->AddPoint(ramp_time, target_rad);      // ramp to target
    slip_func->AddPoint(ramp_time + 100.0, target_rad);  // hold forever
    rig.SetSlipAngleFunction(slip_func);
    rig.SetTimeDelay(T_DELAY);
    rig.Initialize(ChTireTestRig::Mode::TEST);

#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(sys.get());
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("Slip Angle Verify: " + std::to_string((int)target_deg) + " deg");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
    vis->AddLightDirectional();
#endif

    double t = 0;
    double render_step = 1.0 / 60.0;
    double next_render = 0;
    bool printed_target = false;
    bool printed_hold = false;

    while (t < t_hold) {
#ifdef CHRONO_IRRLICHT
        if (t >= next_render) {
            auto& loc = rig.GetPos();
            // Camera behind and above, looking at tire
            vis->UpdateCamera(loc + ChVector3d(-1.0, 3.0, 1.5), loc);
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();

            // Draw HUD text
            auto device = vis->GetDevice();
            auto font = device->getGUIEnvironment()->getBuiltInFont();
            if (font) {
                double alpha_now = rig.GetSlipAngle();
                double alpha_deg = alpha_now * CH_RAD_TO_DEG;
                bool at_target = (t >= t_target);

                std::string line1 = "Target: " + std::to_string((int)target_deg) + " deg";
                std::string line2 = "Actual: " + std::to_string((int)std::round(alpha_deg)) + " deg ("
                    + std::to_string(alpha_deg).substr(0, 5) + " deg)";
                std::string line3 = at_target ? ">>> AT TARGET — HOLDING <<<" : "Ramping...";
                std::string line4 = "t = " + std::to_string(t).substr(0, 4) + " s";
                std::string line5 = "Kappa(rig) = " + std::to_string(rig.GetLongitudinalSlip()).substr(0, 6);

                irr::core::recti pos1(10, 10, 400, 30);
                irr::core::recti pos2(10, 30, 400, 50);
                irr::core::recti pos3(10, 50, 400, 70);
                irr::core::recti pos4(10, 70, 400, 90);
                irr::core::recti pos5(10, 90, 400, 110);

                auto white = irr::video::SColor(255, 255, 255, 255);
                auto green = irr::video::SColor(255, 0, 255, 0);

                font->draw(irr::core::stringw(line1.c_str()).c_str(), pos1, white);
                font->draw(irr::core::stringw(line2.c_str()).c_str(), pos2, white);
                font->draw(irr::core::stringw(line3.c_str()).c_str(), pos3, at_target ? green : white);
                font->draw(irr::core::stringw(line4.c_str()).c_str(), pos4, white);
                font->draw(irr::core::stringw(line5.c_str()).c_str(), pos5, white);
            }

            vis->EndScene();
            next_render += render_step;
        }
#endif

        rig.Advance(STEP_SIZE);
        t += STEP_SIZE;

        // Print when we first reach the target
        if (!printed_target && t >= t_target) {
            double alpha_actual = rig.GetSlipAngle() * CH_RAD_TO_DEG;
            double kappa_actual = rig.GetLongitudinalSlip();
            auto force = rig.ReportTireForce();
            std::cout << ">>> Target reached at t=" << std::setprecision(2) << t << "s\n"
                      << "    Commanded alpha: " << std::setprecision(1) << target_deg << " deg\n"
                      << "    Actual alpha:    " << std::setprecision(2) << alpha_actual << " deg\n"
                      << "    Actual kappa:    " << std::setprecision(4) << kappa_actual << "\n"
                      << "    Fx=" << std::setprecision(0) << force.force.x()
                      << " Fy=" << force.force.y()
                      << " Fz=" << force.force.z() << "\n";
            printed_target = true;
        }

        if (!printed_hold && t >= t_target + 1.0) {
            std::cout << "    (holding for visual inspection — close window to continue)\n";
            printed_hold = true;
        }
    }

    // Keep window open until user closes it
#ifdef CHRONO_IRRLICHT
    std::cout << ">>> Holding display — close window or press Q to move to next angle\n";
    while (vis->Run()) {
        auto& loc = rig.GetPos();
        vis->UpdateCamera(loc + ChVector3d(-1.0, 3.0, 1.5), loc);
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }
#endif

    sys->Clear();
    sys.reset();
}

int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);

    std::vector<double> angles;
    if (argc > 1) {
        for (int i = 1; i < argc; i++)
            angles.push_back(std::atof(argv[i]));
    } else {
        angles = {10, 20, 30, 40, 50};
    }

    std::cout << "Slip Angle Visual Verification\n"
              << "Angles to test:";
    for (auto a : angles) std::cout << " " << a << "°";
    std::cout << "\n\nClose each window to advance to the next angle.\n";

#ifndef CHRONO_IRRLICHT
    std::cerr << "ERROR: This tool requires CHRONO_IRRLICHT.\n";
    return 1;
#endif

    for (auto deg : angles) {
        RunAngleTest(deg);
    }

    std::cout << "\n=== All angles verified ===\n";
    return 0;
}
