
#ifndef CH_ROM_PFDRIVER_H
#define CH_ROM_PFDRIVER_H

#include "../../ChApiHil.h"
#include "../veh/Ch_8DOF_vehicle.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include <string>

namespace chrono {
namespace hil {

class CH_HIL_API ChROM_PathFollowerDriver {

public:
  ChROM_PathFollowerDriver(std::shared_ptr<Ch_8DOF_vehicle> m_rom,
                           std::shared_ptr<ChBezierCurve> curve,
                           double target_speed, double look_ahead_dist,
                           double PID_st_kp, double PID_st_ki, double PID_st_kd,
                           double PID_sp_kp, double PID_sp_ki,
                           double PID_sp_kd);

  void Advance(double step);
  DriverInputs GetDriverInput();
  void SetCruiseSpeed(double target_speed);

private:
  std::shared_ptr<ChBezierCurve> m_curve;
  std::shared_ptr<ChBezierCurveTracker> m_tracker;
  std::shared_ptr<Ch_8DOF_vehicle> m_rom;

  // target spped
  double m_target_speed;

  // steering PID control
  double m_st_kp;
  double m_st_ki;
  double m_st_kd;

  // speed PID control
  double m_sp_kp;
  double m_sp_ki;
  double m_sp_kd;

  // steering errs
  double m_st_err;   // cached error
  double m_st_err_d; // d_err
  double m_st_err_i; // i_err

  // speeed errs
  double m_sp_err;   // cached error
  double m_sp_err_d; // d_err
  double m_sp_err_i; // i_err

  // look ahead distance
  double m_dist;

  DriverInputs m_inputs;
};

} // namespace hil
} // namespace chrono

#endif
