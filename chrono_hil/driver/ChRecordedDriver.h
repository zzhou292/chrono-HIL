#pragma once

#include <vector>

#include "../ChApiHil.h"
#include "chrono_vehicle/ChDriver.h"

namespace chrono
{
namespace hil
{

/// Driver that replays recorded steering/throttle/brake inputs as a function of time.
class CH_HIL_API ChRecordedDriver : public vehicle::ChDriver
{
public:
  struct Sample
  {
    double time;
    vehicle::DriverInputs inputs;
  };

  ChRecordedDriver(vehicle::ChVehicle &vehicle, std::vector<Sample> samples);

  void Reset();
  void Synchronize(double time) override;
  void Advance(double step) override;

  const std::vector<Sample> &GetSamples() const { return m_samples; }

private:
  std::vector<Sample> m_samples;
  size_t m_last_index = 0;
};

} // namespace hil
} // namespace chrono
