#include "ChRecordedDriver.h"

#include <algorithm>

namespace chrono
{
namespace hil
{

ChRecordedDriver::ChRecordedDriver(vehicle::ChVehicle &vehicle, std::vector<Sample> samples)
    : vehicle::ChDriver(vehicle), m_samples(std::move(samples))
{
  if (m_samples.empty())
  {
    Sample s;
    s.time = 0.0;
    s.inputs = vehicle::DriverInputs();
    m_samples.push_back(s);
  }
  std::sort(m_samples.begin(), m_samples.end(), [](const Sample &a, const Sample &b) { return a.time < b.time; });
  m_last_index = 0;
  const auto &first = m_samples.front();
  m_steering = first.inputs.m_steering;
  m_throttle = first.inputs.m_throttle;
  m_braking = first.inputs.m_braking;
  m_clutch = 0.0;
}

void ChRecordedDriver::Reset()
{
  m_last_index = 0;
  const auto &first = m_samples.front();
  m_steering = first.inputs.m_steering;
  m_throttle = first.inputs.m_throttle;
  m_braking = first.inputs.m_braking;
  m_clutch = 0.0;
}

void ChRecordedDriver::Synchronize(double time)
{
  if (m_samples.empty())
  {
    m_steering = 0.0;
    m_throttle = 0.0;
    m_braking = 0.0;
    return;
  }

  if (time <= m_samples.front().time)
  {
    const auto &inp = m_samples.front().inputs;
    m_steering = inp.m_steering;
    m_throttle = inp.m_throttle;
    m_braking = inp.m_braking;
    m_last_index = 0;
    return;
  }

  if (time >= m_samples.back().time)
  {
    const auto &inp = m_samples.back().inputs;
    m_steering = inp.m_steering;
    m_throttle = inp.m_throttle;
    m_braking = inp.m_braking;
    m_last_index = m_samples.size() - 1;
    return;
  }

  while (m_last_index + 1 < m_samples.size() && time >= m_samples[m_last_index + 1].time)
  {
    ++m_last_index;
  }
  while (m_last_index > 0 && time < m_samples[m_last_index].time)
  {
    --m_last_index;
  }

  const Sample &s0 = m_samples[m_last_index];
  const Sample &s1 = m_samples[m_last_index + 1];
  double dt = s1.time - s0.time;
  double alpha = (dt > 1e-8) ? (time - s0.time) / dt : 0.0;
  alpha = std::max(0.0, std::min(1.0, alpha));

  m_steering = s0.inputs.m_steering + alpha * (s1.inputs.m_steering - s0.inputs.m_steering);
  m_throttle = s0.inputs.m_throttle + alpha * (s1.inputs.m_throttle - s0.inputs.m_throttle);
  m_braking = s0.inputs.m_braking + alpha * (s1.inputs.m_braking - s0.inputs.m_braking);
}

void ChRecordedDriver::Advance(double step)
{
  (void)step;
}

} // namespace hil
} // namespace chrono
