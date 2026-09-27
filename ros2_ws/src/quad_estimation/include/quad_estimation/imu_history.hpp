// Short history of raw gyro samples, used to average the body rate over an
// optical-flow integration window (the flow sensor reports no gyro of its own).
#pragma once

#include <algorithm>
#include <cstdint>

#include "quad_estimation/eskf.hpp"
#include "quad_estimation/fixed_ring.hpp"

namespace quad_estimation
{

struct GyroSample
{
  std::int64_t t_end_us{0};   // end of the integration interval
  std::int64_t dt_us{0};      // interval length
  Vec3 gyro{Vec3::Zero()};    // average rate over the interval [rad/s]
};

// 64 samples ~ 0.77 s at 83 Hz: comfortably longer than flow latency + window.
using GyroHistory = FixedRing<GyroSample, 64>;

// Time-weighted mean of the gyro over [t0_us, t1_us]. Returns false unless the
// history covers at least `min_coverage` of the window. O(capacity), bounded.
[[nodiscard]] inline bool average_gyro(
  const GyroHistory & hist, std::int64_t t0_us, std::int64_t t1_us, double min_coverage,
  Vec3 & mean_out) noexcept
{
  if (t1_us <= t0_us) {
    return false;
  }
  Vec3 sum = Vec3::Zero();
  std::int64_t covered = 0;
  for (std::size_t i = 0; i < hist.size(); ++i) {
    const GyroSample & s = hist[i];
    const std::int64_t a = std::max(s.t_end_us - s.dt_us, t0_us);
    const std::int64_t b = std::min(s.t_end_us, t1_us);
    if (b > a) {
      sum += s.gyro * static_cast<double>(b - a);
      covered += b - a;
    }
  }
  if (static_cast<double>(covered) < min_coverage * static_cast<double>(t1_us - t0_us)) {
    return false;
  }
  mean_out = sum / static_cast<double>(covered);
  return true;
}

}  // namespace quad_estimation
