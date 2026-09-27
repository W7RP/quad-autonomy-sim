// Static (coarse) alignment: while the vehicle sits still, average the IMU to
// get roll/pitch from gravity and the gyro bias, then yaw from the magnetometer.
// Restarts automatically if the vehicle moves during the averaging window.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "quad_estimation/eskf.hpp"

namespace quad_estimation
{

struct AlignmentConfig
{
  int samples{100};                  // IMU samples to average (~1.2 s at 83 Hz)
  double max_gyro_norm{0.05};        // [rad/s] above this we are not still
  double max_accel_dev{0.5};         // [m/s^2] | |a| - g | above this: not still
  double gravity{9.80665};
};

class StaticAlignment
{
public:
  explicit StaticAlignment(AlignmentConfig cfg) noexcept
  : cfg_(cfg) {}

  // Returns true when enough still samples have been collected.
  bool add_imu(const Vec3 & gyro, const Vec3 & accel) noexcept
  {
    const bool still = gyro.norm() < cfg_.max_gyro_norm &&
      std::abs(accel.norm() - cfg_.gravity) < cfg_.max_accel_dev;
    if (!still) {
      reset();
      ++restarts_;
      return false;
    }
    gyro_sum_ += gyro;
    accel_sum_ += accel;
    ++count_;
    return ready();
  }

  [[nodiscard]] bool ready() const noexcept {return count_ >= cfg_.samples;}
  [[nodiscard]] Vec3 gyro_bias() const noexcept {return gyro_sum_ / static_cast<double>(std::max(count_, 1));}

  // Attitude from the mean specific force (roll, pitch) and a magnetometer
  // vector (yaw). At rest in FRD the accelerometer reads -g along world down.
  [[nodiscard]] bool attitude(const Vec3 & mag_body, double declination, Quat & q_out) const noexcept
  {
    if (!ready()) {
      return false;
    }
    const Vec3 a = accel_sum_ / static_cast<double>(count_);
    const double roll = std::atan2(-a.y(), -a.z());
    const double pitch = std::atan2(a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
    const Quat tilt = Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX());
    double yaw = 0.0;
    if (!mag_heading(mag_body, tilt, declination, yaw)) {
      return false;
    }
    q_out = Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * tilt;
    return true;
  }

  void reset() noexcept
  {
    gyro_sum_.setZero();
    accel_sum_.setZero();
    count_ = 0;
  }
  [[nodiscard]] std::uint64_t restarts() const noexcept {return restarts_;}

private:
  AlignmentConfig cfg_;
  Vec3 gyro_sum_{Vec3::Zero()};
  Vec3 accel_sum_{Vec3::Zero()};
  int count_{0};
  std::uint64_t restarts_{0};
};

}  // namespace quad_estimation
