// PX4 (NED world, FRD body) <-> ROS REP-103 (ENU world, FLU body) conversions.
// Conversions happen only at the boundary: the filter itself is NED/FRD.
#pragma once

#include "quad_estimation/eskf.hpp"

namespace quad_estimation::frames
{

// NED vector -> ENU vector (x_e = y_n, y_e = x_n, z_e = -z_n).
[[nodiscard]] inline Vec3 ned_to_enu(const Vec3 & v) noexcept {return {v.y(), v.x(), -v.z()};}

// FRD vector -> FLU vector (and back: the map is its own inverse).
[[nodiscard]] inline Vec3 frd_to_flu(const Vec3 & v) noexcept {return {v.x(), -v.y(), -v.z()};}

// Attitude body(FRD)->NED  ==>  body(FLU)->ENU:  q_enu_flu = Q_ENU_NED * q * Q_FRD_FLU.
[[nodiscard]] inline Quat ned_frd_to_enu_flu(const Quat & q_ned_frd) noexcept
{
  // Both constant maps are 180-degree rotations:
  //   NED->ENU about (1,1,0)/sqrt(2);  FLU->FRD about x.
  static const Quat kEnuNed(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  static const Quat kFrdFlu(0.0, 1.0, 0.0, 0.0);
  return (kEnuNed * q_ned_frd * kFrdFlu).normalized();
}

}  // namespace quad_estimation::frames
