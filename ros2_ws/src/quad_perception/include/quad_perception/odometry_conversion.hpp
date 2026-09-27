// PX4 VehicleOdometry (NED world, FRD body) -> ROS REP-103 odometry (ENU world,
// FLU body). ROS-free so it can be unit-tested; the node only moves fields.
#pragma once

#include <optional>

#include "quad_estimation/frames.hpp"

namespace quad_perception
{

using quad_estimation::Quat;
using quad_estimation::Vec3;

struct Px4Odometry
{
  Vec3 position_ned{Vec3::Zero()};
  Quat q_ned_frd{Quat::Identity()};
  Vec3 velocity{Vec3::Zero()};
  int velocity_frame{1};           // px4_msgs VehicleOdometry::VELOCITY_FRAME_*
  Vec3 angular_velocity_frd{Vec3::Zero()};
  Vec3 position_variance_ned{Vec3::Zero()};
  Vec3 orientation_variance{Vec3::Zero()};  // body-axis, rad^2
  Vec3 velocity_variance{Vec3::Zero()};
};

struct RosOdometry
{
  Vec3 position_enu;
  Quat q_enu_flu;
  Vec3 velocity_body_flu;          // nav_msgs twist is in the child (body) frame
  Vec3 angular_velocity_flu;
  Vec3 position_variance_enu;
  Vec3 orientation_variance;
  Vec3 velocity_variance;
};

inline constexpr int kVelocityFrameNed = 1;
inline constexpr int kVelocityFrameBodyFrd = 3;

// std::nullopt if the input is unusable (non-finite, or a velocity frame this
// bridge does not handle).
[[nodiscard]] inline std::optional<RosOdometry> to_ros(const Px4Odometry & in) noexcept
{
  namespace f = quad_estimation::frames;
  if (!in.position_ned.allFinite() || !in.q_ned_frd.coeffs().allFinite() ||
    !in.velocity.allFinite() || in.q_ned_frd.norm() < 0.5)
  {
    return std::nullopt;
  }
  const Quat q = in.q_ned_frd.normalized();
  Vec3 v_body_frd;
  if (in.velocity_frame == kVelocityFrameNed) {
    v_body_frd = q.conjugate() * in.velocity;
  } else if (in.velocity_frame == kVelocityFrameBodyFrd) {
    v_body_frd = in.velocity;
  } else {
    return std::nullopt;
  }
  RosOdometry out;
  out.position_enu = f::ned_to_enu(in.position_ned);
  out.q_enu_flu = f::ned_frd_to_enu_flu(q);
  out.velocity_body_flu = f::frd_to_flu(v_body_frd);
  out.angular_velocity_flu = f::frd_to_flu(in.angular_velocity_frd);
  // Diagonal variances: NED -> ENU swaps x/y; FRD <-> FLU only flips signs.
  out.position_variance_enu = {in.position_variance_ned.y(), in.position_variance_ned.x(),
    in.position_variance_ned.z()};
  out.orientation_variance = in.orientation_variance;
  out.velocity_variance = in.velocity_variance;
  return out;
}

}  // namespace quad_perception
