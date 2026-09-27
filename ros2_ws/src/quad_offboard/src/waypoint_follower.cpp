#include "quad_offboard/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>

namespace quad_offboard
{

double norm(const Vec3 & v) noexcept
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

WaypointFollower::WaypointFollower(FollowerConfig config) noexcept
: config_(config) {}

bool WaypointFollower::set_route(std::span<const Vec3> waypoints_ned, double yaw_rad) noexcept
{
  if (waypoints_ned.empty() || waypoints_ned.size() > kMaxWaypoints) {
    return false;
  }
  std::copy(waypoints_ned.begin(), waypoints_ned.end(), route_.begin());
  std::fill(yaw_.begin(), yaw_.begin() + static_cast<std::ptrdiff_t>(waypoints_ned.size()), yaw_rad);
  count_ = waypoints_ned.size();
  active_ = 0;
  return true;
}

bool WaypointFollower::set_route(
  std::span<const Vec3> waypoints_ned, std::span<const double> yaws_rad) noexcept
{
  if (waypoints_ned.empty() || waypoints_ned.size() > kMaxWaypoints ||
    yaws_rad.size() != waypoints_ned.size())
  {
    return false;
  }
  std::copy(waypoints_ned.begin(), waypoints_ned.end(), route_.begin());
  std::copy(yaws_rad.begin(), yaws_rad.end(), yaw_.begin());
  count_ = waypoints_ned.size();
  active_ = 0;
  return true;
}

std::optional<VelocityCommand> WaypointFollower::step(
  const Vec3 & position_ned, const Vec3 & velocity_ned) noexcept
{
  // At most one waypoint advance per step keeps the call O(1) and makes the
  // transition visible in logs, at the cost of one control period of latency.
  if (!finished()) {
    const Vec3 & wp = route_[active_];
    const Vec3 err{wp.x - position_ned.x, wp.y - position_ned.y, wp.z - position_ned.z};
    if (norm(err) < config_.acceptance_radius_m &&
      norm(velocity_ned) < config_.acceptance_speed_mps)
    {
      ++active_;
    }
  }
  if (finished()) {
    return std::nullopt;
  }

  const Vec3 & wp = route_[active_];
  Vec3 v{
    config_.position_gain * (wp.x - position_ned.x),
    config_.position_gain * (wp.y - position_ned.y),
    config_.position_gain * (wp.z - position_ned.z)};

  // Saturate horizontal speed as a vector (preserves direction), vertical separately:
  // multirotors have very different horizontal and vertical authority.
  const double v_xy = std::hypot(v.x, v.y);
  if (v_xy > config_.cruise_speed_mps) {
    const double s = config_.cruise_speed_mps / v_xy;
    v.x *= s;
    v.y *= s;
  }
  v.z = std::clamp(v.z, -config_.vertical_speed_mps, config_.vertical_speed_mps);

  return VelocityCommand{v, yaw_[active_]};
}

double step_yaw(double current, double target, double max_step) noexcept
{
  const double diff = std::remainder(target - current, 2.0 * M_PI);  // shortest, in [-pi, pi]
  const double step = (max_step <= 0.0) ? diff : std::clamp(diff, -max_step, max_step);
  return std::remainder(current + step, 2.0 * M_PI);
}

void travel_yaws(
  std::span<const Vec3> route_ned, double initial_yaw_rad, std::span<double> yaws_out,
  double min_leg_m) noexcept
{
  double yaw = initial_yaw_rad;
  for (std::size_t i = 0; i < route_ned.size() && i < yaws_out.size(); ++i) {
    if (i > 0) {
      const double dn = route_ned[i].x - route_ned[i - 1].x;
      const double de = route_ned[i].y - route_ned[i - 1].y;
      if (std::hypot(dn, de) >= min_leg_m) {
        yaw = std::atan2(de, dn);  // NED: heading measured from north towards east
      }
    }
    yaws_out[i] = yaw;
  }
}

std::array<Vec3, 5> make_square(const Vec3 & origin_ned, double side_m, double altitude_m) noexcept
{
  const double z = origin_ned.z - altitude_m;  // NED: up is negative z
  const double x0 = origin_ned.x;
  const double y0 = origin_ned.y;
  return {{
    {x0, y0, z},
    {x0 + side_m, y0, z},
    {x0 + side_m, y0 + side_m, z},
    {x0, y0 + side_m, z},
    {x0, y0, z},
  }};
}

}  // namespace quad_offboard
