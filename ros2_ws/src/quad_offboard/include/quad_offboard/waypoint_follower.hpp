// Pure C++20 waypoint-following logic for offboard velocity control.
//
// Deliberately free of ROS and PX4 types: the node owns I/O, this class owns the
// decision of "which velocity do I command now". That keeps it unit-testable
// without a simulator and reusable unchanged on a real companion computer.
//
// Frame convention: PX4 local NED (x north, y east, z down, metres). Callers
// convert from ENU/FLU before calling in if they ever need to.
//
// Real-time notes: all storage is fixed-size (std::array), no allocation after
// construction, and step() is O(1) with no branches that depend on history
// length. This is the pattern the Phase 2 estimator follows as well.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>

namespace quad_offboard
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

[[nodiscard]] double norm(const Vec3 & v) noexcept;

struct FollowerConfig
{
  double cruise_speed_mps{2.0};        // horizontal speed cap
  double vertical_speed_mps{1.0};      // vertical speed cap
  double position_gain{0.8};           // P gain [1/s]: velocity = gain * position error
  double acceptance_radius_m{0.3};     // waypoint reached when closer than this...
  double acceptance_speed_mps{0.3};    // ...and slower than this
};

struct VelocityCommand
{
  Vec3 velocity_ned;  // m/s
  double yaw_rad;     // heading to hold
};

class WaypointFollower
{
public:
  static constexpr std::size_t kMaxWaypoints = 64;

  explicit WaypointFollower(FollowerConfig config) noexcept;

  // Replace the route. Returns false (and leaves the route untouched) when the
  // input is empty or longer than kMaxWaypoints.
  bool set_route(std::span<const Vec3> waypoints_ned, double yaw_rad) noexcept;

  // Advance to the next waypoint if the current one is reached, then return the
  // velocity command toward it. std::nullopt once the route is finished.
  [[nodiscard]] std::optional<VelocityCommand> step(
    const Vec3 & position_ned, const Vec3 & velocity_ned) noexcept;

  [[nodiscard]] bool finished() const noexcept {return active_ >= count_;}
  [[nodiscard]] std::size_t active_index() const noexcept {return active_;}
  [[nodiscard]] std::size_t size() const noexcept {return count_;}

private:
  FollowerConfig config_;
  std::array<Vec3, kMaxWaypoints> route_{};
  std::size_t count_{0};
  std::size_t active_{0};
  double yaw_rad_{0.0};
};

// Closed square of side `side_m` at `altitude_m` above the start point, with
// its first corner at `origin_ned`. Includes the return to the first corner.
[[nodiscard]] std::array<Vec3, 5> make_square(
  const Vec3 & origin_ned, double side_m, double altitude_m) noexcept;

}  // namespace quad_offboard
