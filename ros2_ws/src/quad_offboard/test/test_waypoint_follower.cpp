#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "quad_offboard/waypoint_follower.hpp"

using quad_offboard::FollowerConfig;
using quad_offboard::make_square;
using quad_offboard::Vec3;
using quad_offboard::WaypointFollower;

TEST(MakeSquare, ClosedAndAboveOrigin)
{
  const auto sq = make_square({1.0, 2.0, -0.1}, 5.0, 3.0);
  EXPECT_DOUBLE_EQ(sq.front().x, sq.back().x);
  EXPECT_DOUBLE_EQ(sq.front().y, sq.back().y);
  for (const auto & p : sq) {
    EXPECT_DOUBLE_EQ(p.z, -3.1);  // NED: 3 m above a start at z=-0.1
  }
  EXPECT_DOUBLE_EQ(sq[2].x, 6.0);
  EXPECT_DOUBLE_EQ(sq[2].y, 7.0);
}

TEST(WaypointFollower, RejectsEmptyAndOversizedRoutes)
{
  WaypointFollower f{FollowerConfig{}};
  EXPECT_FALSE(f.set_route({}, 0.0));
  std::vector<Vec3> too_long(WaypointFollower::kMaxWaypoints + 1);
  EXPECT_FALSE(f.set_route(too_long, 0.0));
  EXPECT_TRUE(f.finished());
}

TEST(WaypointFollower, SaturatesHorizontalAndVerticalSeparately)
{
  FollowerConfig cfg;
  cfg.cruise_speed_mps = 2.0;
  cfg.vertical_speed_mps = 1.0;
  cfg.position_gain = 10.0;
  WaypointFollower f{cfg};
  const Vec3 wp{100.0, 100.0, -50.0};
  ASSERT_TRUE(f.set_route({&wp, 1}, 0.5));

  const auto cmd = f.step({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(std::hypot(cmd->velocity_ned.x, cmd->velocity_ned.y), 2.0, 1e-9);
  EXPECT_NEAR(cmd->velocity_ned.x, cmd->velocity_ned.y, 1e-9);  // direction preserved
  EXPECT_DOUBLE_EQ(cmd->velocity_ned.z, -1.0);                   // climbing = negative z
  EXPECT_DOUBLE_EQ(cmd->yaw_rad, 0.5);
}

TEST(WaypointFollower, RequiresLowSpeedToAcceptWaypoint)
{
  WaypointFollower f{FollowerConfig{}};
  const std::vector<Vec3> route{{0.0, 0.0, -3.0}, {5.0, 0.0, -3.0}};
  ASSERT_TRUE(f.set_route(route, 0.0));

  // On the waypoint but still moving fast: not accepted yet.
  (void)f.step({0.0, 0.0, -3.0}, {2.0, 0.0, 0.0});
  EXPECT_EQ(f.active_index(), 0U);

  (void)f.step({0.0, 0.0, -3.0}, {0.0, 0.0, 0.0});
  EXPECT_EQ(f.active_index(), 1U);
}

// Integrate a perfect velocity-tracking vehicle through the full square and
// check the follower terminates.
TEST(WaypointFollower, FliesSquareToCompletion)
{
  WaypointFollower f{FollowerConfig{}};
  const auto sq = make_square({0.0, 0.0, 0.0}, 5.0, 3.0);
  ASSERT_TRUE(f.set_route(sq, 0.0));

  Vec3 pos{};
  Vec3 vel{};
  constexpr double dt = 0.05;
  int steps = 0;
  while (steps < 10000) {
    const auto cmd = f.step(pos, vel);
    if (!cmd) {
      break;
    }
    vel = cmd->velocity_ned;
    pos.x += vel.x * dt;
    pos.y += vel.y * dt;
    pos.z += vel.z * dt;
    ++steps;
  }
  EXPECT_TRUE(f.finished());
  EXPECT_LT(steps, 10000);
  // Finished means the last corner (the start, at altitude) was accepted.
  const Vec3 err{pos.x - 0.0, pos.y - 0.0, pos.z + 3.0};
  EXPECT_LT(quad_offboard::norm(err), FollowerConfig{}.acceptance_radius_m);
}

TEST(WaypointFollower, PerWaypointYaw)
{
  WaypointFollower f{FollowerConfig{}};
  const std::vector<Vec3> route{{0.0, 0.0, -2.0}, {5.0, 0.0, -2.0}};
  const std::vector<double> yaws{0.1, 0.7};
  ASSERT_FALSE(f.set_route(route, std::vector<double>{0.1}));  // length mismatch
  ASSERT_TRUE(f.set_route(route, yaws));
  EXPECT_DOUBLE_EQ(f.step({0.0, 0.0, 0.0}, {}).value().yaw_rad, 0.1);
  (void)f.step({0.0, 0.0, -2.0}, {});  // reach the first waypoint
  EXPECT_DOUBLE_EQ(f.step({0.0, 0.0, -2.0}, {}).value().yaw_rad, 0.7);
}

TEST(TravelYaws, FacesEachLegAndHoldsThroughClimb)
{
  // climb, then east, then north, then west (NED: x north, y east)
  const std::vector<Vec3> route{{0, 0, -2}, {0, 10, -2}, {10, 10, -2}, {10, 0, -2}};
  std::vector<double> yaws(route.size());
  quad_offboard::travel_yaws(route, 1.0, yaws);
  EXPECT_DOUBLE_EQ(yaws[0], 1.0);                 // climb keeps the initial heading
  EXPECT_NEAR(yaws[1], M_PI / 2, 1e-12);          // east
  EXPECT_NEAR(yaws[2], 0.0, 1e-12);               // north
  EXPECT_NEAR(std::abs(yaws[3]), M_PI / 2, 1e-12);
  EXPECT_LT(yaws[3], 0.0);                        // west
}

TEST(StepYaw, RateLimitedShortestWay)
{
  using quad_offboard::step_yaw;
  EXPECT_DOUBLE_EQ(step_yaw(0.0, 1.0, 0.0), 1.0);             // unlimited: jump
  EXPECT_NEAR(step_yaw(0.0, 1.0, 0.1), 0.1, 1e-12);            // limited
  EXPECT_NEAR(step_yaw(0.95, 1.0, 0.1), 1.0, 1e-12);           // no overshoot
  // 170 deg -> -170 deg is a 20 deg turn through 180, not 340 the long way.
  const double from = M_PI * 170 / 180, to = -M_PI * 170 / 180;
  EXPECT_NEAR(step_yaw(from, to, 0.1), std::remainder(from + 0.1, 2 * M_PI), 1e-12);
  double y = from;
  for (int i = 0; i < 10; ++i) {
    y = step_yaw(y, to, 0.1);
  }
  EXPECT_NEAR(y, to, 1e-12);
}
