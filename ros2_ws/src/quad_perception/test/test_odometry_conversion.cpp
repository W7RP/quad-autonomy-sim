#include <gtest/gtest.h>

#include <cmath>

#include "quad_perception/odometry_conversion.hpp"

using quad_perception::Px4Odometry;
using quad_perception::Quat;
using quad_perception::Vec3;

TEST(OdometryConversion, FacingEastFlyingNorthEast)
{
  // NED yaw 90 deg: facing east. World velocity: 1 m/s north, 2 m/s east, 0.5 down.
  Px4Odometry in;
  in.position_ned = {1.0, 2.0, -3.0};
  in.q_ned_frd = Quat(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));
  in.velocity = {1.0, 2.0, 0.5};
  in.velocity_frame = quad_perception::kVelocityFrameNed;
  in.position_variance_ned = {0.1, 0.2, 0.3};

  const auto out = quad_perception::to_ros(in);
  ASSERT_TRUE(out.has_value());
  EXPECT_LT((out->position_enu - Vec3(2.0, 1.0, 3.0)).norm(), 1e-12);
  // ENU yaw of a vehicle facing east is 0: body x (forward) = world +x (east).
  EXPECT_LT(((out->q_enu_flu * Vec3::UnitX()) - Vec3::UnitX()).norm(), 1e-12);
  // Body FLU: forward = east speed 2, left = north speed 1, up = -down = -0.5.
  EXPECT_LT((out->velocity_body_flu - Vec3(2.0, 1.0, -0.5)).norm(), 1e-12);
  EXPECT_LT((out->position_variance_enu - Vec3(0.2, 0.1, 0.3)).norm(), 1e-12);
}

TEST(OdometryConversion, BodyFrameVelocityMatchesWorldFrameVelocity)
{
  Px4Odometry a;
  a.q_ned_frd = Quat(Eigen::AngleAxisd(0.7, Vec3::UnitZ()) * Eigen::AngleAxisd(0.2, Vec3::UnitX()));
  a.velocity = {1.0, -0.5, 0.2};
  a.velocity_frame = quad_perception::kVelocityFrameNed;
  Px4Odometry b = a;
  b.velocity = a.q_ned_frd.conjugate() * a.velocity;
  b.velocity_frame = quad_perception::kVelocityFrameBodyFrd;
  EXPECT_LT((quad_perception::to_ros(a)->velocity_body_flu -
    quad_perception::to_ros(b)->velocity_body_flu).norm(), 1e-12);
}

TEST(OdometryConversion, RejectsUnusableInput)
{
  Px4Odometry in;
  in.velocity_frame = 2;  // FRD world frame with arbitrary heading: not handled
  EXPECT_FALSE(quad_perception::to_ros(in).has_value());
  in.velocity_frame = quad_perception::kVelocityFrameNed;
  in.position_ned.x() = std::nan("");
  EXPECT_FALSE(quad_perception::to_ros(in).has_value());
}
