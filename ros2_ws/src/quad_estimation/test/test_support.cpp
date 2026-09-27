#include <gtest/gtest.h>

#include <cmath>

#include "quad_estimation/fixed_ring.hpp"
#include "quad_estimation/frames.hpp"
#include "quad_estimation/imu_history.hpp"
#include "quad_estimation/static_alignment.hpp"
#include "quad_estimation/timing_stats.hpp"

using namespace quad_estimation;

TEST(FixedRing, FifoOrderAndOverwriteAccounting)
{
  FixedRing<int, 3> r;
  EXPECT_TRUE(r.empty());
  for (int i = 1; i <= 5; ++i) {
    r.push(i);
  }
  EXPECT_EQ(r.size(), 3U);
  EXPECT_EQ(r.overwritten(), 2U);
  EXPECT_EQ(r.front(), 3);
  EXPECT_EQ(r[2], 5);
  r.pop_front();
  EXPECT_EQ(r.front(), 4);
  r.clear();
  EXPECT_TRUE(r.empty());
  r.pop_front();  // safe on empty
  EXPECT_TRUE(r.empty());
}

TEST(GyroAverage, TimeWeightedOverPartialOverlap)
{
  GyroHistory h;
  // Three 10 ms samples ending at 10, 20, 30 ms with rates 1, 2, 3 rad/s on x.
  for (int i = 1; i <= 3; ++i) {
    h.push({i * 10'000, 10'000, Vec3(i, 0.0, 0.0)});
  }
  Vec3 m;
  ASSERT_TRUE(average_gyro(h, 5'000, 25'000, 0.8, m));  // 5 ms of 1, 10 of 2, 5 of 3
  EXPECT_NEAR(m.x(), (5 * 1 + 10 * 2 + 5 * 3) / 20.0, 1e-12);
  EXPECT_FALSE(average_gyro(h, 25'000, 60'000, 0.8, m));  // mostly in the future
}

TEST(Frames, ConsistentWithVectorMaps)
{
  const Quat q_ned_frd(Eigen::AngleAxisd(0.7, Vec3::UnitZ()) *
    Eigen::AngleAxisd(0.2, Vec3::UnitY()) * Eigen::AngleAxisd(-0.1, Vec3::UnitX()));
  const Quat q_enu_flu = frames::ned_frd_to_enu_flu(q_ned_frd);
  const Vec3 u_flu(0.3, -1.2, 0.5);
  // Rotating a FLU vector to ENU must equal: FLU->FRD, rotate to NED, NED->ENU.
  const Vec3 direct = q_enu_flu * u_flu;
  const Vec3 via = frames::ned_to_enu(q_ned_frd * frames::frd_to_flu(u_flu));
  EXPECT_LT((direct - via).norm(), 1e-12);
  // Level, facing north in NED == facing +y (north) in ENU: yaw 90 deg.
  const Quat north = frames::ned_frd_to_enu_flu(Quat::Identity());
  EXPECT_LT(((north * Vec3::UnitX()) - Vec3::UnitY()).norm(), 1e-12);
}

TEST(StaticAlignment, LevelsAndHeadsFromStillData)
{
  StaticAlignment a{AlignmentConfig{}};
  const Quat truth(Eigen::AngleAxisd(1.0, Vec3::UnitZ()) *
    Eigen::AngleAxisd(0.05, Vec3::UnitY()) * Eigen::AngleAxisd(-0.03, Vec3::UnitX()));
  const Vec3 f_body = truth.conjugate() * Vec3(0.0, 0.0, -9.80665);
  const Vec3 bias(0.002, -0.001, 0.003);
  bool ready = false;
  for (int i = 0; i < 100; ++i) {
    ready = a.add_imu(bias, f_body);
  }
  ASSERT_TRUE(ready);
  EXPECT_LT((a.gyro_bias() - bias).norm(), 1e-12);
  Quat q;
  ASSERT_TRUE(a.attitude(truth.conjugate() * Vec3(0.21, 0.0, 0.43), 0.0, q));
  EXPECT_LT(q.angularDistance(truth), 1e-9);

  // Motion restarts the averaging.
  EXPECT_FALSE(a.add_imu(Vec3(0.5, 0.0, 0.0), f_body));
  EXPECT_FALSE(a.ready());
  EXPECT_EQ(a.restarts(), 1U);
}

TEST(TimingStats, PercentileAndOverruns)
{
  TimingStats s(100);
  for (int i = 0; i < 99; ++i) {
    s.record(20);
  }
  s.record(500);
  const auto snap = s.snapshot();
  EXPECT_EQ(snap.count, 100U);
  EXPECT_EQ(snap.max_us, 500);
  EXPECT_EQ(snap.overruns, 1U);
  EXPECT_EQ(snap.p99_us, 25);  // 99 of 100 samples sit in the 20..25 us bin
}

// ---------------------------------------------------------------------------
// EstimatorPipeline: alignment and timeline robustness (the Phase 1 timesync
// glitch: one absurd stamp, then a backward clock step).

#include "quad_estimation/params.hpp"
#include "quad_estimation/pipeline.hpp"

namespace
{
constexpr std::int64_t kT0 = 1'790'000'000'000'000;  // ~epoch-like PX4 time, as seen live
constexpr std::int64_t kDt = 12'000;

ImuInput still_imu(std::int64_t t)
{
  return {t, kDt, Vec3(0.001, -0.002, 0.0005), Vec3(0.0, 0.0, -9.80665)};
}

// Feed still data until the pipeline has aligned; returns the next IMU time.
std::int64_t align(EstimatorPipeline & p)
{
  std::int64_t t = kT0;
  p.on_mag({t, Vec3(0.21, 0.0, 0.43)});
  p.on_range({t, 0.2});
  for (int i = 0; i < 200 && !p.filter().initialized(); ++i) {
    t += kDt;
    p.on_imu(still_imu(t));
  }
  return t + kDt;
}
}  // namespace

TEST(Pipeline, AlignsFromStillDataThenPredicts)
{
  PipelineConfig cfg;
  EstimatorPipeline p(cfg);
  const std::int64_t t = align(p);
  ASSERT_TRUE(p.filter().initialized());
  EXPECT_EQ(p.counters().initializations.load(), 1U);
  EXPECT_NEAR(p.filter().ground_z(), 0.2, 1e-9);
  EXPECT_LT((p.filter().state().bg - Vec3(0.001, -0.002, 0.0005)).norm(), 1e-9);
  // Level, facing magnetic north (declination 0).
  EXPECT_LT(p.filter().state().q.angularDistance(Quat::Identity()), 1e-6);
  EXPECT_TRUE(p.on_imu(still_imu(t)));
}

TEST(Pipeline, IgnoresOneOffGlitch)
{
  EstimatorPipeline p{PipelineConfig{}};
  const std::int64_t t = align(p);
  p.on_imu(still_imu(t));
  // Raw boot time (36.58 s) instead of synced time, then back to normal.
  EXPECT_TRUE(p.on_imu(still_imu(36'580'000)));
  EXPECT_EQ(p.time_us(), t + kDt);             // stayed on the timeline
  EXPECT_TRUE(p.on_imu(still_imu(t + 2 * kDt)));
  EXPECT_EQ(p.time_us(), t + 2 * kDt);
  EXPECT_EQ(p.counters().suspect_stamps.load(), 1U);
  EXPECT_EQ(p.counters().clock_steps.load(), 0U);
}

TEST(Pipeline, ReanchorsOnConfirmedStepsBothDirections)
{
  for (const std::int64_t step : {6'427'000LL, -310'000LL}) {  // both seen in SITL
    EstimatorPipeline p{PipelineConfig{}};
    const std::int64_t t = align(p);
    p.on_imu(still_imu(t));
    p.on_imu(still_imu(t + kDt + step));       // suspect: held on the old timeline
    EXPECT_EQ(p.time_us(), t + kDt);
    p.on_imu(still_imu(t + 2 * kDt + step));   // confirmed: re-anchor
    EXPECT_EQ(p.time_us(), t + 2 * kDt + step);
    EXPECT_EQ(p.counters().clock_steps.load(), 1U);
    // Measurements on the new timeline are accepted again.
    p.on_range({t + 2 * kDt + step, 0.2});
    p.on_imu(still_imu(t + 3 * kDt + step));
    EXPECT_EQ(p.counters().range.fused.load(), 1U);
    EXPECT_LT(p.filter().state().v.norm(), 1e-3);
  }
}

TEST(Pipeline, HoldsSampleOverMissingImuInterval)
{
  EstimatorPipeline p{PipelineConfig{}};
  std::int64_t t = align(p);
  p.on_imu(still_imu(t));
  EXPECT_TRUE(p.on_imu(still_imu(t + 2 * kDt)));  // one interval missing
  EXPECT_EQ(p.counters().imu_gaps.load(), 1U);
}

TEST(Params, UnitsAndNamesRoundTrip)
{
  PipelineConfig c;
  EXPECT_TRUE(set_pipeline_param(c, "mag_declination_deg", -2.56));
  EXPECT_NEAR(c.eskf.mag_declination, -2.56 * M_PI / 180.0, 1e-12);
  EXPECT_TRUE(set_pipeline_param(c, "max_measurement_delay_s", 0.25));
  EXPECT_EQ(c.max_delay_us, 250'000);
  EXPECT_TRUE(set_pipeline_param(c, "gravity", 9.8));
  EXPECT_DOUBLE_EQ(c.alignment.gravity, 9.8);
  EXPECT_FALSE(set_pipeline_param(c, "no_such_param", 1.0));
  for (const ParamSpec & s : pipeline_params()) {
    const double v = s.get(c);
    EXPECT_TRUE(set_pipeline_param(c, s.name, v)) << s.name;
    EXPECT_NEAR(s.get(c), v, 1e-9) << s.name;
  }
}
