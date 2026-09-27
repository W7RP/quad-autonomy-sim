// Filter-core tests. Compiled with EIGEN_RUNTIME_NO_MALLOC (see CMakeLists.txt),
// so Eigen aborts on any heap allocation while the trap is armed.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include <Eigen/Eigenvalues>

#include "quad_estimation/eskf.hpp"

using namespace quad_estimation;

namespace
{

NominalState random_state(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  NominalState x;
  x.p = Vec3(3.0 * u(rng), 3.0 * u(rng), -1.5 + 0.5 * u(rng));   // 1..2 m above ground_z=0
  x.v = Vec3(2.0 * u(rng), 2.0 * u(rng), 0.5 * u(rng));
  // Tilted up to ~20 deg, any yaw.
  x.q = Quat(Eigen::AngleAxisd(3.0 * u(rng), Vec3::UnitZ()) *
    Eigen::AngleAxisd(0.35 * u(rng), Vec3::UnitY()) *
    Eigen::AngleAxisd(0.35 * u(rng), Vec3::UnitX()));
  x.bg = Vec3(0.01 * u(rng), 0.01 * u(rng), 0.01 * u(rng));
  x.ba = Vec3(0.1 * u(rng), 0.1 * u(rng), 0.1 * u(rng));
  return x;
}

// Central differences of a measurement function over the error state.
template<int M, typename F>
Eigen::Matrix<double, M, kErrDim> numeric_jacobian(const NominalState & x, F && h)
{
  constexpr double eps = 1e-6;
  Eigen::Matrix<double, M, kErrDim> J;
  for (int i = 0; i < kErrDim; ++i) {
    ErrVec d = ErrVec::Zero();
    d[i] = eps;
    J.col(i) = (h(retract(x, d)) - h(retract(x, -d))) / (2.0 * eps);
  }
  return J;
}

}  // namespace

TEST(Jacobians, RangeMatchesFiniteDifferences)
{
  std::mt19937 rng(1);
  for (int trial = 0; trial < 50; ++trial) {
    const NominalState x = random_state(rng);
    const auto analytic = range_model(x, 0.0).H;
    const auto numeric = numeric_jacobian<1>(x, [](const NominalState & s) {
          return Eigen::Matrix<double, 1, 1>(range_model(s, 0.0).predicted);
        });
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-6) << "trial " << trial;
  }
}

TEST(Jacobians, FlowMatchesFiniteDifferences)
{
  std::mt19937 rng(2);
  for (int trial = 0; trial < 50; ++trial) {
    const NominalState x = random_state(rng);
    const auto analytic = flow_model(x, 0.0).H;
    const auto numeric = numeric_jacobian<2>(x, [](const NominalState & s) {
          return flow_model(s, 0.0).predicted;
        });
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-5) << "trial " << trial;
  }
}

TEST(Jacobians, HeadingMatchesFiniteDifferences)
{
  std::mt19937 rng(3);
  for (int trial = 0; trial < 50; ++trial) {
    const NominalState x = random_state(rng);
    const double y0 = heading_model(x).predicted;
    const auto analytic = heading_model(x).H;
    // Unwrap around y0 so +-pi crossings don't produce a 2*pi "derivative".
    const auto numeric = numeric_jacobian<1>(x, [y0](const NominalState & s) {
          return Eigen::Matrix<double, 1, 1>(y0 + wrap_pi(heading_model(s).predicted - y0));
        });
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-6) << "trial " << trial;
  }
}

TEST(Math, QuatExpSmallAndLarge)
{
  const Vec3 small(1e-12, -2e-12, 3e-12);
  EXPECT_NEAR(quat_exp(small).angularDistance(Quat::Identity()), small.norm(), 1e-15);
  const Vec3 big(0.3, -0.2, 0.9);
  EXPECT_NEAR(quat_exp(big).angularDistance(Quat::Identity()), big.norm(), 1e-12);
  EXPECT_NEAR(std::abs(wrap_pi(3.0 * M_PI)), M_PI, 1e-12);  // +pi and -pi are the same angle
  EXPECT_NEAR(wrap_pi(-0.5), -0.5, 1e-15);
}

TEST(MagHeading, RecoversYawUnderTilt)
{
  const Vec3 field_ned(0.21, 0.0, 0.43);  // no declination in the field itself
  for (double yaw : {-2.5, -0.4, 0.0, 1.2, 3.0}) {
    const Quat q(Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(0.2, Vec3::UnitY()) *
      Eigen::AngleAxisd(-0.15, Vec3::UnitX()));
    const Vec3 mag_body = q.conjugate() * field_ned;
    double yaw_out = 0.0;
    ASSERT_TRUE(mag_heading(mag_body, q, 0.0, yaw_out));
    EXPECT_NEAR(wrap_pi(yaw_out - yaw), 0.0, 1e-9);
    ASSERT_TRUE(mag_heading(mag_body, q, 0.1, yaw_out));
    EXPECT_NEAR(wrap_pi(yaw_out - yaw - 0.1), 0.0, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// Synthetic flight with exactly known truth.

namespace
{
struct Truth
{
  Vec3 p, v, a;   // NED
  Quat q;
  Vec3 w;         // body rate
};

// Smooth 3D path: circle of radius 3 m plus a slow altitude oscillation, with a
// constant yaw rate and a gently oscillating roll/pitch.
Truth truth_at(double t)
{
  constexpr double r = 3.0, om = 0.4, h0 = 2.0;
  Truth s;
  s.p = Vec3(r * std::sin(om * t), r * (1.0 - std::cos(om * t)), -h0 + 0.5 * std::sin(0.3 * t)) -
    Vec3(0.0, 0.0, -h0);
  s.v = Vec3(r * om * std::cos(om * t), r * om * std::sin(om * t), 0.15 * std::cos(0.3 * t));
  s.a = Vec3(-r * om * om * std::sin(om * t), r * om * om * std::cos(om * t),
    -0.045 * std::sin(0.3 * t));
  const double yaw = 0.2 * t;
  const double roll = 0.1 * std::sin(0.7 * t);
  const double pitch = 0.1 * std::cos(0.5 * t);
  s.q = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
    Eigen::AngleAxisd(roll, Vec3::UnitX()));
  return s;
}

Vec3 body_rate(double t)
{
  // Numerical: w = Log(q(t)^-1 q(t+h)) / h  (small h, central).
  constexpr double h = 1e-5;
  const Quat dq = truth_at(t - h).q.conjugate() * truth_at(t + h).q;
  const Eigen::AngleAxisd aa(dq);
  return aa.axis() * aa.angle() / (2.0 * h);
}
}  // namespace

TEST(SyntheticFlight, TracksTruthAndEstimatesGyroBias)
{
  EskfConfig cfg;
  Eskf f(cfg);
  std::mt19937 rng(42);
  std::normal_distribution<double> n(0.0, 1.0);

  const Vec3 true_bg(0.004, -0.003, 0.002);
  const Vec3 field_ned(0.21, 0.0, 0.43);
  constexpr double h0 = 2.0;       // IMU height above ground at t=0
  constexpr double dt = 0.012;     // matches the PX4 SITL IMU interval
  const double sq_dt = std::sqrt(dt);

  // Initialise with the true attitude but zero bias knowledge.
  f.initialize(truth_at(0.0).q, Vec3::Zero(), h0);

  double sum_nees = 0.0;
  int nees_n = 0;
  double sq_vel_err = 0.0, sq_tilt_err = 0.0, sq_yaw_err = 0.0, tilt_var_reported = 0.0;
  int err_n = 0;
  Vec3 flow_gyro_acc = Vec3::Zero();   // measured gyro over the flow window
  Vec3 true_rate_acc = Vec3::Zero();   // true body rate over the same window

  Eigen::internal::set_is_malloc_allowed(false);  // trap any Eigen allocation
  for (int k = 1; k <= static_cast<int>(60.0 / dt); ++k) {
    const double t = k * dt;
    const double tm = t - 0.5 * dt;  // interval midpoint ~ interval average
    const Truth s = truth_at(tm);
    const Vec3 w = body_rate(tm);
    const Vec3 f_body = s.q.conjugate() * (s.a - Vec3(0.0, 0.0, cfg.gravity));
    const Vec3 gyro_m = w + true_bg + Vec3(n(rng), n(rng), n(rng)) * cfg.gyro_noise / sq_dt;
    const Vec3 accel_m = f_body + Vec3(n(rng), n(rng), n(rng)) * cfg.accel_noise / sq_dt;
    f.predict(gyro_m, accel_m, dt);
    flow_gyro_acc += gyro_m;
    true_rate_acc += w;

    const Truth now = truth_at(t);
    const Eigen::Matrix3d R = now.q.toRotationMatrix();
    const double hagl = h0 - now.p.z();
    if (k % 4 == 0) {  // range + flow at ~20 Hz (flow window = last 4 IMU samples)
      (void)f.update_range(hagl / R(2, 2) + 0.02 * n(rng));
      const Vec3 vb = R.transpose() * now.v;
      const Vec2 w_xy = (true_rate_acc / 4.0).head<2>();
      const Vec2 flow_rate = w_xy - (R(2, 2) / hagl) * Vec2(vb.y(), -vb.x()) +
        Vec2(n(rng), n(rng)) * 0.05;
      (void)f.update_flow(flow_rate, (flow_gyro_acc / 4.0).head<2>(), 1.0);
      flow_gyro_acc.setZero();
      true_rate_acc.setZero();
    }
    if (k % 7 == 0) {
      (void)f.update_heading(now.q.conjugate() * field_ned + Vec3(n(rng), n(rng), n(rng)) * 0.002);
    }

    if (t > 10.0) {  // after convergence
      const NominalState & x = f.state();
      ErrVec e = ErrVec::Zero();
      e.segment<3>(kV) = now.v - x.v;
      const Quat dq = x.q.conjugate() * now.q;
      const Eigen::AngleAxisd aa(dq);
      e.segment<3>(kTh) = aa.axis() * aa.angle();
      // NEES over velocity + attitude (6 dof).
      Eigen::Matrix<double, 6, 1> ev;
      ev << e.segment<3>(kV), e.segment<3>(kTh);
      Eigen::Matrix<double, 6, 6> Pv;
      Pv << f.covariance().block<3, 3>(kV, kV), f.covariance().block<3, 3>(kV, kTh),
        f.covariance().block<3, 3>(kTh, kV), f.covariance().block<3, 3>(kTh, kTh);
      sum_nees += ev.dot(Pv.ldlt().solve(ev));
      ++nees_n;
      sq_vel_err += e.segment<3>(kV).squaredNorm();
      // Split the attitude error into tilt (roll/pitch) and yaw in the world frame:
      // they are observed by different sensors (gravity+flow vs magnetometer).
      const Vec3 e_world = x.q * Vec3(e.segment<3>(kTh));
      sq_tilt_err += e_world.head<2>().squaredNorm();
      sq_yaw_err += e_world.z() * e_world.z();
      const Mat3 R_est = x.q.toRotationMatrix();
      const Mat3 P_th_world = R_est * f.covariance().block<3, 3>(kTh, kTh) * R_est.transpose();
      tilt_var_reported += P_th_world(0, 0) + P_th_world(1, 1);
      ++err_n;
    }
  }
  Eigen::internal::set_is_malloc_allowed(true);

  const double vel_rmse = std::sqrt(sq_vel_err / err_n);
  const double tilt_rmse_deg = std::sqrt(sq_tilt_err / err_n) * 180.0 / M_PI;
  const double yaw_rmse_deg = std::sqrt(sq_yaw_err / err_n) * 180.0 / M_PI;
  const double tilt_sigma_deg = std::sqrt(tilt_var_reported / err_n) * 180.0 / M_PI;
  const double mean_nees = sum_nees / nees_n;
  std::printf("synthetic: vel RMSE %.3f m/s, tilt RMSE %.3f deg, yaw RMSE %.3f deg, "
    "(filter's own tilt sigma %.3f deg), mean NEES %.2f (6 dof), bg est %.4f %.4f %.4f vs true %.4f %.4f %.4f\n", vel_rmse,
    tilt_rmse_deg, yaw_rmse_deg, tilt_sigma_deg, mean_nees,
    f.state().bg.x(), f.state().bg.y(), f.state().bg.z(), true_bg.x(), true_bg.y(), true_bg.z());

  EXPECT_LT(vel_rmse, 0.10);
  // Tilt is limited by the tilt / horizontal-accel-bias ambiguity (only broken
  // as the vehicle yaws). What matters most is that the filter knows it:
  // actual error must not exceed what the covariance claims.
  EXPECT_LT(tilt_rmse_deg, 1.0);
  EXPECT_LT(tilt_rmse_deg, 1.5 * tilt_sigma_deg);
  EXPECT_LT(yaw_rmse_deg, 2.0);  // magnetometer heading noise is 0.05 rad (2.9 deg) per sample
  EXPECT_LT((f.state().bg - true_bg).norm(), 0.0015);
  // Consistency: a well-tuned filter has mean NEES ~ dof (6). Accept a broad
  // band: the synthetic IMU uses midpoint sampling, a small model mismatch.
  EXPECT_GT(mean_nees, 1.0);
  EXPECT_LT(mean_nees, 18.0);
}

TEST(Updates, OutlierIsGatedAndStateUntouched)
{
  Eskf f{EskfConfig{}};
  f.initialize(Quat::Identity(), Vec3::Zero(), 2.0);
  const NominalState before = f.state();
  const auto r = f.update_range(10.0);  // predicted 2.0 m, sigma ~0.05 m
  EXPECT_FALSE(r.accepted);
  EXPECT_GT(r.nis, EskfConfig{}.gate_1dof);
  EXPECT_EQ(f.state().p, before.p);
}

TEST(Updates, SkippedWhenTooTiltedOrTooLow)
{
  Eskf f{EskfConfig{}};
  f.initialize(Quat(Eigen::AngleAxisd(0.9, Vec3::UnitX())), Vec3::Zero(), 2.0);  // ~52 deg
  EXPECT_FALSE(f.update_range(2.0).accepted);
  EXPECT_EQ(f.update_range(2.0).nis, 0.0);  // skipped, not gated

  Eskf g{EskfConfig{}};
  g.initialize(Quat::Identity(), Vec3::Zero(), 0.05);  // below min_hagl
  EXPECT_EQ(g.update_flow(Vec2::Zero(), Vec2::Zero(), 1.0).nis, 0.0);
}

TEST(Covariance, StaysSymmetricPositiveDefinite)
{
  Eskf f{EskfConfig{}};
  f.initialize(Quat::Identity(), Vec3::Zero(), 1.0);
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 20000; ++k) {  // ~4 min at 83 Hz, hovering
    f.predict(Vec3::Zero(), Vec3(0.0, 0.0, -9.80665), 0.012);
    if (k % 4 == 0) {
      (void)f.update_range(1.0);
      (void)f.update_flow(Vec2::Zero(), Vec2::Zero(), 1.0);
    }
    if (k % 7 == 0) {
      (void)f.update_heading(Vec3(0.21, 0.0, 0.43));
    }
  }
  Eigen::internal::set_is_malloc_allowed(true);
  const Cov & P = f.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Cov> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  EXPECT_LT(f.state().v.norm(), 1e-3);
}
