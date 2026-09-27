#include "quad_estimation/eskf.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace quad_estimation
{

// ---------------------------------------------------------------------------
// Helpers

Mat3 skew(const Vec3 & w) noexcept
{
  Mat3 m;
  m << 0.0, -w.z(), w.y(),
    w.z(), 0.0, -w.x(),
    -w.y(), w.x(), 0.0;
  return m;
}

Quat quat_exp(const Vec3 & rv) noexcept
{
  const double angle = rv.norm();
  if (angle < 1e-9) {
    // First-order: exact enough at this size and avoids dividing by ~0.
    return Quat(1.0, 0.5 * rv.x(), 0.5 * rv.y(), 0.5 * rv.z()).normalized();
  }
  const Vec3 axis = rv / angle;
  return Quat(Eigen::AngleAxisd(angle, axis));
}

double wrap_pi(double a) noexcept
{
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  a = std::fmod(a + std::numbers::pi, kTwoPi);
  if (a < 0.0) {
    a += kTwoPi;
  }
  return a - std::numbers::pi;
}

NominalState retract(const NominalState & x, const ErrVec & dx) noexcept
{
  NominalState out = x;
  out.p += dx.segment<3>(kP);
  out.v += dx.segment<3>(kV);
  out.q = (x.q * quat_exp(dx.segment<3>(kTh))).normalized();
  out.bg += dx.segment<3>(kBg);
  out.ba += dx.segment<3>(kBa);
  return out;
}

// ---------------------------------------------------------------------------
// Measurement models
//
// Perturbation identities used below, for a body-frame attitude error
// R' = R (I + [dth]x):
//   d(R22)       = R20 * dth_y - R21 * dth_x
//   d(R^T v)     = [R^T v]x dth           (v held fixed)
// Both are checked numerically in test/test_eskf.cpp.

RangeModel range_model(const NominalState & x, double ground_z) noexcept
{
  const Mat3 R = x.q.toRotationMatrix();
  const double r22 = R(2, 2);
  const double h = ground_z - x.p.z();

  RangeModel m;
  m.predicted = h / r22;
  m.H.setZero();
  m.H(0, kP + 2) = -1.0 / r22;
  const double d_r22 = -h / (r22 * r22);  // d(predicted)/d(R22)
  m.H(0, kTh + 0) = d_r22 * -R(2, 1);
  m.H(0, kTh + 1) = d_r22 * R(2, 0);
  return m;
}

FlowModel flow_model(const NominalState & x, double ground_z) noexcept
{
  const Mat3 R = x.q.toRotationMatrix();
  const Vec3 vb = R.transpose() * x.v;
  const double h = ground_z - x.p.z();
  const double r22 = R(2, 2);
  const double s = r22 / h;
  const Vec2 base(vb.y(), -vb.x());  // flow per unit scale

  FlowModel m;
  m.predicted = s * base + x.bg.head<2>();
  m.H.setZero();

  // Velocity: vb = R^T v -> rows 1 and 0 of R^T are columns 1 and 0 of R.
  m.H.block<1, 3>(0, kV) = s * R.col(1).transpose();
  m.H.block<1, 3>(1, kV) = -s * R.col(0).transpose();

  // Attitude: through vb ([vb]x) and through the scale s (R22).
  const Mat3 vbx = skew(vb);
  Eigen::Matrix<double, 1, 3> d_r22;
  d_r22 << -R(2, 1), R(2, 0), 0.0;
  m.H.block<1, 3>(0, kTh) = s * vbx.row(1) + base.x() / h * d_r22;
  m.H.block<1, 3>(1, kTh) = -s * vbx.row(0) + base.y() / h * d_r22;

  // Height: s = R22 / (ground_z - p_z)  ->  ds/dp_z = R22 / h^2.
  m.H(0, kP + 2) = base.x() * r22 / (h * h);
  m.H(1, kP + 2) = base.y() * r22 / (h * h);

  // Gyro bias (x, y).
  m.H(0, kBg + 0) = 1.0;
  m.H(1, kBg + 1) = 1.0;
  return m;
}

HeadingModel heading_model(const NominalState & x) noexcept
{
  const Mat3 R = x.q.toRotationMatrix();
  HeadingModel m;
  m.predicted = std::atan2(R(1, 0), R(0, 0));
  m.H.setZero();
  // yaw = atan2(R10, R00) with R' = R (I + [dth]x):
  //   dR00 = R01 dth_z - R02 dth_y,  dR10 = R11 dth_z - R12 dth_y
  const double den = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
  m.H(0, kTh + 1) = (R(1, 0) * R(0, 2) - R(0, 0) * R(1, 2)) / den;
  m.H(0, kTh + 2) = (R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1)) / den;
  return m;
}

bool mag_heading(const Vec3 & mag_body, const Quat & q, double declination, double & yaw_out) noexcept
{
  // Level the measurement with roll/pitch only: rotate to world, then undo yaw.
  const Mat3 R = q.toRotationMatrix();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Vec3 m_world = R * mag_body;
  const Vec3 m_level = Eigen::AngleAxisd(-yaw, Vec3::UnitZ()) * m_world;
  if (m_level.head<2>().norm() < 0.05 * mag_body.norm()) {
    return false;  // field nearly vertical here: heading unobservable
  }
  yaw_out = wrap_pi(std::atan2(-m_level.y(), m_level.x()) + declination);
  return true;
}

// ---------------------------------------------------------------------------
// Filter

Eskf::Eskf(const EskfConfig & config) noexcept
: cfg_(config) {}

void Eskf::initialize(const Quat & attitude, const Vec3 & gyro_bias, double ground_z) noexcept
{
  x_ = NominalState{};
  x_.q = attitude.normalized();
  x_.bg = gyro_bias;
  ground_z_ = ground_z;

  ErrVec sigma;
  sigma << Vec3::Constant(cfg_.init_pos_sigma),
    Vec3::Constant(cfg_.init_vel_sigma),
    cfg_.init_tilt_sigma, cfg_.init_tilt_sigma, cfg_.init_yaw_sigma,
    Vec3::Constant(cfg_.init_gyro_bias_sigma),
    Vec3::Constant(cfg_.init_accel_bias_sigma);
  P_ = sigma.array().square().matrix().asDiagonal();
  initialized_ = true;
}

void Eskf::predict(const Vec3 & gyro, const Vec3 & accel, double dt) noexcept
{
  if (!initialized_ || !(dt > 0.0)) {
    return;
  }
  const Vec3 w = gyro - x_.bg;
  const Vec3 a = accel - x_.ba;
  const Mat3 R = x_.q.toRotationMatrix();
  const Vec3 acc_world = R * a + Vec3(0.0, 0.0, cfg_.gravity);
  const Quat dq = quat_exp(w * dt);

  // Nominal state (first-order in dt; at ~83-250 Hz the error is far below the
  // IMU noise).
  x_.p += x_.v * dt + 0.5 * acc_world * dt * dt;
  x_.v += acc_world * dt;
  x_.q = (x_.q * dq).normalized();

  // Error-state transition F (Sola, "Quaternion kinematics for the ESKF", 5.4).
  Cov F = Cov::Identity();
  F.block<3, 3>(kP, kV) = Mat3::Identity() * dt;
  F.block<3, 3>(kV, kTh) = -R * skew(a) * dt;
  F.block<3, 3>(kV, kBa) = -R * dt;
  F.block<3, 3>(kTh, kTh) = dq.toRotationMatrix().transpose();
  F.block<3, 3>(kTh, kBg) = -Mat3::Identity() * dt;

  // Discrete process noise from the continuous densities.
  ErrVec q_diag;
  q_diag << Vec3::Zero(),
    Vec3::Constant(cfg_.accel_noise * cfg_.accel_noise * dt),
    Vec3::Constant(cfg_.gyro_noise * cfg_.gyro_noise * dt),
    Vec3::Constant(cfg_.gyro_bias_walk * cfg_.gyro_bias_walk * dt),
    Vec3::Constant(cfg_.accel_bias_walk * cfg_.accel_bias_walk * dt);

  Cov FP;
  FP.noalias() = F * P_;
  P_.noalias() = FP * F.transpose();
  P_.diagonal() += q_diag;
  P_ = 0.5 * (P_ + P_.transpose());  // keep it exactly symmetric
}

bool Eskf::geometry_ok() const noexcept
{
  const double r22 = x_.q.toRotationMatrix()(2, 2);
  return r22 > cfg_.max_tilt_cos && hagl() > cfg_.min_hagl;
}

UpdateResult Eskf::update_range(double range_m) noexcept
{
  if (!initialized_ || !geometry_ok() || !std::isfinite(range_m)) {
    return {};
  }
  const RangeModel m = range_model(x_, ground_z_);
  Eigen::Matrix<double, 1, 1> y;
  y << range_m - m.predicted;
  Eigen::Matrix<double, 1, 1> R;
  R << cfg_.range_noise * cfg_.range_noise;
  return fuse<1>(y, m.H, R, cfg_.gate_1dof);
}

UpdateResult Eskf::update_flow(const Vec2 & pixel_flow_rate, const Vec2 & gyro_xy, double quality) noexcept
{
  if (!initialized_ || !geometry_ok() || !pixel_flow_rate.allFinite() || !gyro_xy.allFinite()) {
    return {};
  }
  const FlowModel m = flow_model(x_, ground_z_);
  const Vec2 z = gyro_xy - pixel_flow_rate;
  const Vec2 y = z - m.predicted;
  const double q = std::clamp(quality, 0.0, 1.0);
  const double sigma = cfg_.flow_noise_worst - q * (cfg_.flow_noise_worst - cfg_.flow_noise_best);
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma * sigma;
  return fuse<2>(y, m.H, R, cfg_.gate_2dof);
}

UpdateResult Eskf::update_heading(const Vec3 & mag_body) noexcept
{
  if (!initialized_ || !mag_body.allFinite()) {
    return {};
  }
  double yaw_meas = 0.0;
  if (!mag_heading(mag_body, x_.q, cfg_.mag_declination, yaw_meas)) {
    return {};
  }
  const HeadingModel m = heading_model(x_);
  Eigen::Matrix<double, 1, 1> y;
  y << wrap_pi(yaw_meas - m.predicted);
  Eigen::Matrix<double, 1, 1> R;
  R << cfg_.heading_noise * cfg_.heading_noise;
  return fuse<1>(y, m.H, R, cfg_.gate_1dof);
}

template<int M>
UpdateResult Eskf::fuse(
  const Eigen::Matrix<double, M, 1> & y,
  const Eigen::Matrix<double, M, kErrDim> & H,
  const Eigen::Matrix<double, M, M> & R,
  double gate) noexcept
{
  const Eigen::Matrix<double, kErrDim, M> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, M, M> S = H * PHt + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();  // M <= 2: closed form
  const double nis = y.dot(S_inv * y);
  if (!std::isfinite(nis) || nis > gate) {
    return {false, nis};
  }
  const Eigen::Matrix<double, kErrDim, M> K = PHt * S_inv;
  const ErrVec dx = K * y;

  // Joseph form: stays symmetric positive semi-definite despite rounding.
  const Cov I_KH = Cov::Identity() - K * H;
  Cov tmp;
  tmp.noalias() = I_KH * P_;
  P_.noalias() = tmp * I_KH.transpose();
  P_.noalias() += K * R * K.transpose();

  // Inject the error into the nominal state, then reset the error to zero.
  // The reset Jacobian G = I - [dth/2]x on the attitude block keeps P
  // consistent with the re-centred error definition.
  x_ = retract(x_, dx);
  Cov G = Cov::Identity();
  G.block<3, 3>(kTh, kTh) -= 0.5 * skew(dx.segment<3>(kTh));
  tmp.noalias() = G * P_;
  P_.noalias() = tmp * G.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  return {true, nis};
}

template UpdateResult Eskf::fuse<1>(
  const Eigen::Matrix<double, 1, 1> &, const Eigen::Matrix<double, 1, kErrDim> &,
  const Eigen::Matrix<double, 1, 1> &, double) noexcept;
template UpdateResult Eskf::fuse<2>(
  const Eigen::Matrix<double, 2, 1> &, const Eigen::Matrix<double, 2, kErrDim> &,
  const Eigen::Matrix<double, 2, 2> &, double) noexcept;

}  // namespace quad_estimation
