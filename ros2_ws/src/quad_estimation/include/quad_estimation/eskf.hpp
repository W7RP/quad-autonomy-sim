// Error-state extended Kalman filter (ESKF) for a multirotor: IMU-driven
// prediction, fused with a downward rangefinder, downward optical flow and
// magnetometer heading.
//
// ROS-free on purpose: the node owns threads, timestamps and message I/O; this
// class only does the math, so it can be unit-tested (including analytic vs
// numeric Jacobians) and reused on a companion computer unchanged.
//
// Conventions
//   World frame: local NED (x north, y east, z down), origin = IMU position at
//                initialisation. Body frame: FRD (PX4 convention).
//   q rotates body -> world (Hamilton, Eigen). R = q.toRotationMatrix().
//   Error state (15): [dp(3) dv(3) dtheta(3) dbg(3) dba(3)], with dtheta a
//   BODY-frame (right-multiplied) rotation error: q_true = q * Exp(dtheta).
//   Ground: flat, at world z = ground_z (set at initialisation from the range
//   reading). Terrain following is a Phase 3 concern.
//
// Real-time properties (see docs/phase2_state_estimation.md)
//   * Every matrix is fixed-size Eigen: no heap allocation, ever. The unit
//     tests run predict/update with Eigen's runtime malloc check armed.
//   * predict() and every update are O(1): fixed 15x15 algebra, no loops that
//     depend on history length or data. Worst-case cost is the cost.
//   * All public methods are noexcept and never throw or log.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quad_estimation
{

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

inline constexpr int kErrDim = 15;
using ErrVec = Eigen::Matrix<double, kErrDim, 1>;
using Cov = Eigen::Matrix<double, kErrDim, kErrDim>;

// Error-state block offsets.
inline constexpr int kP = 0;
inline constexpr int kV = 3;
inline constexpr int kTh = 6;
inline constexpr int kBg = 9;
inline constexpr int kBa = 12;

struct NominalState
{
  Vec3 p{Vec3::Zero()};         // position, world NED [m]
  Vec3 v{Vec3::Zero()};         // velocity, world NED [m/s]
  Quat q{Quat::Identity()};     // attitude, body FRD -> world NED
  Vec3 bg{Vec3::Zero()};        // gyro bias [rad/s]
  Vec3 ba{Vec3::Zero()};        // accelerometer bias [m/s^2]
};

struct EskfConfig
{
  double gravity{9.80665};  // [m/s^2]

  // IMU noise, continuous-time densities.
  double gyro_noise{0.004};       // [rad/s/sqrt(Hz)]
  double accel_noise{0.06};       // [m/s^2/sqrt(Hz)]
  double gyro_bias_walk{1e-4};    // [rad/s^2/sqrt(Hz)]
  double accel_bias_walk{3e-3};   // [m/s^3/sqrt(Hz)]

  // Measurement noise (1 sigma).
  double range_noise{0.05};            // [m]
  double flow_noise_best{0.15};        // [rad/s] at flow quality 1.0
  double flow_noise_worst{0.5};        // [rad/s] at flow quality 0.0
  double heading_noise{0.05};          // [rad]
  double mag_declination{0.0};         // [rad], true = magnetic + declination

  // Innovation gates on the normalised innovation squared (chi-square).
  double gate_1dof{16.0};   // ~4 sigma
  double gate_2dof{25.0};   // generous: flow is heavy-tailed

  // Measurement validity.
  double min_hagl{0.08};              // [m] below this, flow/range are not trusted
  double max_tilt_cos{0.8};           // reject range/flow when cos(tilt) < this

  // Initial 1-sigma uncertainty.
  double init_pos_sigma{0.01};
  double init_vel_sigma{0.05};
  double init_tilt_sigma{0.03};       // [rad] roll/pitch from accelerometer levelling
  double init_yaw_sigma{0.1};         // [rad]
  double init_gyro_bias_sigma{0.005};
  double init_accel_bias_sigma{0.2};
};

struct UpdateResult
{
  bool accepted{false};
  double nis{0.0};  // normalised innovation squared (0 if the update was skipped)
};

// ---- Measurement models: prediction + analytic Jacobian w.r.t. the error state.
// Exposed so tests can check each Jacobian against finite differences.

struct RangeModel
{
  double predicted{};                         // slant range along body +z [m]
  Eigen::Matrix<double, 1, kErrDim> H;
};
// Downward rangefinder over flat ground: r = (ground_z - p_z) / R22.
[[nodiscard]] RangeModel range_model(const NominalState & x, double ground_z) noexcept;

struct FlowModel
{
  Vec2 predicted;                             // [rad/s], see flow measurement below
  Eigen::Matrix<double, 2, kErrDim> H;
};
// Downward optical flow in PX4's sign convention (mirrors EKF2's predictFlow):
//   z = gyro_xy_measured - pixel_flow / dt
//   h(x) = (R22 / hagl) * [v_body_y, -v_body_x] + bg_xy
// The gyro bias term appears because z is built from the RAW gyro; it lets flow
// help estimate bg_xy.
[[nodiscard]] FlowModel flow_model(const NominalState & x, double ground_z) noexcept;

struct HeadingModel
{
  double predicted{};                         // yaw of body x-axis in NED [rad]
  Eigen::Matrix<double, 1, kErrDim> H;
};
[[nodiscard]] HeadingModel heading_model(const NominalState & x) noexcept;

// Heading implied by a body-frame magnetometer vector, levelled with the given
// attitude's roll/pitch, including declination. Returns false if the
// horizontal field is too weak to be meaningful.
[[nodiscard]] bool mag_heading(
  const Vec3 & mag_body, const Quat & attitude, double declination, double & yaw_out) noexcept;

// ---- Small math helpers (also used by tests).
[[nodiscard]] Mat3 skew(const Vec3 & w) noexcept;
[[nodiscard]] Quat quat_exp(const Vec3 & rotation_vector) noexcept;
[[nodiscard]] double wrap_pi(double angle) noexcept;
// q * Exp(dtheta) etc.: apply an error-state vector to a nominal state.
[[nodiscard]] NominalState retract(const NominalState & x, const ErrVec & dx) noexcept;

class Eskf
{
public:
  explicit Eskf(const EskfConfig & config) noexcept;

  // Start (or restart) the filter at rest: p = v = 0, the given attitude and
  // gyro bias, flat ground at world z = ground_z.
  void initialize(const Quat & attitude, const Vec3 & gyro_bias, double ground_z) noexcept;
  [[nodiscard]] bool initialized() const noexcept {return initialized_;}

  // Strapdown propagation with one IMU sample: average body rate [rad/s] and
  // average specific force [m/s^2] over the interval dt [s]. Raw values; the
  // filter removes its own bias estimates.
  void predict(const Vec3 & gyro, const Vec3 & accel, double dt) noexcept;

  UpdateResult update_range(double range_m) noexcept;
  // pixel_flow_rate: pixel_flow / integration time [rad/s];
  // gyro_xy: raw gyro averaged over the same window [rad/s]; quality in [0, 1].
  UpdateResult update_flow(const Vec2 & pixel_flow_rate, const Vec2 & gyro_xy, double quality) noexcept;
  UpdateResult update_heading(const Vec3 & mag_body) noexcept;

  [[nodiscard]] const NominalState & state() const noexcept {return x_;}
  [[nodiscard]] const Cov & covariance() const noexcept {return P_;}
  [[nodiscard]] double ground_z() const noexcept {return ground_z_;}
  [[nodiscard]] double hagl() const noexcept {return ground_z_ - x_.p.z();}
  [[nodiscard]] const EskfConfig & config() const noexcept {return cfg_;}

private:
  template<int M>
  UpdateResult fuse(
    const Eigen::Matrix<double, M, 1> & innovation,
    const Eigen::Matrix<double, M, kErrDim> & H,
    const Eigen::Matrix<double, M, M> & R,
    double gate) noexcept;

  [[nodiscard]] bool geometry_ok() const noexcept;

  EskfConfig cfg_;
  NominalState x_;
  Cov P_{Cov::Zero()};
  double ground_z_{0.0};
  bool initialized_{false};
};

}  // namespace quad_estimation
