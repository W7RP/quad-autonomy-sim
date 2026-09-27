// Everything the estimator does per sensor sample, independent of ROS:
// timeline handling, static alignment, measurement buffering, gyro averaging
// for flow, and fusion order. The ROS node (eskf_node) and the offline replay
// tool (eskf_replay) both drive this one class, so what is tuned in replay is
// exactly what runs live.
//
// Threading: all on_*() calls must come from ONE thread (the node's sensor
// thread). Counters are atomics so another thread may read them at any time.
#pragma once

#include <atomic>
#include <cstdint>

#include "quad_estimation/eskf.hpp"
#include "quad_estimation/fixed_ring.hpp"
#include "quad_estimation/imu_history.hpp"
#include "quad_estimation/static_alignment.hpp"

namespace quad_estimation
{

struct ImuInput
{
  std::int64_t t_us{};      // end of the integration interval (PX4 time)
  std::int64_t dt_us{};     // integration interval
  Vec3 gyro{Vec3::Zero()};  // average rate over the interval [rad/s], FRD
  Vec3 accel{Vec3::Zero()}; // average specific force over the interval [m/s^2], FRD
};
struct FlowInput
{
  std::int64_t t_us{};        // end of the flow integration window
  std::int64_t window_us{};
  Vec2 pixel_flow{Vec2::Zero()};  // integrated flow [rad], PX4 sign convention
  int quality{};              // 0..255
};
struct RangeInput
{
  std::int64_t t_us{};
  double range{};             // [m], already validated (downward, in range)
};
struct MagInput
{
  std::int64_t t_us{};
  Vec3 mag{Vec3::Zero()};     // body FRD [Gauss]
};

struct PipelineConfig
{
  EskfConfig eskf;
  AlignmentConfig alignment;
  int flow_min_quality{1};
  std::int64_t max_delay_us{300'000};
};

// Per-measurement-type outcome counters.
struct FuseCounters
{
  std::atomic<std::uint64_t> fused{0};
  std::atomic<std::uint64_t> rejected{0};   // failed the innovation gate
  std::atomic<std::uint64_t> skipped{0};    // invalid input or geometry
  std::atomic<std::uint64_t> stale{0};      // too old when the filter reached it
  std::atomic<double> last_nis{0.0};

  void count(const UpdateResult & r) noexcept;
};

struct PipelineCounters
{
  FuseCounters range;
  FuseCounters flow;
  FuseCounters heading;
  std::atomic<std::uint64_t> imu_gaps{0};
  std::atomic<std::uint64_t> imu_bad{0};
  std::atomic<std::uint64_t> clock_steps{0};                 // confirmed re-anchors
  std::atomic<std::uint64_t> suspect_stamps{0};              // IMU stamps off the timeline
  std::atomic<std::uint64_t> off_timeline_measurements{0};   // measurements dropped for that
  std::atomic<std::uint64_t> initializations{0};
};

class EstimatorPipeline
{
public:
  explicit EstimatorPipeline(const PipelineConfig & cfg) noexcept;

  // Returns true when the filter state advanced (initialised and predicted).
  bool on_imu(const ImuInput & in) noexcept;
  void on_flow(const FlowInput & in) noexcept;
  void on_range(const RangeInput & in) noexcept;
  void on_mag(const MagInput & in) noexcept;

  [[nodiscard]] const Eskf & filter() const noexcept {return eskf_;}
  [[nodiscard]] std::int64_t time_us() const noexcept {return filter_time_us_;}
  [[nodiscard]] Vec3 body_rate() const noexcept {return last_gyro_ - eskf_.state().bg;}
  [[nodiscard]] std::uint8_t reset_counter() const noexcept {return reset_counter_;}
  [[nodiscard]] const PipelineCounters & counters() const noexcept {return n_;}
  [[nodiscard]] std::uint64_t alignment_restarts() const noexcept {return alignment_.restarts();}

  // An IMU stamp further than this from where the timeline predicts it is a
  // glitch or a clock step (never ordinary jitter: the IMU interval is ~12 ms).
  static constexpr std::int64_t kStepThresholdUs = 100'000;
  static constexpr std::int64_t kMaxMeasurementSkewUs = 1'000'000;

private:
  void try_initialize(const Vec3 & gyro, const Vec3 & accel) noexcept;
  void fuse_pending() noexcept;
  [[nodiscard]] bool plausible_stamp(std::int64_t t_us) noexcept;
  void clear_buffers() noexcept;

  PipelineConfig cfg_;
  Eskf eskf_;
  StaticAlignment alignment_;
  GyroHistory gyro_hist_;
  FixedRing<FlowInput, 16> flow_buf_;
  FixedRing<RangeInput, 16> range_buf_;
  FixedRing<MagInput, 8> mag_buf_;
  std::int64_t filter_time_us_{0};
  bool have_time_{false};
  std::int64_t candidate_us_{0};   // stamp of an unconfirmed jump
  bool have_candidate_{false};
  Vec3 last_gyro_{Vec3::Zero()};
  std::uint8_t reset_counter_{0};
  PipelineCounters n_;
};

}  // namespace quad_estimation
