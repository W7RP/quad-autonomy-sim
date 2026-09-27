#include "quad_estimation/pipeline.hpp"

#include <cmath>
#include <cstdlib>

namespace quad_estimation
{

void FuseCounters::count(const UpdateResult & r) noexcept
{
  if (r.accepted) {
    fused.fetch_add(1, std::memory_order_relaxed);
  } else if (r.nis > 0.0) {
    rejected.fetch_add(1, std::memory_order_relaxed);
  } else {
    skipped.fetch_add(1, std::memory_order_relaxed);
  }
  if (r.nis > 0.0) {
    last_nis.store(r.nis, std::memory_order_relaxed);
  }
}

EstimatorPipeline::EstimatorPipeline(const PipelineConfig & cfg) noexcept
: cfg_(cfg), eskf_(cfg.eskf), alignment_(cfg.alignment) {}

void EstimatorPipeline::clear_buffers() noexcept
{
  flow_buf_.clear();
  range_buf_.clear();
  mag_buf_.clear();
}

bool EstimatorPipeline::on_imu(const ImuInput & in) noexcept
{
  if (in.dt_us <= 0 || in.dt_us > 100'000 || !in.gyro.allFinite() || !in.accel.allFinite()) {
    n_.imu_bad.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // IMU integration always uses PX4's own interval (dt_us), never stamp
  // differences, so timestamp problems cannot corrupt integration. Stamps only
  // order measurements against the filter clock. Two kinds of stamp jump occur
  // with uXRCE-DDS timesync, and they need opposite handling:
  //   * a one-off glitch (seen: one sample stamped with raw PX4 boot time):
  //     ignore it and stay on the current timeline;
  //   * a persistent clock step (seen: +6 s steps when the simulator ran at
  //     0.66x real time, -0.31 s on re-convergence): re-anchor to it.
  // They look identical on the first sample, so a jump is only accepted as a
  // step once the next sample confirms the new timeline.
  std::int64_t stamp = in.t_us;
  std::int64_t dt_used_us = in.dt_us;
  if (have_time_) {
    const std::int64_t expected = filter_time_us_ + in.dt_us;
    const std::int64_t err = stamp - expected;
    if (std::llabs(err) > kStepThresholdUs) {
      const bool confirms = have_candidate_ &&
        std::llabs(stamp - (candidate_us_ + in.dt_us)) < kStepThresholdUs;
      if (confirms) {
        // Clock step: re-anchor. Buffered measurements carry old-timeline
        // stamps, so drop them (and the gyro history used to average for flow).
        n_.clock_steps.fetch_add(1, std::memory_order_relaxed);
        clear_buffers();
        gyro_hist_.clear();
        have_candidate_ = false;
      } else {
        // First sample off the timeline: stay put until the next one decides.
        n_.suspect_stamps.fetch_add(1, std::memory_order_relaxed);
        candidate_us_ = stamp;
        have_candidate_ = true;
        stamp = expected;
      }
    } else {
      have_candidate_ = false;
      if (err >= in.dt_us / 2) {
        // Missing IMU interval(s): hold this sample over the gap.
        n_.imu_gaps.fetch_add(1, std::memory_order_relaxed);
        dt_used_us = stamp - filter_time_us_;
      }
    }
  }
  filter_time_us_ = stamp;
  have_time_ = true;
  gyro_hist_.push({stamp, dt_used_us, in.gyro});
  last_gyro_ = in.gyro;

  if (!eskf_.initialized()) {
    try_initialize(in.gyro, in.accel);
    return false;
  }
  eskf_.predict(in.gyro, in.accel, static_cast<double>(dt_used_us) * 1e-6);
  fuse_pending();
  return true;
}

void EstimatorPipeline::try_initialize(const Vec3 & gyro, const Vec3 & accel) noexcept
{
  if (!alignment_.add_imu(gyro, accel) || mag_buf_.empty() || range_buf_.empty()) {
    return;
  }
  Quat q;
  if (!alignment_.attitude(mag_buf_[mag_buf_.size() - 1].mag, cfg_.eskf.mag_declination, q)) {
    return;
  }
  const double range = range_buf_[range_buf_.size() - 1].range;
  const double ground_z = range * q.toRotationMatrix()(2, 2);  // ground below the IMU
  eskf_.initialize(q, alignment_.gyro_bias(), ground_z);
  clear_buffers();
  ++reset_counter_;
  n_.initializations.fetch_add(1, std::memory_order_relaxed);
}

// Fuse, in order, every buffered measurement the filter clock has reached.
// Range first: flow's scale depends on height.
void EstimatorPipeline::fuse_pending() noexcept
{
  const std::int64_t now = filter_time_us_;
  while (!range_buf_.empty() && range_buf_.front().t_us <= now) {
    const RangeInput r = range_buf_.front();
    range_buf_.pop_front();
    if (now - r.t_us > cfg_.max_delay_us) {
      n_.range.stale.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    n_.range.count(eskf_.update_range(r.range));
  }
  while (!flow_buf_.empty() && flow_buf_.front().t_us <= now) {
    const FlowInput f = flow_buf_.front();
    flow_buf_.pop_front();
    if (now - f.t_us > cfg_.max_delay_us) {
      n_.flow.stale.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    Vec3 gyro_mean;
    if (!average_gyro(gyro_hist_, f.t_us - f.window_us, f.t_us, 0.8, gyro_mean)) {
      n_.flow.skipped.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    const Vec2 rate = f.pixel_flow / (static_cast<double>(f.window_us) * 1e-6);
    n_.flow.count(eskf_.update_flow(rate, gyro_mean.head<2>(), f.quality / 255.0));
  }
  while (!mag_buf_.empty() && mag_buf_.front().t_us <= now) {
    const MagInput g = mag_buf_.front();
    mag_buf_.pop_front();
    if (now - g.t_us > cfg_.max_delay_us) {
      n_.heading.stale.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    n_.heading.count(eskf_.update_heading(g.mag));
  }
}

bool EstimatorPipeline::plausible_stamp(std::int64_t t_us) noexcept
{
  // Far off the filter clock: a glitch, or the new side of a clock step the IMU
  // has not confirmed yet. Either way it cannot be ordered against the filter.
  if (have_time_ && std::llabs(t_us - filter_time_us_) > kMaxMeasurementSkewUs) {
    n_.off_timeline_measurements.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void EstimatorPipeline::on_flow(const FlowInput & in) noexcept
{
  if (in.window_us > 0 && in.quality >= cfg_.flow_min_quality && in.pixel_flow.allFinite() &&
    plausible_stamp(in.t_us))
  {
    flow_buf_.push(in);
  } else {
    n_.flow.skipped.fetch_add(1, std::memory_order_relaxed);
  }
}

void EstimatorPipeline::on_range(const RangeInput & in) noexcept
{
  if (std::isfinite(in.range) && plausible_stamp(in.t_us)) {
    range_buf_.push(in);
  } else {
    n_.range.skipped.fetch_add(1, std::memory_order_relaxed);
  }
}

void EstimatorPipeline::on_mag(const MagInput & in) noexcept
{
  if (in.mag.allFinite() && plausible_stamp(in.t_us)) {
    mag_buf_.push(in);
  } else {
    n_.heading.skipped.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace quad_estimation
