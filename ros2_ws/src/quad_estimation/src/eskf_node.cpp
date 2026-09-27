// Phase 2 estimator node: runs the ESKF on PX4 sensor data received over
// uXRCE-DDS and publishes the estimate. PX4's own EKF2 keeps flying the
// vehicle; this runs alongside it ("shadow mode") and is evaluated against
// simulator ground truth by scripts/eval_estimation.py. Ground truth is never
// subscribed to here.
//
// Threads, executors and callback groups (the real-time design)
// --------------------------------------------------------------
//   sensor thread   SingleThreadedExecutor spinning ONLY `sensor_group_`
//                   (MutuallyExclusive): IMU, flow, range and mag subscriptions.
//                   They feed EstimatorPipeline (pipeline.hpp), which owns all
//                   filter state; being single-threaded, it needs no locks.
//                   Measurement callbacks only copy a few numbers into fixed
//                   rings; the IMU callback predicts, then fuses every buffered
//                   measurement whose timestamp it has reached.
//   output thread   SingleThreadedExecutor spinning the node's default group
//                   plus `output_group_`: the 50 Hz publish timer, the 1 Hz
//                   diagnostics timer, and parameter services. Everything that
//                   allocates (message serialisation, logging, strings) lives
//                   here, off the hot path.
//   hand-off        The sensor thread copies a small POD snapshot under a mutex
//                   using try_lock(): it NEVER blocks. If the output thread holds
//                   the lock at that instant, that one snapshot is skipped (and
//                   counted); the next IMU sample, ~12 ms later, writes a fresh one.
//
// Hot-path guarantees (measured, not assumed; see /diagnostics):
//   * bounded time: every callback does O(1) work with compile-time bounds
//     (fixed 15x15 algebra; rings of fixed capacity);
//   * no heap allocation in our callback bodies (alloc_probe counts any);
//   * subscriptions take messages from pre-allocated pools
//     (MessagePoolMemoryStrategy), so rclcpp does not allocate a new message
//     per sample either.
//
// Timestamps: all inputs carry PX4 time after uXRCE-DDS timesync. IMU
// integration uses PX4's own interval (gyro_integral_dt), never stamp
// differences, so a timestamp glitch cannot corrupt integration. Stamps are
// only used to order/associate measurements; absurd jumps are ignored and
// genuine clock steps re-anchor the timeline (both counted).

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <pthread.h>
#include <sched.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <px4_msgs/msg/vehicle_magnetometer.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "quad_estimation/alloc_probe.hpp"
#include "quad_estimation/frames.hpp"
#include "quad_estimation/params.hpp"
#include "quad_estimation/pipeline.hpp"
#include "quad_estimation/timing_stats.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::DistanceSensor;
using px4_msgs::msg::SensorCombined;
using px4_msgs::msg::SensorOpticalFlow;
using px4_msgs::msg::VehicleMagnetometer;
using px4_msgs::msg::VehicleOdometry;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

namespace quad_estimation
{

namespace
{
// Everything the output thread needs, copied out of the filter. Trivially
// copyable on purpose (the copy is the whole critical section).
struct Snapshot
{
  bool initialized{false};
  std::int64_t t_us{0};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
  Quat q{Quat::Identity()};
  Vec3 omega{Vec3::Zero()};    // bias-corrected body rate, FRD
  Vec3 bg{Vec3::Zero()};
  Vec3 ba{Vec3::Zero()};
  Vec3 pos_var{Vec3::Zero()};
  Vec3 vel_var{Vec3::Zero()};
  Vec3 att_var{Vec3::Zero()};
  double hagl{0.0};
  std::uint8_t reset_counter{0};
};

std::int64_t elapsed_us(std::chrono::steady_clock::time_point since) noexcept
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - since).count();
}
}  // namespace

class EskfNode : public rclcpp::Node
{
public:
  EskfNode()
  : Node("eskf"),
    pipeline_(load_pipeline_config()),
    imu_timing_(declare_parameter<std::int64_t>("imu_callback_budget_us", 1000)),
    meas_timing_(declare_parameter<std::int64_t>("measurement_callback_budget_us", 200))
  {
    const auto ns = declare_parameter<std::string>("px4_namespace", "");
    const double publish_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<int>("sensor_thread_priority", 0);  // read by main()

    // Groups. The sensor group is NOT auto-added with the node: main() gives it
    // its own executor and thread.
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sensor_group_;
    const auto qos = rclcpp::SensorDataQoS();  // PX4 publishes best-effort

    imu_sub_ = create_subscription<SensorCombined>(
      ns + "/fmu/out/sensor_combined", qos,
      [this](const SensorCombined & m) {on_imu(m);}, opts,
      std::make_shared<MessagePoolMemoryStrategy<SensorCombined, 8>>());
    flow_sub_ = create_subscription<SensorOpticalFlow>(
      ns + "/fmu/out/sensor_optical_flow", qos,
      [this](const SensorOpticalFlow & m) {on_flow(m);}, opts,
      std::make_shared<MessagePoolMemoryStrategy<SensorOpticalFlow, 8>>());
    range_sub_ = create_subscription<DistanceSensor>(
      ns + "/fmu/out/distance_sensor", qos,
      [this](const DistanceSensor & m) {on_range(m);}, opts,
      std::make_shared<MessagePoolMemoryStrategy<DistanceSensor, 8>>());
    mag_sub_ = create_subscription<VehicleMagnetometer>(
      ns + "/fmu/out/vehicle_magnetometer", qos,
      [this](const VehicleMagnetometer & m) {on_mag(m);}, opts,
      std::make_shared<MessagePoolMemoryStrategy<VehicleMagnetometer, 8>>());

    odom_ned_pub_ = create_publisher<VehicleOdometry>("~/odometry_ned", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz)),
      [this]() {publish_estimate();}, output_group_);
    diag_timer_ = create_wall_timer(1s, [this]() {publish_diagnostics();}, output_group_);

    RCLCPP_INFO(get_logger(), "ESKF up: IMU %s, publishing %s at %.0f Hz",
      imu_sub_->get_topic_name(), odom_ned_pub_->get_topic_name(), publish_hz);
  }

  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr sensor_group() const {return sensor_group_;}

private:
  PipelineConfig load_pipeline_config()
  {
    PipelineConfig cfg;
    for (const ParamSpec & p : pipeline_params()) {
      const double v = p.is_int ?
        static_cast<double>(declare_parameter<std::int64_t>(p.name,
        static_cast<std::int64_t>(p.get(cfg)))) :
        declare_parameter<double>(p.name, p.get(cfg));
      set_pipeline_param(cfg, p.name, v);
    }
    return cfg;
  }

  // ------------------------------------------------------ sensor thread
  // Callbacks convert PX4 messages to pipeline inputs and time themselves.
  // Everything inside HotPathScope is counted by the allocation probe.

  void on_imu(const SensorCombined & m) noexcept
  {
    alloc_probe::HotPathScope hot;
    const auto t0 = std::chrono::steady_clock::now();
    const ImuInput in{static_cast<std::int64_t>(m.timestamp),
      static_cast<std::int64_t>(m.gyro_integral_dt),
      Vec3(m.gyro_rad[0], m.gyro_rad[1], m.gyro_rad[2]),
      Vec3(m.accelerometer_m_s2[0], m.accelerometer_m_s2[1], m.accelerometer_m_s2[2])};
    if (pipeline_.on_imu(in)) {
      write_snapshot();
    }
    imu_timing_.record(elapsed_us(t0));
  }

  void on_flow(const SensorOpticalFlow & m) noexcept
  {
    alloc_probe::HotPathScope hot;
    const auto t0 = std::chrono::steady_clock::now();
    pipeline_.on_flow({static_cast<std::int64_t>(m.timestamp_sample),
        static_cast<std::int64_t>(m.integration_timespan_us),
        Vec2(m.pixel_flow[0], m.pixel_flow[1]), m.quality});
    meas_timing_.record(elapsed_us(t0));
  }

  void on_range(const DistanceSensor & m) noexcept
  {
    alloc_probe::HotPathScope hot;
    const auto t0 = std::chrono::steady_clock::now();
    const bool valid = m.orientation == DistanceSensor::ROTATION_DOWNWARD_FACING &&
      m.current_distance >= m.min_distance && m.current_distance <= m.max_distance &&
      m.signal_quality != 0;  // -1 = unknown is accepted
    if (valid) {
      pipeline_.on_range({static_cast<std::int64_t>(m.timestamp), m.current_distance});
    }
    meas_timing_.record(elapsed_us(t0));
  }

  void on_mag(const VehicleMagnetometer & m) noexcept
  {
    alloc_probe::HotPathScope hot;
    const auto t0 = std::chrono::steady_clock::now();
    pipeline_.on_mag({static_cast<std::int64_t>(m.timestamp_sample),
        Vec3(m.magnetometer_ga[0], m.magnetometer_ga[1], m.magnetometer_ga[2])});
    meas_timing_.record(elapsed_us(t0));
  }

  void write_snapshot() noexcept
  {
    std::unique_lock lock(snapshot_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      snapshot_skips_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const Eskf & f = pipeline_.filter();
    const NominalState & x = f.state();
    const Cov & P = f.covariance();
    snapshot_.initialized = true;
    snapshot_.t_us = pipeline_.time_us();
    snapshot_.p = x.p;
    snapshot_.v = x.v;
    snapshot_.q = x.q;
    snapshot_.omega = pipeline_.body_rate();
    snapshot_.bg = x.bg;
    snapshot_.ba = x.ba;
    snapshot_.pos_var = P.diagonal().segment<3>(kP);
    snapshot_.vel_var = P.diagonal().segment<3>(kV);
    snapshot_.att_var = P.diagonal().segment<3>(kTh);
    snapshot_.hagl = f.hagl();
    snapshot_.reset_counter = pipeline_.reset_counter();
  }

  // ------------------------------------------------------ output thread
  Snapshot read_snapshot()
  {
    std::lock_guard lock(snapshot_mutex_);
    return snapshot_;
  }

  void publish_estimate()
  {
    const Snapshot s = read_snapshot();
    if (!s.initialized || s.t_us == last_published_t_us_) {
      return;  // nothing new since the last publish (IMU ~83 Hz vs 50 Hz timer)
    }
    last_published_t_us_ = s.t_us;
    const auto now_us = static_cast<std::uint64_t>(now().nanoseconds() / 1000);

    VehicleOdometry o{};
    o.timestamp = now_us;
    o.timestamp_sample = static_cast<std::uint64_t>(s.t_us);
    o.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    o.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
    for (int i = 0; i < 3; ++i) {
      o.position[i] = static_cast<float>(s.p[i]);
      o.velocity[i] = static_cast<float>(s.v[i]);
      o.angular_velocity[i] = static_cast<float>(s.omega[i]);
      o.position_variance[i] = static_cast<float>(s.pos_var[i]);
      o.velocity_variance[i] = static_cast<float>(s.vel_var[i]);
      o.orientation_variance[i] = static_cast<float>(s.att_var[i]);
    }
    o.q = {static_cast<float>(s.q.w()), static_cast<float>(s.q.x()),
      static_cast<float>(s.q.y()), static_cast<float>(s.q.z())};
    o.reset_counter = s.reset_counter;
    o.quality = 100;
    odom_ned_pub_->publish(o);

    // REP-103 view of the same estimate for the rest of the ROS graph.
    nav_msgs::msg::Odometry r;
    r.header.stamp = now();
    r.header.frame_id = "odom";
    r.child_frame_id = "base_link";
    const Vec3 p_enu = frames::ned_to_enu(s.p);
    const Quat q_enu = frames::ned_frd_to_enu_flu(s.q);
    const Vec3 v_body_flu = frames::frd_to_flu(s.q.conjugate() * s.v);
    const Vec3 w_flu = frames::frd_to_flu(s.omega);
    r.pose.pose.position.x = p_enu.x();
    r.pose.pose.position.y = p_enu.y();
    r.pose.pose.position.z = p_enu.z();
    r.pose.pose.orientation.w = q_enu.w();
    r.pose.pose.orientation.x = q_enu.x();
    r.pose.pose.orientation.y = q_enu.y();
    r.pose.pose.orientation.z = q_enu.z();
    r.twist.twist.linear.x = v_body_flu.x();
    r.twist.twist.linear.y = v_body_flu.y();
    r.twist.twist.linear.z = v_body_flu.z();
    r.twist.twist.angular.x = w_flu.x();
    r.twist.twist.angular.y = w_flu.y();
    r.twist.twist.angular.z = w_flu.z();
    // Diagonal covariances only. Position: NED x/y swap to ENU. Attitude error
    // is body-frame, and FRD<->FLU only flips signs, so variances carry over.
    // Twist variance uses the world-frame velocity variance as an approximation.
    r.pose.covariance[0] = s.pos_var.y();
    r.pose.covariance[7] = s.pos_var.x();
    r.pose.covariance[14] = s.pos_var.z();
    r.pose.covariance[21] = s.att_var.x();
    r.pose.covariance[28] = s.att_var.y();
    r.pose.covariance[35] = s.att_var.z();
    r.twist.covariance[0] = s.vel_var.x();
    r.twist.covariance[7] = s.vel_var.y();
    r.twist.covariance[14] = s.vel_var.z();
    odom_pub_->publish(r);
  }

  void publish_diagnostics()
  {
    const auto imu = imu_timing_.snapshot();
    const auto meas = meas_timing_.snapshot();
    const Snapshot s = read_snapshot();
    const auto hot_allocs = alloc_probe::hot_path_allocations();
    const auto thread_allocs = alloc_probe::tracked_thread_allocations();
    const PipelineCounters & c = pipeline_.counters();

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "eskf: filter";
    st.hardware_id = "quad_estimation";
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = s.initialized ? "running" : "aligning (keep the vehicle still)";
    if (hot_allocs > 0) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "heap allocation detected in hot path";
    } else if (imu.overruns > 0) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "IMU callback exceeded its time budget";
    }
    auto kv = [&st](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        st.values.push_back(e);
      };
    auto n = [](auto v) {return std::to_string(v);};
    kv("imu_callbacks", n(imu.count));
    kv("imu_cb_mean_us", n(imu.mean_us));
    kv("imu_cb_p99_us", n(imu.p99_us));
    kv("imu_cb_max_us", n(imu.max_us));
    kv("imu_cb_budget_us", n(imu.budget_us));
    kv("imu_cb_overruns", n(imu.overruns));
    kv("meas_cb_max_us", n(meas.max_us));
    kv("hot_path_allocations", n(hot_allocs));
    kv("sensor_thread_allocations", n(thread_allocs));
    kv("snapshot_skips", n(snapshot_skips_.load()));
    kv("imu_gaps", n(c.imu_gaps.load()));
    kv("imu_bad", n(c.imu_bad.load()));
    kv("clock_steps", n(c.clock_steps.load()));
    kv("suspect_stamps", n(c.suspect_stamps.load()));
    kv("off_timeline_measurements", n(c.off_timeline_measurements.load()));
    kv("initializations", n(c.initializations.load()));
    kv("alignment_restarts", n(pipeline_.alignment_restarts()));
    const std::array<std::pair<const char *, const FuseCounters *>, 3> sensors{{
      {"range", &c.range}, {"flow", &c.flow}, {"heading", &c.heading}}};
    for (const auto & [name, fc] : sensors) {
      const std::string base(name);
      kv(base + "_fused", n(fc->fused.load()));
      kv(base + "_rejected", n(fc->rejected.load()));
      kv(base + "_skipped", n(fc->skipped.load()));
      kv(base + "_stale", n(fc->stale.load()));
      kv(base + "_last_nis", n(fc->last_nis.load()));
    }
    kv("hagl_m", n(s.hagl));
    kv("gyro_bias", n(s.bg.x()) + " " + n(s.bg.y()) + " " + n(s.bg.z()));
    kv("accel_bias", n(s.ba.x()) + " " + n(s.ba.y()) + " " + n(s.ba.z()));
    arr.status.push_back(st);
    diag_pub_->publish(arr);

    if (++diag_ticks_ % 10 == 0) {
      RCLCPP_INFO(get_logger(),
        "%s | imu cb mean %.0f us p99 %ld us max %ld us, overruns %lu | hot-path allocs %lu | "
        "fused range/flow/heading %lu/%lu/%lu, rejected %lu/%lu/%lu",
        st.message.c_str(), imu.mean_us, static_cast<long>(imu.p99_us),
        static_cast<long>(imu.max_us), static_cast<unsigned long>(imu.overruns),
        static_cast<unsigned long>(hot_allocs),
        static_cast<unsigned long>(c.range.fused.load()),
        static_cast<unsigned long>(c.flow.fused.load()),
        static_cast<unsigned long>(c.heading.fused.load()),
        static_cast<unsigned long>(c.range.rejected.load()),
        static_cast<unsigned long>(c.flow.rejected.load()),
        static_cast<unsigned long>(c.heading.rejected.load()));
    }
  }

  // ---------------------------------------------------------------- state
  // Sensor-thread-owned (touched only from sensor_group_ callbacks).
  EstimatorPipeline pipeline_;

  // Shared with the output thread (atomics / try_lock-guarded snapshot).
  TimingStats imu_timing_;
  TimingStats meas_timing_;
  std::atomic<std::uint64_t> snapshot_skips_{0};
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;

  // Output-thread-owned.
  std::uint64_t diag_ticks_{0};
  std::int64_t last_published_t_us_{-1};

  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr output_group_;
  rclcpp::Subscription<SensorCombined>::SharedPtr imu_sub_;
  rclcpp::Subscription<SensorOpticalFlow>::SharedPtr flow_sub_;
  rclcpp::Subscription<DistanceSensor>::SharedPtr range_sub_;
  rclcpp::Subscription<VehicleMagnetometer>::SharedPtr mag_sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_ned_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace quad_estimation

namespace
{
// Optional SCHED_FIFO for the sensor thread. Needs CAP_SYS_NICE / rtprio
// limits: typically granted on a companion computer, not inside WSL2.
void set_realtime_priority(int priority, const rclcpp::Logger & logger)
{
  if (priority <= 0) {
    RCLCPP_INFO(logger, "sensor thread: default scheduling (sensor_thread_priority=0)");
    return;
  }
  sched_param sp{};
  sp.sched_priority = priority;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc == 0) {
    RCLCPP_INFO(logger, "sensor thread: SCHED_FIFO priority %d", priority);
  } else {
    RCLCPP_WARN(logger, "sensor thread: SCHED_FIFO %d refused (%s); continuing with default "
      "scheduling", priority, std::strerror(rc));
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<quad_estimation::EskfNode>();
  const int priority = static_cast<int>(node->get_parameter("sensor_thread_priority").as_int());

  rclcpp::executors::SingleThreadedExecutor sensor_exec;
  sensor_exec.add_callback_group(node->sensor_group(), node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor output_exec;
  output_exec.add_node(node);

  std::thread sensor_thread([&]() {
      quad_estimation::alloc_probe::track_this_thread();
      set_realtime_priority(priority, node->get_logger());
      sensor_exec.spin();
    });
  output_exec.spin();  // returns on Ctrl+C (rclcpp's signal handler shuts the context down)

  sensor_exec.cancel();
  sensor_thread.join();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
