// Phase 1 offboard node: take off, fly a square with velocity setpoints, land.
//
// Talks to PX4 only through the uXRCE-DDS /fmu/{in,out} topics, so the same node
// runs unchanged against SITL or a real Pixhawk connected to a companion
// computer over serial/Ethernet (only the agent's transport changes).
//
// Executor / callback-group design:
//   Everything (two subscriptions + the control timer) sits in ONE
//   MutuallyExclusive callback group on a SingleThreadedExecutor. Callbacks
//   therefore never run concurrently, so the latest-sample members below need no
//   locks and the timer always sees a consistent snapshot. Subscription callbacks
//   only copy a few scalars (bounded time, no allocation). This is plenty for a
//   20 Hz outer loop; Phase 2 splits estimation into its own group/thread.
//
// Safety behaviour:
//   The node only commands the vehicle while PX4 reports nav_state == OFFBOARD.
//   If PX4 leaves offboard (RC takeover, failsafe, operator mode switch) the node
//   aborts and stops publishing setpoints; it never tries to re-engage by itself.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "quad_offboard/px4_topics.hpp"
#include "quad_offboard/waypoint_follower.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace quad_offboard
{

enum class Phase : std::uint8_t
{
  kWaitForPx4,   // no valid local position / status yet
  kPrime,        // stream setpoints so PX4 accepts the offboard switch
  kEngage,       // request OFFBOARD + ARM until both are confirmed
  kMission,      // follow the route
  kLand,         // NAV_LAND sent, waiting for auto-disarm
  kDone,
  kAborted,
};

constexpr std::string_view to_string(Phase p)
{
  switch (p) {
    case Phase::kWaitForPx4: return "WAIT_FOR_PX4";
    case Phase::kPrime: return "PRIME";
    case Phase::kEngage: return "ENGAGE";
    case Phase::kMission: return "MISSION";
    case Phase::kLand: return "LAND";
    case Phase::kDone: return "DONE";
    case Phase::kAborted: return "ABORTED";
  }
  return "?";
}

class OffboardSquareNode : public rclcpp::Node
{
public:
  OffboardSquareNode()
  : Node("offboard_square"),
    follower_(load_follower_config())
  {
    const auto ns = declare_parameter<std::string>("px4_namespace", "");
    side_m_ = declare_parameter<double>("square_side_m", 5.0);
    altitude_m_ = declare_parameter<double>("altitude_m", 3.0);
    const double rate_hz = declare_parameter<double>("control_rate_hz", 20.0);
    px4_timeout_s_ = declare_parameter<double>("px4_timeout_s", 120.0);
    engage_timeout_s_ = declare_parameter<double>("engage_timeout_s", 20.0);
    shutdown_when_done_ = declare_parameter<bool>("shutdown_when_done", true);

    if (rate_hz < 5.0) {
      // PX4 drops out of offboard if setpoints arrive slower than 2 Hz
      // (COM_OF_LOSS_T); keep a wide margin.
      throw std::invalid_argument("control_rate_hz must be >= 5");
    }

    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = group_;

    // PX4 publishes best-effort; a reliable subscriber would never match.
    const auto px4_qos = rclcpp::SensorDataQoS();

    status_sub_ = create_subscription<VehicleStatus>(
      px4_topic<VehicleStatus>(ns, "/fmu/out/vehicle_status"), px4_qos,
      [this](const VehicleStatus & msg) {
        nav_state_ = msg.nav_state;
        arming_state_ = msg.arming_state;
        target_system_ = msg.system_id;
        have_status_ = true;
      }, sub_opts);

    local_pos_sub_ = create_subscription<VehicleLocalPosition>(
      px4_topic<VehicleLocalPosition>(ns, "/fmu/out/vehicle_local_position"), px4_qos,
      [this](const VehicleLocalPosition & msg) {
        pos_ = {msg.x, msg.y, msg.z};
        vel_ = {msg.vx, msg.vy, msg.vz};
        heading_ = msg.heading;
        pos_valid_ = msg.xy_valid && msg.z_valid && msg.v_xy_valid && msg.v_z_valid;
      }, sub_opts);

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      px4_topic<OffboardControlMode>(ns, "/fmu/in/offboard_control_mode"), 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(
      px4_topic<TrajectorySetpoint>(ns, "/fmu/in/trajectory_setpoint"), 10);
    command_pub_ = create_publisher<VehicleCommand>(
      px4_topic<VehicleCommand>(ns, "/fmu/in/vehicle_command"), 10);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {on_timer();}, group_);

    phase_start_ = now();  // the first phase's timeout counts from startup
    RCLCPP_INFO(
      get_logger(), "square %.1f m at %.1f m AGL, %.0f Hz; listening on %s",
      side_m_, altitude_m_, rate_hz, status_sub_->get_topic_name());
  }

  [[nodiscard]] Phase phase() const {return phase_;}

private:
  FollowerConfig load_follower_config()
  {
    FollowerConfig c;
    c.cruise_speed_mps = declare_parameter<double>("cruise_speed_mps", c.cruise_speed_mps);
    c.vertical_speed_mps = declare_parameter<double>("vertical_speed_mps", c.vertical_speed_mps);
    c.position_gain = declare_parameter<double>("position_gain", c.position_gain);
    c.acceptance_radius_m = declare_parameter<double>("acceptance_radius_m", c.acceptance_radius_m);
    c.acceptance_speed_mps =
      declare_parameter<double>("acceptance_speed_mps", c.acceptance_speed_mps);
    return c;
  }

  void set_phase(Phase next)
  {
    if (next != phase_) {
      RCLCPP_INFO(get_logger(), "%s -> %s", to_string(phase_).data(), to_string(next).data());
      phase_ = next;
      phase_start_ = now();
    }
  }

  [[nodiscard]] double seconds_in_phase() {return (now() - phase_start_).seconds();}
  [[nodiscard]] bool in_offboard() const
  {
    return nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  [[nodiscard]] bool armed() const {return arming_state_ == VehicleStatus::ARMING_STATE_ARMED;}

  void on_timer()
  {
    switch (phase_) {
      case Phase::kWaitForPx4:
        if (have_status_ && pos_valid_) {
          // Route is anchored at wherever the vehicle is sitting when PX4 is ready.
          const auto square = make_square(pos_, side_m_, altitude_m_);
          follower_.set_route(square, heading_);
          hold_yaw_ = heading_;
          set_phase(Phase::kPrime);
        } else if (seconds_in_phase() > px4_timeout_s_) {
          RCLCPP_ERROR(get_logger(), "no valid PX4 status/local position within %.0f s "
            "(status %s, position %s)", px4_timeout_s_, have_status_ ? "ok" : "missing",
            pos_valid_ ? "valid" : "invalid");
          set_phase(Phase::kAborted);
        }
        return;

      case Phase::kPrime:
        publish_velocity({0.0, 0.0, 0.0}, hold_yaw_);
        // PX4 requires a setpoint stream before it accepts the mode switch; 1 s is ample.
        if (seconds_in_phase() > 1.0) {
          set_phase(Phase::kEngage);
        }
        return;

      case Phase::kEngage:
        publish_velocity({0.0, 0.0, 0.0}, hold_yaw_);
        if (in_offboard() && armed()) {
          set_phase(Phase::kMission);
          return;
        }
        if ((now() - last_command_).seconds() > 1.0) {
          // Re-sent once a second: arming is rejected until the EKF and preflight
          // checks are happy, which can take a few seconds after SITL starts.
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);  // custom, OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
          last_command_ = now();
        }
        if (seconds_in_phase() > engage_timeout_s_) {
          RCLCPP_ERROR(get_logger(), "PX4 did not enter OFFBOARD+ARMED within %.0f s",
            engage_timeout_s_);
          set_phase(Phase::kAborted);
        }
        return;

      case Phase::kMission: {
          if (!in_offboard()) {
            RCLCPP_WARN(get_logger(), "PX4 left OFFBOARD (nav_state=%u); handing control back",
              nav_state_);
            set_phase(Phase::kAborted);
            return;
          }
          const std::size_t before = follower_.active_index();
          const auto cmd = follower_.step(pos_, vel_);
          if (follower_.active_index() != before && !follower_.finished()) {
            RCLCPP_INFO(get_logger(), "waypoint %zu/%zu reached", before + 1, follower_.size());
          }
          if (!cmd) {
            RCLCPP_INFO(get_logger(), "route complete, landing");
            send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            set_phase(Phase::kLand);
            return;
          }
          publish_velocity(cmd->velocity_ned, cmd->yaw_rad);
          return;
        }

      case Phase::kLand:
        // PX4 auto-disarms after touchdown (COM_DISARM_LAND).
        if (!armed()) {
          RCLCPP_INFO(get_logger(), "landed and disarmed");
          set_phase(Phase::kDone);
        }
        return;

      case Phase::kDone:
      case Phase::kAborted:
        if (shutdown_when_done_) {
          timer_->cancel();
          rclcpp::shutdown();
        }
        return;
    }
  }

  void publish_velocity(const Vec3 & v_ned, double yaw)
  {
    const auto stamp_us = static_cast<std::uint64_t>(now().nanoseconds() / 1000);

    OffboardControlMode mode{};
    mode.timestamp = stamp_us;
    mode.velocity = true;
    offboard_mode_pub_->publish(mode);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    TrajectorySetpoint sp{};
    sp.timestamp = stamp_us;
    sp.position = {kNaN, kNaN, kNaN};  // NaN = "not controlled" to PX4
    sp.velocity = {static_cast<float>(v_ned.x), static_cast<float>(v_ned.y),
      static_cast<float>(v_ned.z)};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = static_cast<float>(yaw);
    sp.yawspeed = kNaN;
    setpoint_pub_->publish(sp);
  }

  void send_command(std::uint32_t command, float p1 = 0.0F, float p2 = 0.0F)
  {
    VehicleCommand cmd{};
    cmd.timestamp = static_cast<std::uint64_t>(now().nanoseconds() / 1000);
    cmd.command = command;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  // Configuration
  double side_m_{};
  double altitude_m_{};
  double px4_timeout_s_{};
  double engage_timeout_s_{};
  bool shutdown_when_done_{};

  // Latest PX4 state (written by subscriptions, read by the timer; same
  // mutually-exclusive group, so no locking).
  bool have_status_{false};
  bool pos_valid_{false};
  std::uint8_t nav_state_{0};
  std::uint8_t arming_state_{0};
  std::uint8_t target_system_{1};
  Vec3 pos_{};
  Vec3 vel_{};
  double heading_{0.0};

  // Mission state
  WaypointFollower follower_;
  double hold_yaw_{0.0};
  Phase phase_{Phase::kWaitForPx4};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace quad_offboard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<quad_offboard::OffboardSquareNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  const bool ok = node->phase() == quad_offboard::Phase::kDone;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ok ? 0 : 1;
}
