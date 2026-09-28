// PX4 odometry -> ROS odometry + TF (odom -> base_link) for mapping.
//
// PX4's EKF2 (this vehicle has GPS) provides the odometry RTAB-Map builds the
// map on. Phase 2's ESKF is not used here: it assumes flat ground under the
// rangefinder, which a cluttered world violates (docs/phase3_perception_slam.md).
//
// Time: every Phase 3 node runs on simulation time (/clock from Gazebo), and
// Gazebo stamps the camera images with that clock. PX4's stamps are in a
// different time base (PX4 time + uXRCE-DDS timesync offset, with steps), so
// this node stamps each sample with the ROS (simulation) clock on arrival. The
// cost is the DDS transport latency, a few ms: millimetres at mapping speeds.
// Samples are also dropped if they arrive out of order.
//
// Validity: nothing is published until PX4 first reports the estimate usable
// (vehicle_local_position: xy_valid, z_valid, heading_good_for_control). Before
// EKF2 aligns its heading, vehicle_odometry carries an identity attitude, which
// is yaw 90 deg off here; RTAB-Map's first two map nodes, taken on the ground
// with it, drew a phantom box_tall into the Phase 4 map.

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "quad_perception/odometry_conversion.hpp"

using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;

namespace quad_perception
{

class Px4OdometryBridge : public rclcpp::Node
{
public:
  Px4OdometryBridge()
  : Node("px4_odometry_bridge")
  {
    const auto ns = declare_parameter<std::string>("px4_namespace", "");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<VehicleOdometry>(
      ns + "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry & m) {on_odometry(m);});
    // Versioned PX4 topics carry a _vN suffix (as quad_offboard/px4_topics.hpp).
    std::string lpos_topic = ns + "/fmu/out/vehicle_local_position";
    if (VehicleLocalPosition::MESSAGE_VERSION != 0U) {
      lpos_topic += "_v" + std::to_string(VehicleLocalPosition::MESSAGE_VERSION);
    }
    lpos_sub_ = create_subscription<VehicleLocalPosition>(lpos_topic, rclcpp::SensorDataQoS(),
        [this](const VehicleLocalPosition & m) {
          // Latched: gate only the start-up alignment. A later flicker (seen
          // at takeoff) must not cut the odometry the planner flies on.
          if (!valid_ && m.xy_valid && m.z_valid && m.heading_good_for_control) {
            RCLCPP_INFO(get_logger(), "PX4 estimate valid: publishing");
            valid_ = true;
          }
        });
    RCLCPP_INFO(get_logger(), "%s -> %s (%s -> %s)", sub_->get_topic_name(),
      pub_->get_topic_name(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void on_odometry(const VehicleOdometry & m)
  {
    if (!valid_) {
      return;
    }
    if (m.pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ignoring odometry with pose_frame %u (need NED)", m.pose_frame);
      return;
    }
    Px4Odometry in;
    in.position_ned = {m.position[0], m.position[1], m.position[2]};
    in.q_ned_frd = Quat(m.q[0], m.q[1], m.q[2], m.q[3]);
    in.velocity = {m.velocity[0], m.velocity[1], m.velocity[2]};
    in.velocity_frame = m.velocity_frame;
    in.angular_velocity_frd = {m.angular_velocity[0], m.angular_velocity[1],
      m.angular_velocity[2]};
    in.position_variance_ned = {m.position_variance[0], m.position_variance[1],
      m.position_variance[2]};
    in.orientation_variance = {m.orientation_variance[0], m.orientation_variance[1],
      m.orientation_variance[2]};
    in.velocity_variance = {m.velocity_variance[0], m.velocity_variance[1],
      m.velocity_variance[2]};
    const auto out = to_ros(in);
    if (!out) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ignoring unusable odometry (velocity_frame %u)", m.velocity_frame);
      return;
    }
    const rclcpp::Time stamp = now();
    if (stamp <= last_stamp_) {
      return;  // same sim-clock tick or clock went backwards: TF would reject it
    }
    last_stamp_ = stamp;

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = out->position_enu.x();
    o.pose.pose.position.y = out->position_enu.y();
    o.pose.pose.position.z = out->position_enu.z();
    o.pose.pose.orientation.w = out->q_enu_flu.w();
    o.pose.pose.orientation.x = out->q_enu_flu.x();
    o.pose.pose.orientation.y = out->q_enu_flu.y();
    o.pose.pose.orientation.z = out->q_enu_flu.z();
    o.twist.twist.linear.x = out->velocity_body_flu.x();
    o.twist.twist.linear.y = out->velocity_body_flu.y();
    o.twist.twist.linear.z = out->velocity_body_flu.z();
    o.twist.twist.angular.x = out->angular_velocity_flu.x();
    o.twist.twist.angular.y = out->angular_velocity_flu.y();
    o.twist.twist.angular.z = out->angular_velocity_flu.z();
    for (int i = 0; i < 3; ++i) {
      o.pose.covariance[i * 7] = out->position_variance_enu[i];
      o.pose.covariance[(i + 3) * 7] = out->orientation_variance[i];
      o.twist.covariance[i * 7] = out->velocity_variance[i];
    }
    pub_->publish(o);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = o.header;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = o.pose.pose.position.x;
      t.transform.translation.y = o.pose.pose.position.y;
      t.transform.translation.z = o.pose.pose.position.z;
      t.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(t);
    }
  }

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  bool valid_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace quad_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quad_perception::Px4OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
