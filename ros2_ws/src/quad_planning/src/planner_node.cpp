// Phase 4 planner: occupancy from RTAB-Map's map plus the live depth camera,
// RRT* + shortcut smoothing, continuous path validation and replanning, and a
// simple goal-sequence mission.
//
// Frames: everything is planned in `odom` (ENU): PX4's local frame in ROS
// conventions (px4_odometry_bridge). Paths in that frame can be flown directly
// by quad_offboard, with no dependence on RTAB-Map's map -> odom corrections.
// RTAB-Map's obstacle cloud (frame `map`) is transformed into `odom` with TF.
//
// Threads: two MutuallyExclusive callback groups on a 2-thread executor.
//   data group      odometry, depth images and the RTAB-Map cloud. Each
//                   callback converts its input to points in `odom` and stores
//                   them under `data_mutex_` (short critical sections).
//   planning group  a 5 Hz timer: rebuild + inflate the grid from the stored
//                   points, advance the mission, validate the current path,
//                   and replan (RRT*, bounded by a time budget) if needed.
// Planning is soft real-time. Tens to hundreds of ms per replan is fine: while
// the planner thinks, PX4 holds the last setpoint and quad_offboard keeps
// streaming it. It never runs on the flight controller's control loops.
//
// Known simplification: the grid only ever marks. An obstacle that moves away
// leaves its old cells occupied in RTAB-Map's cloud until the map updates,
// which is conservative. Live depth points expire after live_decay_s.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "quad_planning/rrt_star.hpp"
#include "quad_planning/voxel_grid.hpp"

using namespace std::chrono_literals;

namespace quad_planning
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner"),
    raw_(load_grid_spec()),
    inflated_(raw_.spec()),
    hard_(raw_.spec())
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    inflate_cells_ = static_cast<int>(std::ceil(
        declare_parameter<double>("inflation_m", 0.75) / raw_.spec().resolution));
    // Hysteresis: paths are PLANNED with inflation_m but only declared blocked
    // when they violate the tighter hard_inflation_m. Without it, every depth
    // frame nudging a cell into the margin of a path that skirts an obstacle
    // triggered a replan (~16 per leg, 0.1-0.4 s apart), each with a different
    // random RRT* path.
    hard_cells_ = static_cast<int>(std::ceil(
        declare_parameter<double>("hard_inflation_m", 0.55) / raw_.spec().resolution));
    ground_z_ = declare_parameter<double>("ground_z", -0.24);  // odom z of the ground (start height)
    ground_margin_ = declare_parameter<double>("ground_margin", 0.25);
    depth_max_range_ = declare_parameter<double>("depth_max_range", 6.0);
    depth_stride_ = static_cast<int>(declare_parameter<int>("depth_pixel_stride", 4));
    live_decay_ = declare_parameter<double>("live_decay_s", 4.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.6);
    dump_dir_ = declare_parameter<std::string>("debug_dump_dir", "");
    dump_max_ = static_cast<int>(declare_parameter<int>("debug_dump_max", 5));
    const auto events_file = declare_parameter<std::string>("events_file", "");
    if (!events_file.empty()) {
      events_out_.open(events_file, std::ios::out | std::ios::trunc);
    }
    const auto goals = declare_parameter<std::vector<double>>("goals_enu", std::vector<double>{});
    if (goals.empty() || goals.size() % 3 != 0) {
      throw std::invalid_argument("goals_enu must hold x, y, z triples (odom frame, ENU)");
    }
    for (std::size_t i = 0; i < goals.size(); i += 3) {
      goals_.emplace_back(goals[i], goals[i + 1], goals[i + 2]);
    }

    // Where the planner may route (sampling box), separate from the grid that
    // holds observations. Unmapped space is treated as free (optimistic), so
    // without this box the first missions happily routed around the whole
    // field through never-seen space, out to x = 15 m.
    const auto amin = declare_parameter<std::vector<double>>("arena_min_xy", {-4.0, -4.0});
    const auto amax = declare_parameter<std::vector<double>>("arena_max_xy", {14.0, 14.0});
    RrtConfig rc;
    rc.bounds_min = Vec3(amin.at(0), amin.at(1), declare_parameter<double>("z_min", 1.0));
    rc.bounds_max = Vec3(amax.at(0), amax.at(1), declare_parameter<double>("z_max", 2.8));
    horizon_m_ = declare_parameter<double>("validation_horizon_m", 5.0);
    rc.step = declare_parameter<double>("rrt_step", 1.0);
    rc.rewire_radius = declare_parameter<double>("rrt_rewire_radius", 2.5);
    rc.time_budget = std::chrono::milliseconds(declare_parameter<int>("rrt_time_budget_ms", 400));
    rc.max_nodes = static_cast<int>(declare_parameter<int>("rrt_max_nodes", 4000));
    rrt_ = std::make_unique<RrtStar>(rc);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    data_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = data_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 20,
        [this](const nav_msgs::msg::Odometry & m) {
          std::lock_guard lock(data_mutex_);
          pos_ = Vec3(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
          have_pos_ = true;
        }, opts);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",
        rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::CameraInfo & m) {
          fx_ = m.k[0];
          fy_ = m.k[4];
          cx_ = m.k[2];
          cy_ = m.k[5];
        }, opts);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw",
        rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image & m) {on_depth(m);}, opts);
    // RTAB-Map publishes the assembled obstacle cloud latched.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_obstacles",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const sensor_msgs::msg::PointCloud2 & m) {on_map_cloud(m);}, opts);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/path", latched);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/planner/mission_complete", latched);
    event_pub_ = create_publisher<std_msgs::msg::String>("/planner/events", 50);

    const double rate = declare_parameter<double>("plan_rate_hz", 5.0);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
        [this]() {tick();}, plan_group_);
    RCLCPP_INFO(get_logger(), "planner: %zu goals, grid %dx%dx%d @ %.2f m, inflation %d cells",
      goals_.size(), raw_.spec().nx, raw_.spec().ny, raw_.spec().nz, raw_.spec().resolution,
      inflate_cells_);
  }

private:
  GridSpec load_grid_spec()
  {
    GridSpec s;
    const auto o = declare_parameter<std::vector<double>>("grid_origin", {-8.0, -12.0, -0.5});
    const auto size = declare_parameter<std::vector<double>>("grid_size", {30.0, 30.0, 4.5});
    s.resolution = declare_parameter<double>("grid_resolution", 0.1);
    s.origin = Vec3(o.at(0), o.at(1), o.at(2));
    s.nx = static_cast<int>(std::round(size.at(0) / s.resolution));
    s.ny = static_cast<int>(std::round(size.at(1) / s.resolution));
    s.nz = static_cast<int>(std::round(size.at(2) / s.resolution));
    return s;
  }

  // ------------------------------------------------------------ data group
  std::optional<Eigen::Isometry3d> lookup(const std::string & from, const rclcpp::Time & stamp)
  {
    try {
      return tf2::transformToEigen(tf_buffer_->lookupTransform(odom_frame_, from, stamp, 50ms));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  bool keep(const Vec3 & p) const
  {
    return p.allFinite() && p.z() > ground_z_ + ground_margin_;
  }

  void on_depth(const sensor_msgs::msg::Image & m)
  {
    if (fx_ <= 0.0 || m.encoding != "32FC1") {
      return;
    }
    const auto T = lookup(m.header.frame_id, m.header.stamp);
    if (!T) {
      return;
    }
    std::vector<Vec3> pts;
    pts.reserve(static_cast<std::size_t>((m.width / depth_stride_ + 1) * (m.height / depth_stride_ + 1)));
    for (std::uint32_t v = 0; v < m.height; v += depth_stride_) {
      const auto * row = reinterpret_cast<const float *>(&m.data[v * m.step]);
      for (std::uint32_t u = 0; u < m.width; u += depth_stride_) {
        const double z = row[u];
        if (!(z > 0.3 && z < depth_max_range_)) {
          continue;
        }
        const Vec3 p = *T * Vec3((u - cx_) * z / fx_, (v - cy_) * z / fy_, z);
        if (keep(p)) {
          pts.push_back(p);
        }
      }
    }
    std::lock_guard lock(data_mutex_);
    live_.push_back({now().seconds(), std::move(pts)});
    while (!live_.empty() && live_.front().stamp < now().seconds() - live_decay_) {
      live_.pop_front();
    }
  }

  void on_map_cloud(const sensor_msgs::msg::PointCloud2 & m)
  {
    const auto T = lookup(m.header.frame_id, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
    if (!T) {
      return;
    }
    std::vector<Vec3> pts;
    pts.reserve(static_cast<std::size_t>(m.width) * m.height);
    for (sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
      x != x.end(); ++x, ++y, ++z)
    {
      const Vec3 p = *T * Vec3(*x, *y, *z);
      if (keep(p)) {
        pts.push_back(p);
      }
    }
    std::lock_guard lock(data_mutex_);
    map_points_ = std::move(pts);
  }

  // -------------------------------------------------------- planning group
  void event(const std::string & json)
  {
    std_msgs::msg::String s;
    s.data = json;
    event_pub_->publish(s);
    if (events_out_.is_open()) {
      events_out_ << json << '\n' << std::flush;  // one JSON object per line
    }
    RCLCPP_INFO(get_logger(), "%s", json.c_str());
  }

  bool rebuild_grid(Vec3 & pos)
  {
    {
      std::lock_guard lock(data_mutex_);
      if (!have_pos_) {
        return false;
      }
      pos = pos_;
      raw_.clear();
      for (const auto & p : map_points_) {
        raw_.mark(p);
      }
      n_map_points_ = map_points_.size();
      n_live_points_ = 0;
      const double cutoff = now().seconds() - live_decay_;
      for (const auto & f : live_) {
        if (f.stamp >= cutoff) {
          for (const auto & p : f.points) {
            raw_.mark(p);
          }
          n_live_points_ += f.points.size();
        }
      }
    }
    raw_.dilate_into(inflate_cells_, inflated_);
    raw_.dilate_into(hard_cells_, hard_);
    return true;
  }

  // The vehicle can legitimately be inside the inflated margin (passing an
  // obstacle at 0.6 m). Plan from the nearest free point instead of failing.
  std::optional<Vec3> free_near(const Vec3 & p) const
  {
    if (!inflated_.occupied(p)) {
      return p;
    }
    for (double r = 0.2; r <= 1.2; r += 0.2) {
      for (int k = 0; k < 16; ++k) {
        const double a = k * M_PI / 8.0;
        for (const double dz : {0.0, 0.3, -0.3}) {
          const Vec3 q = p + Vec3(r * std::cos(a), r * std::sin(a), dz);
          if (!inflated_.occupied(q)) {
            return q;
          }
        }
      }
    }
    return std::nullopt;
  }

  bool path_blocked(const Vec3 & pos) const
  {
    if (path_.empty()) {
      return true;
    }
    // Against the HARD margin (see hard_cells_), and only within the
    // validation horizon along the path: a blockage further out is re-checked
    // as the vehicle approaches, when the map there is better. Checking the
    // whole path made far-away map updates trigger replan after replan.
    double budget = horizon_m_;
    Vec3 a = pos;
    for (std::size_t i = next_wp_; i < path_.size() && budget > 0.0; ++i) {
      const Vec3 & b = path_[i];
      const double len = (b - a).norm();
      const Vec3 end = len > budget ? a + (b - a) * (budget / len) : b;
      if (!hard_.segment_free(a, end)) {
        return true;
      }
      budget -= len;
      a = b;
    }
    return false;
  }

  void tick()
  {
    if (done_) {
      return;
    }
    Vec3 pos;
    const auto t0 = std::chrono::steady_clock::now();
    if (!rebuild_grid(pos)) {
      return;
    }
    grid_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // Wait for takeoff: the flight node climbs in place first, and a path
    // planned from the ground would replace that climb.
    if (pos.z() < rrt_->config().bounds_min.z() - 0.3) {
      return;
    }
    const Vec3 & goal = goals_[goal_idx_];
    if ((pos - goal).norm() < goal_tolerance_) {
      std::ostringstream e;
      e << "{\"event\":\"goal_reached\",\"goal\":" << goal_idx_ << ",\"t\":" << now().seconds() << "}";
      event(e.str());
      path_.clear();
      fail_streak_ = 0;
      if (++goal_idx_ >= goals_.size()) {
        done_ = true;
        std_msgs::msg::Bool b;
        b.data = true;
        done_pub_->publish(b);
        dump_grid("final_map", "");
        event("{\"event\":\"mission_complete\",\"t\":" + std::to_string(now().seconds()) + "}");
        return;
      }
    }
    // Track progress along the current path (for validation only).
    while (!path_.empty() && next_wp_ + 1 < path_.size() && (pos - path_[next_wp_]).norm() < 0.8) {
      ++next_wp_;
    }
    std::string reason;
    if (path_.empty()) {
      reason = "new_goal";
    } else if (path_blocked(pos)) {
      reason = "path_blocked";
    } else {
      return;
    }
    replan(pos, reason);
  }

  void replan(const Vec3 & pos, const std::string & reason)
  {
    const Vec3 & goal = goals_[goal_idx_];
    const auto start = free_near(pos);
    const auto target = free_near(goal);
    std::ostringstream e;
    e << "{\"event\":\"plan\",\"reason\":\"" << reason << "\",\"goal\":" << goal_idx_
      << ",\"t\":" << now().seconds() << ",\"pos\":" << json(pos)
      << ",\"map_points\":" << n_map_points_ << ",\"live_points\":" << n_live_points_
      << ",\"grid_ms\":" << grid_ms_;
    if (start) {
      e << ",\"start\":" << json(*start);
    }
    if (target) {
      e << ",\"target\":" << json(*target);
    }
    if (!start || !target) {
      e << ",\"ok\":false,\"why\":\"" << (!start ? "start" : "goal") << " not free\"}";
      event(e.str());
      publish_hold();
      return;
    }
    // Same seed for every plan towards a goal: on a slightly changed map, a
    // replan then tends to reproduce a similar path rather than a random new
    // one (fewer direction flips while replanning).
    // After a failure, the next attempt uses a new seed: with the same seed
    // and a near-identical map, a failed search just fails again.
    rrt_->reseed(1000u + 100u * static_cast<std::uint32_t>(goal_idx_) +
      static_cast<std::uint32_t>(fail_streak_));
    ++plans_;
    const auto free = [this](const Vec3 & a, const Vec3 & b) {return inflated_.segment_free(a, b);};
    const auto raw_path = rrt_->plan(*start, *target, free);
    const auto & st = rrt_->stats();
    e << ",\"plan_ms\":" << st.elapsed_ms << ",\"iterations\":" << st.iterations;
    if (!raw_path) {
      e << ",\"ok\":false,\"why\":\"no path within budget\",\"nodes\":" << st.nodes << "}";
      event(e.str());
      ++fail_streak_;
      if (dumps_ < dump_max_) {
        std::ostringstream h;
        h << "# pos " << pos.transpose() << "\n# start " << start->transpose()
          << "\n# target " << target->transpose() << "\n";
        dump_grid("plan_fail_" + std::to_string(dumps_++), h.str());
      }
      publish_hold();  // the next tick tries again with fresh data
      return;
    }
    fail_streak_ = 0;
    path_ = shortcut(*raw_path, free);
    if ((path_.front() - pos).norm() > 1e-6) {
      path_.insert(path_.begin(), pos);  // from where the vehicle actually is
    }
    next_wp_ = std::min<std::size_t>(1, path_.size() - 1);
    e << ",\"ok\":true,\"waypoints\":" << path_.size() << ",\"length_m\":" << path_length(path_)
      << ",\"occupied_cells\":" << inflated_.count_occupied() << ",\"path\":[";
    for (std::size_t i = 0; i < path_.size(); ++i) {
      e << (i ? "," : "") << "[" << path_[i].x() << "," << path_[i].y() << "," << path_[i].z() << "]";
    }
    e << "]}";
    event(e.str());

    nav_msgs::msg::Path out;
    out.header.stamp = now();
    out.header.frame_id = odom_frame_;
    for (const auto & p : path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }
    path_pub_->publish(out);
  }

  static std::string json(const Vec3 & v)
  {
    std::ostringstream o;
    o << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
    return o.str();
  }

  // Diagnostics for offline inspection (scripts/eval_mission.py): the points
  // behind the grid, one "source x y z" per line, source 0 = RTAB-Map's map,
  // 1 = live depth. Written for failed plans (up to debug_dump_max) and at the
  // end of the mission.
  void dump_grid(const std::string & name, const std::string & header)
  {
    if (dump_dir_.empty()) {
      return;
    }
    std::ofstream f(dump_dir_ + "/" + name + ".xyz");
    f << "# t " << now().seconds() << " goal " << goal_idx_ << "\n" << header;
    std::lock_guard lock(data_mutex_);
    for (const auto & p : map_points_) {
      f << "0 " << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }
    for (const auto & fr : live_) {
      for (const auto & p : fr.points) {
        f << "1 " << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
      }
    }
  }

  // An empty path tells quad_offboard to stop and hold position: never leave
  // it following an old path that planning has just found to be blocked.
  void publish_hold()
  {
    path_.clear();
    nav_msgs::msg::Path out;
    out.header.stamp = now();
    out.header.frame_id = odom_frame_;
    path_pub_->publish(out);
  }

  struct LiveFrame
  {
    double stamp;
    std::vector<Vec3> points;
  };

  // config
  std::string odom_frame_;
  int inflate_cells_{8};
  int hard_cells_{6};
  double ground_z_{-0.24};
  double ground_margin_{0.25};
  double depth_max_range_{6.0};
  int depth_stride_{4};
  double live_decay_{4.0};
  double goal_tolerance_{0.6};
  double horizon_m_{5.0};
  std::string dump_dir_;
  int dump_max_{5};
  int dumps_{0};
  std::vector<Vec3> goals_;
  std::ofstream events_out_;  // planning thread only

  // shared (data_mutex_)
  std::mutex data_mutex_;
  Vec3 pos_{Vec3::Zero()};
  bool have_pos_{false};
  std::vector<Vec3> map_points_;
  std::deque<LiveFrame> live_;

  // data-group only
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

  // planning-group only
  VoxelGrid raw_;
  VoxelGrid inflated_;  // planning margin
  VoxelGrid hard_;      // validation margin (tighter)
  std::unique_ptr<RrtStar> rrt_;
  std::vector<Vec3> path_;
  std::size_t next_wp_{0};
  std::size_t goal_idx_{0};
  bool done_{false};
  int plans_{0};
  int fail_streak_{0};
  double grid_ms_{0.0};  // last grid rebuild + inflation [ms]
  std::size_t n_map_points_{0};
  std::size_t n_live_points_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr data_group_;
  rclcpp::CallbackGroup::SharedPtr plan_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace quad_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<quad_planning::PlannerNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
