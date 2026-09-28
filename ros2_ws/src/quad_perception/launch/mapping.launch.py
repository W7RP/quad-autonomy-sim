"""Phase 3 live mapping: Gazebo RGB-D -> ROS, PX4 odometry -> TF, RTAB-Map.

    ros2 launch quad_perception mapping.launch.py [rviz:=true] [database:=/path/rtabmap.db]
                                                  [odom_source:=px4|gt] [loop_closure:=true|false]

odom_source:=px4 (default) maps on PX4 EKF2's odometry, as a real vehicle would.
odom_source:=gt is a SIMULATION-ONLY diagnostic: Gazebo's true pose, stamped on
the camera's clock. Comparing the two separates odometry errors from mapping
errors.

loop_closure:=false turns off RTAB-Map's loop-closure and proximity detection:
the map is then built on the odometry poses alone. Phase 4 plans on the map in
the odometry frame and uses this. With GPS-aided EKF2 there is no drift to
correct, and in the repetitive demo world the visual loop closures bent the
graph: the same database scored 7.5 cm median surface error with its closures
vs 1.9 cm on odometry poses (docs/phase4_autonomy.md).

Expects the simulator to be running with the x500_mapper vehicle
(scripts/sim.sh --model x500_mapper --world cluttered). All nodes use
simulation time, so camera and odometry stamps share one clock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Camera mount, base_link (FLU) -> rgbd_link. MUST match the <pose> of rgbd_link
# in sim/models/x500_mapper/model.sdf: 15 cm forward, 5 cm up, 15 deg down.
CAMERA_XYZ = ("0.15", "0", "0.05")
CAMERA_RPY = ("0", "0.2618", "0")
# rgbd_link (x forward) -> rgbd_optical_frame (z forward, x right, y down).
OPTICAL_RPY = ("-1.5707963", "0", "-1.5707963")


def static_tf(parent, child, xyz, rpy):
    return Node(
        package="tf2_ros", executable="static_transform_publisher",
        name=f"tf_{parent}_to_{child}",
        arguments=["--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
                   "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
                   "--frame-id", parent, "--child-frame-id", child],
        parameters=[{"use_sim_time": True}])


# RTAB-Map parameters are strings. Kp/MaxFeatures -1 disables loop-closure
# detection (no visual vocabulary); proximity detection is a separate switch.
NO_LOOP_CLOSURE = {"Kp/MaxFeatures": "-1", "RGBD/ProximityBySpace": "false"}


def rtabmap_node(context):
    pkg = FindPackageShare("quad_perception")
    params = [PathJoinSubstitution([pkg, "config", "rtabmap.yaml"]),
              {"database_path": LaunchConfiguration("database")}]
    if LaunchConfiguration("loop_closure").perform(context) == "false":
        params.append(NO_LOOP_CLOSURE)
    return [Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
                 parameters=params,
                 remappings=[("rgb/image", "/camera/color/image_raw"),
                             ("depth/image", "/camera/depth/image_raw"),
                             ("rgb/camera_info", "/camera/color/camera_info")],
                 arguments=["--delete_db_on_start"])]


def generate_launch_description():
    pkg = FindPackageShare("quad_perception")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("odom_source", default_value="px4", choices=["px4", "gt"]),
        DeclareLaunchArgument("loop_closure", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("database", default_value="/tmp/quad_rtabmap.db",
                              description="RTAB-Map database (deleted at start)"),

        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             parameters=[{"config_file": PathJoinSubstitution([pkg, "config", "gz_bridge.yaml"]),
                          "use_sim_time": True}]),

        static_tf("base_link", "rgbd_link", CAMERA_XYZ, CAMERA_RPY),
        static_tf("rgbd_link", "rgbd_optical_frame", ("0", "0", "0"), OPTICAL_RPY),

        Node(package="quad_perception", executable="px4_odometry_bridge",
             parameters=[{"use_sim_time": True}],
             condition=LaunchConfigurationEquals("odom_source", "px4")),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_gt_odom",
             parameters=[{"config_file": PathJoinSubstitution([pkg, "config", "gz_bridge_gt_odom.yaml"]),
                          "use_sim_time": True}],
             condition=LaunchConfigurationEquals("odom_source", "gt")),

        OpaqueFunction(function=rtabmap_node),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", PathJoinSubstitution([pkg, "config", "mapping.rviz"])],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
