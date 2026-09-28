"""Phase 4 autonomy stack: Phase 3 mapping (RTAB-Map on PX4 odometry) + planner.

    ros2 launch quad_planning autonomy.launch.py [mission:=/path/mission.yaml] [rviz:=true]

Expects the simulator running with x500_mapper (scripts/sim.sh --model
x500_mapper --world demo_final). Fly it with quad_offboard in planner mode:
    ros2 run quad_offboard offboard_square --ros-args -p route_source:=planner ...
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning = FindPackageShare("quad_planning")
    return LaunchDescription([
        DeclareLaunchArgument("mission", default_value=PathJoinSubstitution(
            [planning, "config", "mission_demo.yaml"])),
        DeclareLaunchArgument("database", default_value="/tmp/quad_rtabmap.db"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("events_file", default_value=""),
        DeclareLaunchArgument("debug_dump_dir", default_value="",
                              description="write the occupancy behind failed plans here"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("quad_perception"), "launch", "mapping.launch.py"])),
            # Mapping on odometry poses: the planner flies in the odometry
            # frame, and loop closures only bent the map (mapping.launch.py).
            launch_arguments={"database": LaunchConfiguration("database"),
                              "rviz": LaunchConfiguration("rviz"),
                              "loop_closure": "false"}.items()),
        Node(package="quad_planning", executable="planner_node", name="planner",
             parameters=[PathJoinSubstitution([planning, "config", "planner.yaml"]),
                         LaunchConfiguration("mission"),
                         {"events_file": LaunchConfiguration("events_file"),
                          "debug_dump_dir": LaunchConfiguration("debug_dump_dir")}]),
    ])
