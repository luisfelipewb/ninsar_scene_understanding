"""Live calibration: the detector node plus RViz, ready for `ros2 bag play`."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = "ninsar_lidar_calib"


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare(PACKAGE)
    params_file = LaunchConfiguration("params_file")
    detections = LaunchConfiguration("detections")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([share, "config", "lidar_calib.yaml"]),
            description="calibration parameter file; every value is live-tunable",
        ),
        DeclareLaunchArgument(
            "detections",
            default_value="detections.jsonl",
            description="where accepted detections are appended",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(
            package=PACKAGE,
            executable="calib_node",
            name="lidar_calib_node",
            output="screen",
            parameters=[params_file, {"detections_path": detections}],
            # bags carry their own clock; without this TF lookups miss
            arguments=["--ros-args", "-p", "use_sim_time:=true"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(use_rviz),
            arguments=[
                "-d", PathJoinSubstitution([share, "rviz", "calibration.rviz"]),
            ],
            parameters=[{"use_sim_time": True}],
        ),
    ])
