"""The tunable TF publisher plus rqt_reconfigure, ready to dial in an extrinsic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "ninsar_tf_tools"


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare(PACKAGE)
    params_file = LaunchConfiguration("params_file")
    node_name = LaunchConfiguration("node_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    static_tf = LaunchConfiguration("static_tf")
    use_rqt = LaunchConfiguration("rqt")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([share, "config", "tf_publisher.yaml"]),
            description="parameter file; every value but static_tf is live-tunable",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="tf_publisher",
            description="node name; give each instance its own to run several",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="true when driving this off `ros2 bag play`, which "
                        "carries its own clock",
        ),
        DeclareLaunchArgument(
            "static_tf",
            default_value="false",
            description="latch on /tf_static instead of broadcasting on /tf; "
                        "immune to clock mismatches, so it is the safe choice "
                        "over bag playback",
        ),
        DeclareLaunchArgument("rqt", default_value="true"),

        Node(
            package=PACKAGE,
            executable="tf_publisher",
            name=node_name,
            output="screen",
            parameters=[params_file, {
                # value_type is explicit: a launch argument arrives as the
                # string "false", which a bool parameter will not accept
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "static_tf": ParameterValue(static_tf, value_type=bool),
            }],
        ),
        # rqt_reconfigure is left on wall time on purpose: it only talks to the
        # parameter services, and giving it sim time makes its own timers stall
        # whenever /clock does.
        Node(
            package="rqt_reconfigure",
            executable="rqt_reconfigure",
            name="rqt_reconfigure",
            condition=IfCondition(use_rqt),
        ),
    ])
