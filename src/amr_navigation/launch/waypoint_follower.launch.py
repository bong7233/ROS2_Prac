from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("amr_navigation"), "config", "waypoint_follower.yaml"]
    )
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the waypoint follower parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock from simulation when true.",
            ),
            Node(
                package="amr_navigation",
                executable="waypoint_follower_node",
                name="waypoint_follower",
                output="screen",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
