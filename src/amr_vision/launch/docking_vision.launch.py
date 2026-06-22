from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("amr_vision"), "config", "docking.yaml"]
    )
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common_params = [config_file, {"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the amr_vision docking parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock from simulation when true.",
            ),
            Node(
                package="amr_vision",
                executable="mock_dock_camera_node",
                name="mock_dock_camera",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="amr_vision",
                executable="aruco_docking_node",
                name="aruco_docking",
                output="screen",
                parameters=common_params,
            ),
        ]
    )
