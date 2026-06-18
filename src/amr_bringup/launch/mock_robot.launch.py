from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def configured_node(package, executable, name, config_file, use_sim_time):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
    )


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("amr_bringup"), "config", "mock_robot.yaml"]
    )

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the AMR mock robot parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock from simulation when true.",
            ),
            configured_node(
                "amr_battery_driver",
                "mock_battery_driver_node",
                "mock_battery_driver",
                config_file,
                use_sim_time,
            ),
            configured_node(
                "amr_io_driver",
                "mock_io_driver_node",
                "mock_io_driver",
                config_file,
                use_sim_time,
            ),
            configured_node(
                "amr_motor_driver",
                "mock_motor_driver_node",
                "mock_motor_driver",
                config_file,
                use_sim_time,
            ),
            configured_node(
                "amr_safety_monitor",
                "safety_monitor_node",
                "safety_monitor",
                config_file,
                use_sim_time,
            ),
            configured_node(
                "amr_base_controller",
                "diff_drive_base_controller_node",
                "diff_drive_base_controller",
                config_file,
                use_sim_time,
            ),
            configured_node(
                "amr_system_manager",
                "system_manager_node",
                "system_manager",
                config_file,
                use_sim_time,
            ),
        ]
    )

