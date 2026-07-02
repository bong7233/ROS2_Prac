from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_odom = LaunchConfiguration("use_odom")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_odom",
                default_value="false",
                description="Move the modelled sensor with /odom when true.",
            ),
            Node(
                package="amr_lidar_driver",
                executable="mock_lidar_driver_node",
                name="mock_lidar_driver",
                output="screen",
                parameters=[
                    {
                        "use_odom": use_odom,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
