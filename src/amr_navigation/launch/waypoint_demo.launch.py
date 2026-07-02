"""Waypoint navigation demo: mock robot + waypoint follower (auto-started).

Brings up the mock robot stack and the waypoint follower with ``auto_start``
enabled, so ``ros2 launch amr_navigation waypoint_demo.launch.py`` drives the
robot around the default square route. The follower writes ``/cmd_vel``, which
flows through the safety monitor to the base controller and updates ``/odom``,
which the follower consumes - a closed loop with no Gazebo.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    loop = LaunchConfiguration("loop")
    follower_config = PathJoinSubstitution(
        [FindPackageShare("amr_navigation"), "config", "waypoint_follower.yaml"]
    )

    mock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_bringup"), "launch", "mock_robot.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    follower = Node(
        package="amr_navigation",
        executable="waypoint_follower_node",
        name="waypoint_follower",
        output="screen",
        parameters=[
            follower_config,
            {"auto_start": True, "loop": loop, "use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("loop", default_value="false"),
            mock_robot,
            follower,
        ]
    )
