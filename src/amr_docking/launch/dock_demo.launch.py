"""Full docking demo: mock camera + ArUco detector + docking controller.

Brings up the ``amr_vision`` perception pipeline and the docking controller with
``auto_start`` enabled, so ``ros2 launch amr_docking dock_demo.launch.py`` shows
the controller reacting to the docking error end to end. The controller writes to
``/cmd_vel``; bring up the mock robot (or Gazebo) separately to actually move the
base and pass the command through the safety monitor.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_config = PathJoinSubstitution(
        [FindPackageShare("amr_docking"), "config", "docking_controller.yaml"]
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("amr_vision"),
                    "launch",
                    "docking_vision.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    controller = Node(
        package="amr_docking",
        executable="docking_controller_node",
        name="docking_controller",
        output="screen",
        parameters=[
            controller_config,
            {"auto_start": True, "use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            vision,
            controller,
        ]
    )
