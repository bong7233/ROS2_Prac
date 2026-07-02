"""Mock robot with sensor TF and laser scan, for RViz and Nav2 readiness.

Extends ``mock_robot.launch.py`` with the pieces a navigation stack needs but the
headless mock omits, matching the project's documented TF design
(``odom -> base_link``, ``base_link -> laser``):

* ``base_link -> lidar_link`` / ``base_link -> camera_link`` static transforms
  (offsets taken from the URDF), so sensor frames exist without pulling in the
  full robot_state_publisher/xacro pipeline.
* ``mock_lidar_driver`` - publishes ``/scan`` (driven by ``/odom``).

The base controller already publishes ``odom -> base_link`` and ``/odom``, so with
this launch the core Nav2 inputs (TF, odometry, scan) are present without Gazebo.
RViz is optional via the ``rviz`` argument.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_tf(name, xyz, parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution(
        [FindPackageShare("amr_bringup"), "config", "mock_robot.yaml"]
    )

    mock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_bringup"), "launch", "mock_robot.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": config_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("config_file", default_value=default_config),
            mock_robot,
            # Sensor mounts (offsets match amr_description URDF joints).
            static_tf("lidar_static_tf", ["0.23", "0.0", "0.175"], "base_link", "lidar_link"),
            static_tf("camera_static_tf", ["0.40", "0.0", "0.10"], "base_link", "camera_link"),
            Node(
                package="amr_lidar_driver",
                executable="mock_lidar_driver_node",
                name="mock_lidar_driver",
                output="screen",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
