"""Full integrated AMR demo, no Gazebo required.

Brings up the whole stack and wires the command pipeline through the twist mux:

    waypoint follower -> cmd_vel_nav  ┐
    docking controller -> cmd_vel_dock├─ twist_mux -> /cmd_vel -> safety
    (operator)          -> cmd_vel_teleop ┘                        -> base -> /odom

Defaults: the waypoint follower auto-starts and loops a square patrol; docking is
idle until enabled. Priority is teleop > dock > nav, so publishing
``/cmd_vel_teleop`` or calling ``/enable_docking`` overrides the patrol.

    ros2 launch amr_bringup full_system.launch.py
    ros2 service call /enable_docking std_srvs/srv/SetBool "{data: true}"
    ros2 topic pub /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MARKER_ID = 3
MARKER_LENGTH = 0.20


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    marker_world_x = LaunchConfiguration("marker_world_x")
    marker_world_y = LaunchConfiguration("marker_world_y")

    sim_time = {"use_sim_time": use_sim_time}

    mock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_bringup"), "launch", "mock_robot.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    twist_mux = Node(
        package="amr_twist_mux",
        executable="twist_mux_node",
        name="twist_mux",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("amr_twist_mux"), "config", "twist_mux.yaml"]
            ),
            sim_time,
        ],
    )

    mock_lidar = Node(
        package="amr_lidar_driver",
        executable="mock_lidar_driver_node",
        name="mock_lidar_driver",
        output="screen",
        parameters=[{"use_odom": True, **sim_time}],
    )

    mock_camera = Node(
        package="amr_vision",
        executable="mock_dock_camera_node",
        name="mock_dock_camera",
        output="screen",
        parameters=[
            {
                "use_odom": True,
                "marker_id": MARKER_ID,
                "marker_length_m": MARKER_LENGTH,
                "marker_world_x": marker_world_x,
                "marker_world_y": marker_world_y,
                "camera_forward_offset_m": 0.40,
                **sim_time,
            }
        ],
    )

    detector = Node(
        package="amr_vision",
        executable="aruco_docking_node",
        name="aruco_docking",
        output="screen",
        parameters=[
            {"marker_id": MARKER_ID, "marker_length_m": MARKER_LENGTH, **sim_time}
        ],
    )

    docking_controller = Node(
        package="amr_docking",
        executable="docking_controller_node",
        name="docking_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("amr_docking"), "config", "docking_controller.yaml"]
            ),
            {"auto_start": False, **sim_time},
        ],
        remappings=[("cmd_vel", "cmd_vel_dock")],
    )

    waypoint_follower = Node(
        package="amr_navigation",
        executable="waypoint_follower_node",
        name="waypoint_follower",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("amr_navigation"), "config", "waypoint_follower.yaml"]
            ),
            {"auto_start": True, "loop": True, **sim_time},
        ],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("marker_world_x", default_value="2.0"),
            DeclareLaunchArgument("marker_world_y", default_value="0.35"),
            mock_robot,
            twist_mux,
            mock_lidar,
            mock_camera,
            detector,
            docking_controller,
            waypoint_follower,
        ]
    )
