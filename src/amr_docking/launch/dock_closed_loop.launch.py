"""Fully closed-loop mock docking demo (no Gazebo required).

Brings up the mock robot stack, a mock camera that renders a marker fixed in the
world (odom) frame, the ArUco detector, and the docking controller. As the
controller drives ``/cmd_vel``, it flows through safety -> base controller ->
``/odom``; the camera re-renders the marker from the new robot pose, so the
docking error genuinely closes the loop:

    controller -> /cmd_vel -> safety -> base -> /odom -> camera -> detector ->
    /docking_state -> controller

The robot starts at the origin and the marker sits ahead and slightly to one
side, so you can watch the ALIGN -> APPROACH -> DOCKED progression.
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
    controller_config = PathJoinSubstitution(
        [FindPackageShare("amr_docking"), "config", "docking_controller.yaml"]
    )

    mock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_bringup"), "launch", "mock_robot.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
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
                "publish_rate_hz": 15.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    detector = Node(
        package="amr_vision",
        executable="aruco_docking_node",
        name="aruco_docking",
        output="screen",
        parameters=[
            {
                "marker_id": MARKER_ID,
                "marker_length_m": MARKER_LENGTH,
                "use_sim_time": use_sim_time,
            }
        ],
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
            DeclareLaunchArgument("marker_world_x", default_value="2.0"),
            DeclareLaunchArgument("marker_world_y", default_value="0.35"),
            mock_robot,
            mock_camera,
            detector,
            controller,
        ]
    )
