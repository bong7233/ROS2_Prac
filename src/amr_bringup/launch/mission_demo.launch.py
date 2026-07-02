"""Autonomous mission demo: battery-aware patrol and charge, no Gazebo.

Brings up the full stack with the waypoint follower and docking controller idle,
and lets the mission coordinator drive them based on battery and dock status:

    PATROL (nav) --battery low--> RETURN_TO_DOCK (dock) --docked--> CHARGING
       ^                                                              |
       +------------------------ battery full -----------------------+

Drive the battery with the FAE tool to watch the transitions (the mock battery
discharges slowly and does not self-charge):

    ros2 launch amr_bringup mission_demo.launch.py
    ros2 run amr_tools fault_scenario battery-low      # -> RETURN_TO_DOCK, docks
    ros2 run amr_tools fault_scenario battery-normal    # 0.85 < full, stays CHARGING
    ros2 service call /set_battery_percentage \
      amr_interfaces/srv/SetBatteryPercentage "{percentage: 0.95}"   # -> PATROL
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
                "marker_world_x": 2.0,
                "marker_world_y": 0.35,
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
            {"auto_start": False, "loop": True, **sim_time},
        ],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    mission_coordinator = Node(
        package="amr_mission",
        executable="mission_coordinator_node",
        name="mission_coordinator",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("amr_mission"), "config", "mission.yaml"]
            ),
            {"auto_start": True, **sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            mock_robot,
            twist_mux,
            mock_lidar,
            mock_camera,
            detector,
            docking_controller,
            waypoint_follower,
            mission_coordinator,
        ]
    )
