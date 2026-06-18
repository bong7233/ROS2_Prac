from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    world_file = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")
    robot_sdf = LaunchConfiguration("robot_sdf")
    robot_xacro = LaunchConfiguration("robot_xacro")
    rviz_config = LaunchConfiguration("rviz_config")
    mock_config = LaunchConfiguration("mock_config")

    default_world = PathJoinSubstitution(
        [FindPackageShare("amr_sim"), "worlds", "amr_warehouse.sdf"]
    )
    default_bridge_config = PathJoinSubstitution(
        [FindPackageShare("amr_sim"), "config", "gazebo_bridge.yaml"]
    )
    default_robot_sdf = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "models", "amr_demo_robot", "model.sdf"]
    )
    default_robot_xacro = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr_demo_robot.urdf.xacro"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "rviz", "amr_gazebo.rviz"]
    )
    default_mock_config = PathJoinSubstitution(
        [FindPackageShare("amr_sim"), "config", "gazebo_robot.yaml"]
    )

    robot_description = {
        "robot_description": Command(["xacro ", robot_xacro]),
        "use_sim_time": use_sim_time,
    }

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    robot_sdf,
                    "-name",
                    "amr_demo_robot",
                    "-allow_renaming",
                    "false",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.02",
                ],
                output="screen",
            )
        ],
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
            "config_file": bridge_config,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("bridge_config", default_value=default_bridge_config),
            DeclareLaunchArgument("robot_sdf", default_value=default_robot_sdf),
            DeclareLaunchArgument("robot_xacro", default_value=default_robot_xacro),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("mock_config", default_value=default_mock_config),
            gazebo,
            spawn_robot,
            bridge,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            configured_node(
                "amr_battery_driver",
                "mock_battery_driver_node",
                "mock_battery_driver",
                mock_config,
                use_sim_time,
            ),
            configured_node(
                "amr_io_driver",
                "mock_io_driver_node",
                "mock_io_driver",
                mock_config,
                use_sim_time,
            ),
            configured_node(
                "amr_motor_driver",
                "mock_motor_driver_node",
                "mock_motor_driver",
                mock_config,
                use_sim_time,
            ),
            configured_node(
                "amr_safety_monitor",
                "safety_monitor_node",
                "safety_monitor",
                mock_config,
                use_sim_time,
            ),
            configured_node(
                "amr_system_manager",
                "system_manager_node",
                "system_manager",
                mock_config,
                use_sim_time,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
