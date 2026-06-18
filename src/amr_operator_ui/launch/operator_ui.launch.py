from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="amr_operator_ui",
                executable="amr_operator_ui_node",
                name="amr_operator_ui",
                output="screen",
            )
        ]
    )
