from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="setpoint_gen",
            executable="brightness_setpoint",
            parameters=[
                {"threshold": 200}
            ]
        ),
        Node(
            package="setpoint_gen",
            executable="sub_moving"
        )
    ])