from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="setpoint_gen",
            executable="setpoint_gen",
        ),
        Node(
            package="setpoint_gen",
            executable="sub_moving"
        )
    ])