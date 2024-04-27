from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar',
            executable='move_robot_node',
            output='screen'),
    ])