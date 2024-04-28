from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reworked_services',
            executable='move_client_node',
            output='screen'),
    ])