from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar',
            executable='simple_publisher_composable_node',
            output='screen'),
    ])