from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reworked_services',
            executable='service_client_v2_node',
            output='screen'),
    ])