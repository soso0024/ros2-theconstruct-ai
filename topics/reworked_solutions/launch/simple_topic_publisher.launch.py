from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar',
            executable='simple_publisher_node',
            output='screen'),
    ])

# FOR EXAM
# ---------------------------------------------------------------------------------------
# from simple_launch import SimpleLauncher

# def generate_launch_description():
#     sl = SimpleLauncher()

#     sl.node(package = 'sofar', executable = 'simple_publisher_node', output = 'screen')

#     return sl.launch_description()