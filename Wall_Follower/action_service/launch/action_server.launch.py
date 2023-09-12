from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_service',
            executable='action_server_node',
            output='screen'),
    ])