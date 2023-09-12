from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_service_pkg',
            executable='find_wall_server',
            output='screen'),
        
        Node(
            package='wall_service_pkg',
            executable='service_client_v2',
            output='screen')
    ])