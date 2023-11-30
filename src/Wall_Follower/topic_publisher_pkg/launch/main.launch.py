from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_service_pkg',
            executable='find_wall_server_stddv',
            output='screen'),
        Node(
            package='action_service',
            executable='action_server_node',
            output='screen'),
        Node(
            package='topic_publisher_pkg',
            executable='move_robot_service_action_node_v1',
            output='screen')
    ])

