#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_diff_driver_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_base_controller"],
        remappings=[("/cmd_vel_unstamped", "/cmd_vel")],
        output="screen",
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_controller,
            spawn_diff_driver_controller
        ]
    )