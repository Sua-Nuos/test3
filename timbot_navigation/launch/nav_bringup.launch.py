#!/usr/bin/env python3 
import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text="WARN"),
        # default_value=TextSubstitution(text="ERROR"),
        description="Logging level"
    )
    
    # Get the path to the YAML file
    config_file = os.path.join(
        get_package_share_directory('timbot_navigation'),
        'config',
        'ultrasonics_params.yaml'
    )

    node = Node(
        package='timbot_navigation',  
        executable='range_publisher.py',
        name='range_publisher',
        output="screen",
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        log_level_arg,  # เพิ่มเข้าไปใน LaunchDescription
        node
    ])
