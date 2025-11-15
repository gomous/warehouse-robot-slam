#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directories
    pkg_lio_sam = get_package_share_directory('lio_sam')
    pkg_custom_robot = get_package_share_directory('custom_robot_description')
    
    # Use CUSTOM parameters from your robot package
    params_file = os.path.join(pkg_custom_robot, 'config', 'lio_sam_params.yaml')
    
    # Launch LIO-SAM with your custom parameters
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')
        ),
        launch_arguments={
            'params_file': params_file  # This overrides the default params
        }.items()
    )
    
    return LaunchDescription([
        lio_sam_launch
    ])
