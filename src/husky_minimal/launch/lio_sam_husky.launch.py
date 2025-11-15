#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_lio_sam = get_package_share_directory('lio_sam')
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    
    # Use Husky-specific parameters
    params_file = os.path.join(pkg_husky_minimal, 'config', 'lio_sam_params.yaml')
    
    # Launch LIO-SAM with Husky parameters
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')
        ),
        launch_arguments={
            'params_file': params_file
        }.items()
    )
    
    return LaunchDescription([
        lio_sam_launch
    ])
