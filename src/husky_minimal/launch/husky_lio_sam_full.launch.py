#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    pkg_lio_sam = get_package_share_directory('lio_sam')
    
    params_file = os.path.join(pkg_husky_minimal, 'config', 'lio_sam_params.yaml')
    
    # Launch Gazebo + Husky
    gazebo_husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_husky_minimal, 'launch', 'gazebo_husky.launch.py')
        )
    )
    
    # Launch LIO-SAM
    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')
        ),
        launch_arguments={'params_file': params_file}.items()
    )
    
    # Teleop
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )
    
    return LaunchDescription([
        gazebo_husky,
        lio_sam,
        teleop
    ])
