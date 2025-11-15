#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directories
    pkg_custom_robot = get_package_share_directory('custom_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    urdf_file = os.path.join(pkg_custom_robot, 'urdf', 'custom_robot.urdf.xacro')
    world_file = os.path.join(pkg_custom_robot, 'worlds', 'custom_warehouse.world')
    # world_file = os.path.join(pkg_custom_robot, 'worlds', 'warehouse.world')
    
    # Process URDF
    import xacro
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'custom_robot', '-topic', 'robot_description']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            name='teleop_keyboard'
        ),
    ])