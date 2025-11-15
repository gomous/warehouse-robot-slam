import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky_ros2.urdf.xacro')
    # urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky.urdf.xacro')
    rviz_config = os.path.join(pkg_husky_minimal, 'rviz', 'husky.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Convert xacro output to string explicitly
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher GUI (THIS WAS MISSING!)
        # This publishes joint states so RViz can show the transforms
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])