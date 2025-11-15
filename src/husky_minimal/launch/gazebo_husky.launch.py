# import yaml
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():
#     # Package Directories
#     pkg_husky_minimal = get_package_share_directory('husky_minimal')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     # Paths
#     urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky_ros2.urdf.xacro')
#     # urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky.urdf.xacro')
#     controller_config = os.path.join(pkg_husky_minimal, 'config', 'husky_controllers.yaml')

#     # Launch Arguments
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='0.0')
#     y_pose = LaunchConfiguration('y_pose', default='0.0')
#     z_pose = LaunchConfiguration('z_pose', default='0.5')

#     # Gazebo launch
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
#         ),
#     )

#     # Convert the Xacro output to a proper string parameter
#     robot_description = ParameterValue(
#         Command(['xacro ', urdf_file]),
#         value_type=str
#     )

#     # Robot State Publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'robot_description': robot_description
#         }]
#     )

#     # Spawn Robot in Gazebo
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', 'robot_description',
#             '-entity', 'husky',
#             '-x', x_pose,
#             '-y', y_pose,
#             '-z', z_pose
#         ],
#         output='screen'
#     )

#     # Joint State Broadcaster Spawner (delayed to ensure Gazebo is ready)
#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#         output='screen'
#     )

#     # Diff Drive Controller Spawner (starts after joint_state_broadcaster)
#     diff_drive_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
#         output='screen'
#     )

#     # Delay controller spawners until after robot is spawned in Gazebo
#     delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=spawn_entity,
#             on_exit=[joint_state_broadcaster_spawner],
#         )
#     )

#     # Start diff_drive_controller after joint_state_broadcaster
#     delay_diff_drive_controller_after_joint_state = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=joint_state_broadcaster_spawner,
#             on_exit=[diff_drive_controller_spawner],
#         )
#     )

#     # Launch Description
#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='true'),
#         DeclareLaunchArgument('x_pose', default_value='0.0'),
#         DeclareLaunchArgument('y_pose', default_value='0.0'),
#         DeclareLaunchArgument('z_pose', default_value='0.5'),
#         gazebo,
#         robot_state_publisher,
#         spawn_entity,
#         delay_joint_state_broadcaster_after_spawn,
#         delay_diff_drive_controller_after_joint_state,
#     ])



import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package Directories
    pkg_husky_minimal = get_package_share_directory('husky_minimal')
    pkg_custom_robot = get_package_share_directory('custom_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    urdf_file = os.path.join(pkg_husky_minimal, 'urdf', 'husky_ros2.urdf.xacro')
    controller_config = os.path.join(pkg_husky_minimal, 'config', 'husky_controllers.yaml')
    # world_file = os.path.join(pkg_custom_robot, 'worlds', 'warehouse.world')
    world_file = os.path.join(pkg_custom_robot, 'worlds', 'custom_warehouse.world')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')

    # Gazebo launch with warehouse world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Convert the Xacro output to a proper string parameter
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'husky',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner (delayed to ensure Gazebo is ready)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Diff Drive Controller Spawner (starts after joint_state_broadcaster)
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay controller spawners until after robot is spawned in Gazebo
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Start diff_drive_controller after joint_state_broadcaster
    delay_diff_drive_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # Launch Description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position to spawn the robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position to spawn the robot'),
        DeclareLaunchArgument('z_pose', default_value='0.5', description='Z position to spawn the robot'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster_after_spawn,
        delay_diff_drive_controller_after_joint_state,
    ])