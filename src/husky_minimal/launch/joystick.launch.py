from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node - reads joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # Change if your joystick is on a different device
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Teleop twist joy - converts joystick to velocity commands
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,      # Left stick up/down for forward/backward
                'axis_angular.yaw': 0,   # Left stick left/right for turning
                'scale_linear.x': 1.0,   # Max linear speed
                'scale_angular.yaw': 1.5, # Max angular speed
                'enable_button': 4,      # L1/LB button to enable (hold to move)
                'enable_turbo_button': 5, # R1/RB button for turbo mode
            }]
        ),
    ])
