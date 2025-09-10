from launch import LaunchDescription
from launch_ros.actions import Node


# ros2 param set /delay_node delay_sec 2.0  # pour mode Lune
# ros2 param set /delay_node delay_sec 0.0  # pour mode Terre

def generate_launch_description():
    return LaunchDescription([
        # Delay node C++ pour les commandes utilisateur
        Node(
            package='demo_delayed_teleop',
            executable='delay_node',
            name='delay_node'
        ),
        # Safety node Python
        Node(
            package='demo_delayed_teleop',
            executable='safety_node.py',
            name='safety_node'
        ),
        # Color detector node Python
        Node(
            package='demo_delayed_teleop',
            executable='color_detector.py',
            name='color_detector'
        ),
        # joy_node utilisateur
        Node(
            package='joy',
            executable='joy_node',
            name='joy_user',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        # joy_node sécurité
        Node(
            package='joy',
            executable='joy_node',
            name='joy_safety',
            output='screen',
            parameters=[{'dev': '/dev/input/js1'}]
        )
    ])
