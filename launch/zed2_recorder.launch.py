from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_open_capture',
            executable='anyrosDataCollector',
            name='anyrosDataCollector',
            output='screen',
            parameters=[
                {'fps': 15}
            ]
        )
    ])