from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ros2_snc_9',
            executable='hazard_detection_executable',
            name='hazard_detection',
            output='screen'
        )
    ])
