from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_snc_9',
            executable='position_tracking_node.py',
            name='position_tracking',
            parameters=[
                {'tracking_interval': 2},
                {'path_history_topic': 'robot_path_explore'},
                {'return_path_topic': 'robot_path_return'},
                {'base_frame': 'base_link'},
                {'map_frame': 'map'} 
            ]
        ),
    ])