from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Position Tracking & Return home node
        Node(
            package='ros2_snc_9',
            executable='position_tracking_executable',
            name='position_tracking',
            parameters=[
                {'tracking_interval': 2.0},
                {'path_history_topic': 'path_explore'},
                {'return_path_topic': 'path_return'},
                {'base_frame': 'base_link'},
                {'map_frame': 'map'},
                {'follow_waypoints_action': 'follow_waypoints'},
                {'path_history_max_length' : 1000} 
            ]
        ),

        # Start State Management node
        Node(
            package='ros2_snc_9',
            executable='state_node_executable',
            name='state_server',
            output='screen'
        ),
    ])