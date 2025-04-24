from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Path to the Explore Lite launch file
    explore_lite_launch_file = os.path.join(
        get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')
    
    find_object_2d_launch_file = os.path.join(
        get_package_share_directory('ros2_snc_9'), 'find_object_2d.launch.py' 
    )
    
    return LaunchDescription([
        # Include Explore Lite launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(find_object_2d_launch_file)
        ),

        # Include Find Object Launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_lite_launch_file)
        ),

        # Start State Management node
        Node(
            package='ros2_snc_9',
            executable='state_node_executable',
            name='state_server',
            output='screen'
        ),

        # Start Position Tracking & Return home node
        Node(
            package='ros2_snc_9',
            executable='position_tracking_executable',
            name='position_tracking',
            output='screen',
            parameters=[
                {'tracking_interval': 3.0},
                {'path_history_topic': 'path_explore'},
                {'return_path_topic': 'path_return'},
                {'base_frame': 'base_link'},
                {'map_frame': 'map'},
                {'follow_waypoints_action': 'follow_waypoints'},
                {'path_history_max_length' : 1000} 
            ]
        ),

        # Start Nav Node
        Node(
            package='ros2_snc_9',
            executable='nav_node_executable',
            name='navigation_node',
            output='screen'
        ),

        # Start Marker Detection Node
        Node(
            package='ros2_snc_9',
            executable='hazard_detection_executable',
            name='hazard_detection',
            output='screen'
        ),

        
    ])