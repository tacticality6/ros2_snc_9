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
    
    return LaunchDescription([
        # Include Explore Lite launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_lite_launch_file),
            launch_arguments=[{'transform_tolerance':0.25}]
        ),

        Node(
            package='ros2_snc_9',
            executable='state_node_executable',
            name='state_server',
            output='screen'
        ),

        # Start Nav Node
        Node(
            package='ros2_snc_9',
            executable='nav_node_executable',
            name='navigation_node',
            output='screen'
        ),
    ])