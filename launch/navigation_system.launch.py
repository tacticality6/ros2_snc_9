from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Nav2 launch file
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    
    # Path to the Explore Lite launch file
    explore_lite_launch_file = os.path.join(
        get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')
    
    return LaunchDescription([
        # Include Nav2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file)
        ),

        # Include Explore Lite launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_lite_launch_file)
        ),

        # Start Custom Node
        Node(
            package='your_package',
            executable='your_custom_node',
            name='custom_node',
            output='screen'
        )
    ])