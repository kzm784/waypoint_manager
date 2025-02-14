import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Sync Template Server Node
    sync_template_server_node = Node(
        package='waypoint_function_server_template',
        executable='sync_template_server',
        namespace='waypoint_function',
        output='screen',
    )
    ld.add_action(sync_template_server_node)
    
    # Async Template Server Node
    async_template_server_node = Node(
        package='waypoint_function_server_template',
        executable='async_template_server',
        namespace='waypoint_function',
        output='screen',
    )
    ld.add_action(async_template_server_node)

    return ld