import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Host Server Node
    host_server_node = Node(
        package='waypoint_function_main_server',
        executable='host_server',
        namespace='waypoint_function',
        output='screen',
    )
    ld.add_action(host_server_node)

    # Wait Server Node
    wait_server_node = Node(
        package='waypoint_function_main_server',
        executable='wait_server',
        namespace='waypoint_function',
        output='screen'
    )
    ld.add_action(wait_server_node)

    return ld