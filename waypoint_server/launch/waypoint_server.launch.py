import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    waypoint_server_node = Node(
        package='waypoint_server',
        executable='waypoint_server_node',
        name='waypoint_server_node',
        output='screen'
    )
    
    ld.add_action(waypoint_server_node)

    return ld