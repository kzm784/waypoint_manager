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
        package='waypoint_event_handler',
        executable='host_server',
        namespace='waypoint_event',
        output='screen',
    )
    ld.add_action(host_server_node)

    # Skip Server Node
    skip_server_node = Node(
        package='waypoint_event_handler',
        executable='skip_server',
        namespace='waypoint_event',
        output='screen'
    )
    ld.add_action(skip_server_node)

    # Wait Server Node
    wait_server_node = Node(
        package='waypoint_event_handler',
        executable='wait_server',
        namespace='waypoint_event',
        output='screen'
    )
    ld.add_action(wait_server_node)

    # Speak Server Node
    speak_server_node = Node(
        package='waypoint_event_handler',
        executable='speak_server',
        namespace='waypoint_event',
        output='screen'
    )
    ld.add_action(speak_server_node)

    # Map Change Server Node
    map_change_server_node = Node(
        package='waypoint_event_handler',
        executable='map_change_server',
        namespace='waypoint_event',
        output='screen'
    )
    ld.add_action(map_change_server_node)


    return ld