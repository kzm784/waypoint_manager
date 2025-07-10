import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    host_server_node = Node(
        package="waypoint_function_server",
        executable='host_server',
        name='host_server',
        output='screen',
    )
    
    wait_server_node = Node(
        package="waypoint_function_wait_server",
        executable='wait_server',
        name='wait_server',
        output='screen',
    )

    key_wait_server_node = Node(
        package="waypoint_function_wait_server",
        executable='key_wait_server',
        name='key_wait_server',
        output='screen',
    )

    test_server_node = Node(
        package="waypoint_function_server_example",
        executable='test_server',
        name='test_server',
        output='screen',
    )

    skip_server_node = Node(
        package="waypoint_function_skip_server",
        executable='skip_server',
        name='skip_server',
        output = 'screen',
        remappings=[
            {'current_pose', 'current_pose'},
            {'scan', 'scan'}
        ]
    )
    
    ld.add_action(host_server_node)
    # ld.add_action(test_server_node)
    ld.add_action(wait_server_node)
    ld.add_action(key_wait_server_node)
    ld.add_action(skip_server_node)


    return ld