import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the waypoint_navigator config
    waypoint_function_server_config = launch.substitutions.LaunchConfiguration(
        'waypoint_function_server_config',
        default=os.path.join(
            get_package_share_directory('waypoint_function_server'),
                'config',
                'config_waypoint_function_server.yaml'
        )
    )

    host_server_node = Node(
        package="waypoint_function_server",
        executable='host_server',
        name='host_server_node',
        output='screen',
    )
    
    wait_server_node = Node(
        package="waypoint_function_example_server",
        executable='wait_server',
        name='wait_server_node',
        output='screen',
    )

    pause_server_node = Node(
        package="waypoint_function_example_server",
        executable='pause_server',
        name='pause_server_node',
        output='screen',
    )

    test_server_node = Node(
        package="waypoint_function_example_server",
        executable='test_server',
        name='test_server_node',
        output='screen',
    )

    skip_server_node = Node(
        package="waypoint_function_example_server",
        executable='skip_server',
        name='skip_server_node',
        output = 'screen',
        parameters=[waypoint_function_server_config],
        remappings=[
            ("current_pose", "amcl_pose"),
            ("scan", "scan"),
        ],
    )
    
    ld.add_action(host_server_node)
    # ld.add_action(test_server_node)
    ld.add_action(wait_server_node)
    ld.add_action(pause_server_node)
    ld.add_action(skip_server_node)


    return ld