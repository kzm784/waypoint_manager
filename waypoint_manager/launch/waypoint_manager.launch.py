import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    declare_waypoints_csv = DeclareLaunchArgument(
        'waypoints',
        description='Path to the waypoints CSV file.'
    )
    waypoints_csv_path = LaunchConfiguration('waypoints')

    waypoint_manager_config = LaunchConfiguration(
        'waypoint_manager_config',
        default=os.path.join(
            get_package_share_directory('waypoint_manager'),
            'config',
            'config_waypoint_manager.yaml'
        )
    )

    function_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('waypoint_function_server'),
                'launch',
                'waypoint_function_server.launch.py'
            )
        ),
        launch_arguments={
            'waypoit_function_server_config' : waypoint_manager_config,
        }.items()
    )

    waypoint_visualizer_node = Node(
        package="waypoint_visualizer",
        executable='waypoint_visualizer_node',
        name='waypoint_visualizer_node',
        output='screen',
        parameters=[waypoint_manager_config, {
            'waypoints_csv': waypoints_csv_path
        }]
    )

    waypoint_navigator_node = Node(
        package='waypoint_navigator',
        executable='waypoint_navigator_node',
        name='waypoint_navigator_node',
        output='screen',
        parameters=[waypoint_manager_config, {
            'waypoints_csv': waypoints_csv_path
        }]
    )

    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            waypoint_visualizer_node,
            waypoint_navigator_node
        ]
    )

    ld.add_action(declare_waypoints_csv)
    ld.add_action(function_server_launch)
    ld.add_action(delayed_nodes)

    return ld
