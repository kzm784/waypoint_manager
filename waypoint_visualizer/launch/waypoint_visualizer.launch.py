import os
import launch
import launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    declare_waypoints_csv = DeclareLaunchArgument(
        'waypoints',
        description='Path to the waypoints CSV file.'
    )

    waypoints_csv_path = LaunchConfiguration('waypoints')

    waypoint_visualizer_config = LaunchConfiguration(
        'waypoint_visualizer_config',
        default=os.path.join(
            get_package_share_directory('waypoint_manager'),
            'config',
            'config_waypoint_manager.yaml'
        )
    )

    waypoint_visualizer_node = Node(
        package="waypoint_visualizer",
        executable='waypoint_visualizer_node',
        name='waypoint_visualizer_node',
        output='screen',
        parameters=[waypoint_visualizer_config, {
            'waypoints_csv': waypoints_csv_path
        }]
    )

    ld.add_action(declare_waypoints_csv)
    ld.add_action(waypoint_visualizer_node)

    return ld
