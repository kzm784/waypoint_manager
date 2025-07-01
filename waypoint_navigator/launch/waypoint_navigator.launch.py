import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the waypoint CSV
    navigation_data_dir = os.getenv('NAVIGATION_DATA_DIR')
    navigation_data_name = os.getenv('NAVIGATION_DATA_NAME')
    waypoints_csv_path = os.path.join(
        navigation_data_dir,
        navigation_data_name,
        f"{navigation_data_name}_wp.csv"
    )

    # Set the path to the waypoint_navigator config
    waypoint_navigator_config = launch.substitutions.LaunchConfiguration(
        'waypoint_navigator_config',
        default=os.path.join(
            get_package_share_directory('waypoint_navigator'),
                'config',
                'config_waypoint_navigator.yaml'
        )
    )

    waypoint_navigator_node = Node(
        package='waypoint_navigator',
        executable='waypoint_navigator_node',
        name='waypoint_navigator_node',
        output='screen',
        parameters=[waypoint_navigator_config, {
                'waypoints_csv': waypoints_csv_path
        }]
    )
    
    ld.add_action(waypoint_navigator_node)

    return ld