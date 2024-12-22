from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wayopint_recorder_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'recorder_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='wayopint_recorder',
            name='wayopint_recorder_node',
            output='screen',
            remappings=[('/current_pose', '/current_pose')],
            parameters=[wayopint_recorder_config]
        ),

         Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
        ),    
    ])