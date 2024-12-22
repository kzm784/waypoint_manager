from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('waypoint_manager')
    creater_config_file_path = os.path.join(package_dir, 'config', 'creater_config.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'creater.rviz')

    # RViz2 Node
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Map Server Node
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[creater_config_file_path],
    )

    # Lifecycle Manager Node
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # Waypoint Creater Node
    start_waypoint_creater_cmd = Node(
        package='waypoint_manager',
        executable='waypoint_creater',
        name='waypoint_creater_node',
        output='screen',
        emulate_tty=True,
        remappings=[('/pose', '/goal_pose')],
        parameters=[creater_config_file_path],
    )

    # Combined TimerAction for delayed startup
    delayed_actions = TimerAction(
        period=5.0,
        actions=[
            start_map_server_cmd,
            start_lifecycle_manager_cmd,
            start_waypoint_creater_cmd,
        ],
    )

    # Launch Description
    ld = LaunchDescription()

    # Add Actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(delayed_actions)

    return ld
