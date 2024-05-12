from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='nav2_controller',
            name='nav2_controller',
            output='screen',
            parameters=['/home/aidan/ana_bot/src/ana/config/nav2_params.yaml']),
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=['/path/to/your/rtabmap_params.yaml']),
        Node(
            package='nav2_bringup',
            executable='nav2_planner',
            name='nav2_planner',
            output='screen',
            parameters=['/home/aidan/ana_bot/src/ana/config/nav2_params.yaml']),
        Node(
            package='nav2_bringup',
            executable='nav2_recoveries',
            name='nav2_recoveries',
            output='screen',
            parameters=['/home/aidan/ana_bot/src/ana/config/nav2_params.yaml']),
    ])
