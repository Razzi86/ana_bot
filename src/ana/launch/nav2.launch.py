from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_params_path = os.path.join('/home/aidan/ana_bot/src/ana/config', 'nav2_params.yaml')  # Define the path to nav2_params.yaml

    return LaunchDescription([
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            namespace='',
            name='map_server',
            output='screen',
            parameters=[nav2_params_path, {
                'use_sim_time': True,
                'yaml_filename': '/home/aidan/ana_bot/src/ana/rtab_maps/occupancy_grid.yaml',
                'subscribe_to_costmap_topic': True,
                'costmap_topic_name': '/costmap_new'  # Explicitly set this parameter here
            }]
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'map_server', 'configure'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'map_server', 'activate'],
            output='screen'
        ),
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            namespace='',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'amcl', 'configure'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'amcl', 'activate'],
            output='screen'
        ),
        # Node(
        #     package='twist_mux',
        #     executable='twist_mux',
        #     namespace='',
        #     name='twist_mux',
        #     output='screen',
        #     parameters=[os.path.join(get_package_share_directory('ana'), 'config', 'twist_mux.yaml')],
        #     remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_unstamped')]
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
        #     launch_arguments={'use_sim_time': 'true'}.items()
        # ),
        # Assuming 'controller_server' handles the costmap:
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            namespace='',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}],  # Ensure the path to nav2_params.yaml is included if needed
            remappings=[('/global_costmap/costmap', '/costmap_new')]
        )
    ])