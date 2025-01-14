import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name='ana'

    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=true'])

    # Updated part of the script
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_config, value_type=str), 
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # static_transform_odom_to_base_link = ExecuteProcess(
    # cmd=[
    #     'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
    #     '0', '0', '0', '0', '0', '0', '1',  # x, y, z, qx, qy, qz, qw
    #     'odom', 'base_link'  # parent_frame_id, child_frame_id
    # ],
    # output='screen')

    # depth_live_filter_node = Node(
    #     package='ana',  # Adjust if your package name is different
    #     executable='depth_live_filter_node',  # The executable name as defined in CMakeLists.txt
    #     name='depth_live_filter',  # Optional: Specify a custom node name
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    occupancy_grid_subscriber_node = Node(
        package='ana',  # Make sure 'ana' matches your actual package name
        executable='occupancy_grid_subscriber_node',  # The executable name as defined in CMakeLists.txt
        name='occupancy_grid_subscriber',  # Optional: Specify a custom node name
        output='screen',
        parameters=[{'use_sim_time': True}]  # Add any specific parameters required by your node
    )

        
    # below automatically does the sim_time, gazebo_ros, robot_description, and joint_state_publisher terminal commands
    rsp_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    nav2_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','navigation_launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','navigation_launch.py'
    #     )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )
    
    # parameter file changes gazebo refresh from 10 to 400hz
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world_file = '/home/aidan/ana_bot/src/ana/worlds/outside4.world'
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 
                    launch_arguments={'world': world_file}.items()
    )

    # spawn_entity = Node(
    #     package='gazebo_ros', executable='spawn_entity.py', 
    #     arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
    #     output='screen')

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'my_bot',
            '-x', '0.0',  # X coordinate
            '-y', '0.0',  # Y coordinate
            '-z', '0.0',  # Z coordinate (height)
            '-Y', '0.0'   # Yaw orientation
        ],
        output='screen',
        parameters=[{'use_sim_time': True}] # ADDED
    )

    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}] # ADDED
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}] # ADDED
    )

    # rtab_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','rtab_3D.launch.py'
    #     )]),     
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #     }.items()
    # )

    pcd_publisher_node = Node(
        package='ana',  # Replace 'ana' with your package name if different
        executable='publish_pcd_node',  # The name of your executable
        name='pcd_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static transform publisher from map to odom
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch
    return LaunchDescription([
        # depth_live_filter_node,
        joint_state_publisher,
        rsp_node,
        # rtab_node,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        node_robot_state_publisher,
        pcd_publisher_node,
        nav2_node,
        twist_mux,
        # localization_launch
        # static_transform_odom_to_base_link
        occupancy_grid_subscriber_node,
        static_transform_publisher
    ])