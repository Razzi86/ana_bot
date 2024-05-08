import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name='ana'
    
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=true'])

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}, {'use_sim_time':True}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
        
    # below automatically does the sim_time, gazebo_ros, robot_description, and joint_state_publisher terminal commands

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # parameter file changes gazebo refresh from 10 to 400hz
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world_file = '/home/aidan/ana_bot/src/ana/worlds/obstacles.world'
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 
                    launch_arguments={'world': world_file}.items()

    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    # Launch
    return LaunchDescription([
        joint_state_publisher,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        node_robot_state_publisher,
    ])