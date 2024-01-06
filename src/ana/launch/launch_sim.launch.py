import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='ana'
    
    # below automatically does the sim_time, gazebo_ros, robot_description, and joint_state_publisher terminal commands

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # parameter file changes gazebo refresh from 10 to 400hz
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])