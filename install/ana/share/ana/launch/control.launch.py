from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare any launch arguments if needed
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) time'),

        # Add nodes for controlling the robot
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
            output="screen"
        ),
    ])
