from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_node',
            parameters=[{
                'frame_id': 'velodyne',
                'model': 'VLP16',  # Ensure this is VLP16
                'port': 2368,
                'rpm': 600.0,
            }]
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[{
                'calibration': '/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml',
                'model': 'VLP16'  # Add model parameter for the correct sensor
            }],
            remappings=[
                ('velodyne_packets', '/velodyne_packets'),
                ('velodyne_points', '/velodyne_points')
            ]
        ),
        # Automatically launch RViz with the specified configuration file
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/aidan/ros2_ws/src/ana_bot/src/ana/config/velodyne_rviz.rviz'],
            output='screen'
        )
    ])
