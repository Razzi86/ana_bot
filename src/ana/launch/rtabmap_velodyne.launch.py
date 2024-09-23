from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_scan_cloud': True,  # Enable point cloud input
                'frame_id': 'velodyne',  # Adjust to match your Velodyne's frame
                'scan_cloud_topic': '/velodyne_points'  # Velodyne point cloud topic
            }],
            remappings=[
                ('/scan_cloud', '/velodyne_points'),  # Remap to match Velodyne topic
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/your/rviz/config/file'],  # Adjust path
            output='screen'
        )
    ])
