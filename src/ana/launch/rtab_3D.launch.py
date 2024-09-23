# Example:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_examples vlp16.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    # deskewing = LaunchConfiguration('deskewing') # READS TIME STAMPS IS -REAL- LIDAR, NOT VIRTUAL
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # DeclareLaunchArgument(
        #     'deskewing', default_value='true',
        #     description='Enable lidar deskewing'),
          
        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
            #   'deskewing':deskewing,
              'use_sim_time':use_sim_time,
              # RTAB-Map's internal parameters are strings:
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '25',
              'Icp/VoxelSize': '0.05', # resolution
              'Icp/Epsilon': '0.001',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/MaxTranslation': '2',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.05', # lower = easier to add new keyframes
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '9000',
              'OdomF2M/BundleAdjustment': 'false'
            }],
            remappings=[
              ('scan_cloud', '/velodyne_points')
            ]),
            
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
              'max_clouds':10,
              'fixed_frame_id':'',
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('cloud', 'odom_filtered_input_scan')
            ]),
            
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
              # RTAB-Map's internal parameters are strings:
              'Db/IncrementalMemory': 'true',  # Ensure incremental memory is enabled
              'RGBD/ProximityMaxGraphDepth': '10',
              'RGBD/ProximityPathMaxNeighbors': '5',
              'RGBD/AngularUpdate': '0.01',
              'RGBD/LinearUpdate': '0.01',
              'RGBD/CreateOccupancyGrid': 'true',
              'Mem/NotLinkedNodesKept': 'true',
              'Mem/STMSize': '99999', # ram (how much it keeps), was 50
              'Mem/LaserScanNormalK': '0', # no pruning, was 20
              'Mem/RecentWmRatio': '1.0', # keep everything in working memory, was 0.2
              'Reg/Strategy': '1',
              'Icp/VoxelSize': '0.1',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '25',
              'Icp/MaxTranslation': '0.5',
              'Icp/MaxRotation': '0.1',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '3', # lower = more keyframes on smaller movements
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.2',
              'Kp/MaxFeatures': '9000',  # Increase keypoint extraction
              'Vis/MinInliers': '200'  # Example setting for minimum inliers
            }],
            remappings=[
              ('scan_cloud', '/velodyne_points') # changing this caused SLAM to work, but there are two clouds
            ],
            arguments=[
              '-d' # This will delete the previous database (~/.ros/rtabmap.db)
            ]), 
     
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'subscribe_odom_info':True,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
               ('scan_cloud', 'odom_filtered_input_scan')
            ]),
    ])


