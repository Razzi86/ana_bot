from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters = {
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'subscribe_depth': False, # depth camera
        'use_action_for_goal': True,
        'qos_image': qos,
        'qos_imu': qos,
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',  # Disable imu constraints as we are already in 2D
        'max_depth': 10.0,
        'Mem/IncrementalMemory': 'False',  # Disable memory incrementation
        'Mem/InitWMWithAllNodes': 'True',  # Use all nodes in working memory for localization
        'Rtabmap/DetectionRate': '0',  # Disable new map creation
        'Kp/MaxFeatures': '0',  # Disable feature extraction
        'RGBD/ProximityBySpace': 'false',  # Disable joining new clouds by space
        'RGBD/Mapping': 'false'  # Disable mapping features
    }

    remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/filtered/depth/image_raw')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),

        # Localization node:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters, {'map_file_path': '/home/aidan/ana_bot/src/ana/rtab_maps/occupancy_grid.yaml'}],  # Path to your saved map
            remappings=remappings,
            arguments=['-d']  # Optionally delete the existing database on start
        ),

        # Visualization node:
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings
        )
    ])





# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')
#     qos = LaunchConfiguration('qos')
#     localization = LaunchConfiguration('localization')

#     parameters={
#           'frame_id':'base_link',
#           'use_sim_time':use_sim_time,
#           'subscribe_depth':True,
#           'use_action_for_goal':True,
#           'qos_image':qos,
#           'qos_imu':qos,
#           'Reg/Force3DoF':'true',
#           'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
#           'max_depth': 10.0
#     }

#     remappings=[
#           ('rgb/image', '/camera/image_raw'), # RGB images used by RTAB-Map to detect visual features and recognize previously visited areas, crucial for SLAM (Simultaneous Localization and Mapping)
#           ('rgb/camera_info', '/camera/camera_info'), # carries metadata about the camera's calibration and settings, which is essential for accurate image processing and depth estimation in RTAB-Map
#           # ('depth/image', '/camera/depth/image_raw') # Original mapping for depth images if using unfiltered data.
#           ('depth/image', '/filtered/depth/image_raw') # provides filtered depth data, which complements the RGB images by adding 3D structural information to the visual features, enhancing the 3D mapping and feature matching capabilities of RTAB-Map.
#           ]

#     return LaunchDescription([

#         # Launch arguments
#         DeclareLaunchArgument(
#             'use_sim_time', default_value='true',
#             description='Use simulation (Gazebo) clock if true'),

#         DeclareLaunchArgument(
#             'qos', default_value='2',
#             description='QoS used for input sensor topics'),

#         DeclareLaunchArgument(
#             'localization', default_value='false',
#             description='Launch in localization mode.'),

#         # Nodes to launch

#         # SLAM mode:
#         Node(
#             condition=UnlessCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters],
#             remappings=remappings,
#             arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)

#         # Localization mode:
#         Node(
#             condition=IfCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters,
#               {'Mem/IncrementalMemory':'False',
#                'Mem/InitWMWithAllNodes':'True'}],
#             remappings=remappings),

#         Node(
#             package='rtabmap_viz', executable='rtabmap_viz', output='screen',
#             parameters=[parameters],
#             remappings=remappings),
#         # Localization mode:
#         Node(
#             condition=IfCondition(localization),
#             package='rtabmap_slam', 
#             executable='rtabmap', 
#             output='screen',
#             parameters=[parameters,
#                 {'Mem/IncrementalMemory':'False',
#                 'Mem/InitWMWithAllNodes':'True',
#                 'map_file_path': '/home/aidan/ana_bot/src/ana/rtab_maps/occupancy_grid.yaml'}],  # Path to your saved map
#             remappings=remappings),
#     ])
