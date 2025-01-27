from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    package_dir = get_package_share_directory('realsense_robot')
    
    # Paths to config files
    rviz_config = os.path.join(package_dir, 'config', 'mapping.rviz')
    
    #----------------------------------------------------------------------
    # 1) RealSense Camera Node
    #----------------------------------------------------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'gyro_fps': '200',
            'accel_fps': '200',
            'depth_fps': '30',
            'color_fps': '30',
            'tf_publish_rate': '200.0',
            'enable_imu': 'true'
        }.items()
    )

    #----------------------------------------------------------------------
    # 2) IMU Filter Node for Quaternion
    #----------------------------------------------------------------------
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.1
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/rtabmap/imu')
        ]
    )

    #----------------------------------------------------------------------
    # 3) RTAB-Map RGBD Odometry Node
    #----------------------------------------------------------------------
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '10',
            'Vis/InlierDistance': '0.1',
            'Vis/EstimationType': '0',
            'Vis/MaxDepth': '10.0',
            'Vis/MinDepth': '0.3',
            'OdomF2M/MaxSize': '1000',
            'OdomF2M/MaxNewFeatures': '200',
            'Odom/Strategy': '1',
            'Odom/FilteringStrategy': '1',
            'Odom/ResetCountdown': '0',
            'approx_sync': True,
            'queue_size': 10,
            'qos_image': 2,
            'qos_imu': 2,
            'wait_imu_to_init': True,
            'publish_tf': True,
            'guess_frame_id': 'camera_link',
            'Imu/LinearVelocity': 'true',
            'Imu/MaxGyroBias': '0.01'
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('imu', '/rtabmap/imu')
        ]
    )

    #----------------------------------------------------------------------
    # 4) RTAB-Map Main Node
    #----------------------------------------------------------------------
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'approx_sync': False,
            'queue_size': 10,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/CreateOccupancyGrid': 'true',
            'Grid/FromDepth': 'true',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '10.0',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Optimizer/GravitySigma': '0.3',
            'wait_for_transform': '0.2',
            'map_always_update': 'true',
            'map_negative_poses_ignored': 'false',
            'map_negative_scan_empty_ray_tracing': 'true',
            'qos_image': 2,
            'qos_imu': 2,
            'wait_imu_to_init': True,
            'database_path': '~/.ros/rtabmap.db',
            'Imu/LinearVelocity': 'true',
            'Imu/MaxGyroBias': '0.01'
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('imu', '/rtabmap/imu'),
            ('odom', '/rtabmap/odom')
        ]
    )

    #----------------------------------------------------------------------
    # 5) Static Transform Publishers for Camera Frames
    #----------------------------------------------------------------------
    map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    camera_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    camera_optical_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_link',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame']
    )

    camera_depth_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_link',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_depth_optical_frame']
    )

    #----------------------------------------------------------------------
    # 6) RViz for Visualization
    #----------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Return final LaunchDescription
    return LaunchDescription([
        realsense_launch,
        imu_filter_node,
        map_odom,
        odom_base,
        camera_base_link,
        camera_optical_link,
        camera_depth_link,
        rtabmap_odom_node,
        rtabmap_node,
        rviz_node
    ])
