from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    package_dir = get_package_share_directory('realsense_robot')
    
    # (Optional) rtabmap package dirs, if needed
    # rtabmap_dir = get_package_share_directory('rtabmap_ros')
    # rtabmap_odom_dir = get_package_share_directory('rtabmap_odom')
    # rtabmap_slam_dir = get_package_share_directory('rtabmap_slam')
    
    # Configure RViz
    rviz_config = os.path.join(package_dir, 'config', 'detection.rviz')
    
    # Include the RealSense camera launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'pointcloud.ordered_pc': 'false',
            'pointcloud.filter.magnitude_threshold': '0.01',
            'pointcloud.filter.remove_radius_outlier': 'true',
            'pointcloud.filter.radius': '0.05',
            'pointcloud.filter.min_neighbors': '10',
            'publish_tf': 'false',   # Let RTAB-Map handle transforms
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200',
            'accel_fps': '200',
            'unite_imu_method': '1',
            'enable_rgbd': 'true',
            'depth_module.exposure': '8500',
            'depth_module.gain': '16',
            'motion_module.enable_motion_correction': 'true'
        }.items()
    )
    
    # -------------------------------------------------------------------------
    # Do NOT publish static transforms for map->odom or odom->camera_link.
    # RTAB-Map will publish those dynamic transforms.
    # -------------------------------------------------------------------------
    
    # Static transform for camera depth optical frame
    camera_optical_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_transform',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link', 'camera_depth_optical_frame'
        ]
    )

    # Static transform for camera color frame
    camera_color_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_transform',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link', 'camera_color_optical_frame'
        ]
    )

    # Static transform for camera IMU
    camera_imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_imu_transform',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'camera_link', 'camera_imu_optical_frame'
        ]
    )

    # IMU filter node to preprocess IMU data for RTAB-Map
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.1,
            'zeta': 0.0,
            'fixed_frame': 'map',
            'orientation_stddev': 0.01,
            'remove_gravity_vector': True,
            'publish_debug_topics': True,
            'use_sim_time': False
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/rtabmap/imu')
        ]
    )
    
    # RTAB-Map odometry node
    # => Publishes odom->camera_link transform when publish_tf=True
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odometry',
        output='screen',
        parameters=[{
            # Timestamps / frames
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'tf_delay': 0.0,
            'tf_tolerance': 0.5,
            'wait_imu_to_init': True,    # Make sure IMU is publishing

            # Sync + queue
            'approx_sync': False,
            'queue_size': 10,
            'qos_image': 2,
            'qos_imu': 2,

            # The tricky parameters => set them as strings if your node expects strings!
            'Odom/Strategy': '0',         # string
            'Odom/GuessMotion': 'true',   # string
            'Odom/FillInfoData': 'true',  # string

            'Vis/MinInliers': '10',       # string
            'Vis/InlierDistance': '0.1',  # string

            # GFTT parameters must be strings too:
            'GFTT/MinDistance': '5',
            'GFTT/QualityLevel': '0.001',
            'GFTT/MaxFeatures': '500',

            'use_sim_time': False
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('imu', '/rtabmap/imu')
        ]
    )

    # Main RTAB-Map node
    # => Publishes map->odom transform when publish_tf=True
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # TF frames
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'tf_delay': 0.0,
            'tf_tolerance': 0.5,
            'wait_imu_to_init': True,

            # Subscription settings
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'subscribe_scan': 'false',
            'subscribe_odom_info': 'true',
            'subscribe_user_data': 'false',
            'subscribe_scan_cloud': 'true',
            'approx_sync': 'false',
            'queue_size': '10',
            'qos_image': '2',
            'qos_imu': '2',

            # Memory / SLAM
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true',
            'Mem/STMSize': '30',
            'Mem/UseOdomFeatures': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/MaxDepth': '3.0',
            'Vis/MinInliers': '10',
            'Vis/InlierDistance': '0.1',

            # Local cloud filtering
            'cloud_decimation': '4',
            'cloud_max_depth': '3.0',
            'cloud_min_depth': '0.2',
            'cloud_voxel_size': '0.05',

            # Some parameters are declared as bool in your build:
            'cloud_output_voxelized': True,  # <--- bool

            # 3D grid map in your build is actually string-based
            'Grid/FromDepth': 'false',
            'Grid/RayTracing': 'false',
            'Grid/3D': 'true',
            'Grid/MapFrameProjection': 'false',
            'Grid/Enabled': 'true',
            'Grid/RangeMax': '4.0',
            'Grid/CellSize': '0.05',

            # Global assembled point cloud
            'Cloud/Assemble': 'true',     # <--- string
            'Cloud/Regenerate': 'true',   # <--- string

            'use_sim_time': False
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('odom', '/rtabmap/odom'),
            ('imu', '/rtabmap/imu'),
            ('scan_cloud', '/camera/camera/depth/color/points')
        ]
    )
    
    # Optional custom mapping node
    mapping_node = Node(
        package='realsense_robot',
        executable='mapping_node.py',
        name='mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'max_points': 20000,
            'voxel_size': 0.03,
            'nb_neighbors': 20,
            'std_ratio': 1.5,
            'use_rtabmap_pose': True
        }],
        remappings=[
            ('/camera/pointcloud', '/camera/camera/depth/color/points'),
            ('/camera/pose', '/rtabmap/odom'),
            ('/map/pointcloud', '/rtabmap/cloud_map')
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        realsense_launch,
        camera_optical_transform,
        camera_color_transform,
        camera_imu_transform,
        imu_filter_node,
        rtabmap_odom_node,
        rtabmap_node,
        mapping_node,
        rviz_node
    ])
