from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    package_dir = get_package_share_directory('realsense_robot')
    
    # Configure RViz and parameter files
    rviz_config = os.path.join(package_dir, 'config', 'detection.rviz')
    rtabmap_odom_params = os.path.join(package_dir, 'config', 'rtabmap_odom.yaml')
    rtabmap_params = os.path.join(package_dir, 'config', 'rtabmap.yaml')
    
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
            'pointcloud.ordered_pc': 'true',
            'pointcloud.filter.magnitude_threshold': '0.01',
            'pointcloud.filter.remove_radius_outlier': 'true',
            'pointcloud.filter.radius': '0.05',
            'pointcloud.filter.min_neighbors': '10',
            'publish_tf': 'false',
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
    
    # Static transforms
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

    camera_imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_imu_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_imu_optical_frame']
    )

    # IMU filter node
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
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odometry',
        output='screen',
        parameters=[rtabmap_odom_params],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('imu', '/rtabmap/imu')
        ]
    )

    # RTAB-Map SLAM node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
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
    
    # Mapping node
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
