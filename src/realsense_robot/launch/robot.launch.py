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
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200',
            'accel_fps': '200',
            'unite_imu_method': '1',
            'motion_module.enable_motion_correction': 'true'
        }.items()
    )
    
    # Detection node
    detection_node = Node(
        package='realsense_robot',
        executable='detection_node.py',
        name='detection_node',
        output='screen',
        remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/aligned_depth_to_color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/aligned_depth_to_color/camera_info', '/camera/camera/aligned_depth_to_color/camera_info')
        ]
    )
    
    # Point cloud processing node
    pointcloud_node = Node(
        package='realsense_robot',
        executable='pointcloud_node.py',
        name='pointcloud_node',
        output='screen'
    )
    
    # IMU node
    imu_node = Node(
        package='realsense_robot',
        executable='imu_node.py',
        name='imu_node',
        output='screen'
    )
    
    # Mapping node
    mapping_node = Node(
        package='realsense_robot',
        executable='mapping_node.py',
        name='mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'max_points': 500000,
            'voxel_size': 0.02,
            'nb_neighbors': 30,
            'std_ratio': 1.5
        }],
        remappings=[
            ('/camera/pointcloud', '/camera/camera/depth/color/points'),
            ('/camera/pose', '/camera/camera/pose')
        ]
    )
    
    # Web interface node
    web_interface_node = Node(
        package='realsense_robot',
        executable='web_interface.py',
        name='web_interface',
        output='screen'
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
        detection_node,
        pointcloud_node,
        imu_node,
        mapping_node,
        web_interface_node,
        rviz_node
    ]) 