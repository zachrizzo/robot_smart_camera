#####################################################################
# robot.launch.py (Minimal, no advanced params)
#####################################################################
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the RealSense launch directory
    rtabmap_dir = get_package_share_directory('rtabmap_launch')

    return LaunchDescription([
        # RealSense camera node (optimized configuration)
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='/camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_pointcloud': False,
                'enable_sync': True,
                'align_depth.enable': True,
                'initial_reset': True,
                'depth_module.profile': '424x240x30',  # Increased resolution
                'rgb_camera.profile': '424x240x30',    # Increased resolution
                'depth_qos': 'SYSTEM_DEFAULT',         # Changed QoS
                'color_qos': 'SYSTEM_DEFAULT',         # Changed QoS
                'tf_publish_rate': 10.0                # Increased TF rate
            }],
            output='screen'
        ),

        # Static TF publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_link',
            arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_link', 'camera_color_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_depth_link',
            arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_link', 'camera_depth_optical_frame']
        ),

        # RTAB-Map RGBD Odometry node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'queue_size': 20,
                'qos_image': 2,
                'qos_camera_info': 2,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'wait_for_transform': 0.2
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')
            ],
            output='screen'
        ),

        # RTAB-Map node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'qos_image': 2,
                'qos_camera_info': 2,
                'queue_size': 20,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'wait_for_transform': 0.2,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'true',
                'RGBD/OptimizeMaxError': '0.1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'Optimizer/Strategy': '0',
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '5.0',
                'Reg/Strategy': '0',           # 0=Visual, 1=ICP, 2=Visual+ICP
                'Vis/MinInliers': '20',        # Minimum visual correspondences to accept a transformation
                'RGBD/NeighborLinkRefining': 'true'  # Local loop closure detection
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')
            ],
            output='screen'
        ),

        # Object Detection node
        Node(
            package='realsense_robot',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),

        # AI Agent Node for voice control and scene description
        Node(
            package='realsense_robot',
            executable='agent_node',
            name='agent_node',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'src/realsense_robot/config/mapping.rviz']
        )
    ])
