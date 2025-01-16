from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'realsense_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('lib', package_name), [
            'realsense_robot/camera_node.py',
            'realsense_robot/pointcloud_node.py',
            'realsense_robot/detection_node.py',
            'realsense_robot/web_interface.py',
            'realsense_robot/imu_node.py',
            'realsense_robot/mapping_node.py'
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'torch',
        'torchvision',
        'ultralytics',
        'pyrealsense2',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'visualization_msgs',
        'tf2_ros',
        'geometry_msgs',
        'transforms3d',
        'open3d',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RealSense robot package with point cloud and object detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = realsense_robot.camera_node:main',
            'pointcloud_node = realsense_robot.pointcloud_node:main',
            'detection_node = realsense_robot.detection_node:main',
            'imu_node = realsense_robot.imu_node:main',
            'mapping_node = realsense_robot.mapping_node:main',
            'web_interface = realsense_robot.web_interface:main',
        ],
    }
) 