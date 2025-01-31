from setuptools import setup, find_packages
import os
import sys
from glob import glob

package_name = 'realsense_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python>=4.5.0',
        'numpy>=1.20.0',
        'transforms3d',
        'aiohttp>=3.9.0',
        'websockets>=10.0',
        'pyserial>=3.5',
        'ultralytics>=8.0.196',
        'langchain>=0.1.0',
        'langchain-core>=0.1.0',
        'langchain-anthropic>=0.1.0',
        'langchain-openai>=0.0.8',
        'langgraph>=0.0.27',
        'anthropic>=0.18.1',
        'openai>=1.14.0',
        'ollama>=0.1.6',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 package for RealSense camera with AI capabilities',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = realsense_robot.camera_node:main',
            'detection_node = realsense_robot.detection_node:main',
            'mapping_node = realsense_robot.mapping_node:main',
            'imu_node = realsense_robot.imu_node:main',
            'pointcloud_node = realsense_robot.pointcloud_node:main',
            'web_interface = realsense_robot.web_interface:main',
            'pico_servo_node = realsense_robot.pico_servo_node:main',
            'agent_node = realsense_robot.agent_node:main',
        ],
    },
    python_requires='>=3.8',
    options={
        'build_scripts': {
            'executable': '/home/zach/Desktop/robot1/.venv/bin/python3'
        }
    }
) 