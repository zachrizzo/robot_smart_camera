from setuptools import setup
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
    ],
    install_requires=[
        'setuptools',
        'torch==2.2.1',
        'torchvision==0.17.1',
        'ultralytics==8.0.196',
        'opencv-python>=4.5.0',
        'numpy>=1.20.0',
    ],
    zip_safe=True,
    maintainer='zach',
    maintainer_email='zach@todo.todo',
    description='RealSense robot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = realsense_robot.detection_node:main',
        ],
    },
    scripts=[],
    py_modules=[],
) 