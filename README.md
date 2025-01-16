# RealSense Robot System

This project implements a robot system using ROS 2 and Intel RealSense camera for object detection and distance measurement.



## How to run the code:

```bash
cd ~/Desktop/robot1 && colcon build --symlink-install && source install/setup.bash && ros2 launch realsense_robot robot.launch.py 
```



## System Requirements

- Ubuntu 24.04
- ROS 2 Humble
- Python 3.10+
- Intel RealSense Camera

## Installation

1. Install ROS 2 Humble:
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

2. Install Intel RealSense SDK:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y
```

3. Install ROS 2 RealSense package:
```bash
sudo apt install ros-humble-realsense2-camera -y
```

4. Install Python dependencies:
```bash
pip install torch torchvision ultralytics opencv-python flask
```

## Building the workspace

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Clone the repository
git clone <repository_url> ~/realsense_ws
cd ~/realsense_ws

# Build the workspace
colcon build
source install/setup.bash
```

## Running the System

1. Launch the complete system:
```bash
ros2 launch realsense_robot robot.launch.py
```

2. Access the web interface:
Open your browser and navigate to `http://localhost:8080`

## Features

- Real-time camera feed streaming
- Object detection using YOLOv8
- Distance measurement for detected objects
- Web-based visualization interface
- ROS 2 integration for modularity and extensibility

## Project Structure

```
realsense_robot/
├── launch/
│   └── robot.launch.py
├── realsense_robot/
│   ├── detector_node.py
│   ├── camera_node.py
│   └── web_interface.py
├── config/
│   └── camera_params.yaml
└── setup.py
``` # robot_smart_camera



I see the issue now. The RealSense camera publishes IMU data on separate topics for accelerometer and gyroscope:
/camera/camera/accel/sample
/camera/camera/gyro/sample