# RealSense Robot Package

A ROS2 package for Intel RealSense cameras with point cloud visualization and object detection capabilities.

## Hardware Requirements

- Intel RealSense D435i camera
- USB 3.0 port and cable recommended for full functionality
  - USB 3.0: Full resolution (1280x720 @ 30fps) with motion sensors enabled
  - USB 2.0/2.1: Reduced resolution (424x240 @ 6fps) without motion sensors

## Features

- Point cloud generation and visualization
- Real-time object detection using YOLOv8
- Automatic USB bandwidth management
- 3D visualization in RViz2

## Installation

1. Install ROS2 Jazzy
2. Install RealSense dependencies:
```bash
sudo apt-get install ros-jazzy-realsense2-camera
```

3. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/realsense_robot.git
```

4. Install Python dependencies (using virtual environment recommended):
```bash
python3 -m venv venv
source venv/bin/activate
pip install torch torchvision ultralytics opencv-python
```

5. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select realsense_robot
source install/setup.bash
```

## Usage

Launch the camera node with point cloud and object detection:
```bash
ros2 launch realsense_robot camera_launch.py
```

### USB Bandwidth Considerations

The package automatically detects your USB connection type and adjusts settings accordingly:

- **USB 3.0 or higher**:
  - Resolution: 1280x720
  - Frame rate: 30 FPS
  - Motion sensors: Enabled (for D435i)
  - Point cloud: Full resolution

- **USB 2.0/2.1**:
  - Resolution: 424x240
  - Frame rate: 6 FPS
  - Motion sensors: Disabled to conserve bandwidth
  - Point cloud: Reduced resolution

To manually override motion sensor settings:
```bash
# Explicitly disable motion sensors
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_pointcloud:=true -p pointcloud.enable:=true -p align_depth.enable:=true -p enable_motion_module:=false -p enable_accel:=false -p enable_gyro:=false
```

## Visualization

1. Launch RViz2:
```bash
rviz2
```

2. Configure RViz2:
   - Set "Fixed Frame" to "camera_link"
   - Add PointCloud2 display and set topic to "/camera/pointcloud"
   - Add Image display and set topic to "/camera/detection/image" for object detection visualization
   - Add MarkerArray display and set topic to "/camera/detection/markers" for 3D labels
   - Set Point Size to 5 for better visibility
   - Set Color Transformer to RGB8 for color visualization

### Object Detection

The system uses YOLOv8 for real-time object detection:
- Detects 80+ common object classes
- Displays bounding boxes and labels in real-time
- Publishes detection results as:
  - Annotated images (`/camera/detection/image`)
  - 3D markers (`/camera/detection/markers`)
- Confidence threshold: 0.5 (adjustable)

## Troubleshooting

1. **Frame Timeout Errors**:
   - Check USB connection type (`lsusb -t`)
   - Use USB 3.0 port and cable for best performance
   - If using USB 2.0/2.1, reduced performance is expected

2. **Motion Sensor Issues**:
   - Motion sensors are automatically disabled for USB 2.0/2.1 connections
   - Only enabled when using USB 3.0 or higher with D435i camera

3. **Point Cloud Not Visible**:
   - Ensure camera is within 0.5-2 meters of objects
   - Check if topics are publishing: `ros2 topic list`
   - Verify RViz2 settings match configuration above

4. **Object Detection Issues**:
   - Ensure good lighting conditions
   - Keep objects within 0.5-3 meters range
   - Check GPU availability for better performance
   - Monitor system resources if detection is slow

## License

[Your License] 