# RealSense Robot Package

A ROS2 package for Intel RealSense cameras with SLAM and 3D mapping capabilities using RTAB-Map.

## Hardware Requirements

- Intel RealSense D435i camera
- USB 3.0 port and cable recommended for full functionality
  - USB 3.0: Full resolution (424x240 @ 30fps) with mapping enabled
  - USB 2.0/2.1: Reduced performance, not recommended for mapping

## Features

- Visual SLAM using RTAB-Map
- Real-time 3D mapping and point cloud generation
- Visual odometry for camera pose estimation
- Optimized for real-time performance
- 3D visualization in RViz2

## Installation

1. Install ROS2 Jazzy
2. Install dependencies:
```bash
sudo apt-get install ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros
```

3. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/realsense_robot.git
```

4. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select realsense_robot
source install/setup.bash
```

## Usage

### Launch Mapping System

1. Start the mapping system:
```bash
ros2 launch realsense_robot robot.launch.py
```

2. View the map in RViz:
   - The launch file will automatically start RViz with the correct configuration
   - If you need to start RViz manually:
     ```bash
     rviz2 -d src/realsense_robot/config/mapping.rviz
     ```

### Running Instructions
- To run the system, execute the following command:
```bash
rm -rf ~/.ros/rtabmap.db* && colcon build --packages-select realsense_robot && source install/setup.bash && ros2 launch realsense_robot robot.launch.py
```

### Mapping Tips

1. **Camera Movement**:
   - Move the camera slowly and smoothly
   - Keep some overlap between views
   - Maintain good lighting conditions
   - Avoid rapid rotations

2. **RViz Visualization**:
   - Map Frame: Set to "map"
   - Point Cloud Topics:
     - `/rtabmap/cloud_map`: Full 3D point cloud map
     - `/rtabmap/grid_map`: 3D occupancy grid
   - Set "Reliability Policy" to "Best Effort" for all topics

### Configuration

The package is configured for optimal performance with these settings:

- Camera Resolution: 424x240 @ 30fps
- Depth Alignment: Enabled
- Visual Odometry:
  - Min Inliers: 20
  - Max Depth: 4.0m
  - Queue Size: 20
- RTAB-Map:
  - Update Rate: 1Hz
  - Grid Cell Size: 0.05m
  - Grid Range: 5.0m
  - Loop Closure: Enabled

## Troubleshooting

1. **No Map Visible**:
   - Ensure camera is moving slowly
   - Check if odometry is working (`ros2 topic echo /odom`)
   - Verify RTAB-Map topics are publishing (`ros2 topic list | grep rtabmap`)

2. **Poor Mapping Quality**:
   - Ensure good lighting conditions
   - Move camera slower
   - Check visual odometry quality in terminal output
   - Reduce camera movement speed

3. **Performance Issues**:
   - Monitor CPU usage
   - Reduce resolution if needed
   - Ensure USB 3.0 connection
   - Check for synchronization warnings in logs

## License

[Your License] 