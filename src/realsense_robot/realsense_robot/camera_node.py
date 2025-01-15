#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import json

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Get device and check USB type
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("No RealSense devices found!")
        
        device = devices[0]
        usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
        self.is_usb3 = "3." in usb_type
        
        self.get_logger().info(f'Device USB type: {usb_type}')
        
        # Configure streams based on USB capability
        if self.is_usb3:
            self.get_logger().info('Using full resolution with motion module (USB 3.0+)')
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            # Only enable motion sensors for USB 3.0 and D435i
            if device.supports(rs.camera_info.product_id) and device.get_info(rs.camera_info.product_id) == "0B3A":
                self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
                self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
                self.has_motion = True
                self.get_logger().info('Motion sensors enabled for USB 3.0+ connection')
            else:
                self.has_motion = False
                self.get_logger().info('Device does not support motion sensors')
        else:
            self.get_logger().info('Using reduced resolution without motion module (USB 2.0/2.1)')
            self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 6)
            self.config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 6)
            self.has_motion = False
            
            # Disable motion sensors for USB 2.0/2.1
            try:
                device.query_sensors()
                for sensor in device.query_sensors():
                    if sensor.is_motion_sensor():
                        sensor.stop()
                        sensor.close()
                        self.get_logger().info('Motion sensor explicitly disabled for USB 2.0/2.1')
            except Exception as e:
                self.get_logger().warn(f'Failed to stop motion sensor: {str(e)}')
            
            # Explicitly disable motion streams
            advanced_mode = rs.rs400_advanced_mode(device)
            if advanced_mode.is_enabled():
                try:
                    json_string = advanced_mode.serialize_json()
                    json_object = json.loads(json_string)
                    if 'motion_module' in json_object:
                        json_object['motion_module']['enable'] = 0
                        advanced_mode.load_json(json.dumps(json_object))
                        self.get_logger().info('Motion module disabled via advanced mode')
                except Exception as e:
                    self.get_logger().warn(f'Failed to disable motion module in advanced mode: {str(e)}')
        
        # Start streaming
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info('Pipeline started successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to start pipeline: {str(e)}')
            raise
        
        # Get depth scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # Get camera intrinsics
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        # Create publishers
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.detection_pub = self.create_publisher(Image, '/camera/detection/image', 10)
        
        # Create timer for publishing frames (adjust rate based on USB type)
        timer_period = 1.0/6.0 if not self.is_usb3 else 1.0/30.0
        self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('RealSense camera node initialized with detection and point cloud')

    def create_point_cloud(self, depth_image, color_image):
        points = []
        for y in range(0, depth_image.shape[0], 2):  # Skip every other pixel for performance
            for x in range(0, depth_image.shape[1], 2):
                depth = depth_image[y, x] * self.depth_scale
                if depth > 0:
                    # Deproject from pixel to point in 3D
                    px = (x - self.depth_intrinsics.ppx) / self.depth_intrinsics.fx
                    py = (y - self.depth_intrinsics.ppy) / self.depth_intrinsics.fy
                    point = [px * depth, py * depth, depth]
                    color = color_image[y, x]
                    points.append(point + [color[2], color[1], color[0]])  # RGB
        
        if not points:
            return None
            
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='r', offset=12, datatype=pc2.PointField.UINT8, count=1),
            pc2.PointField(name='g', offset=13, datatype=pc2.PointField.UINT8, count=1),
            pc2.PointField(name='b', offset=14, datatype=pc2.PointField.UINT8, count=1),
        ]
        
        header = self.get_clock().now().to_msg()
        pc = pc2.create_cloud(header, fields, points)
        pc.header.frame_id = "camera_link"
        return pc

    def detect_objects(self, color_image):
        # Run inference
        results = self.model(color_image)
        
        # Draw results on image
        img = results.render()[0]
        return np.ascontiguousarray(img)

    def timer_callback(self):
        try:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
                
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create camera info message
            camera_info = CameraInfo()
            camera_info.header.stamp = self.get_clock().now().to_msg()
            camera_info.header.frame_id = "camera_link"
            
            # Generate point cloud
            pc_msg = self.create_point_cloud(depth_image, color_image)
            
            # Perform object detection
            detection_image = self.detect_objects(color_image)
            
            # Convert to ROS messages
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
            detection_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
            
            # Set headers
            now = self.get_clock().now().to_msg()
            color_msg.header.stamp = now
            depth_msg.header.stamp = now
            detection_msg.header.stamp = now
            color_msg.header.frame_id = "camera_link"
            depth_msg.header.frame_id = "camera_link"
            detection_msg.header.frame_id = "camera_link"
            
            # Publish messages
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.camera_info_pub.publish(camera_info)
            if pc_msg:
                self.pointcloud_pub.publish(pc_msg)
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in camera node: {str(e)}')

    def __del__(self):
        self.pipeline.stop()
        if self.has_motion:
            self.get_logger().info('Stopping Motion Module')
        self.get_logger().info('Stopping camera node')

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 