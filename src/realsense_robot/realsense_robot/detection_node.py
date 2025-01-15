#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import os
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import struct

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # Create publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable color and depth streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start streaming
        self.pipeline_profile = self.pipeline.start(self.config)
        
        # Get device and set advanced mode
        device = self.pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.enable_auto_exposure):
            depth_sensor.set_option(rs.option.enable_auto_exposure, 1)
        
        # Create align object
        self.align = rs.align(rs.stream.color)
        
        # Load YOLO model
        conda_env_path = '/home/zach/anaconda3/envs/realsense_env'
        model_path = os.path.join(conda_env_path, 'yolov8n.pt')
        if not os.path.exists(model_path):
            print(f'Downloading YOLOv8 model to {model_path}')
            model = YOLO('yolov8n.pt')
            model.save(model_path)
        self.model = YOLO(model_path)
        
        # Create timer for processing frames
        self.create_timer(1/30, self.process_frame)  # 30 Hz
        
        print('Detection node initialized')
    
    def process_frame(self):
        try:
            # Get frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create point cloud
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
            
            # Create PointCloud2 message
            header = self.get_header()
            pc2_msg = pc2.create_cloud_xyz32(header, vertices)
            self.pc_pub.publish(pc2_msg)
            
            # Publish color image
            img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            img_msg.header = header
            self.image_pub.publish(img_msg)
            
            # Run detection
            results = self.model(color_image)
            
            # Create marker array for detections
            marker_array = MarkerArray()
            
            # Process results
            for r in results:
                boxes = r.boxes
                for i, box in enumerate(boxes):
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # Get center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    try:
                        distance = depth_frame.get_distance(center_x, center_y)
                        if distance > 0:
                            # Get 3D point from depth
                            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], distance)
                            
                            # Create marker for detection
                            marker = Marker()
                            marker.header = header
                            marker.ns = "detections"
                            marker.id = i
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            
                            # Set marker position
                            marker.pose.position.x = point_3d[0]
                            marker.pose.position.y = point_3d[1]
                            marker.pose.position.z = point_3d[2]
                            
                            # Set marker size
                            marker.scale.x = 0.1
                            marker.scale.y = 0.1
                            marker.scale.z = 0.1
                            
                            # Set marker color
                            marker.color.r = 0.0
                            marker.color.g = 1.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0
                            
                            # Add text marker
                            cls = int(box.cls[0])
                            conf = float(box.conf[0])
                            name = self.model.names[cls]
                            text_marker = Marker()
                            text_marker.header = header
                            text_marker.ns = "labels"
                            text_marker.id = i + 1000
                            text_marker.type = Marker.TEXT_VIEW_FACING
                            text_marker.action = Marker.ADD
                            text_marker.pose.position = marker.pose.position
                            text_marker.pose.position.z += 0.1
                            text_marker.text = f"{name} {conf:.2f}"
                            text_marker.scale.z = 0.1
                            text_marker.color.r = 1.0
                            text_marker.color.g = 1.0
                            text_marker.color.b = 1.0
                            text_marker.color.a = 1.0
                            
                            marker_array.markers.append(marker)
                            marker_array.markers.append(text_marker)
                            
                    except Exception as e:
                        self.get_logger().warn(f"Error processing detection: {e}")
            
            # Publish markers
            self.marker_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {e}")
    
    def get_header(self):
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"
        return header
    
    def __del__(self):
        self.pipeline.stop()

def main():
    rclpy.init()
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 