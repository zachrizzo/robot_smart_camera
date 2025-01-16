#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import struct

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # Create subscribers
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        
        # Create publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)
        self.image_pub = self.create_publisher(Image, '/detection_image', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Store latest frames
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        
        # Load YOLO model
        model_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'robot1', '.venv', 'yolov8n.pt')
        # self.get_logger().info(f'Attempting to load YOLO model from: {model_path}')
        if not os.path.exists(model_path):
            # self.get_logger().info(f'Model not found. Downloading YOLOv8 model to {model_path}')
            model = YOLO('yolov8n.pt')
            model.save(model_path)
        try:
            self.model = YOLO(model_path)
            # self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            # self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            raise e
        
        # Create timer for processing frames
        self.create_timer(1/30, self.process_frame)  # 30 Hz
        
        # self.get_logger().info('Detection node initialized')
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def color_callback(self, msg):
        try:
            # self.get_logger().info(f'Received color frame with encoding: {msg.encoding}, size: {msg.width}x{msg.height}')
            self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.get_logger().info(f'Successfully converted color frame to OpenCV format: shape={self.latest_color_frame.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {str(e)}')
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')
    
    def process_frame(self):
        if self.latest_color_frame is None:
            # self.get_logger().warn('No color frame available')
            return
        if self.latest_depth_frame is None:
            # self.get_logger().warn('No depth frame available')
            return
        if self.camera_info is None:
            # self.get_logger().warn('No camera info available')
            return
        
        try:
            # Log frame processing
            # self.get_logger().info('Processing new frame')
            
            # Create a copy of the frame for visualization
            vis_frame = self.latest_color_frame.copy()
            
            # Verify frame dimensions
            # self.get_logger().info(f'Frame shape: {vis_frame.shape}')
            
            # Run detection
            # self.get_logger().info('Running YOLO detection on frame')
            results = self.model(self.latest_color_frame)
            # self.get_logger().info('YOLO detection completed')
            
            # Create marker array for detections
            marker_array = MarkerArray()
            
            # Process results
            for r in results:
                boxes = r.boxes
                # self.get_logger().info(f'Found {len(boxes)} detections')
                
                for i, box in enumerate(boxes):
                    # Get confidence and class
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = self.model.names[cls]
                    # self.get_logger().info(f'Detection {i}: {name} with confidence {conf:.2f}')
                    
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # Draw bounding box and label on the visualization frame
                    cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(vis_frame, f'{name} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Get center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    try:
                        # Get depth value (in meters)
                        depth = self.latest_depth_frame[center_y, center_x] / 1000.0  # Convert mm to meters
                        # self.get_logger().info(f'Depth at center point: {depth:.2f}m')
                        
                        if depth > 0:
                            # Convert to 3D point
                            fx = self.camera_info.k[0]  # focal length x
                            fy = self.camera_info.k[4]  # focal length y
                            cx = self.camera_info.k[2]  # optical center x
                            cy = self.camera_info.k[5]  # optical center y
                            
                            # Calculate 3D point
                            x = (center_x - cx) * depth / fx
                            y = (center_y - cy) * depth / fy
                            z = depth
                            
                            # self.get_logger().info(f'3D position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
                            
                            # Create marker for detection
                            marker = Marker()
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.header.frame_id = "camera_link"
                            marker.ns = "detections"
                            marker.id = i
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            
                            # Set marker position
                            marker.pose.position.x = z  # RealSense camera coordinate system to ROS coordinate system
                            marker.pose.position.y = -x
                            marker.pose.position.z = -y
                            
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
                            text_marker = Marker()
                            text_marker.header = marker.header
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
                            # self.get_logger().info(f'Added markers for {name} at position (x={z:.2f}, y={-x:.2f}, z={-y:.2f})')
                            
                    except Exception as e:
                        self.get_logger().warn(f"Error processing detection: {e}")
            
            # Publish the annotated image
            try:
                if vis_frame is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(vis_frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.image_pub.publish(img_msg)
                    # self.get_logger().info(f'Published annotated image with shape {vis_frame.shape}')
                else:
                    # self.get_logger().warn('Visualization frame is None')
                    pass
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {e}')
            
            # Publish markers
            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)
                # self.get_logger().info(f'Published {len(marker_array.markers)} markers')
            else:
                # self.get_logger().info('No markers to publish')
                pass
            
        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {e}")

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