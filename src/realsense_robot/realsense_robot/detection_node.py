#!"/home/zach/Desktop/robot1/.venv/bin/python3"

import cv2
import numpy as np
import os
import sys
sys.path.append('/home/zach/Desktop/robot1/.venv/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import struct
from collections import defaultdict
from ultralytics import YOLO

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
        
        # Store detected objects with their positions
        self.detected_objects = defaultdict(list)  # {class_name: [(x, y, z, confidence), ...]}
        self.next_marker_id = 0
        
        # Initialize YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully')
        
        # Define colors for different classes (BGR format)
        self.class_colors = {
            'person': (0, 0, 255),    # Red
            'car': (255, 0, 0),       # Blue
            'truck': (255, 128, 0),   # Light Blue
            'bicycle': (0, 255, 0),   # Green
            'motorcycle': (0, 255, 255), # Yellow
            'dog': (128, 0, 255),     # Purple
            'cat': (255, 0, 255),     # Pink
            'chair': (128, 128, 0),   # Teal
            'laptop': (0, 128, 255),  # Orange
            'cell phone': (128, 0, 128) # Dark Purple
        }
        
        # Set confidence threshold
        self.confidence_threshold = 0.85
        
        # Create timer for processing frames
        self.create_timer(1/30, self.process_frame)  # 30 Hz
        
        # Create timer for publishing persistent markers
        self.create_timer(1.0, self.publish_persistent_markers)  # 1 Hz
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def color_callback(self, msg):
        try:
            self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {str(e)}')
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')
    
    def create_marker(self, position, class_name, confidence, marker_id, is_persistent=False):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"  # Changed to map frame for persistence
        marker.ns = "detections"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Set marker size
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Set marker color based on class
        color = self.class_colors.get(class_name, (128, 128, 128))  # Default to gray if class not found
        marker.color.r = color[2] / 255.0  # Convert BGR to RGB
        marker.color.g = color[1] / 255.0
        marker.color.b = color[0] / 255.0
        marker.color.a = 0.8
        
        # Set marker lifetime
        if is_persistent:
            marker.lifetime.sec = 0  # Persistent
        else:
            marker.lifetime.sec = 1  # Short-lived for current detections
        
        # Add text marker
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "labels"
        text_marker.id = marker_id + 10000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = marker.pose.position
        text_marker.pose.position.z += 0.2
        text_marker.text = f"{class_name} {confidence:.2f}"
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.lifetime = marker.lifetime
        
        return [marker, text_marker]
    
    def update_detected_object(self, class_name, position, confidence):
        # Only update if confidence is above threshold
        if confidence < self.confidence_threshold:
            return
            
        # Check if we already have a similar detection nearby
        similar_found = False
        for i, (pos, conf) in enumerate(self.detected_objects[class_name]):
            dist = np.sqrt(sum((np.array(position) - np.array(pos))**2))
            if dist < 0.3:  # 30cm threshold for considering it the same object
                # Update position with moving average
                alpha = 0.3
                new_pos = tuple(alpha * np.array(position) + (1 - alpha) * np.array(pos))
                new_conf = max(confidence, conf)
                self.detected_objects[class_name][i] = (new_pos, new_conf)
                similar_found = True
                break
        
        if not similar_found:
            self.detected_objects[class_name].append((position, confidence))
    
    def publish_persistent_markers(self):
        marker_array = MarkerArray()
        
        # Add all persistent detections
        for class_name, detections in self.detected_objects.items():
            for position, confidence in detections:
                markers = self.create_marker(position, class_name, confidence, self.next_marker_id, True)
                marker_array.markers.extend(markers)
                self.next_marker_id += 1
        
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)
    
    def process_frame(self):
        if any(x is None for x in [self.latest_color_frame, self.latest_depth_frame, self.camera_info]):
            return
        
        try:
            # Create a copy of the frame for visualization
            vis_frame = self.latest_color_frame.copy()
            
            # Run YOLO detection
            results = self.model.predict(vis_frame)
            
            # Create marker array for current detections
            marker_array = MarkerArray()
            
            # Dictionary to count detections by class
            detection_counts = defaultdict(int)
            
            # Process results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get confidence and class
                    confidence = float(box.conf)
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    
                    # Skip if confidence is below threshold
                    if confidence < self.confidence_threshold:
                        continue
                        
                    detection_counts[class_name] += 1
                    
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Get color for this class
                    color = self.class_colors.get(class_name, (128, 128, 128))
                    
                    # Draw bounding box and label
                    cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(vis_frame, f'{class_name} {confidence:.2f}', (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Get center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    try:
                        # Get depth value (in meters)
                        depth = self.latest_depth_frame[center_y, center_x] / 1000.0  # Convert mm to meters
                        
                        if depth > 0:
                            # Convert to 3D point
                            fx = self.camera_info.k[0]  # focal length x
                            fy = self.camera_info.k[4]  # focal length y
                            cx = self.camera_info.k[2]  # optical center x
                            cy = self.camera_info.k[5]  # optical center y
                            
                            # Calculate 3D point in camera frame
                            x = (center_x - cx) * depth / fx
                            y = (center_y - cy) * depth / fy
                            z = depth
                            
                            # Convert to map frame coordinates
                            position = (z, -x, -y)  # Simple conversion for visualization
                            
                            # Update detected objects
                            self.update_detected_object(class_name, position, confidence)
                            
                            # Create current detection markers
                            markers = self.create_marker(position, class_name, confidence, self.next_marker_id)
                            marker_array.markers.extend(markers)
                            self.next_marker_id += 1
                            
                    except Exception as e:
                        self.get_logger().warn(f"Error processing detection: {e}")
            
            # Log detection counts
            if detection_counts:
                log_msg = "Detected: " + ", ".join([f"{count} {cls}" for cls, count in detection_counts.items()])
                self.get_logger().info(log_msg)
            
            # Publish the annotated image
            try:
                if vis_frame is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(vis_frame, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {e}')
            
            # Publish current detection markers
            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)
            
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