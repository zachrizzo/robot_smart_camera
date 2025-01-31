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
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA, String
import struct
from collections import defaultdict
from ultralytics import YOLO
import tf2_ros
import random

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
        self.objects_pub = self.create_publisher(String, '/detection/objects', 10)  # New publisher for object info
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Store latest frames
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        
        # Store detected objects with their positions and timestamps
        self.detected_objects = defaultdict(list)  # {class_name: [(x, y, z, confidence, last_update_time), ...]}
        self.next_marker_id = 0
        
        # Initialize YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully')
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Random color generator for classes
        self.class_colors = {}
        
        # Set parameters
        self.confidence_threshold = 0.85
        self.distance_filter_threshold = 0.5  # 50cm threshold for position updates
        self.depth_filter_window = 5  # Number of depth values to average
        self.depth_max_distance = 5.0  # Maximum valid depth in meters
        self.position_update_alpha = 0.2  # Position update smoothing factor (reduced for more stability)
        self.min_update_interval = 1.0  # Minimum time between position updates (seconds)
        self.object_timeout = 5.0  # Time before removing an object that hasn't been seen (seconds)
        
        # Create timer for processing frames
        self.create_timer(1/15, self.process_frame)  # Reduced to 15 Hz for stability
        
        # Create timer for publishing persistent markers
        self.create_timer(0.1, self.publish_persistent_markers)  # Increased to 10 Hz for smoother updates
    
    def get_random_color(self, class_name):
        if class_name not in self.class_colors:
            # Generate random BGR color
            color = (
                random.randint(50, 255),
                random.randint(50, 255),
                random.randint(50, 255)
            )
            self.class_colors[class_name] = color
        return self.class_colors[class_name]
    
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
    
    def filter_depth(self, depth_frame, center_x, center_y):
        """Apply median filtering to depth values around the center point"""
        window_size = 5
        half_size = window_size // 2
        
        # Extract depth window
        y_start = max(0, center_y - half_size)
        y_end = min(depth_frame.shape[0], center_y + half_size + 1)
        x_start = max(0, center_x - half_size)
        x_end = min(depth_frame.shape[1], center_x + half_size + 1)
        
        depth_window = depth_frame[y_start:y_end, x_start:x_end]
        
        # Filter out zeros and invalid depths
        valid_depths = depth_window[(depth_window > 0) & (depth_window < self.depth_max_distance * 1000)]
        
        if len(valid_depths) > 0:
            # Return median of valid depths
            return np.median(valid_depths) / 1000.0  # Convert to meters
        return None
    
    def create_marker(self, position, class_name, confidence, marker_id, is_persistent=False):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
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
        
        # Get color for this class
        color = self.get_random_color(class_name)
        marker.color.r = color[2] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.b = color[0] / 255.0
        marker.color.a = 0.8
        
        # Set marker lifetime
        if is_persistent:
            marker.lifetime.sec = 2  # Keep markers visible for 2 seconds even if not updated
        else:
            marker.lifetime.sec = 1
        
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
        current_time = self.get_clock().now().to_msg().sec
        
        # Only update if confidence is above threshold
        if confidence < self.confidence_threshold:
            return
            
        position_np = np.array(position)
        similar_found = False
        min_dist = float('inf')
        best_match_idx = -1
        
        # Clean up old detections first
        self.detected_objects[class_name] = [
            (pos, conf, time) for pos, conf, time in self.detected_objects[class_name]
            if (current_time - time) < self.object_timeout
        ]
        
        # Find the closest existing detection
        for i, (pos, conf, last_update) in enumerate(self.detected_objects[class_name]):
            pos_np = np.array(pos)
            dist = np.sqrt(np.sum((position_np - pos_np)**2))
            
            if dist < min_dist:
                min_dist = dist
                best_match_idx = i
        
        # Update existing detection if close enough and enough time has passed
        if min_dist < self.distance_filter_threshold:
            pos, conf, last_update = self.detected_objects[class_name][best_match_idx]
            
            # Only update if enough time has passed or confidence is significantly higher
            if (current_time - last_update) >= self.min_update_interval or confidence > conf + 0.1:
                pos_np = np.array(pos)
                
                # Update position with stronger weight to previous position
                new_pos = tuple(
                    self.position_update_alpha * position_np + 
                    (1 - self.position_update_alpha) * pos_np
                )
                
                # Use max confidence
                new_conf = max(confidence, conf)
                
                self.detected_objects[class_name][best_match_idx] = (new_pos, new_conf, current_time)
            similar_found = True
        
        # Add new detection if no similar object found and confidence is high
        if not similar_found and confidence > self.confidence_threshold + 0.05:
            self.detected_objects[class_name].append((tuple(position), confidence, current_time))
    
    def transform_point_to_map(self, point_camera):
        try:
            # Look up transform from camera to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_color_optical_frame',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract translation
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            # Quaternion to rotation matrix conversion
            qw = rot.w
            qx = rot.x
            qy = rot.y
            qz = rot.z
            
            # First row of the rotation matrix
            r00 = 1 - 2 * (qy * qy + qz * qz)
            r01 = 2 * (qx * qy - qz * qw)
            r02 = 2 * (qx * qz + qy * qw)
            
            # Second row of the rotation matrix
            r10 = 2 * (qx * qy + qz * qw)
            r11 = 1 - 2 * (qx * qx + qz * qz)
            r12 = 2 * (qy * qz - qx * qw)
            
            # Third row of the rotation matrix
            r20 = 2 * (qx * qz - qy * qw)
            r21 = 2 * (qy * qz + qx * qw)
            r22 = 1 - 2 * (qx * qx + qy * qy)
            
            # Apply rotation and translation
            # Remember: camera optical frame has z forward, x right, y down
            # Convert from camera optical frame to map frame
            x = point_camera[0]  # camera x (right)
            y = point_camera[1]  # camera y (down)
            z = point_camera[2]  # camera z (forward)
            
            # Apply rotation
            px = r00 * x + r01 * y + r02 * z
            py = r10 * x + r11 * y + r12 * z
            pz = r20 * x + r21 * y + r22 * z
            
            # Apply translation
            point_map = [
                px + trans.x,
                py + trans.y,
                pz + trans.z
            ]
            
            return point_map
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error: {str(e)}')
            return None
    
    def publish_persistent_markers(self):
        current_time = self.get_clock().now().to_msg().sec
        marker_array = MarkerArray()
        
        # Add all persistent detections that haven't timed out
        for class_name, detections in self.detected_objects.items():
            for position, confidence, last_update in detections:
                if (current_time - last_update) < self.object_timeout:
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
            
            # Dictionary to count detections by class and store distances
            detection_info = defaultdict(list)
            
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
                        
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Get color for this class
                    color = self.get_random_color(class_name)
                    
                    # Draw bounding box and label
                    cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(vis_frame, f'{class_name} {confidence:.2f}', (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Get center point
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    try:
                        # Get filtered depth value (in meters)
                        depth = self.filter_depth(self.latest_depth_frame, center_x, center_y)
                        
                        if depth is not None and depth > 0:
                            # Convert to 3D point in camera frame
                            fx = self.camera_info.k[0]  # focal length x
                            fy = self.camera_info.k[4]  # focal length y
                            cx = self.camera_info.k[2]  # optical center x
                            cy = self.camera_info.k[5]  # optical center y
                            
                            # Calculate 3D point in camera frame
                            x = (center_x - cx) * depth / fx
                            y = (center_y - cy) * depth / fy
                            z = depth
                            
                            # Transform point to map frame
                            point_map = self.transform_point_to_map([x, y, z])
                            
                            if point_map is not None:
                                # Store detection info
                                detection_info[class_name].append({
                                    'distance': depth,
                                    'position': point_map,
                                    'confidence': confidence
                                })
                                
                                # Update detected objects
                                self.update_detected_object(class_name, point_map, confidence)
                                
                                # Create current detection markers
                                markers = self.create_marker(point_map, class_name, confidence, self.next_marker_id)
                                marker_array.markers.extend(markers)
                                self.next_marker_id += 1
                            
                    except Exception as e:
                        self.get_logger().warn(f"Error processing detection: {e}")
            
            # Publish detection info
            if detection_info:
                info_msg = String()
                info_str = ""
                for obj_class, detections in detection_info.items():
                    for det in detections:
                        dist = det['distance']
                        pos = det['position']
                        conf = det['confidence']
                        info_str += f"{obj_class}: {dist:.2f}m away at position ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) with {conf:.2f} confidence\n"
                info_msg.data = info_str
                self.objects_pub.publish(info_msg)
            
            # Log detection counts
            if detection_info:
                log_msg = "Detected: " + ", ".join([f"{len(dets)} {cls}" for cls, dets in detection_info.items()])
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