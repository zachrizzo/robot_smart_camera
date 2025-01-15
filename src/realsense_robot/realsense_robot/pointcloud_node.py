#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import struct

class PointCloudGenerator(Node):
    def __init__(self):
        super().__init__('pointcloud_generator')
        
        self.bridge = CvBridge()
        
        # Subscribe to the aligned depth and color images
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/color/camera_info')
        
        # Synchronize the subscriptions
        self.ts = message_filters.TimeSynchronizer(
            [self.depth_sub, self.color_sub, self.camera_info_sub], 10)
        self.ts.registerCallback(self.callback)
        
        # Create publisher for point cloud
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/camera/pointcloud', 10)
        
        self.get_logger().info('Point cloud generator node initialized')
    
    def callback(self, depth_msg, color_msg, info_msg):
        try:
            # Convert images to numpy arrays
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
            color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            
            # Get camera intrinsics
            fx = info_msg.k[0]
            fy = info_msg.k[4]
            cx = info_msg.k[2]
            cy = info_msg.k[5]
            
            # Create point cloud
            height, width = depth_image.shape
            points = []
            
            for v in range(0, height, 2):  # Skip pixels for performance
                for u in range(0, width, 2):
                    depth = float(depth_image[v, u]) / 1000.0  # Convert to meters
                    if depth > 0:
                        # Calculate 3D point
                        x = (u - cx) * depth / fx
                        y = (v - cy) * depth / fy
                        z = depth
                        
                        # Get color
                        b, g, r = color_image[v, u]
                        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                        
                        points.append([x, y, z, rgb])
            
            if not points:
                return
            
            # Create point cloud message
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]
            
            header = depth_msg.header
            pc_msg = pc2.create_cloud(header, fields, points)
            
            # Publish point cloud
            self.cloud_pub.publish(pc_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 