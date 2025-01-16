#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import numpy as np
import open3d as o3d
import struct
from sensor_msgs_py import point_cloud2

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Initialize subscribers with best effort QoS for better real-time performance
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/camera/camera/pose',
            self.pose_callback,
            qos
        )
        
        # Initialize publisher for the global map
        self.map_pub = self.create_publisher(
            PointCloud2,
            '/map/pointcloud',
            10
        )
        
        # Initialize variables
        self.current_pose = None
        self.global_pcd = o3d.geometry.PointCloud()
        self.max_points = 500000  # Reduced to prevent memory issues
        self.voxel_size = 0.02    # Increased voxel size for better performance
        
        # Parameters for outlier removal
        self.nb_neighbors = 30  # Increased for better noise removal
        self.std_ratio = 1.5    # Reduced for more aggressive outlier removal
        
        # Create timer for publishing global map
        self.create_timer(0.2, self.publish_map)  # Reduced to 5Hz for better performance
        
        self.get_logger().info('Mapping node initialized with topics:')
        self.get_logger().info(f' - Point cloud input: /camera/camera/depth/color/points')
        self.get_logger().info(f' - Pose input: /camera/camera/pose')
        self.get_logger().info(f' - Map output: /map/pointcloud')
        
    def pointcloud_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().debug('No pose data received yet, skipping point cloud')
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            field_names = [field.name for field in msg.fields]
            cloud_data = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
            points = np.array(list(cloud_data))
            
            if len(points) == 0:
                self.get_logger().debug('Received empty point cloud')
                return
                
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            
            # Check if RGB data is available
            rgb_idx = [i for i, name in enumerate(field_names) if name in ['rgb', 'rgba']]
            if rgb_idx:
                rgb = points[:, rgb_idx[0]].copy()
                rgb.dtype = np.uint32
                r = (rgb >> 16) & 0x0000ff
                g = (rgb >> 8) & 0x0000ff
                b = rgb & 0x0000ff
                colors = np.vstack([r, g, b]).T.astype(np.float64) / 255.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Remove outliers
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=self.nb_neighbors,
                std_ratio=self.std_ratio
            )
            
            # Transform point cloud to global frame using current pose
            T = np.eye(4)
            T[:3, 3] = [self.current_pose.position.x,
                        self.current_pose.position.y,
                        self.current_pose.position.z]
            q = [self.current_pose.orientation.w,
                 self.current_pose.orientation.x,
                 self.current_pose.orientation.y,
                 self.current_pose.orientation.z]
            T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            
            pcd.transform(T)
            
            # Merge with global point cloud
            if len(self.global_pcd.points) == 0:
                self.global_pcd = pcd
            else:
                # Voxel downsample before merging to reduce memory usage
                pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
                self.global_pcd += pcd
                self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # Limit total number of points
            if len(self.global_pcd.points) > self.max_points:
                points_np = np.asarray(self.global_pcd.points)
                indices = np.random.choice(len(points_np), self.max_points, replace=False)
                self.global_pcd.points = o3d.utility.Vector3dVector(points_np[indices])
                if len(self.global_pcd.colors) > 0:
                    colors_np = np.asarray(self.global_pcd.colors)
                    self.global_pcd.colors = o3d.utility.Vector3dVector(colors_np[indices])
            
            self.get_logger().debug(f'Updated global map. Total points: {len(self.global_pcd.points)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def publish_map(self):
        if len(self.global_pcd.points) == 0:
            return
            
        try:
            # Convert Open3D point cloud to numpy arrays
            points = np.asarray(self.global_pcd.points)
            colors = np.asarray(self.global_pcd.colors) if len(self.global_pcd.colors) > 0 else None
            
            # Create point cloud fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            if colors is not None:
                # Convert colors to uint32 RGB format
                colors_uint8 = (colors * 255).astype(np.uint8)
                rgb = np.zeros(len(points), dtype=np.uint32)
                rgb = np.bitwise_or(rgb, colors_uint8[:, 0].astype(np.uint32) << 16)
                rgb = np.bitwise_or(rgb, colors_uint8[:, 1].astype(np.uint32) << 8)
                rgb = np.bitwise_or(rgb, colors_uint8[:, 2].astype(np.uint32))
                
                # Add RGB field
                fields.append(PointField(
                    name='rgb',
                    offset=12,
                    datatype=PointField.UINT32,
                    count=1
                ))
                
                # Combine points and colors
                cloud_data = np.zeros(len(points), dtype=[
                    ('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32),
                    ('rgb', np.uint32)
                ])
                cloud_data['x'] = points[:, 0]
                cloud_data['y'] = points[:, 1]
                cloud_data['z'] = points[:, 2]
                cloud_data['rgb'] = rgb
            else:
                cloud_data = points
            
            # Create and publish the point cloud message
            header = point_cloud2.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'world'
            
            pc2_msg = point_cloud2.create_cloud(header, fields, cloud_data)
            self.map_pub.publish(pc2_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing map: {str(e)}')
        
def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 