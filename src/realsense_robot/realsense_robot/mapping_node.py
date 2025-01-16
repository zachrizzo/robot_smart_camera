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
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            qos
        )
        
        # Use best effort QoS for pose data
        pose_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/camera/pose',
            self.pose_callback,
            pose_qos
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
        self.previous_pcd = None
        self.max_points = 1000000  # Increased for higher resolution
        self.voxel_size = 0.005   # Decreased for higher resolution (5mm)
        
        # Parameters for outlier removal
        self.nb_neighbors = 50     # Increased for better noise removal
        self.std_ratio = 2.0      # Less aggressive to keep more detail
        
        # Parameters for radius outlier removal
        self.radius = 0.02        # 2cm radius - decreased for finer detail
        self.min_points = 5       # Reduced minimum points for higher detail
        
        # ICP parameters
        self.icp_threshold = 0.02  # 2cm threshold for more precise alignment
        self.icp_iterations = 30   # Reduced for better performance
        
        # Create timer for publishing global map
        self.create_timer(0.1, self.publish_map)  # Increased publish rate
        
        # Initialize CUDA for GPU acceleration if available
        try:
            if o3d.core.cuda.is_available():
                o3d.core.Device("CUDA:0")
                self.get_logger().info('CUDA acceleration enabled')
            else:
                self.get_logger().info('CUDA not available, using CPU')
        except:
            self.get_logger().warn('Failed to initialize CUDA, using CPU')
        
        self.get_logger().info('Mapping node initialized with high-resolution parameters')
        
    def filter_point_cloud(self, pcd):
        """Apply multiple filtering steps to clean the point cloud."""
        try:
            # Convert to CUDA tensor if available
            if o3d.core.cuda.is_available():
                pcd = pcd.to(o3d.core.Device("CUDA:0"))
            
            # Voxel downsampling
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # Statistical outlier removal
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=self.nb_neighbors,
                std_ratio=self.std_ratio
            )
            
            # Radius outlier removal - only if points are sparse
            if len(pcd.points) < self.max_points // 2:
                pcd, _ = pcd.remove_radius_outlier(
                    nb_points=self.min_points,
                    radius=self.radius
                )
            
            return pcd
        except Exception as e:
            self.get_logger().warn(f'Error in filtering: {str(e)}')
            return pcd
        
    def register_point_cloud(self, source, target):
        """Register two point clouds using ICP with GPU acceleration if available."""
        if len(source.points) < 10 or len(target.points) < 10:
            return source, np.eye(4)
            
        try:
            # Estimate normals if they don't exist
            if not source.has_normals():
                source.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
                )
            if not target.has_normals():
                target.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
                )
            
            # Use GPU-accelerated ICP if available
            if o3d.core.cuda.is_available():
                result = o3d.pipelines.registration.registration_icp(
                    source.to(o3d.core.Device("CUDA:0")),
                    target.to(o3d.core.Device("CUDA:0")),
                    self.icp_threshold,
                    np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_iterations)
                )
            else:
                result = o3d.pipelines.registration.registration_icp(
                    source, target,
                    self.icp_threshold,
                    np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_iterations)
                )
            
            # Transform the source point cloud
            source.transform(result.transformation)
            return source, result.transformation
        except Exception as e:
            self.get_logger().warn(f'Error in registration: {str(e)}')
            return source, np.eye(4)

    def pointcloud_callback(self, msg):
        if self.current_pose is None:
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            field_names = [field.name for field in msg.fields]
            cloud_data = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
            points = np.array(list(cloud_data))
            
            if len(points) == 0:
                return
                
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            # Reshape points array if it's 1-dimensional
            if len(points.shape) == 1:
                # Assuming the points are stored as a structured array
                xyz = np.zeros((len(points), 3))
                xyz[:, 0] = points['x']
                xyz[:, 1] = points['y']
                xyz[:, 2] = points['z']
            else:
                xyz = points[:, :3]
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # Check if RGB data is available
            rgb_idx = [i for i, name in enumerate(field_names) if name in ['rgb', 'rgba']]
            if rgb_idx:
                if len(points.shape) == 1:
                    rgb = points['rgb']
                else:
                    rgb = points[:, rgb_idx[0]]
                rgb.dtype = np.uint32
                r = (rgb >> 16) & 0x0000ff
                g = (rgb >> 8) & 0x0000ff
                b = rgb & 0x0000ff
                colors = np.vstack([r, g, b]).T.astype(np.float64) / 255.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Initial filtering of new point cloud
            pcd = self.filter_point_cloud(pcd)
            
            # Transform to global frame using pose
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
            
            # Register with global map if it exists
            if len(self.global_pcd.points) > 0:
                pcd, transform = self.register_point_cloud(pcd, self.global_pcd)
                self.get_logger().info(f'ICP registration fitness: {transform[0, 0]:.3f}')
            
            # Merge with global map
            if len(self.global_pcd.points) == 0:
                self.global_pcd = pcd
            else:
                self.global_pcd += pcd
                # Filter the combined point cloud
                self.global_pcd = self.filter_point_cloud(self.global_pcd)
            
            # Limit total number of points
            if len(self.global_pcd.points) > self.max_points:
                points_np = np.asarray(self.global_pcd.points)
                # Use uniform sampling instead of random
                every_nth = len(points_np) // self.max_points
                indices = np.arange(0, len(points_np), every_nth)[:self.max_points]
                self.global_pcd.points = o3d.utility.Vector3dVector(points_np[indices])
                if len(self.global_pcd.colors) > 0:
                    colors_np = np.asarray(self.global_pcd.colors)
                    self.global_pcd.colors = o3d.utility.Vector3dVector(colors_np[indices])
            
            self.get_logger().info(f'Updated global map. Total points: {len(self.global_pcd.points)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(f'Received pose: position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})')
        
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