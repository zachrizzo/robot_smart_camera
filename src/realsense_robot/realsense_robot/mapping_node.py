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
        
        # Subscribe to RTAB-Map's pose if enabled
        self.declare_parameter('use_rtabmap_pose', True)
        self.use_rtabmap_pose = self.get_parameter('use_rtabmap_pose').value
        
        if self.use_rtabmap_pose:
            # Subscribe to RTAB-Map's odometry
            self.pose_sub = self.create_subscription(
                PoseStamped,
                '/rtabmap/odom',
                self.pose_callback,
                pose_qos
            )
            # Subscribe to RTAB-Map's map data
            self.map_sub = self.create_subscription(
                PointCloud2,
                '/rtabmap/cloud_map',
                self.rtabmap_cloud_callback,
                qos
            )
        else:
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
        self.max_points = 1000000  # Maximum points in global map
        self.voxel_size = 0.005   # 5mm voxel size for high resolution
        
        # Parameters for outlier removal
        self.nb_neighbors = 50     
        self.std_ratio = 2.0      
        
        # Parameters for radius outlier removal
        self.radius = 0.02        
        self.min_points = 5       
        
        # ICP parameters
        self.icp_threshold = 0.02  
        self.icp_iterations = 30   
        
        # Add buffer for recent point clouds and their poses
        self.recent_clouds = []
        self.recent_poses = []
        self.max_recent_clouds = 5
        
        # Add movement threshold to only update on significant motion
        self.min_movement = 0.01  # 1cm minimum movement
        self.last_update_pose = None
        
        # Add voxel grid for maintaining source of truth
        self.voxel_grid = {}  # Dictionary to store occupied voxels
        self.confidence_threshold = 0.6  # Minimum confidence for a point to be kept
        
        # Add drift correction parameters
        self.initial_pose = None
        self.cumulative_transform = np.eye(4)
        self.drift_correction = np.eye(4)
        self.drift_threshold = 0.1  # 10cm threshold for drift correction
        
        # Create timer for publishing global map
        self.create_timer(0.1, self.publish_map)
        
        # Initialize CUDA for GPU acceleration if available
        try:
            if o3d.core.cuda.is_available():
                o3d.core.Device("CUDA:0")
                self.get_logger().info('CUDA acceleration enabled')
            else:
                self.get_logger().info('CUDA not available, using CPU')
        except:
            self.get_logger().warn('Failed to initialize CUDA, using CPU')
        
        self.get_logger().info('Mapping node initialized with RTAB-Map integration')

    def has_moved_enough(self, current_pose):
        """Check if the camera has moved enough to warrant a new update."""
        if self.last_update_pose is None:
            return True
            
        position_diff = np.array([
            current_pose.position.x - self.last_update_pose.position.x,
            current_pose.position.y - self.last_update_pose.position.y,
            current_pose.position.z - self.last_update_pose.position.z
        ])
        
        return np.linalg.norm(position_diff) > self.min_movement

    def correct_point_cloud_axes(self, xyz):
        """Correct point cloud axes to match world frame."""
        # Convert from camera frame to world frame
        # Camera: x=right, y=down, z=forward
        # World: x=forward, y=left, z=up
        corrected_xyz = np.zeros_like(xyz)
        corrected_xyz[:, 0] = xyz[:, 2]  # camera z -> world x
        corrected_xyz[:, 1] = -xyz[:, 0]  # camera -x -> world y
        corrected_xyz[:, 2] = -xyz[:, 1]  # camera -y -> world z
        return corrected_xyz

    def update_recent_clouds(self, pcd, pose):
        """Update the buffer of recent point clouds and their poses."""
        self.recent_clouds.append(pcd)
        self.recent_poses.append(pose)
        if len(self.recent_clouds) > self.max_recent_clouds:
            self.recent_clouds.pop(0)
            self.recent_poses.pop(0)

    def merge_recent_clouds(self):
        """Merge recent point clouds to reduce noise and improve registration."""
        if not self.recent_clouds:
            return None
            
        merged = self.recent_clouds[0]
        for pcd in self.recent_clouds[1:]:
            merged += pcd
        
        return self.filter_point_cloud(merged)

    def filter_point_cloud(self, pcd):
        """Apply multiple filtering steps to clean the point cloud."""
        try:
            # Convert to CUDA tensor if available
            if o3d.core.cuda.is_available():
                pcd = pcd.to(o3d.core.Device("CUDA:0"))
            
            # Remove NaN points
            pcd = pcd.remove_non_finite_points()
            
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
            
            # Estimate normals if they don't exist
            if not pcd.has_normals():
                pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.1,
                        max_nn=30
                    )
                )
                pcd.orient_normals_towards_camera_location(np.array([0., 0., 0.]))
            
            return pcd
        except Exception as e:
            self.get_logger().warn(f'Error in filtering: {str(e)}')
            return None
        
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

    def get_voxel_key(self, point):
        """Convert a point to its voxel grid key."""
        x = int(point[0] / self.voxel_size)
        y = int(point[1] / self.voxel_size)
        z = int(point[2] / self.voxel_size)
        return (x, y, z)

    def update_voxel_grid(self, pcd, confidence=1.0):
        """Update the voxel grid with new points."""
        try:
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) if len(pcd.colors) > 0 else None
            normals = np.asarray(pcd.normals) if pcd.has_normals() else None
            
            updated_points = []
            updated_colors = []
            updated_normals = []
            
            for i, point in enumerate(points):
                key = self.get_voxel_key(point)
                
                if key not in self.voxel_grid:
                    self.voxel_grid[key] = {
                        'point': point,
                        'confidence': confidence,
                        'color': colors[i] if colors is not None else None,
                        'normal': normals[i] if normals is not None else None
                    }
                    updated_points.append(point)
                    if colors is not None:
                        updated_colors.append(colors[i])
                    if normals is not None:
                        updated_normals.append(normals[i])
                else:
                    # Update existing voxel with weighted average
                    existing = self.voxel_grid[key]
                    weight = existing['confidence'] / (existing['confidence'] + confidence)
                    new_weight = 1 - weight
                    
                    # Update point position
                    existing['point'] = weight * existing['point'] + new_weight * point
                    existing['confidence'] = min(1.0, existing['confidence'] + confidence)
                    
                    # Update color if available
                    if colors is not None and existing['color'] is not None:
                        existing['color'] = weight * existing['color'] + new_weight * colors[i]
                    
                    # Update normal if available
                    if normals is not None and existing['normal'] is not None:
                        normal = weight * existing['normal'] + new_weight * normals[i]
                        normal /= np.linalg.norm(normal)
                        existing['normal'] = normal
                    
                    if existing['confidence'] >= self.confidence_threshold:
                        updated_points.append(existing['point'])
                        if existing['color'] is not None:
                            updated_colors.append(existing['color'])
                        if existing['normal'] is not None:
                            updated_normals.append(existing['normal'])
            
            # Create new point cloud from updated voxels
            new_pcd = o3d.geometry.PointCloud()
            new_pcd.points = o3d.utility.Vector3dVector(np.array(updated_points))
            if updated_colors:
                new_pcd.colors = o3d.utility.Vector3dVector(np.array(updated_colors))
            if updated_normals:
                new_pcd.normals = o3d.utility.Vector3dVector(np.array(updated_normals))
            
            return new_pcd
        except Exception as e:
            self.get_logger().error(f'Error updating voxel grid: {str(e)}')
            return pcd

    def get_transform_matrix(self, from_pose, to_pose):
        """Calculate transform matrix between two poses."""
        T = np.eye(4)
        
        # Calculate rotation difference
        q1 = [from_pose.orientation.w, from_pose.orientation.x,
              from_pose.orientation.y, from_pose.orientation.z]
        q2 = [to_pose.orientation.w, to_pose.orientation.x,
              to_pose.orientation.y, to_pose.orientation.z]
        
        R1 = o3d.geometry.get_rotation_matrix_from_quaternion(q1)
        R2 = o3d.geometry.get_rotation_matrix_from_quaternion(q2)
        T[:3, :3] = R2 @ np.linalg.inv(R1)
        
        # Calculate translation difference
        t = np.array([
            to_pose.position.x - from_pose.position.x,
            to_pose.position.y - from_pose.position.y,
            to_pose.position.z - from_pose.position.z
        ])
        T[:3, 3] = t
        
        return T

    def check_drift(self, current_transform):
        """Check and correct for drift in the transformation."""
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            return current_transform
            
        # If using RTAB-Map pose, skip drift correction
        if self.use_rtabmap_pose:
            return current_transform
            
        # Calculate expected transform from initial pose
        expected_transform = self.get_transform_matrix(self.initial_pose, self.current_pose)
        
        # Calculate drift
        drift = np.linalg.inv(expected_transform) @ current_transform
        drift_translation = np.linalg.norm(drift[:3, 3])
        
        if drift_translation > self.drift_threshold:
            # Apply correction
            self.drift_correction = expected_transform @ np.linalg.inv(current_transform)
            current_transform = expected_transform
            self.get_logger().info(f'Drift correction applied: {drift_translation:.3f}m')
        
        return current_transform
        
    def rtabmap_cloud_callback(self, msg):
        """Handle incoming point cloud map from RTAB-Map"""
        if not self.use_rtabmap_pose:
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
            if len(points.shape) == 1:
                xyz = np.zeros((len(points), 3))
                xyz[:, 0] = points['x']
                xyz[:, 1] = points['y']
                xyz[:, 2] = points['z']
            else:
                xyz = points[:, :3]
            
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # Add colors if available
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
            
            # Filter and update global map
            filtered_pcd = self.filter_point_cloud(pcd)
            if filtered_pcd is not None:
                self.global_pcd = self.update_voxel_grid(filtered_pcd, confidence=1.0)
                self.get_logger().info(f'Updated global map from RTAB-Map. Total points: {len(self.global_pcd.points)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing RTAB-Map cloud: {str(e)}')
            
    def pointcloud_callback(self, msg):
        if self.current_pose is None:
            return
            
        if not self.has_moved_enough(self.current_pose):
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            field_names = [field.name for field in msg.fields]
            cloud_data = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
            points = np.array(list(cloud_data))
            
            if len(points) == 0:
                return
                
            # Create Open3D point cloud with corrected axes
            pcd = o3d.geometry.PointCloud()
            if len(points.shape) == 1:
                xyz = np.zeros((len(points), 3))
                xyz[:, 0] = points['x']
                xyz[:, 1] = points['y']
                xyz[:, 2] = points['z']
            else:
                xyz = points[:, :3]
                
            # Correct point cloud axes before any other processing
            xyz = self.correct_point_cloud_axes(xyz)
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # Add colors if available
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
            
            # Initial filtering
            pcd = self.filter_point_cloud(pcd)
            if pcd is None:
                return
                
            if self.use_rtabmap_pose:
                # When using RTAB-Map, let it handle the transformations
                self.update_recent_clouds(pcd, self.current_pose)
                merged_pcd = self.merge_recent_clouds()
                if merged_pcd is not None:
                    self.global_pcd = self.update_voxel_grid(merged_pcd, confidence=0.8)
            else:
                # Transform to world coordinates
                transform = self.get_transform_matrix(
                    self.initial_pose if self.initial_pose else self.current_pose,
                    self.current_pose
                )
                transform = self.check_drift(transform)
                pcd.transform(transform)
                
                # Update recent clouds buffer with pose information
                self.update_recent_clouds(pcd, self.current_pose)
                
                # Merge recent clouds for better registration
                merged_pcd = self.merge_recent_clouds()
                if merged_pcd is None:
                    return
                
                # Register with global map
                if len(self.global_pcd.points) > 0:
                    merged_pcd, transform = self.register_point_cloud(merged_pcd, self.global_pcd)
                    fitness = transform[0, 0] if isinstance(transform, np.ndarray) else 0
                    self.get_logger().info(f'ICP registration fitness: {fitness:.3f}')
                
                # Update global map using voxel grid
                self.global_pcd = self.update_voxel_grid(merged_pcd, confidence=0.8)
            
            # Update last update pose
            self.last_update_pose = self.current_pose
            
            self.get_logger().info(f'Updated global map. Total points: {len(self.global_pcd.points)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(f'Received pose: position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})')
        
    def map_callback(self, msg):
        """Handle incoming map updates from RTAB-Map"""
        if self.use_rtabmap_pose:
            # Update the global map based on RTAB-Map's optimized map
            self.get_logger().info('Received map update from RTAB-Map')
        
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
            header.frame_id = 'map'
            
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