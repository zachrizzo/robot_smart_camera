#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import transforms3d
import message_filters

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters
        self.declare_parameter('use_rtabmap_pose', True)
        self.use_rtabmap_pose = self.get_parameter('use_rtabmap_pose').value
        
        # Create subscribers for accelerometer and gyroscope data with compatible QoS
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create synchronized subscribers for accelerometer and gyroscope
        self.accel_sub = message_filters.Subscriber(
            self,
            Imu,
            '/camera/camera/accel/sample',
            qos_profile=qos
        )
        
        self.gyro_sub = message_filters.Subscriber(
            self,
            Imu,
            '/camera/camera/gyro/sample',
            qos_profile=qos
        )
        
        # Subscribe to RTAB-Map's pose if enabled
        if self.use_rtabmap_pose:
            # Subscribe to RTAB-Map's odometry
            self.rtabmap_pose_sub = self.create_subscription(
                PoseStamped,
                '/rtabmap/odom',
                self.rtabmap_pose_callback,
                qos
            )
            # Subscribe to filtered IMU data
            self.filtered_imu_sub = self.create_subscription(
                Imu,
                '/rtabmap/imu',
                self.filtered_imu_callback,
                qos
            )
            # Subscribe to RTAB-Map's map path for loop closures
            self.map_path_sub = self.create_subscription(
                PoseStamped,
                '/rtabmap/mapPath',
                self.map_path_callback,
                qos
            )
            # Subscribe to RTAB-Map's localization pose
            self.localization_sub = self.create_subscription(
                PoseStamped,
                '/rtabmap/localization_pose',
                self.localization_callback,
                qos
            )
            self.rtabmap_pose = None
            self.last_rtabmap_time = None
            self.filtered_imu = None
        
        # Create synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.accel_sub, self.gyro_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.imu_callback)
        
        # Create publishers with the same QoS profile
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize pose tracking
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        self._last_position = self.position.copy()
        
        # Initialize timestamps
        self.last_time = None
        
        # Low-pass filter parameters (alpha closer to 1 = more smoothing)
        self.alpha = 0.7  
        self.accel_lpf = np.array([0.0, 0.0, 0.0])
        self.gyro_lpf = np.array([0.0, 0.0, 0.0])
        
        # Gravity and bias
        self.gravity = np.array([0.0, 0.0, 9.81])
        self.gravity_calibrated = False
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        
        # Calibration buffers
        self.init_readings = []
        self.init_count = 0
        self.bias_samples = 50  # number of samples to estimate gravity + bias
        
        # Thresholds (tune as needed)
        self.accel_threshold = 0.001  # smaller threshold to catch small motions
        self.gyro_threshold = 0.001   # rad/s
        
        # Zero-velocity update
        self.still_frames_required = 5
        self.still_frame_count = 0
        
        # Debug counters
        self.imu_msg_count = 0
        self.pose_pub_count = 0
        self.last_debug_time = self.get_clock().now()
        
        self.get_logger().info('IMU Node initialized with synchronized IMU subscribers')

    def filtered_imu_callback(self, msg):
        """Handle incoming filtered IMU data from Madgwick filter"""
        self.filtered_imu = msg
        
        # Update orientation from filtered IMU if RTAB-Map pose is not available
        if self.use_rtabmap_pose and self.rtabmap_pose is None:
            self.orientation = np.array([
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z
            ])

    def rtabmap_pose_callback(self, msg):
        """Handle incoming RTAB-Map pose updates"""
        self.rtabmap_pose = msg.pose
        current_time = msg.header.stamp
        
        if self.last_rtabmap_time is not None:
            dt = (current_time.sec - self.last_rtabmap_time.sec) + \
                 (current_time.nanosec - self.last_rtabmap_time.nanosec) * 1e-9
            
            # Update position and orientation from RTAB-Map
            self.position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            
            self.orientation = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            
            # Calculate velocity from position change
            if dt > 0:
                new_velocity = (self.position - self._last_position) / dt
                # Low-pass filter the velocity
                self.velocity = self.alpha * new_velocity + (1 - self.alpha) * self.velocity
            
            self._last_position = self.position.copy()
        else:
            self._last_position = self.position.copy()
        
        self.last_rtabmap_time = current_time
        
        # Publish transform
        self.publish_transform(current_time)

    def publish_transform(self, current_time):
        """Publish the current transform and pose"""
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(self.position[2])
        
        t.transform.rotation.w = float(self.orientation[0])
        t.transform.rotation.x = float(self.orientation[1])
        t.transform.rotation.y = float(self.orientation[2])
        t.transform.rotation.z = float(self.orientation[3])
        
        # Only publish transform if not using RTAB-Map (RTAB-Map will handle TF tree)
        if not self.use_rtabmap_pose:
            self.tf_broadcaster.sendTransform(t)
        
        # Create and publish pose
        pose_msg = PoseStamped()
        pose_msg.header = t.header
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        pose_msg.pose.orientation.w = self.orientation[0]
        pose_msg.pose.orientation.x = self.orientation[1]
        pose_msg.pose.orientation.y = self.orientation[2]
        pose_msg.pose.orientation.z = self.orientation[3]
        
        self.pose_pub.publish(pose_msg)
        self.pose_pub_count += 1

    def calibrate_gravity_and_biases(self, accel, gyro):
        """
        Collect initial stationary samples to estimate:
        - Gravity direction
        - Accelerometer bias
        - Gyroscope bias
        """
        if self.init_count < self.bias_samples:
            self.init_readings.append((accel, gyro))
            self.init_count += 1
            return False
        else:
            # Compute mean of all collected samples
            accel_array = np.array([r[0] for r in self.init_readings])
            gyro_array = np.array([r[1] for r in self.init_readings])
            
            mean_accel = np.mean(accel_array, axis=0)
            mean_gyro = np.mean(gyro_array, axis=0)
            
            # Estimate gravity direction from average accel
            norm_accel = np.linalg.norm(mean_accel)
            if norm_accel > 1e-3:
                self.gravity = (mean_accel / norm_accel) * 9.81
            else:
                self.gravity = np.array([0.0, 0.0, 9.81])
            
            # Accelerometer bias: if the device was stationary, the average reading
            # should be gravity. So the difference is the bias.
            self.accel_bias = mean_accel - self.gravity
            
            # Gyro bias is simply the average of gyro readings (assuming stationary)
            self.gyro_bias = mean_gyro
            
            self.gravity_calibrated = True
            self.get_logger().info(f'Gravity calibrated: {self.gravity}')
            self.get_logger().info(f'Accelerometer bias: {self.accel_bias}')
            self.get_logger().info(f'Gyroscope bias: {self.gyro_bias}')
            return True
    
    def imu_callback(self, accel_msg: Imu, gyro_msg: Imu):
        """Process IMU data and update pose if RTAB-Map pose is not available"""
        self.imu_msg_count += 1
        
        # If using RTAB-Map pose and we have a recent one, skip IMU integration
        if self.use_rtabmap_pose and self.rtabmap_pose is not None:
            if self.last_rtabmap_time is not None:
                current_time = self.get_clock().now().to_msg()
                dt = (current_time.sec - self.last_rtabmap_time.sec) + \
                     (current_time.nanosec - self.last_rtabmap_time.nanosec) * 1e-9
                if dt < 0.1:  # If RTAB-Map pose is recent (within 100ms)
                    return
        
        # Extract linear acceleration and angular velocity from the messages
        raw_accel = np.array([
            accel_msg.linear_acceleration.x,
            accel_msg.linear_acceleration.y,
            accel_msg.linear_acceleration.z
        ])
        
        raw_gyro = np.array([
            gyro_msg.angular_velocity.x,
            gyro_msg.angular_velocity.y,
            gyro_msg.angular_velocity.z
        ])
        
        # Print debug info every 1 second
        current_time_ros = self.get_clock().now()
        if (current_time_ros - self.last_debug_time).nanoseconds > 1e9:  # 1 second
            self.get_logger().info(
                f'IMU messages received: {self.imu_msg_count}, '
                f'Poses published: {self.pose_pub_count}'
            )
            self.last_debug_time = current_time_ros
        
        # If we're still calibrating, just collect data
        if not self.gravity_calibrated:
            done = self.calibrate_gravity_and_biases(raw_accel, raw_gyro)
            if not done:
                return
        
        # Remove estimated biases
        accel = raw_accel - self.accel_bias
        gyro = raw_gyro - self.gyro_bias
        
        # Apply low-pass filters
        self.accel_lpf = self.alpha * accel + (1 - self.alpha) * self.accel_lpf
        self.gyro_lpf = self.alpha * gyro + (1 - self.alpha) * self.gyro_lpf
        
        # Threshold small noise to zero (helps reduce drift)
        if np.linalg.norm(self.accel_lpf) < self.accel_threshold:
            self.accel_lpf[:] = 0.0
        if np.linalg.norm(self.gyro_lpf) < self.gyro_threshold:
            self.gyro_lpf[:] = 0.0
        
        current_time = accel_msg.header.stamp
        if self.last_time is not None:
            dt = (current_time.sec - self.last_time.sec) + \
                 (current_time.nanosec - self.last_time.nanosec) * 1e-9
            
            # Update orientation using gyroscope data
            gyro_norm = np.linalg.norm(self.gyro_lpf)
            if gyro_norm > 0:
                angle = gyro_norm * dt
                axis = self.gyro_lpf / gyro_norm
                dq = transforms3d.quaternions.axangle2quat(axis, angle)
                self.orientation = transforms3d.quaternions.qmult(self.orientation, dq)
                # Normalize quaternion
                self.orientation /= np.linalg.norm(self.orientation)
            
            # Rotate the accelerometer reading to the world frame
            rot_mat = transforms3d.quaternions.quat2mat(self.orientation)
            accel_world = rot_mat @ self.accel_lpf
            
            # Check if we are stationary (accel and gyro both near zero)
            if (np.linalg.norm(self.accel_lpf) < self.accel_threshold and
                np.linalg.norm(self.gyro_lpf) < self.gyro_threshold):
                self.still_frame_count += 1
            else:
                self.still_frame_count = 0
            
            # If we've been still for N consecutive frames, force velocity = 0
            if self.still_frame_count >= self.still_frames_required:
                self.velocity[:] = 0.0
            else:
                # Update velocity and position
                self.velocity += accel_world * dt
                # Damp velocity slightly every iteration to reduce drift
                self.velocity *= 0.98
                self.position += self.velocity * dt
            
            # Publish the transform and pose
            self.publish_transform(current_time)
        
        self.last_time = current_time

    def map_path_callback(self, msg):
        """Handle incoming map path updates from RTAB-Map after loop closures"""
        if self.use_rtabmap_pose:
            # Update position and orientation from optimized path
            self.position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            
            self.orientation = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            
            # Publish the updated transform
            self.publish_transform(msg.header.stamp)
            
    def localization_callback(self, msg):
        """Handle incoming localization pose updates from RTAB-Map"""
        if self.use_rtabmap_pose:
            # Update position and orientation from localization
            self.position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            
            self.orientation = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            
            # Publish the updated transform
            self.publish_transform(msg.header.stamp)

def main():
    rclpy.init()
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
