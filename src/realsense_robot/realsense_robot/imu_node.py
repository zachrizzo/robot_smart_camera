#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
import numpy as np
import transforms3d
import message_filters

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Create subscriber for IMU data with compatible QoS
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos
        )
        
        # Create publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize pose tracking
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        
        # Initialize timestamps
        self.last_time = None
        
        # Initialize filters with more responsive parameters
        self.alpha = 0.6  # Increased for faster response
        self.accel_lpf = np.array([0.0, 0.0, 0.0])
        self.gyro_lpf = np.array([0.0, 0.0, 0.0])
        
        # Gravity compensation
        self.gravity = np.array([0.0, 0.0, 9.81])
        self.init_count = 0
        self.init_readings = []
        self.gravity_calibrated = False
        
        # Reduced motion thresholds for better sensitivity
        self.accel_threshold = 0.01  # m/s^2
        self.gyro_threshold = 0.005   # rad/s
        
        # Bias estimation
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.bias_samples = 50  # Reduced for faster startup
        
        self.get_logger().info('IMU Node initialized with more responsive parameters')
        
    def calibrate_gravity(self, accel):
        if not self.gravity_calibrated:
            if self.init_count < self.bias_samples:
                self.init_readings.append(accel)
                self.init_count += 1
            else:
                # Average the readings to get gravity vector and accelerometer bias
                readings = np.array(self.init_readings)
                mean_accel = np.mean(readings, axis=0)
                self.gravity = mean_accel / np.linalg.norm(mean_accel) * 9.81
                self.accel_bias = mean_accel - self.gravity
                self.gravity_calibrated = True
                self.get_logger().info(f'Gravity calibrated: {self.gravity}')
                self.get_logger().info(f'Accelerometer bias: {self.accel_bias}')

    def estimate_gyro_bias(self, gyro):
        if self.init_count < self.bias_samples:
            self.gyro_bias += gyro / self.bias_samples
            return True
        return False
        
    def imu_callback(self, msg: Imu):
        # Extract linear acceleration and angular velocity
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Calibrate gravity and estimate biases
        self.calibrate_gravity(accel)
        if self.estimate_gyro_bias(gyro):
            return
        
        if not self.gravity_calibrated:
            return
        
        # Remove biases
        accel = accel - self.accel_bias
        gyro = gyro - self.gyro_bias
        
        # Apply low pass filter
        self.accel_lpf = self.alpha * accel + (1 - self.alpha) * self.accel_lpf
        self.gyro_lpf = self.alpha * gyro + (1 - self.alpha) * self.gyro_lpf
        
        # Apply motion thresholds
        accel_mask = np.abs(self.accel_lpf) < self.accel_threshold
        gyro_mask = np.abs(self.gyro_lpf) < self.gyro_threshold
        self.accel_lpf[accel_mask] = 0.0
        self.gyro_lpf[gyro_mask] = 0.0
        
        # Get time delta
        current_time = msg.header.stamp
        if self.last_time is not None:
            dt = (current_time.sec - self.last_time.sec) + \
                 (current_time.nanosec - self.last_time.nanosec) * 1e-9
            
            # Update orientation using gyroscope data
            angle = np.linalg.norm(self.gyro_lpf) * dt
            if angle > 0:
                axis = self.gyro_lpf / np.linalg.norm(self.gyro_lpf)
                dq = transforms3d.quaternions.axangle2quat(axis, angle)
                self.orientation = transforms3d.quaternions.qmult(self.orientation, dq)
                # Normalize quaternion to prevent drift
                self.orientation = self.orientation / np.linalg.norm(self.orientation)
            
            # Transform acceleration to world frame and remove gravity
            rot_mat = transforms3d.quaternions.quat2mat(self.orientation)
            accel_world = rot_mat @ self.accel_lpf - self.gravity
            
            # Update velocity and position with improved drift compensation
            if np.linalg.norm(accel_world) > self.accel_threshold:
                self.velocity += accel_world * dt
                # Apply stronger damping to velocity to reduce drift
                self.velocity *= 0.95
                self.position += self.velocity * dt
            else:
                # If acceleration is below threshold, decay velocity faster
                self.velocity *= 0.90
            
            # Create and publish transform
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'world'
            t.child_frame_id = 'camera_link'
            
            t.transform.translation.x = float(self.position[0])
            t.transform.translation.y = float(self.position[1])
            t.transform.translation.z = float(self.position[2])
            
            t.transform.rotation.w = float(self.orientation[0])
            t.transform.rotation.x = float(self.orientation[1])
            t.transform.rotation.y = float(self.orientation[2])
            t.transform.rotation.z = float(self.orientation[3])
            
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
            
            # if np.linalg.norm(self.gyro_lpf) > self.gyro_threshold or np.linalg.norm(accel_world) > self.accel_threshold:
            #     self.get_logger().info(f'Motion detected - Pos: {self.position}, Vel: {self.velocity}, Accel: {accel_world}')
        
        self.last_time = current_time

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