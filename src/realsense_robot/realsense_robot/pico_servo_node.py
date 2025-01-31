#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Float32

class PicoServoNode(Node):
    def __init__(self):
        super().__init__('pico_servo_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Setup serial connection
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Connected to Pico on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Pico: {str(e)}')
            return
            
        # Create subscription for servo angle
        self.servo_sub = self.create_subscription(
            Float32,
            'servo_angle',
            self.servo_callback,
            10
        )
        
        # Create publisher for current servo position
        self.position_pub = self.create_publisher(
            Float32,
            'servo_position',
            10
        )
        
        # Timer for reading from Pico
        self.create_timer(0.1, self.read_from_pico)
        
    def servo_callback(self, msg):
        """Handle incoming servo angle commands"""
        angle = msg.data
        # Ensure angle is within valid range (0-180)
        angle = max(0, min(180, angle))
        
        # Send command to Pico
        command = {'cmd': 'servo', 'angle': angle}
        try:
            self.serial.write(f'{json.dumps(command)}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')
            
    def read_from_pico(self):
        """Read any incoming data from Pico"""
        if not hasattr(self, 'serial'):
            return
            
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode().strip()
                try:
                    data = json.loads(line)
                    if 'position' in data:
                        msg = Float32()
                        msg.data = float(data['position'])
                        self.position_pub.publish(msg)
                except json.JSONDecodeError:
                    pass
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Pico: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PicoServoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 