#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import time

class ManualWheelControl(Node):
    def __init__(self):
        super().__init__('manual_wheel_control')
        
        # ROS2 parameters
        self.declare_parameters(namespace='',
                             parameters=[
                                 ('front_port', '/dev/ttyUSB0'),
                                 ('rear_port', '/dev/ttyUSB1'),
                                 ('baudrate', 115200),
                                 ('timeout', 0.1),
                             ])
        
        # Initialize hoverboard communication
        self._init_hoverboards()
        
        # Create subscriber for wheel speeds
        self.speed_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_speeds',
            self.speed_callback,
            10)
        
        self.get_logger().info("Manual wheel control initialized")

    def _init_hoverboards(self):
        try:
            # Initialize front hoverboard
            self.front_port = serial.Serial(
                self.get_parameter('front_port').value,
                self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info(f"Front hoverboard initialized on {self.front_port.port}")
            
            # Initialize rear hoverboard
            self.rear_port = serial.Serial(
                self.get_parameter('rear_port').value,
                self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info(f"Rear hoverboard initialized on {self.rear_port.port}")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize hoverboards: {str(e)}")
            raise

    def _send_hoverboard_command(self, port, left_speed, right_speed):
        """Send command to a hoverboard"""
        try:
            # Ensure speeds are within valid range for signed short (-32768 to 32767)
            left_speed = max(-32768, min(32767, left_speed))
            right_speed = max(-32768, min(32767, right_speed))
            
            start_frame = 0xABCD
            # Calculate checksum using only the lower 16 bits
            checksum = (start_frame & 0xFFFF) ^ (left_speed & 0xFFFF) ^ (right_speed & 0xFFFF)
            
            # Log the command details
            self.get_logger().info("=== Command Details ===")
            self.get_logger().info(f"Port: {port.port}")
            self.get_logger().info(f"Start frame: 0x{start_frame:04X} (2 bytes)")
            self.get_logger().info(f"Left speed: {left_speed} (0x{left_speed & 0xFFFF:04X}) (2 bytes)")
            self.get_logger().info(f"Right speed: {right_speed} (0x{right_speed & 0xFFFF:04X}) (2 bytes)")
            self.get_logger().info(f"Checksum: 0x{checksum:04X} (2 bytes)")
            
            # Pack the command
            command = struct.pack('<HhhH', start_frame, left_speed, right_speed, checksum)
            
            # Log each byte of the command with position and meaning
            self.get_logger().info("=== Command Bytes ===")
            self.get_logger().info("Position | Byte | Meaning")
            self.get_logger().info("---------|------|---------")
            self.get_logger().info(f"0-1      | 0x{command[0]:02X} 0x{command[1]:02X} | Start frame (0xABCD)")
            self.get_logger().info(f"2-3      | 0x{command[2]:02X} 0x{command[3]:02X} | Left speed")
            self.get_logger().info(f"4-5      | 0x{command[4]:02X} 0x{command[5]:02X} | Right speed")
            self.get_logger().info(f"6-7      | 0x{command[6]:02X} 0x{command[7]:02X} | Checksum")
            self.get_logger().info("=====================")
            
            # Clear any existing data in the buffer
            port.reset_input_buffer()
            port.reset_output_buffer()
            
            # Send the command
            bytes_written = port.write(command)
            self.get_logger().info(f"Bytes written: {bytes_written}")
            
            # Make sure the command is sent
            port.flush()
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
        except struct.error as e:
            self.get_logger().error(f"Failed to pack command: {str(e)}")

    def speed_callback(self, msg):
        """Handle incoming speed commands"""
        if len(msg.data) != 4:
            self.get_logger().error(f"Expected 4 speeds, got {len(msg.data)}")
            return
            
        # Extract speeds for each wheel
        front_left = msg.data[0]
        front_right = msg.data[1]
        rear_left = msg.data[2]
        rear_right = msg.data[3]
        
        # Send commands to hoverboards
        self._send_hoverboard_command(self.front_port, front_left, front_right)
        self._send_hoverboard_command(self.rear_port, rear_left, rear_right)
        
        self.get_logger().info(f"Speeds set: FL={front_left}, FR={front_right}, RL={rear_left}, RR={rear_right}")

    def destroy_node(self):
        self.front_port.close()
        self.rear_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ManualWheelControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 