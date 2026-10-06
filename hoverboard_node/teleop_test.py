#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Teleop node started. Use WASD keys to control:")
        self.get_logger().info("W: Forward, S: Backward, A: Left, D: Right")
        self.get_logger().info("Q: Rotate Left, E: Rotate Right")
        self.get_logger().info("X: Stop, Ctrl+C: Exit")
        
    def get_key(self):
        """Get a single keypress from stdin"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def run(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key()
                    twist = Twist()
                    
                    if key == 'w':
                        twist.linear.x = 0.5
                        self.get_logger().info("Forward")
                    elif key == 's':
                        twist.linear.x = -0.5
                        self.get_logger().info("Backward")
                    elif key == 'a':
                        twist.linear.y = 0.5
                        self.get_logger().info("Left")
                    elif key == 'd':
                        twist.linear.y = -0.5
                        self.get_logger().info("Right")
                    elif key == 'q':
                        twist.angular.z = 0.5
                        self.get_logger().info("Rotate Left")
                    elif key == 'e':
                        twist.angular.z = -0.5
                        self.get_logger().info("Rotate Right")
                    elif key == 'x':
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Stop")
                    elif key == '\x03':  # Ctrl+C
                        break
                    else:
                        continue
                    
                    self.publisher.publish(twist)
                    
        except KeyboardInterrupt:
            pass
        finally:
            # Stop the robot
            twist = Twist()
            self.publisher.publish(twist)
            self.get_logger().info("Teleop stopped")

def main():
    rclpy.init()
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 