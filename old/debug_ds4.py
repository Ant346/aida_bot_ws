#!/usr/bin/env python3
"""
Debug script for DS4 controller input issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class DS4Debugger(Node):
    def __init__(self):
        super().__init__('ds4_debugger')
        
        # Subscribe to raw joy data
        self.joy_sub = self.create_subscription(
            Joy, 
            '/ds4/joy', 
            self.joy_callback, 
            10
        )
        
        # Subscribe to twist data
        self.twist_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.twist_callback, 
            10
        )
        
        self.get_logger().info('DS4 Debugger started')
        self.get_logger().info('Publishing to /debug_cmd_vel for testing')
        
        # Publisher for testing
        self.debug_pub = self.create_publisher(Twist, '/debug_cmd_vel', 10)
        
        # Timer for manual testing
        self.timer = self.create_timer(0.1, self.test_publisher)
        
        self.manual_mode = False
        self.test_value = 0.0
        self.increasing = True
        
    def joy_callback(self, msg):
        """Debug joy input"""
        if len(msg.axes) >= 4:  # DS4 has at least 4 axes
            left_x = msg.axes[0]   # Left stick X (should be linear.x)
            left_y = msg.axes[1]   # Left stick Y (should be linear.y)
            right_x = msg.axes[2]  # Right stick X
            right_y = msg.axes[3]  # Right stick Y
            
            self.get_logger().info(f'Joy Input - Left: ({left_x:.3f}, {left_y:.3f}), Right: ({right_x:.3f}, {right_y:.3f})')
            
            # Check if left stick X is working
            if abs(left_x) > 0.1:
                self.get_logger().info(f'✅ Left stick X working: {left_x:.3f}')
            else:
                self.get_logger().info(f'❌ Left stick X not responding: {left_x:.3f}')
                
            # Check button states
            if msg.buttons:
                self.get_logger().info(f'Buttons: {msg.buttons}')
                
    def twist_callback(self, msg):
        """Debug twist output"""
        self.get_logger().info(f'Twist Output - Linear: ({msg.linear.x:.3f}, {msg.linear.y:.3f}), Angular: {msg.angular.z:.3f}')
        
    def test_publisher(self):
        """Test publisher to verify cmd_vel is working"""
        if self.manual_mode:
            twist = Twist()
            twist.linear.x = self.test_value
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            
            self.debug_pub.publish(twist)
            
            # Oscillate test value
            if self.increasing:
                self.test_value += 0.1
                if self.test_value >= 1.0:
                    self.increasing = False
            else:
                self.test_value -= 0.1
                if self.test_value <= -1.0:
                    self.increasing = True
                    
            if abs(self.test_value) < 0.1:
                self.test_value = 0.0

def main():
    rclpy.init()
    node = DS4Debugger()
    
    print("🎮 DS4 Controller Debugger")
    print("=" * 40)
    print("This will show you:")
    print("1. Raw joy input from DS4 controller")
    print("2. Twist output being published")
    print("3. Test publisher on /debug_cmd_vel")
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 40)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Debugger stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 