#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import numpy as np
import serial
import time
import struct
import threading
from collections import deque
import glob
import os

class HoverboardOmniDriver(Node):
    def __init__(self):
        super().__init__('hoverboard_omni_driver')
        
        self.get_logger().info("Initializing hoverboard driver...")
        
        # ROS2 parameters
        self.declare_parameters(namespace='',
                             parameters=[
                                 ('front_serial', '0001'),# front hoverboard serial number
                                 ('rear_serial', '0002'),# rear hoverboard serial number
                                 ('baudrate', 115200),
                                 ('timeout', 0.05),
                                 ('wheel_radius', 0.095),  # meters
                                 ('wheel_base', 0.635),    # meters (distance between front and rear wheels)
                                 ('wheel_track', 0.72),   # meters (distance between left and right wheels)
                                 ('max_speed', 50),     # 
                                 ('test_speed', 0),    # test speed for individual wheel testing
                                 ('simulation_mode', False),  # Run in simulation mode without hardware
                             ])
        
        self.get_logger().info("Parameters declared")
        
        # Initialize hoverboard communication
        self._init_hoverboards()
        self.get_logger().info("Hoverboards initialized")
        
        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("Subscriber created")
        
        # ROS2 publishers (for feedback)
        self.odom_pub = self.create_publisher(Twist, 'hoverboard_odom', 10)
        self.get_logger().info("Publisher created")
        
        # Create service for testing individual wheels
        self.test_wheel_srv = self.create_service(
            Empty,
            'hoverboard_omni_driver/test_wheel',
            self.test_wheel_callback)
        self.get_logger().info("Test wheel service created")
        
        # Start communication threads
        self.running = True
        self.front_thread = threading.Thread(target=self._front_communication_loop)
        self.front_thread.daemon = True
        self.front_thread.start()
        
        # Start rear communication thread only if rear hoverboard is available
        if hasattr(self, 'rear_port') and self.rear_port is not None:
            self.rear_thread = threading.Thread(target=self._rear_communication_loop)
            self.rear_thread.daemon = True
            self.rear_thread.start()
            self.get_logger().info("Rear communication thread started")
        else:
            self.rear_thread = None
            self.get_logger().info("Rear hoverboard not available - running with front only")
        
        self.get_logger().info("Communication threads started")
        
        # Create timer for odometry publishing
        self.odom_timer = self.create_timer(0.1, self._publish_odometry)
        self.get_logger().info("Odometry timer created")
        
        self.get_logger().info("Omni hoverboard driver initialized successfully")

    
    def _find_ch341_device(self, device_id):
        """Find CH341 device by vendor:product ID"""
        try:
            # First try to find by vendor:product ID
            devices = glob.glob('/dev/serial/by-id/*')
            for device in devices:
                if device_id in device:
                    return device
            
            # If not found, try direct device node
            if os.path.exists('/dev/ttyCH341USB0'):
                return '/dev/ttyCH341USB0'
            if os.path.exists('/dev/ttyCH341USB1'):
                return '/dev/ttyCH341USB1'
                
            self.get_logger().error("No CH341 device found")
            return None
        except Exception as e:
            self.get_logger().error(f"Error finding CH341 device: {str(e)}")
            return None
    
    def _check_device_status(self, port):
        """Check if the CH341 device is properly connected and configured"""
        try:
            if not port.is_open:
                self.get_logger().error(f"Port {port.port} is not open")
                return False
                
            # Try to read device status
            port.write(b'\xE8\x00\x00\x00')  # Status request
            time.sleep(0.01)
            
            if port.in_waiting:
                status = port.read(port.in_waiting)
                self.get_logger().info(f"Device status: {status.hex()}")
                return True
            return False
        except Exception as e:
            self.get_logger().error(f"Error checking device status: {str(e)}")
            return False
    
    def _init_hoverboards(self):
        try:
            # Check if we're in simulation mode
            simulation_mode = self.get_parameter('simulation_mode').value
            
            if simulation_mode:
                self.get_logger().info("Running in SIMULATION MODE - no hardware required")
                self.front_port = None
                self.rear_port = None
                self.front_buffer = bytearray()
                self.rear_buffer = bytearray()
                
                # Kinematic parameters
                self.R = self.get_parameter('wheel_radius').value
                self.L = self.get_parameter('wheel_base').value / 2  # half of wheel base
                self.W = self.get_parameter('wheel_track').value / 2  # half of wheel track
                self.max_speed = self.get_parameter('max_speed').value
                
                # Control variables
                self.lock = threading.Lock()
                self.target_vel = np.zeros(3)  # [vx, vy, ωz]
                self.last_feedback = {
                    'front': {'speedL': 0, 'speedR': 0},
                    'rear': {'speedL': 0, 'speedR': 0}
                }
                
                self.get_logger().info("Simulation mode initialized successfully")
                return
            
            # Hardware mode - find devices
            front_port_path = self._find_ch341_device(self.get_parameter('front_serial').value)
            rear_port_path = self._find_ch341_device(self.get_parameter('rear_serial').value)
            
            # Initialize front hoverboard (required)
            if not front_port_path:
                raise serial.SerialException("Could not find front CH341 device with specified vendor:product ID")
            
            self.front_port = serial.Serial(
                front_port_path,
                self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.front_buffer = bytearray()
            self.get_logger().info(f"Front hoverboard initialized on {self.front_port.port}")
            
            # Check device status after initialization
            if not self._check_device_status(self.front_port):
                self.get_logger().error("Front CH341 device not responding")
            
            # Initialize rear hoverboard (optional)
            self.rear_port = None
            if rear_port_path:
                try:
                    self.rear_port = serial.Serial(
                        rear_port_path,
                        self.get_parameter('baudrate').value,
                        timeout=self.get_parameter('timeout').value,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        xonxoff=False,
                        rtscts=False,
                        dsrdtr=False
                    )
                    self.rear_buffer = bytearray()
                    self.get_logger().info(f"Rear hoverboard initialized on {self.rear_port.port}")
                    
                    # Check device status after initialization
                    if not self._check_device_status(self.rear_port):
                        self.get_logger().error("Rear CH341 device not responding")
                        self.rear_port.close()
                        self.rear_port = None
                        self.get_logger().info("Disabling rear hoverboard due to communication issues")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize rear hoverboard: {str(e)}")
                    self.rear_port = None
                    self.get_logger().info("Continuing with front hoverboard only")
            else:
                self.get_logger().info("Rear hoverboard not found - running with front only")
            
            # Kinematic parameters
            self.R = self.get_parameter('wheel_radius').value
            self.L = self.get_parameter('wheel_base').value / 2  # half of wheel base
            self.W = self.get_parameter('wheel_track').value / 2  # half of wheel track
            self.max_speed = self.get_parameter('max_speed').value
            
            # Control variables
            self.lock = threading.Lock()
            self.target_vel = np.zeros(3)  # [vx, vy, ωz]
            self.last_feedback = {
                'front': {'speedL': 0, 'speedR': 0},
                'rear': {'speedL': 0, 'speedR': 0}
            }
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize hoverboards: {str(e)}")
            raise

    def _find_device_by_serial(self, serial_number):
        """Find device port by serial number"""
        try:
            # List all serial devices
            devices = glob.glob('/dev/serial/by-id/*')
            for device in devices:
                if serial_number in device:
                    # Get the actual device path
                    return device
            return None
        except Exception as e:
            self.get_logger().error(f"Error finding device with serial {serial_number}: {str(e)}")
            return None

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to target velocities"""
        with self.lock:
            # Increase angular velocity (wz) by 2x for faster turning
            self.target_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z * 2.0])

    def _inverse_kinematics(self, vx, vy, wz):
        """
        Convert desired chassis motion to wheel speeds for mecanum drive
        Args:
            vx: forward velocity (m/s)
            vy: strafe velocity (m/s)
            wz: angular velocity (rad/s)
        Returns: (front_left, front_right, rear_left, rear_right) in hoverboard units (-1000 to 1000)
        """
        # Correct mecanum wheel kinematics
        # For mecanum wheels, all wheels should have same speed when moving straight
        fl = (vx - vy + (self.L + self.W)*wz) / self.R
        fr = (vx + vy - (self.L + self.W)*wz) / self.R
        rl = (vx + vy + (self.L + self.W)*wz) / self.R
        rr = (vx - vy - (self.L + self.W)*wz) / self.R
        
        # Debug: show calculations for straight movement
        if abs(vy) < 0.01 and abs(wz) < 0.01:  # Straight movement
            self.get_logger().info(f"STRAIGHT: vx={vx:.2f} -> fl={fl:.2f} fr={fr:.2f} rl={rl:.2f} rr={rr:.2f}")
        
        # Scale to hoverboard units (-1000 to 1000)
        scale = 1000.0 / self.max_speed
        fl = int(fl * scale)
        fr = int(fr * scale)
        rl = int(rl * scale)
        rr = int(rr * scale)
        
        # Clamp values to valid range
        fl = max(-1000, min(1000, fl))
        fr = max(-1000, min(1000, fr))
        rl = max(-1000, min(1000, rl))
        rr = max(-1000, min(1000, rr))
        
        return (fl, fr, rl, rr)

    def _send_hoverboard_command(self, port, left_speed, right_speed, hoverboard_name="Unknown"):
        """Send command to a hoverboard"""
        try:
            # Check if we're in simulation mode
            simulation_mode = self.get_parameter('simulation_mode').value
            
            if simulation_mode:
                # In simulation mode, just log the command
                self.get_logger().info(f"SIMULATION: {hoverboard_name} - Left: {left_speed}, Right: {right_speed}")
                return
            
            # Hardware mode - ensure speeds are within valid range for signed short (-32768 to 32767)
            left_speed = max(-32768, min(32767, left_speed))
            right_speed = max(-32768, min(32767, right_speed))
            
            # Show real-time speed commands with hoverboard identification
            self.get_logger().info(f"SPEED COMMAND [{hoverboard_name}]: Left: {left_speed}, Right: {right_speed}")
            
            start_frame = 0xABCD
            # Calculate checksum using only the lower 16 bits
            checksum = (start_frame & 0xFFFF) ^ (left_speed & 0xFFFF) ^ (right_speed & 0xFFFF)
            
            # Pack the command
            command = struct.pack('<HhhH', start_frame, left_speed, right_speed, checksum)
            
            # Clear any existing data in the buffer
            port.reset_input_buffer()
            port.reset_output_buffer()
            
            # Send the command
            bytes_written = port.write(command)
            
            # Make sure the command is sent
            port.flush()
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command to {hoverboard_name}: {str(e)}")
        except struct.error as e:
            self.get_logger().error(f"Failed to pack command for {hoverboard_name}: {str(e)}")

    def _read_hoverboard_feedback(self, port, buffer):
        """Read and process feedback from hoverboard"""
        try:
            while port.in_waiting:
                byte = port.read(1)
                buffer.extend(byte)
                
                # Look for start frame (0xABCD)
                if len(buffer) >= 2:
                    if buffer[-2] == 0xCD and buffer[-1] == 0xAB:
                        if len(buffer) >= 20:  # Full message length
                            msg = buffer[-20:]
                            feedback = self._process_hoverboard_feedback(msg)
                            if feedback:
                                return feedback
                            buffer.clear()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to read feedback: {str(e)}")
        return None

    def _process_hoverboard_feedback(self, msg):
        """Process 20-byte feedback message"""
        try:
            # Use signed shorts (h) for speed values
            feedback = struct.unpack('<HhhhhhhH2xH', msg)
            calculated_checksum = (feedback[0] ^ feedback[1] ^ feedback[2] ^ 
                                 feedback[3] ^ feedback[4] ^ feedback[5] ^ 
                                 feedback[6] ^ feedback[7])
            
            if calculated_checksum == feedback[8]:
                return {
                    'speedL': feedback[3],  # Left wheel speed
                    'speedR': feedback[4]   # Right wheel speed
                }
            # else:
                # self.get_logger().warning(f"Checksum mismatch: calculated={calculated_checksum}, received={feedback[8]}")
        except struct.error as e:
            self.get_logger().error(f"Failed to unpack feedback: {str(e)}")
            return None

    def _front_communication_loop(self):
        """Send commands to front hoverboard and process feedback"""
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel
                    fl, fr, rl, rr = self._inverse_kinematics(vx, vy, wz)
                
                # Show all info in one line
                self.get_logger().info(f"VEL: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} | WHEELS: FL={fl} FR={fr} RL={rl} RR={rr}")
                
                self._send_hoverboard_command(self.front_port, fl, fr, "FRONT")
                
                # Check if we're in simulation mode
                simulation_mode = self.get_parameter('simulation_mode').value
                
                if simulation_mode:
                    # In simulation mode, simulate feedback
                    with self.lock:
                        self.last_feedback['front'] = {'speedL': fl, 'speedR': fr}
                else:
                    # Hardware mode - read and process feedback
                    feedback = self._read_hoverboard_feedback(self.front_port, self.front_buffer)
                    if feedback:
                        with self.lock:
                            self.last_feedback['front'] = feedback
                
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in front communication loop: {str(e)}")
                time.sleep(1)  # Wait before retrying

    def _rear_communication_loop(self):
        """Send commands to rear hoverboard and process feedback"""
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel
                    fl, fr, rl, rr = self._inverse_kinematics(vx, vy, wz)
                
                # Show rear wheel info in one line
                self.get_logger().info(f"REAR: RL={-rl} RR={-rr}")
                
                # Invert direction for rear wheels to correct rotation direction
                self._send_hoverboard_command(self.rear_port, -rl, -rr, "REAR")
                
                # Check if we're in simulation mode
                simulation_mode = self.get_parameter('simulation_mode').value
                
                if simulation_mode:
                    # In simulation mode, simulate feedback
                    with self.lock:
                        self.last_feedback['rear'] = {'speedL': -rl, 'speedR': -rr}
                else:
                    # Hardware mode - read and process feedback
                    feedback = self._read_hoverboard_feedback(self.rear_port, self.rear_buffer)
                    if feedback:
                        with self.lock:
                            # Invert feedback values to match the inverted command
                            self.last_feedback['rear'] = {'speedL': -feedback['speedL'], 'speedR': -feedback['speedR']}
                
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in rear communication loop: {str(e)}")
                time.sleep(1)  # Wait before retrying

    def _forward_kinematics(self):
        """
        Convert wheel speeds to chassis velocities for mecanum drive
        Returns: (vx, vy, wz) - forward, strafe, and angular velocities (m/s, m/s, rad/s)
        """
        with self.lock:
            # Get wheel speeds in m/s (assuming input is in -1000 to 1000 range)
            fl = self.last_feedback['front']['speedL'] * self.max_speed / 1000
            fr = self.last_feedback['front']['speedR'] * self.max_speed / 1000
            
            # Use rear wheels if available, otherwise use front wheels for rear calculation
            if self.rear_port is not None:
                # Rear wheels are inverted, so we need to invert the feedback values
                rl = -self.last_feedback['rear']['speedL'] * self.max_speed / 1000
                rr = -self.last_feedback['rear']['speedR'] * self.max_speed / 1000
            else:
                # If no rear hoverboard, use front wheels for both front and rear
                rl = fl
                rr = fr
        
        # Mecanum kinematic equations
        vx = (fl + fr + rl + rr) * (self.R/4)
        vy = (-fl + fr - rl + rr) * (self.R/4)
        wz = (-fl + fr + rl - rr) * (self.R/(4*(self.L + self.W)))
        
        return (vx, vy, wz)

    def _publish_odometry(self):
        """Publish current odometry"""
        try:
            vx, vy, wz = self._forward_kinematics()
            
            msg = Twist()
            msg.linear.x = vx
            # Swap vy and wz - publish wz as linear.y and vy as angular.z
            msg.linear.y = wz
            msg.angular.z = vy
            
            self.odom_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing odometry: {str(e)}")

    def test_wheel_callback(self, request, response):
        """Test individual wheels"""
        self.get_logger().info("Starting wheel test")
        test_speed = self.get_parameter('test_speed').value
        self.get_logger().info(f"Using test speed: {test_speed}")
        
        try:
            # Pause the main control loops
            self.running = False
            self.front_thread.join(timeout=1.0)
            if self.rear_thread is not None:
                self.rear_thread.join(timeout=1.0)
            self.get_logger().info("Main control loops paused")
            
            # Test front right wheel only
            self.get_logger().info("Testing front right wheel only")
            self._send_hoverboard_command(self.front_port, 0, test_speed, "FRONT")  # left=0, right=test_speed
            time.sleep(5)
            self._send_hoverboard_command(self.front_port, 0, 0, "FRONT")
            
            # Test rear right wheel only if rear hoverboard is available
            if self.rear_port is not None:
                self.get_logger().info("Testing rear right wheel only")
                # Invert test speed for rear wheels due to inverted direction
                self._send_hoverboard_command(self.rear_port, 0, -test_speed, "REAR")  # left=0, right=-test_speed
                time.sleep(5)
                self._send_hoverboard_command(self.rear_port, 0, 0, "REAR")
            else:
                self.get_logger().info("Rear hoverboard not available - skipping rear wheel test")
            
            # Restart the main control loops
            self.running = True
            self.front_thread = threading.Thread(target=self._front_communication_loop)
            self.front_thread.daemon = True
            self.front_thread.start()
            
            if self.rear_port is not None:
                self.rear_thread = threading.Thread(target=self._rear_communication_loop)
                self.rear_thread.daemon = True
                self.rear_thread.start()
            self.get_logger().info("Main control loops restarted")
            
            self.get_logger().info("Wheel test completed successfully")
        except Exception as e:
            self.get_logger().error(f"Error during wheel test: {str(e)}")
            # Make sure to restart the control loops even if there's an error
            self.running = True
            self.front_thread = threading.Thread(target=self._front_communication_loop)
            self.front_thread.daemon = True
            self.front_thread.start()
            
            if self.rear_port is not None:
                self.rear_thread = threading.Thread(target=self._rear_communication_loop)
                self.rear_thread.daemon = True
                self.rear_thread.start()
        
        return response

    def destroy_node(self):
        self.running = False
        self.front_thread.join()
        if self.rear_thread is not None:
            self.rear_thread.join()
        
        # Only close ports if they exist (not in simulation mode)
        if self.front_port is not None:
            self.front_port.close()
        if self.rear_port is not None:
            self.rear_port.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardOmniDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()