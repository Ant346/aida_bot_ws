#!/usr/bin/env python3
"""Single Odroid host node: cmd_vel → mecanum wheel controllers (hoverboard-style serial protocol)."""
import glob
import os
import struct
import threading
import time

import numpy as np
import rclpy
import serial
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_srvs.srv import Empty


class OdroidDriver(Node):
    """Subscribes to cmd_vel, talks to front/rear motor boards over USB–serial (e.g. CH341)."""

    def __init__(self):
        super().__init__('odroid_driver')

        self.get_logger().info('Initializing odroid driver (wheel controllers + kinematics)...')

        self.declare_parameters(
            '',
            [
                ('front_serial', '0001'),
                ('rear_serial', '0002'),
                ('baudrate', 115200),
                ('timeout', 0.05),
                ('wheel_radius', 0.095),
                ('wheel_base', 0.635),
                ('wheel_track', 0.72),
                ('max_speed', 50),
                ('test_speed', 0),
                ('simulation_mode', False),
                ('cmd_vel_subscribe_stamped', False),
                ('angular_z_scale', 2.0),
                ('wheel_odometry_topic', 'wheel_odometry'),
            ],
        )

        self._init_boards()

        stamped = self.get_parameter('cmd_vel_subscribe_stamped').value
        if stamped:
            self.create_subscription(
                TwistStamped, 'cmd_vel', self._cmd_vel_stamped_callback, 10)
            self.get_logger().info('cmd_vel subscription: TwistStamped')
        else:
            self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            self.get_logger().info('cmd_vel subscription: Twist')

        odom_topic = self.get_parameter('wheel_odometry_topic').value
        self.odom_pub = self.create_publisher(Twist, odom_topic, 10)
        self.get_logger().info(f'Publishing wheel odometry (Twist) on "{odom_topic}"')

        self.create_service(Empty, 'test_wheel', self.test_wheel_callback)

        self.running = True
        self.front_thread = threading.Thread(target=self._front_communication_loop, daemon=True)
        self.front_thread.start()

        if getattr(self, 'rear_port', None) is not None:
            self.rear_thread = threading.Thread(target=self._rear_communication_loop, daemon=True)
            self.rear_thread.start()
            self.get_logger().info('Rear board communication thread started')
        else:
            self.rear_thread = None
            self.get_logger().info('Rear board not available — front only')

        self.create_timer(0.1, self._publish_odometry)
        self.get_logger().info('odroid_driver ready')

    def _find_ch341_device(self, device_id):
        try:
            devices = glob.glob('/dev/serial/by-id/*')
            for device in devices:
                if device_id in device:
                    return device
            if os.path.exists('/dev/ttyCH341USB0'):
                return '/dev/ttyCH341USB0'
            if os.path.exists('/dev/ttyCH341USB1'):
                return '/dev/ttyCH341USB1'
            self.get_logger().error('No CH341 / by-id device matched')
            return None
        except Exception as e:
            self.get_logger().error(f'Error finding serial device: {e!s}')
            return None

    def _check_device_status(self, port):
        try:
            if not port.is_open:
                self.get_logger().error(f'Port {port.port} is not open')
                return False
            port.write(b'\xE8\x00\x00\x00')
            time.sleep(0.01)
            if port.in_waiting:
                status = port.read(port.in_waiting)
                self.get_logger().info(f'Device status: {status.hex()}')
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Error checking device status: {e!s}')
            return False

    def _init_boards(self):
        simulation_mode = self.get_parameter('simulation_mode').value
        if simulation_mode:
            self.get_logger().info('SIMULATION_MODE — no serial hardware')
            self.front_port = None
            self.rear_port = None
            self.front_buffer = bytearray()
            self.rear_buffer = bytearray()
            self._init_kinematics_state()
            return

        front_path = self._find_ch341_device(self.get_parameter('front_serial').value)
        if not front_path:
            raise serial.SerialException(
                'Front board: no serial device (set front_serial or simulation_mode)')

        self.front_port = serial.Serial(
            front_path,
            self.get_parameter('baudrate').value,
            timeout=self.get_parameter('timeout').value,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.front_buffer = bytearray()
        self.get_logger().info(f'Front board on {self.front_port.port}')
        if not self._check_device_status(self.front_port):
            self.get_logger().error('Front device not responding to status poll')

        rear_path = self._find_ch341_device(self.get_parameter('rear_serial').value)
        self.rear_port = None
        if rear_path:
            try:
                self.rear_port = serial.Serial(
                    rear_path,
                    self.get_parameter('baudrate').value,
                    timeout=self.get_parameter('timeout').value,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
                self.rear_buffer = bytearray()
                self.get_logger().info(f'Rear board on {self.rear_port.port}')
                if not self._check_device_status(self.rear_port):
                    self.get_logger().error('Rear device not responding — disabling rear')
                    self.rear_port.close()
                    self.rear_port = None
            except Exception as e:
                self.get_logger().error(f'Rear init failed: {e!s}')
                self.rear_port = None
        else:
            self.get_logger().info('Rear board not found — front only')

        self._init_kinematics_state()

    def _init_kinematics_state(self):
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value / 2.0
        self.W = self.get_parameter('wheel_track').value / 2.0
        self.max_speed = self.get_parameter('max_speed').value
        self.lock = threading.Lock()
        self.target_vel = np.zeros(3)
        self.last_feedback = {
            'front': {'speedL': 0, 'speedR': 0},
            'rear': {'speedL': 0, 'speedR': 0},
        }

    def _cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.cmd_vel_callback(msg.twist)

    def cmd_vel_callback(self, msg: Twist):
        az_scale = float(self.get_parameter('angular_z_scale').value)
        with self.lock:
            self.target_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z * az_scale])

    def _inverse_kinematics(self, vx, vy, wz):
        fl = (vx - vy + (self.L + self.W) * wz) / self.R
        fr = (vx + vy - (self.L + self.W) * wz) / self.R
        rl = (vx + vy + (self.L + self.W) * wz) / self.R
        rr = (vx - vy - (self.L + self.W) * wz) / self.R

        if abs(vy) < 0.01 and abs(wz) < 0.01:
            self.get_logger().debug(
                f'STRAIGHT vx={vx:.2f} fl={fl:.2f} fr={fr:.2f} rl={rl:.2f} rr={rr:.2f}')

        scale = 1000.0 / self.max_speed
        fl = int(fl * scale)
        fr = int(fr * scale)
        rl = int(rl * scale)
        rr = int(rr * scale)

        lim = lambda v: max(-1000, min(1000, v))
        return lim(fl), lim(fr), lim(rl), lim(rr)

    def _send_board_command(self, port, left_speed, right_speed, label='board'):
        try:
            if self.get_parameter('simulation_mode').value:
                self.get_logger().debug(f'SIM {label}: L={left_speed} R={right_speed}')
                return

            left_speed = max(-32768, min(32767, left_speed))
            right_speed = max(-32768, min(32767, right_speed))
            self.get_logger().debug(f'{label}: command L={left_speed} R={right_speed}')

            start_frame = 0xABCD
            checksum = (start_frame & 0xFFFF) ^ (left_speed & 0xFFFF) ^ (right_speed & 0xFFFF)
            command = struct.pack('<HhhH', start_frame, left_speed, right_speed, checksum)
            port.reset_input_buffer()
            port.reset_output_buffer()
            port.write(command)
            port.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'{label} write failed: {e!s}')
        except struct.error as e:
            self.get_logger().error(f'{label} pack failed: {e!s}')

    def _read_board_feedback(self, port, buffer):
        try:
            while port.in_waiting:
                byte = port.read(1)
                buffer.extend(byte)
                if len(buffer) >= 2:
                    if buffer[-2] == 0xCD and buffer[-1] == 0xAB:
                        if len(buffer) >= 20:
                            msg = buffer[-20:]
                            feedback = self._parse_board_feedback(msg)
                            if feedback:
                                buffer.clear()
                                return feedback
        except serial.SerialException as e:
            self.get_logger().error(f'Feedback read failed: {e!s}')
        return None

    def _parse_board_feedback(self, msg):
        try:
            feedback = struct.unpack('<HhhhhhhH2xH', msg)
            calculated = (
                feedback[0] ^ feedback[1] ^ feedback[2] ^ feedback[3]
                ^ feedback[4] ^ feedback[5] ^ feedback[6] ^ feedback[7]
            )
            if calculated == feedback[8]:
                return {'speedL': feedback[3], 'speedR': feedback[4]}
        except struct.error as e:
            self.get_logger().error(f'Unpack feedback: {e!s}')
        return None

    def _front_communication_loop(self):
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel
                    fl, fr, rl, rr = self._inverse_kinematics(vx, vy, wz)

                self.get_logger().debug(
                    f'cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} | FL={fl} FR={fr} RL={rl} RR={rr}')

                self._send_board_command(self.front_port, fl, fr, 'FRONT')

                if self.get_parameter('simulation_mode').value:
                    with self.lock:
                        self.last_feedback['front'] = {'speedL': fl, 'speedR': fr}
                elif self.front_port is not None:
                    feedback = self._read_board_feedback(self.front_port, self.front_buffer)
                    if feedback:
                        with self.lock:
                            self.last_feedback['front'] = feedback

                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Front loop error: {e!s}')
                time.sleep(1.0)

    def _rear_communication_loop(self):
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel
                    fl, fr, rl, rr = self._inverse_kinematics(vx, vy, wz)

                self.get_logger().debug(f'REAR cmd RL={-rl} RR={-rr}')
                self._send_board_command(self.rear_port, -rl, -rr, 'REAR')

                if self.get_parameter('simulation_mode').value:
                    with self.lock:
                        self.last_feedback['rear'] = {'speedL': -rl, 'speedR': -rr}
                elif self.rear_port is not None:
                    feedback = self._read_board_feedback(self.rear_port, self.rear_buffer)
                    if feedback:
                        with self.lock:
                            self.last_feedback['rear'] = {
                                'speedL': -feedback['speedL'],
                                'speedR': -feedback['speedR'],
                            }

                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Rear loop error: {e!s}')
                time.sleep(1.0)

    def _forward_kinematics(self):
        with self.lock:
            fl = self.last_feedback['front']['speedL'] * self.max_speed / 1000.0
            fr = self.last_feedback['front']['speedR'] * self.max_speed / 1000.0
            if self.rear_port is not None:
                rl = -self.last_feedback['rear']['speedL'] * self.max_speed / 1000.0
                rr = -self.last_feedback['rear']['speedR'] * self.max_speed / 1000.0
            else:
                rl, rr = fl, fr

        vx = (fl + fr + rl + rr) * (self.R / 4.0)
        vy = (-fl + fr - rl + rr) * (self.R / 4.0)
        wz = (-fl + fr + rl - rr) * (self.R / (4.0 * (self.L + self.W)))
        return vx, vy, wz

    def _publish_odometry(self):
        try:
            vx, vy, wz = self._forward_kinematics()
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = wz
            msg.angular.z = vy
            self.odom_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'odometry publish: {e!s}')

    def test_wheel_callback(self, request, response):
        self.get_logger().info('test_wheel service: run short spin test')
        test_speed = int(self.get_parameter('test_speed').value)
        if test_speed == 0:
            self.get_logger().warning('test_speed is 0; set param test_speed non-zero')

        try:
            self.running = False
            self.front_thread.join(timeout=1.0)
            if self.rear_thread is not None:
                self.rear_thread.join(timeout=1.0)

            if not self.get_parameter('simulation_mode').value and self.front_port is not None:
                self._send_board_command(self.front_port, 0, test_speed, 'FRONT')
                time.sleep(5)
                self._send_board_command(self.front_port, 0, 0, 'FRONT')

                if self.rear_port is not None:
                    self._send_board_command(self.rear_port, 0, -test_speed, 'REAR')
                    time.sleep(5)
                    self._send_board_command(self.rear_port, 0, 0, 'REAR')
            else:
                self.get_logger().info('test_wheel: simulation or no front port — skip hardware pulse')

            self.running = True
            self.front_thread = threading.Thread(target=self._front_communication_loop, daemon=True)
            self.front_thread.start()
            if self.rear_port is not None:
                self.rear_thread = threading.Thread(target=self._rear_communication_loop, daemon=True)
                self.rear_thread.start()
        except Exception as e:
            self.get_logger().error(f'test_wheel error: {e!s}')
            self.running = True
            self.front_thread = threading.Thread(target=self._front_communication_loop, daemon=True)
            self.front_thread.start()
            if self.rear_port is not None:
                self.rear_thread = threading.Thread(target=self._rear_communication_loop, daemon=True)
                self.rear_thread.start()

        return response

    def destroy_node(self):
        self.running = False
        if self.front_thread.is_alive():
            self.front_thread.join(timeout=2.0)
        if self.rear_thread is not None and self.rear_thread.is_alive():
            self.rear_thread.join(timeout=2.0)
        if self.front_port is not None:
            self.front_port.close()
        if self.rear_port is not None:
            self.rear_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdroidDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
