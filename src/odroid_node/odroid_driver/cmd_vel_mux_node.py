#!/usr/bin/env python3
"""Multiplex Twist teleop vs autonomy using DS4 Status (toggle + watchdog)."""

import rclpy
from ds4_driver_msgs.msg import Status
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('status_topic', '/status')
        self.declare_parameter('teleop_cmd_vel_topic', '/cmd_vel_teleop')
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_nav')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel')
        self.declare_parameter('watchdog_timeout_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('initial_navigation_mode', False)
        self.declare_parameter('allow_nav_when_joy_lost', False)
        self.declare_parameter('toggle_button_field', 'button_cross')
        self.declare_parameter('mode_topic', '/cmd_vel_mux/mode')

        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        teleop_topic = self.get_parameter('teleop_cmd_vel_topic').get_parameter_value().string_value
        nav_topic = self.get_parameter('nav_cmd_vel_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('cmd_vel_out_topic').get_parameter_value().string_value
        mode_topic = self.get_parameter('mode_topic').get_parameter_value().string_value

        self._watchdog_timeout = self.get_parameter(
            'watchdog_timeout_sec'
        ).get_parameter_value().double_value
        self._nav_mode = self.get_parameter(
            'initial_navigation_mode'
        ).get_parameter_value().bool_value
        self._allow_nav_joy_lost = self.get_parameter(
            'allow_nav_when_joy_lost'
        ).get_parameter_value().bool_value
        self._toggle_field = self.get_parameter(
            'toggle_button_field'
        ).get_parameter_value().string_value

        self._last_status_time = None
        self._prev_toggle_pressed = False

        self._teleop = Twist()
        self._nav = Twist()
        self._have_teleop = False
        self._have_nav = False

        _mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub_mode = self.create_publisher(String, mode_topic, _mode_qos)
        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Status, status_topic, self._cb_status, 10)
        self.create_subscription(Twist, teleop_topic, self._cb_teleop, 10)
        self.create_subscription(Twist, nav_topic, self._cb_nav, 10)

        rate = max(self.get_parameter('publish_rate_hz').get_parameter_value().double_value, 1.0)
        self.create_timer(1.0 / rate, self._tick)

        self._publish_mode()
        self.get_logger().info(
            f'cmd_vel_mux: teleop={teleop_topic} nav={nav_topic} out={out_topic} '
            f'status={status_topic} mode_topic={mode_topic} '
            f'mode={"auto" if self._nav_mode else "teleop"} '
            f'toggle={self._toggle_field}'
        )

    def _mode_string(self) -> str:
        return 'auto' if self._nav_mode else 'teleop'

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode_string()
        self._pub_mode.publish(msg)

    def _cb_status(self, msg: Status):
        self._last_status_time = self.get_clock().now()

        try:
            raw = getattr(msg, self._toggle_field)
        except AttributeError:
            self.get_logger().error(f'Unknown toggle_button_field: {self._toggle_field}')
            self._prev_toggle_pressed = False
            return

        pressed = bool(raw)
        if pressed and not self._prev_toggle_pressed:
            self._nav_mode = not self._nav_mode
            self._publish_mode()
            self.get_logger().info(
                'Mode: %s' % ('auto (cmd_nav -> cmd_vel)' if self._nav_mode else 'teleop')
            )
        self._prev_toggle_pressed = pressed

    def _joy_ok(self) -> bool:
        if self._last_status_time is None:
            return False
        dt = (self.get_clock().now() - self._last_status_time).nanoseconds / 1e9
        return dt < self._watchdog_timeout

    def _cb_teleop(self, msg: Twist):
        self._teleop = msg
        self._have_teleop = True

    def _cb_nav(self, msg: Twist):
        self._nav = msg
        self._have_nav = True

    def _tick(self):
        joy_ok = self._joy_ok()

        if not joy_ok:
            if self._allow_nav_joy_lost and self._nav_mode and self._have_nav:
                self._pub.publish(self._nav)
            else:
                self._pub.publish(Twist())
            return

        if self._nav_mode:
            self._pub.publish(self._nav if self._have_nav else Twist())
        else:
            self._pub.publish(self._teleop if self._have_teleop else Twist())


def main():
    rclpy.init()
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
