#!/usr/bin/env python3
"""Клавиатурный телеоп в /cmd_vel_teleop.

Читает клавиатуры через evdev, как аварийный пробел, поэтому фокус терминала
не нужен и устройство не перехватывается. Удержание клавиши держит скорость,
отпускание обнуляет ось. Пробел сюда не входит: его забирает сторож стопа.
"""

import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from evdev import ecodes
except ImportError:
    ecodes = None


# value: (ось, знак). +x вперёд, +y влево, +wz против часовой.
_KEYS = {}
if ecodes is not None:
    _KEYS = {
        ecodes.KEY_W: ('x', 1.0),
        ecodes.KEY_UP: ('x', 1.0),
        ecodes.KEY_S: ('x', -1.0),
        ecodes.KEY_DOWN: ('x', -1.0),
        ecodes.KEY_A: ('y', 1.0),
        ecodes.KEY_LEFT: ('y', 1.0),
        ecodes.KEY_D: ('y', -1.0),
        ecodes.KEY_RIGHT: ('y', -1.0),
        ecodes.KEY_Q: ('z', 1.0),
        ecodes.KEY_E: ('z', -1.0),
    }


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_teleop')
        self.declare_parameter('linear_mps', 0.5)
        self.declare_parameter('strafe_mps', 0.5)
        self.declare_parameter('angular_rps', 0.24)
        self.declare_parameter('publish_rate_hz', 20.0)

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._lin = float(self.get_parameter('linear_mps').value)
        self._strafe = float(self.get_parameter('strafe_mps').value)
        self._yaw = float(self.get_parameter('angular_rps').value)
        self._held = {}
        self._lock = threading.Lock()
        self._stop_reader = False
        self._was_moving = False
        self._pub = self.create_publisher(Twist, topic, 10)
        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self._reader = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f'Клавиатура → {topic}: W/S или стрелки вперёд/назад {self._lin:g} м/с, '
            f'A/D влево/вправо {self._strafe:g} м/с, Q/E поворот {self._yaw:g} рад/с. '
            'Пробел — аварийный стоп.'
        )

    def _keyboard_loop(self):
        try:
            import evdev
            from evdev import ecodes as keycodes
        except ImportError:
            self.get_logger().error('python3-evdev нет — клавиатурный телеоп не работает')
            return

        opened = {}
        while not self._stop_reader and rclpy.ok():
            seen = set()
            for path in evdev.list_devices():
                seen.add(path)
                if path in opened:
                    continue
                try:
                    dev = evdev.InputDevice(path)
                    keys = dev.capabilities().get(keycodes.EV_KEY, [])
                    if keycodes.KEY_W not in keys and keycodes.KEY_UP not in keys:
                        dev.close()
                        continue
                    os.set_blocking(dev.fd, False)
                    opened[path] = dev
                    self.get_logger().info(f'Клавиатура телеопа: {dev.name} ({path})')
                except Exception as exc:
                    self.get_logger().warning(f'Не открыл {path}: {exc}')

            dead = [path for path in opened if path not in seen]
            for path in dead:
                try:
                    opened[path].close()
                except Exception:
                    pass
                opened.pop(path, None)

            if not opened:
                time.sleep(0.5)
                continue

            for dev in list(opened.values()):
                try:
                    for event in iter(lambda: dev.read_one(), None):
                        if event is None:
                            break
                        if event.type != keycodes.EV_KEY or event.code not in _KEYS:
                            continue
                        if event.value == 2:
                            continue
                        axis, sign = _KEYS[event.code]
                        with self._lock:
                            if event.value == 1:
                                self._held[(dev.path, event.code)] = (axis, sign)
                            else:
                                self._held.pop((dev.path, event.code), None)
                except (BlockingIOError, OSError):
                    pass
                except Exception as exc:
                    self.get_logger().warning(f'Чтение {dev.path}: {exc}')
            time.sleep(0.01)

        for dev in opened.values():
            try:
                dev.close()
            except Exception:
                pass

    def _axis(self, held, name, scale):
        signs = [sign for axis, sign in held if axis == name]
        if not signs or any(sign > 0 for sign in signs) and any(sign < 0 for sign in signs):
            return 0.0
        return scale if signs[0] > 0 else -scale

    def _command(self):
        with self._lock:
            held = list(self._held.values())
        msg = Twist()
        msg.linear.x = self._axis(held, 'x', self._lin)
        msg.linear.y = self._axis(held, 'y', self._strafe)
        msg.angular.z = self._axis(held, 'z', self._yaw)
        return msg

    def _tick(self):
        msg = self._command()
        moving = msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.angular.z != 0.0
        # В покое молчим: иначе нули затирают команду из SSH-терминала.
        if moving or self._was_moving:
            self._pub.publish(msg)
        self._was_moving = moving

    def destroy_node(self):
        self._stop_reader = True
        self._pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
