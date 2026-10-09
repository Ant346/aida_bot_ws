#!/usr/bin/env python3
"""Телеоп из SSH-терминала по удержанию клавиши.

Клавиатура Mac не видна в /dev/input робота. Просим терминал присылать
нажатие и отпускание (kitty keyboard protocol). Если он так не умеет,
клавиша считается зажатой, только пока идут повторы, и отпускается,
как только повторы прекратились.
"""

import os
import select
import signal
import sys
import tempfile
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

ESTOP_FLAG = '/tmp/odroid_estop'
ESTOP_REASON = '/tmp/odroid_estop_reason'

# Пока терминал не начал повторять клавишу, держим ход. Дальше хватает
# короткого окна: повтор приходит чаще, отпускание гаснет сразу.
_INITIAL_HOLD_SEC = 0.55
_REPEAT_HOLD_SEC = 0.16
# Терминал часто шлёт «отпускание» на каждый автоповтор. Не сбрасываем
# ход сразу: короткое окно перекрывает повтор, настоящее отпускание гаснет.
_RELEASE_GRACE_SEC = 0.22
_ARM_GRACE_SEC = 0.55

_MOTION = {
    'w': ('x', 1.0),
    's': ('x', -1.0),
    'a': ('y', 1.0),
    'd': ('y', -1.0),
    'q': ('z', 1.0),
    'e': ('z', -1.0),
}
_ARROWS = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}

_USAGE = (
    'Нужен терминал SSH. Запуск из сессии на роботе:\n'
    '  docker exec -it aida_bot_ws-odroid_node-1 bash -lc '
    "'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && "
    "python3 /workspace/src/odroid_node/odroid_driver/keyboard_teleop_ssh.py'"
)


def _write_atomic(path, text):
    directory = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(prefix='.odroid-', dir=directory)
    try:
        os.write(fd, text.encode())
    finally:
        os.close(fd)
    os.replace(tmp, path)


def latch_estop(reason):
    _write_atomic(ESTOP_REASON, reason.strip() + '\n')
    _write_atomic(ESTOP_FLAG, '1\n')


def _yaml_number(key, default):
    path = '/workspace/src/odroid_node/config/odroid_driver.yaml'
    if not os.path.isfile(path):
        path = os.path.join(os.path.dirname(__file__), '..', 'config', 'odroid_driver.yaml')
    try:
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                stripped = line.split('#', 1)[0].strip()
                if stripped.startswith(key + ':'):
                    return float(stripped.split(':', 1)[1].strip())
    except (OSError, ValueError):
        pass
    return default


def _parse_csi(payload):
    """Разбор CSI. Возвращает ('query', flags) или ('key', name, event) или None.

    event: 1 нажатие, 2 повтор, 3 отпускание.
    """
    if not payload:
        return None
    final = payload[-1]
    body = payload[:-1]
    if final == 'u' and body.startswith('?'):
        try:
            return ('query', int(body[1:] or '0'))
        except ValueError:
            return None
    parts = body.split(';') if body else ['']
    code = parts[0].split(':')[0]
    mod = parts[1] if len(parts) > 1 else '1'
    event = 1
    mod_bits = mod.split(':')
    if len(mod_bits) > 1 and mod_bits[1].isdigit():
        event = int(mod_bits[1])
    try:
        modifiers = int(mod_bits[0] or '1')
    except ValueError:
        modifiers = 1
    ctrl = (modifiers - 1) & 4
    if final == 'u' and code.isdigit():
        number = int(code)
        if number == 99 and ctrl:
            return ('key', 'ctrl-c', event)
        if number == 32:
            return ('key', ' ', event)
        if 0 < number < 128:
            name = chr(number).lower()
            if name in _MOTION or name == 'k':
                return ('key', name, event)
        return None
    if final in _ARROWS and code in ('', '1'):
        return ('key', _ARROWS[final], event)
    return None


class KeyboardTeleopSsh(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_ssh')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_teleop')
        self.declare_parameter('linear_mps', _yaml_number('keyboard_linear_mps', 0.5))
        self.declare_parameter('strafe_mps', _yaml_number('keyboard_strafe_mps', 0.5))
        self.declare_parameter('angular_rps', _yaml_number('keyboard_angular_rps', 0.08))
        self.declare_parameter('publish_rate_hz', 20.0)

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._lin = float(self.get_parameter('linear_mps').value)
        self._strafe = float(self.get_parameter('strafe_mps').value)
        self._yaw = float(self.get_parameter('angular_rps').value)
        self._lock = threading.Lock()
        self._held = {}
        self._seq = 0
        self._release_events = False
        self._stop = False
        self._shown = None
        self._pub = self.create_publisher(Twist, topic, 10)
        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

    def note_protocol(self, flags):
        self._release_events = (flags & 0b1010) == 0b1010
        if self._release_events:
            sys.stderr.write('Терминал присылает отпускание: ход только пока клавиша зажата.\r\n')
        else:
            sys.stderr.write(
                'Терминал не присылает отпускание: ход, пока клавиша повторяется.\r\n'
                'На Mac, если удержание показывает акценты, в своём терминале:\r\n'
                '  defaults write -g ApplePressAndHoldEnabled -bool false\r\n'
            )
        sys.stderr.flush()

    def apply_key(self, key, event):
        if key == 'ctrl-c':
            self._stop = True
            return
        if key == ' ':
            if event == 3:
                return
            with self._lock:
                self._held.clear()
            latch_estop('space from ssh teleop')
            sys.stderr.write(
                '\r\nПРОБЕЛ: аварийный стоп. Сброс:\r\n'
                '  docker exec aida_bot_ws-odroid_node-1 bash -lc '
                "'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && "
                "ros2 service call /odroid_estop/clear std_srvs/srv/Empty'\r\n"
            )
            sys.stderr.flush()
            return
        if key == 'k':
            if event == 3:
                return
            with self._lock:
                self._held.clear()
            return
        if key not in _MOTION:
            return
        axis, sign = _MOTION[key]
        now = time.monotonic()
        with self._lock:
            if event == 3:
                item = self._held.get(key)
                if item is None:
                    return
                grace = _RELEASE_GRACE_SEC if item[4] else _ARM_GRACE_SEC
                self._held[key] = (item[0], item[1], now + grace, item[3], item[4])
                return
            self._seq += 1
            prev = self._held.get(key)
            repeating = event == 2 or prev is not None
            if self._release_events:
                deadline = None
            elif prev is not None:
                deadline = now + _REPEAT_HOLD_SEC
            else:
                deadline = now + _INITIAL_HOLD_SEC
            self._held[key] = (axis, sign, deadline, self._seq, repeating)

    def _command(self):
        now = time.monotonic()
        best = {}
        with self._lock:
            expired = [
                key for key, item in self._held.items()
                if item[2] is not None and now >= item[2]
            ]
            for key in expired:
                self._held.pop(key, None)
            for axis, sign, _deadline, seq, _repeating in self._held.values():
                prev = best.get(axis)
                if prev is None or seq >= prev[1]:
                    best[axis] = (sign, seq)
        msg = Twist()
        if 'x' in best:
            msg.linear.x = best['x'][0] * self._lin
        if 'y' in best:
            msg.linear.y = best['y'][0] * self._strafe
        if 'z' in best:
            msg.angular.z = best['z'][0] * self._yaw
        return msg

    def _tick(self):
        msg = self._command()
        shown = (msg.linear.x, msg.linear.y, msg.angular.z)
        if shown != self._shown:
            self._shown = shown
            sys.stderr.write(
                f'\r\nvx={msg.linear.x:+.2f}  vy={msg.linear.y:+.2f}  wz={msg.angular.z:+.2f}\r\n'
            )
            sys.stderr.flush()
        moving = shown != (0.0, 0.0, 0.0)
        if moving or self._shown == shown:
            self._pub.publish(msg)

    def stop_motion(self):
        with self._lock:
            self._held.clear()
        self._pub.publish(Twist())


def _read_event(fd):
    raw = os.read(fd, 1)
    if not raw:
        return ('eof',)
    if raw != b'\x1b':
        return ('plain', raw.decode('latin1', errors='ignore'))
    if not select.select([fd], [], [], 0.02)[0]:
        return ('plain', '\x1b')
    nxt = os.read(fd, 1)
    if nxt == b'[':
        buf = bytearray()
        while select.select([fd], [], [], 0.05)[0]:
            char = os.read(fd, 1)
            if not char:
                break
            buf += char
            if 0x40 <= char[0] <= 0x7E:
                break
        return ('csi', buf.decode('latin1', errors='ignore'))
    if nxt == b'O' and select.select([fd], [], [], 0.02)[0]:
        char = os.read(fd, 1).decode('latin1', errors='ignore')
        return ('key', _ARROWS.get(char, ''), 1)
    return ('plain', nxt.decode('latin1', errors='ignore'))


def _keyboard_loop(node, fd):
    announced = False
    started = time.monotonic()
    while not node._stop and rclpy.ok():
        if not announced and time.monotonic() - started > 0.4 and not node._release_events:
            node.note_protocol(0)
            announced = True
        if not select.select([fd], [], [], 0.05)[0]:
            continue
        event = _read_event(fd)
        kind = event[0]
        if kind == 'eof':
            node._stop = True
            break
        if kind == 'plain':
            key = event[1]
            if key in ('\x03', '\x04'):
                node._stop = True
                break
            if key:
                node.apply_key(key.lower(), 1)
            continue
        if kind == 'csi':
            parsed = _parse_csi(event[1])
            if parsed is None:
                continue
            if parsed[0] == 'query':
                node.note_protocol(parsed[1])
                announced = True
                continue
            _kind, key, ev = parsed
            if key:
                node.apply_key(key, ev)
            continue
        if kind == 'key' and event[1]:
            node.apply_key(event[1], event[2])


def main(args=None):
    if not sys.stdin.isatty():
        print(_USAGE, file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = KeyboardTeleopSsh()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(
        'Окно SSH в фокусе. Ход только пока клавиша удерживается.\n'
        'W/S стрелки — вперёд/назад, A/D — вбок, Q/E — поворот.\n'
        'K — отпустить всё. Пробел — аварийный стоп. Ctrl-C — выход.',
        flush=True,
    )
    reader = threading.Thread(target=_keyboard_loop, args=(node, fd), daemon=True)

    def _on_term(_signum, _frame):
        node._stop = True

    signal.signal(signal.SIGTERM, _on_term)
    try:
        tty.setraw(fd)
        os.write(fd, b'\x1b[>11u\x1b[?u')
        reader.start()
        while rclpy.ok() and not node._stop:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        node.stop_motion()
        try:
            os.write(fd, b'\x1b[<10u')
        except OSError:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print('\nВыход. Команда обнулена.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
