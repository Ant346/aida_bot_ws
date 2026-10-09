#!/usr/bin/env python3
"""Пробел в любом окне останавливает все оси ODrive.

Сторож читает клавиатуры напрямую, поэтому фокус на терминале не нужен.
Пока процесс жив и видит хотя бы одну клавиатуру, он обновляет watchdog.
Пропал сторож или клавиатура — драйвер сам снимает момент.
"""

import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from odroid_driver.motion_guard import (
    DRIVER_HEARTBEAT,
    KEYBOARD,
    WATCHDOG,
    clear_estop,
    ensure_estop_file,
    estop_latched,
    estop_reason,
    file_fresh,
    latch_estop,
    touch,
)

_CAN_SET_AXIS_STATE = 0x007
_CAN_SET_INPUT_VEL = 0x00D
_AXIS_IDLE = 1


class SpaceEstop(Node):
    def __init__(self):
        super().__init__('odroid_estop')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_interface_rear', 'can1')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('axis_id_fl', 0)
        self.declare_parameter('axis_id_fr', 1)
        self.declare_parameter('axis_id_rl', 1)
        self.declare_parameter('axis_id_rr', 0)

        ensure_estop_file()
        self._lock = threading.Lock()
        self._latched = estop_latched()
        self._clear_gen = 0
        self._saw_driver = False
        self._warned_kb = False
        self._buses = []
        self._can = None
        self._devices = []
        self._stop_reader = False
        self._reader = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._reader.start()
        self.create_service(Empty, '~/clear', self._clear_cb)
        self.create_timer(0.05, self._tick)
        self.get_logger().warn(
            'АВАРИЙНЫЙ СТОП: пробел в любом окне снимает момент со всех моторов. '
            'Сброс только сервисом /odroid_estop/clear')

    def _axis_ids(self):
        ids = {0, 1}
        for name in ('axis_id_fl', 'axis_id_fr', 'axis_id_rl', 'axis_id_rr'):
            aid = int(self.get_parameter(name).value)
            if aid >= 0:
                ids.add(aid)
        return sorted(ids)

    def _interfaces(self):
        front = self.get_parameter('can_interface').get_parameter_value().string_value.strip() or 'can0'
        rear = self.get_parameter('can_interface_rear').get_parameter_value().string_value.strip()
        names = [front]
        if rear and rear not in names:
            names.append(rear)
        return names

    def _ensure_buses(self):
        if self._buses:
            return True
        try:
            import can as can_mod
        except ImportError:
            self.get_logger().error('python-can не установлен, прямой CAN-стоп недоступен')
            return False
        self._can = can_mod
        rate = int(self.get_parameter('can_bitrate').value)
        opened = []
        for name in self._interfaces():
            try:
                opened.append(can_mod.interface.Bus(
                    bustype='socketcan', channel=name, bitrate=rate))
            except Exception as exc:
                self.get_logger().error(f'CAN стоп: не открыл {name}: {exc}')
        self._buses = opened
        return bool(self._buses)

    def _blast_idle(self):
        if not self._ensure_buses():
            return
        can_mod = self._can
        for bus in self._buses:
            for axis_id in self._axis_ids():
                try:
                    vel = struct.pack('<ff', 0.0, 0.0)
                    bus.send(can_mod.Message(
                        arbitration_id=(axis_id << 5) | _CAN_SET_INPUT_VEL,
                        data=vel,
                        is_extended_id=False,
                    ))
                    state = int(_AXIS_IDLE).to_bytes(4, 'little')
                    bus.send(can_mod.Message(
                        arbitration_id=(axis_id << 5) | _CAN_SET_AXIS_STATE,
                        data=state,
                        is_extended_id=False,
                    ))
                except Exception as exc:
                    self.get_logger().error(f'CAN стоп axis={axis_id}: {exc}')
                    self._close_buses()
                    return

    def _close_buses(self):
        for bus in self._buses:
            try:
                bus.shutdown()
            except Exception:
                pass
        self._buses = []

    def _keyboard_loop(self):
        try:
            import evdev
            from evdev import ecodes
        except ImportError:
            self.get_logger().error('python3-evdev нет — пробел не слушаю, моторы не разрешу')
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
                    keys = dev.capabilities().get(ecodes.EV_KEY, [])
                    if ecodes.KEY_SPACE not in keys:
                        dev.close()
                        continue
                    os.set_blocking(dev.fd, False)
                    opened[path] = dev
                    self.get_logger().info(f'Клавиатура стопа: {dev.name} ({path})')
                except Exception as exc:
                    self.get_logger().warning(f'Не открыл {path}: {exc}')

            dead = [path for path in opened if path not in seen]
            for path in dead:
                try:
                    opened[path].close()
                except Exception:
                    pass
                opened.pop(path, None)

            with self._lock:
                self._devices = list(opened.values())

            if not opened:
                time.sleep(0.5)
                continue

            # Короткий опрос, чтобы подхватывать новые клавиатуры.
            for dev in list(opened.values()):
                try:
                    for event in iter(lambda: dev.read_one(), None):
                        if event is None:
                            break
                        if (
                            event.type == ecodes.EV_KEY
                            and event.code == ecodes.KEY_SPACE
                            and event.value == 1
                        ):
                            self._on_space(dev.name)
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

    def _on_space(self, source):
        with self._lock:
            already = self._latched
            self._latched = True
        latch_estop(f'space from {source}')
        self._blast_idle()
        if already:
            self.get_logger().warn(f'ПРОБЕЛ ({source}): стоп уже удерживается')
        else:
            self.get_logger().error(f'ПРОБЕЛ ({source}): ВСЕ МОТОРЫ В IDLE')

    def _tick(self):
        with self._lock:
            keyboards = len(self._devices)
            latched = self._latched or estop_latched()
            self._latched = latched
            gen = self._clear_gen
        if keyboards > 0:
            touch(KEYBOARD)
            self._warned_kb = False
        elif not self._warned_kb:
            self.get_logger().error('Клавиатура не открыта — драйвер не даст момент')
            self._warned_kb = True
        touch(WATCHDOG)
        driver_alive = file_fresh(DRIVER_HEARTBEAT, 0.5, time.time())
        if driver_alive:
            self._saw_driver = True
        if not latched and driver_alive:
            return
        self._blast_idle()
        with self._lock:
            still = self._latched and self._clear_gen == gen
        if still:
            latch_estop(estop_reason() or 'space')

    def _clear_cb(self, _request, response):
        with self._lock:
            self._latched = False
            self._clear_gen += 1
        clear_estop()
        self.get_logger().warn('Стоп снят. Моторы снова могут взять момент.')
        return response

    def destroy_node(self):
        self._stop_reader = True
        latch_estop('estop node exit')
        self._blast_idle()
        self._close_buses()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpaceEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
