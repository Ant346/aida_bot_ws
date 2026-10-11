"""Ток и скорость четырёх мотор-колёс с ODrive по CAN, 10 раз в секунду.

Публикует sensor_msgs/JointState в /motor_current, имена fl, fr, rl, rr:
  effort   — измеренный ток Iq, А (пропорционален моменту);
  velocity — скорость вала мотора, об/с (до редуктора);
  position — позиция вала, обороты.
Знак приведён к роботу: плюс — колесо тянет вперёд (с учётом can_invert_*).
Нет ответа от оси за stale_s — в её полях NaN.

ODrive 0.5.x сам эти величины не рассылает, только heartbeat. Поэтому каждый
такт узел шлёт по оси RTR-запросы Get_Iq (0x014) и Get_Encoder_Estimates
(0x009), а ответы ловит своим сокетом. Команд движения не шлёт, с
odroid_driver на одной шине уживается.
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_GET_IQ = 0x014
WHEELS = ('fl', 'fr', 'rl', 'rr')


class MotorCurrent(Node):

    def __init__(self):
        super().__init__('motor_current')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_interface_rear', 'can1')
        for w, aid in zip(WHEELS, (0, 1, 1, 0)):
            self.declare_parameter(f'axis_id_{w}', aid)
            self.declare_parameter(f'can_invert_{w}', False)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('stale_s', 0.25)
        self.declare_parameter('topic', '/motor_current')

        front = str(self.get_parameter('can_interface').value).strip()
        rear = str(self.get_parameter('can_interface_rear').value).strip() or front
        self.stale_s = float(self.get_parameter('stale_s').value)

        # колесо -> (шина, axis_id, знак); axis_id < 0 — колесо выключено
        self.wheels = {}
        for w in WHEELS:
            aid = int(self.get_parameter(f'axis_id_{w}').value)
            if aid < 0:
                continue
            sign = -1.0 if bool(self.get_parameter(f'can_invert_{w}').value) else 1.0
            self.wheels[w] = (front if w in ('fl', 'fr') else rear, aid, sign)

        self.buses = {}
        for name in sorted({b for b, _, _ in self.wheels.values()}):
            ids = [aid for b, aid, _ in self.wheels.values() if b == name]
            bus = can.Bus(interface='socketcan', channel=name)
            bus.set_filters([{'can_id': (a << 5) | c, 'can_mask': 0x7FF}
                             for a in ids for c in (CMD_GET_IQ, CMD_GET_ENCODER_ESTIMATES)])
            self.buses[name] = bus

        self.lock = threading.Lock()
        self.iq = {}    # (шина, axis) -> (ток, время)
        self.enc = {}   # (шина, axis) -> (позиция, скорость, время)
        self.running = True
        self.threads = [threading.Thread(target=self.read, args=(n, b), daemon=True)
                        for n, b in self.buses.items()]
        for t in self.threads:
            t.start()

        self.pub = self.create_publisher(JointState, str(self.get_parameter('topic').value), 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info(
            'ток моторов в ' + str(self.get_parameter('topic').value) + ': '
            + ', '.join(f'{w}={b}/axis{a}' for w, (b, a, _) in self.wheels.items()))

    def read(self, name, bus):
        while self.running:
            try:
                m = bus.recv(0.1)
            except Exception:
                time.sleep(0.1)
                continue
            if m is None or m.is_remote_frame or len(m.data) < 8:
                continue
            axis, cmd = m.arbitration_id >> 5, m.arbitration_id & 0x1F
            a, b = struct.unpack('<ff', bytes(m.data[:8]))
            now = time.monotonic()
            with self.lock:
                if cmd == CMD_GET_IQ:
                    self.iq[(name, axis)] = (b, now)   # a — уставка, b — измеренный
                elif cmd == CMD_GET_ENCODER_ESTIMATES:
                    self.enc[(name, axis)] = (a, b, now)

    def tick(self):
        self.publish()
        # Запросы на одну шину не пачкой: у ODrive три приёмных ящика, и
        # четвёртый кадр подряд (вместе с командами драйвера) терялся —
        # замерено, без паузы пропадало 20% ответов последней оси.
        queues = {}
        for name, aid, _ in self.wheels.values():
            for cmd in (CMD_GET_IQ, CMD_GET_ENCODER_ESTIMATES):
                queues.setdefault(name, []).append((aid << 5) | cmd)
        for i in range(max(len(q) for q in queues.values())):
            if i:
                time.sleep(0.002)
            for name, q in queues.items():
                if i >= len(q):
                    continue
                try:
                    self.buses[name].send(can.Message(
                        arbitration_id=q[i], is_remote_frame=True,
                        dlc=8, is_extended_id=False))
                except Exception as e:
                    self.get_logger().warn(f'запрос {name} 0x{q[i]:03x}: {e}',
                                           throttle_duration_sec=5.0)

    def publish(self):
        now = time.monotonic()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        nan = math.nan
        with self.lock:
            for w, (name, aid, sign) in self.wheels.items():
                iq = self.iq.get((name, aid))
                enc = self.enc.get((name, aid))
                msg.name.append(w)
                msg.effort.append(sign * iq[0] if iq and now - iq[1] < self.stale_s else nan)
                fresh = enc and now - enc[2] < self.stale_s
                msg.position.append(sign * enc[0] if fresh else nan)
                msg.velocity.append(sign * enc[1] if fresh else nan)
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=0.5)
        for bus in self.buses.values():
            bus.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorCurrent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
