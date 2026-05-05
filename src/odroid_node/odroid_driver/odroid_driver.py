#!/usr/bin/env python3
"""cmd_vel → mecanum wheels: UART (hoverboard framing) or SocketCAN (ODrive native protocol)."""

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

# ODrive CAN: (axis_id << 5) | cmd — см. odroid_v_3.6/main.py
_CAN_SET_AXIS_STATE = 0x007
_CAN_SET_INPUT_VEL = 0x00D
_AXIS_IDLE = 1
_AXIS_CLOSED_LOOP_CONTROL = 8

TAU = 2.0 * np.pi


class OdroidDriver(Node):
    """transport=uart_hoverboard (CH341) или socketcan_odrive на `can0` / `can1`."""

    def __init__(self):
        super().__init__('odroid_driver')

        self.get_logger().info('Initializing odroid driver')

        self.declare_parameters(
            '',
            [
                ('transport', 'uart_hoverboard'),  # socketcan_odrive | uart_hoverboard
                ('simulation_mode', False),
                ('cmd_vel_subscribe_stamped', False),
                ('angular_z_scale', 2.0),
                ('wheel_odometry_topic', 'wheel_odometry'),
                ('wheel_radius', 0.095),
                ('wheel_base', 0.635),
                ('wheel_track', 0.72),
                ('max_speed', 50),
                ('test_speed', 3.0),
                ('front_serial', '0001'),
                ('rear_serial', '0002'),
                ('baudrate', 115200),
                ('timeout', 0.05),
                # SocketCAN (интерфейс после slcand + ip link, обычно can0/can1)
                ('can_interface', 'can0'),
                # Если непусто и отличается от can_interface: FL/FR на передней шине,
                # RL/RR на задней (два USB-CAN, см. odroid_v_3.6/README_CAN.md).
                ('can_interface_rear', ''),
                ('can_bitrate', 250000),
                ('axis_id_fl', 0),
                ('axis_id_fr', 1),
                ('axis_id_rl', 2),
                ('axis_id_rr', 3),
                ('can_invert_fl', False),
                ('can_invert_fr', False),
                ('can_invert_rl', True),
                ('can_invert_rr', True),
                ('torque_ff', 0.0),
            ],
        )

        self.front_port = None
        self.rear_port = None
        self.front_buffer = bytearray()
        self.rear_buffer = bytearray()
        self._transport_kind = 'none'  # none | uart | can
        self._can_bus_front = None
        self._can_bus_rear = None
        self._can_unique_buses = []
        self._can_imported = None

        self._init_boards()

        if self.get_parameter('cmd_vel_subscribe_stamped').value:
            self.create_subscription(
                TwistStamped, 'cmd_vel', self._cmd_vel_stamped_cb, 10)
            self.get_logger().info('cmd_vel: TwistStamped')
        else:
            self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            self.get_logger().info('cmd_vel: Twist')

        otopic = self.get_parameter('wheel_odometry_topic').value
        self.odom_pub = self.create_publisher(Twist, otopic, 10)

        self.create_service(Empty, 'test_wheel', self.test_wheel_callback)

        sim = bool(self.get_parameter('simulation_mode').value)

        self.running = True
        self.front_thread = None
        self.rear_thread = None

        if sim:
            self.front_thread = threading.Thread(
                target=self._front_communication_loop, daemon=True)
        elif self._transport_kind == 'can':
            self.front_thread = threading.Thread(
                target=self._can_communication_loop, daemon=True)
        elif self._transport_kind == 'uart':
            self.front_thread = threading.Thread(
                target=self._front_communication_loop, daemon=True)
            if self.rear_port is not None:
                self.rear_thread = threading.Thread(
                    target=self._rear_communication_loop, daemon=True)
        elif self._transport_kind == 'none':
            self.front_thread = threading.Thread(
                target=self._front_communication_loop, daemon=True)

        if self.front_thread is not None:
            self.front_thread.start()
        if self.rear_thread is not None:
            self.rear_thread.start()

        self.create_timer(0.1, self._publish_odometry)
        mode = ('simulation' if sim else self._transport_kind)
        self.get_logger().info(f'odroid_driver ready (mode={mode})')

    def _can_mod(self):
        if self._can_imported is None:
            try:
                import can as can_mod

                self._can_imported = can_mod
            except ImportError as e:
                raise RuntimeError(
                    'Нужен python3-can для transport socketcan_odrive') from e
        return self._can_imported

    def _norm_transport(self):
        t = str(self.get_parameter('transport').value).replace('-', '_').lower()
        if t in ('uart_hoverboard', 'serial', 'uart'):
            return 'uart'
        if t in ('socketcan_odrive', 'can_odrive', 'odrive_can', 'socketcan'):
            return 'can'
        self.get_logger().warning(f'transport={t} — считаем uart_hoverboard')
        return 'uart'

    def _init_boards(self):
        self._transport_kind = 'none'
        sim = bool(self.get_parameter('simulation_mode').value)

        if sim:
            self.get_logger().info('SIMULATION_MODE')
            self._init_kinematic_params()
            return

        t = self._norm_transport()

        if t == 'can':
            can_mod = self._can_mod()
            iface = self.get_parameter('can_interface').get_parameter_value().string_value.strip()
            if not iface:
                iface = 'can0'
            iface_rear = self.get_parameter('can_interface_rear').get_parameter_value().string_value.strip()
            rate = int(self.get_parameter('can_bitrate').value)
            try:
                self._can_bus_front = can_mod.interface.Bus(
                    bustype='socketcan', channel=iface, bitrate=rate)
                self._can_unique_buses = [self._can_bus_front]
            except Exception as e:
                raise RuntimeError(
                    f'Не удалось открыть SocketCAN `{iface}` '
                    '(проверь: sudo odroid_v_3.6/start_can.sh → can0/can1): '
                    f'{e}',
                ) from e

            if iface_rear and iface_rear != iface:
                try:
                    self._can_bus_rear = can_mod.interface.Bus(
                        bustype='socketcan', channel=iface_rear, bitrate=rate)
                    self._can_unique_buses.append(self._can_bus_rear)
                except Exception as e:
                    raise RuntimeError(
                        f'Не удалось открыть SocketCAN заднюю шину `{iface_rear}`: {e}',
                    ) from e
                self.get_logger().info(
                    f'SocketCAN перед `{iface}`, зад `{iface_rear}`, bitrate={rate}')
            else:
                self._can_bus_rear = self._can_bus_front
                self.get_logger().info(f'SocketCAN одна шина `{iface}`, bitrate={rate}')

            self._transport_kind = 'can'

            aids = tuple(
                int(self.get_parameter(n).value)
                for n in ('axis_id_fl', 'axis_id_fr', 'axis_id_rl', 'axis_id_rr'))
            n_en = sum(1 for a in aids if a >= 0)
            if n_en < 4:
                self.get_logger().warning(
                    f'CAN: включено колёс {n_en}/4 (остальные axis_id_* = -1 не получают команд). '
                    'Проверь odroid_driver.yaml.'
                )

            names = ('axis_id_fl', 'axis_id_fr', 'axis_id_rl', 'axis_id_rr')
            started = False
            for i, pname in enumerate(names):
                aid = int(self.get_parameter(pname).value)
                if aid < 0:
                    continue
                bus = self._can_bus_front if i < 2 else self._can_bus_rear
                self._can_set_axis_state(aid, _AXIS_CLOSED_LOOP_CONTROL, bus)
                started = True
            if started:
                time.sleep(0.35)

            self._init_kinematic_params()
            self.get_logger().info(f'CAN axis ids FL FR RL RR = {aids}')
            return

        # UART
        front_path = self._find_ch341_device(self.get_parameter('front_serial').value)
        if not front_path:
            raise serial.SerialException(
                'Нет UART (transport uart_hoverboard). '
                'Для CAN → transport:=socketcan_odrive, см. YAML')

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
        self._check_device_status(self.front_port)

        self.rear_port = None
        rear_path = self._find_ch341_device(self.get_parameter('rear_serial').value)
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
            except Exception as e:
                self.get_logger().warning(f'UART rear недоступен: {e!s}')

        self._transport_kind = 'uart'
        self._init_kinematic_params()

    def _init_kinematic_params(self):
        self.R = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value) / 2.0
        self.W = float(self.get_parameter('wheel_track').value) / 2.0
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.lock = threading.Lock()
        self.target_vel = np.zeros(3)

    def _find_ch341_device(self, device_id):
        try:
            for device in glob.glob('/dev/serial/by-id/*'):
                if device_id in device:
                    return device
            for p in ('/dev/ttyCH341USB0', '/dev/ttyCH341USB1'):
                if os.path.exists(p):
                    return p
            return None
        except Exception as e:
            self.get_logger().error(str(e))
            return None

    def _check_device_status(self, port):
        try:
            if not port.is_open:
                return
            port.write(b'\xE8\x00\x00\x00')
            time.sleep(0.008)
            if port.in_waiting:
                port.read(port.in_waiting)
        except Exception as e:
            self.get_logger().debug(f'Status poll: {e!s}')

    def _cmd_vel_stamped_cb(self, msg):
        self.cmd_vel_callback(msg.twist)

    def cmd_vel_callback(self, msg: Twist):
        k = float(self.get_parameter('angular_z_scale').value)
        with self.lock:
            self.target_vel[:] = msg.linear.x, msg.linear.y, msg.angular.z * k

    def _wheel_omega_rad_s(self, vx, vy, wz):
        r = max(self.R, 1e-6)
        lw = self.L + self.W
        fl = (vx - vy + lw * wz) / r
        fr = (vx + vy - lw * wz) / r
        rl = (vx + vy + lw * wz) / r
        rr = (vx - vy - lw * wz) / r
        return fl, fr, rl, rr

    def _inverse_kinematic_uart_scaled(self, vx, vy, wz):
        fl, fr, rl, rr = self._wheel_omega_rad_s(vx, vy, wz)
        sc = 1000.0 / max(self.max_speed, 1e-6)
        lim = lambda om: max(-1000, min(1000, int(om * sc)))
        return lim(fl), lim(fr), lim(rl), lim(rr)

    def _can_set_axis_state(self, axis_id: int, state: int, bus):
        cid = (axis_id << 5) | _CAN_SET_AXIS_STATE
        data = int(state).to_bytes(4, 'little')
        msg = self._can_mod().Message(arbitration_id=cid, data=data, is_extended_id=False)
        try:
            bus.send(msg)
        except Exception as e:
            self.get_logger().error(f'CAN set_state axis={axis_id}: {e!s}')

    def _can_send_vel_turns(self, axis_id: int, turns_s: float, tq: float, bus):
        if axis_id < 0:
            return
        cid = (axis_id << 5) | _CAN_SET_INPUT_VEL
        data = struct.pack('<ff', float(turns_s), float(tq))
        msg = self._can_mod().Message(arbitration_id=cid, data=data, is_extended_id=False)
        try:
            bus.send(msg)
        except Exception as e:
            self.get_logger().error(f'CAN velocity axis={axis_id}: {e!s}')

    def _can_communication_loop(self):
        tq = float(self.get_parameter('torque_ff').value)
        aids = [
            int(self.get_parameter(k).value)
            for k in ('axis_id_fl', 'axis_id_fr', 'axis_id_rl', 'axis_id_rr')
        ]
        inv_fl = -1.0 if bool(self.get_parameter('can_invert_fl').value) else 1.0
        inv_fr = -1.0 if bool(self.get_parameter('can_invert_fr').value) else 1.0
        inv_rl = -1.0 if bool(self.get_parameter('can_invert_rl').value) else 1.0
        inv_rr = -1.0 if bool(self.get_parameter('can_invert_rr').value) else 1.0

        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel.tolist()

                fl_r, fr_r, rl_r, rr_r = self._wheel_omega_rad_s(vx, vy, wz)
                fl_t = fl_r * inv_fl / TAU
                fr_t = fr_r * inv_fr / TAU
                rl_t = rl_r * inv_rl / TAU
                rr_t = rr_r * inv_rr / TAU

                spins = [fl_t, fr_t, rl_t, rr_t]

                self.get_logger().debug(
                    f'CAN vx vy wz {(vx, vy, wz)} turns/s {spins}')

                for i, (aid, tsp) in enumerate(zip(aids, spins)):
                    bus = self._can_bus_front if i < 2 else self._can_bus_rear
                    self._can_send_vel_turns(aid, tsp, tq, bus)

                time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'CAN loop: {e!s}')
                time.sleep(1.0)

    def _send_board_command(self, port, left_speed, right_speed, label=''):
        try:
            if bool(self.get_parameter('simulation_mode').value):
                return
            if port is None:
                return

            ls = max(-32768, min(32767, left_speed))
            rs = max(-32768, min(32767, right_speed))

            chk = (
                (0xABCD ^ (ls & 0xFFFF) ^ (rs & 0xFFFF)) & 0xFFFF
            )
            buf = struct.pack('<HhhH', 0xABCD, ls, rs, chk)
            port.reset_input_buffer()
            port.reset_output_buffer()
            port.write(buf)
            port.flush()
            self.get_logger().debug(f'UART {label} L={ls} R={rs}')
        except Exception as e:
            self.get_logger().error(str(e))

    def _read_uart_feedback(self, port, buffer):
        try:
            while port and port.in_waiting:
                buffer.extend(port.read(1))
                if len(buffer) >= 2 and buffer[-2] == 0xCD and buffer[-1] == 0xAB:
                    if len(buffer) >= 20:
                        m = bytes(buffer[-20:])
                        fb = struct.unpack('<HhhhhhhH2xH', m)
                        c = fb[0] ^ fb[1] ^ fb[2] ^ fb[3] ^ fb[4] ^ fb[5] ^ fb[6] ^ fb[7]
                        buffer.clear()
                        if c == fb[8]:
                            return {'speedL': fb[3], 'speedR': fb[4]}
        except Exception as e:
            self.get_logger().error(str(e))
        return None

    def _front_communication_loop(self):
        sim = bool(self.get_parameter('simulation_mode').value)
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel.tolist()
                fi, fj, fk, fm = self._inverse_kinematic_uart_scaled(vx, vy, wz)

                self._send_board_command(self.front_port, fi, fj, 'F')

                if not sim and self.front_port is not None:
                    self._read_uart_feedback(self.front_port, self.front_buffer)

                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'UART front loop: {e!s}')
                time.sleep(0.8)

    def _rear_communication_loop(self):
        sim = bool(self.get_parameter('simulation_mode').value)
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    vx, vy, wz = self.target_vel.tolist()
                fi, fj, fk, fm = self._inverse_kinematic_uart_scaled(vx, vy, wz)
                self._send_board_command(self.rear_port, -fk, -fm, 'R')
                if not sim and self.rear_port:
                    self._read_uart_feedback(self.rear_port, self.rear_buffer)
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'UART rear loop: {e!s}')
                time.sleep(0.8)

    def _forward_kinematic_cmd_twist(self):
        with self.lock:
            vx_c, vy_c, wz_c = self.target_vel.tolist()

        fl, fr, rl, rr = self._wheel_omega_rad_s(vx_c, vy_c, wz_c)
        r = self.R
        lw = self.L + self.W
        vx = r * (fl + fr + rl + rr) / 4.0
        vy = r * (-fl + fr - rl + rr) / 4.0
        wz = r * (-fl + fr + rl - rr) / (4.0 * lw)
        # как в старом hoverboard узле для Twist-«одометрии»:
        return vx, wz, vy

    def _publish_odometry(self):
        try:
            lx, lz, ay = self._forward_kinematic_cmd_twist()
            m = Twist()
            m.linear.x = float(lx)
            m.linear.y = float(lz)
            m.angular.z = float(ay)
            self.odom_pub.publish(m)
        except Exception as e:
            self.get_logger().error(str(e))

    def test_wheel_callback(self, request, response):
        self.running = False
        if self.front_thread is not None:
            self.front_thread.join(timeout=1.2)
        if self.rear_thread is not None:
            self.rear_thread.join(timeout=1.2)

        sim = bool(self.get_parameter('simulation_mode').value)
        t = self._norm_transport()

        try:
            if not sim and t == 'can' and self._can_bus_front is not None:
                turns_s = float(self.get_parameter('test_speed').value)
                aid0 = int(self.get_parameter('axis_id_fl').value)
                iq = float(self.get_parameter('torque_ff').value)
                vt = turns_s if abs(turns_s) > 1e-6 else 3.0
                self.get_logger().info(f'test_wheel CAN FL @ {vt} turns/s')
                if aid0 >= 0:
                    self._can_send_vel_turns(aid0, vt, iq, self._can_bus_front)
                    time.sleep(2.0)
                    self._can_send_vel_turns(aid0, 0.0, iq, self._can_bus_front)
                else:
                    self.get_logger().warning('axis_id_fl < 0')
            elif not sim and self.front_port:
                tsp = max(
                    80,
                    abs(int(float(self.get_parameter('test_speed').value))))
                self._send_board_command(self.front_port, 0, tsp, 'test')
                time.sleep(3.0)
                self._send_board_command(self.front_port, 0, 0, 'test')
            else:
                self.get_logger().info('test_wheel: skip (sim or no iface)')
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.running = True
            sim2 = bool(self.get_parameter('simulation_mode').value)
            self.front_thread = None
            self.rear_thread = None
            if sim2:
                self.front_thread = threading.Thread(
                    target=self._front_communication_loop, daemon=True)
            elif self._transport_kind == 'can':
                self.front_thread = threading.Thread(
                    target=self._can_communication_loop, daemon=True)
            elif self._transport_kind == 'uart':
                self.front_thread = threading.Thread(
                    target=self._front_communication_loop, daemon=True)
                if self.rear_port is not None:
                    self.rear_thread = threading.Thread(
                        target=self._rear_communication_loop, daemon=True)

            if self.front_thread:
                self.front_thread.start()
            if self.rear_thread:
                self.rear_thread.start()

        return response

    def destroy_node(self):
        self.running = False
        if self.front_thread and self.front_thread.is_alive():
            self.front_thread.join(timeout=2.0)
        if self.rear_thread and self.rear_thread.is_alive():
            self.rear_thread.join(timeout=2.0)

        self._shutdown_can_axes()

        try:
            if self.front_port is not None:
                self.front_port.close()
            if self.rear_port is not None:
                self.rear_port.close()
        except Exception:
            pass
        super().destroy_node()

    def _shutdown_can_axes(self):
        if self._can_bus_front is None:
            return

        aids = [
            int(self.get_parameter(k).value)
            for k in ('axis_id_fl', 'axis_id_fr', 'axis_id_rl', 'axis_id_rr')
        ]
        try:
            for i, aid in enumerate(aids):
                if aid < 0:
                    continue
                bus = self._can_bus_front if i < 2 else self._can_bus_rear
                self._can_send_vel_turns(aid, 0.0, 0.0, bus)
                self._can_set_axis_state(aid, _AXIS_IDLE, bus)
        except Exception as e:
            self.get_logger().warning(f'CAN stop: {e!s}')
        for bus in self._can_unique_buses:
            try:
                bus.shutdown()
            except Exception:
                pass
        self._can_bus_front = None
        self._can_bus_rear = None
        self._can_unique_buses = []


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
