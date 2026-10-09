"""ODrive 0.5.1 CANSimple: ошибки осей из heartbeat и их сброс."""

import struct
import threading
import time

CMD_HEARTBEAT = 0x001
CMD_CLEAR_ERRORS = 0x018

HEARTBEAT_TIMEOUT_S = 0.5

AXIS_ERRORS = {
    0x00000001: 'INVALID_STATE',
    0x00000002: 'DC_BUS_UNDER_VOLTAGE',
    0x00000004: 'DC_BUS_OVER_VOLTAGE',
    0x00000008: 'CURRENT_MEASUREMENT_TIMEOUT',
    0x00000010: 'BRAKE_RESISTOR_DISARMED',
    0x00000020: 'MOTOR_DISARMED',
    0x00000040: 'MOTOR_FAILED',
    0x00000080: 'SENSORLESS_ESTIMATOR_FAILED',
    0x00000100: 'ENCODER_FAILED',
    0x00000200: 'CONTROLLER_FAILED',
    0x00000400: 'POS_CTRL_DURING_SENSORLESS',
    0x00000800: 'WATCHDOG_TIMER_EXPIRED',
    0x00001000: 'MIN_ENDSTOP_PRESSED',
    0x00002000: 'MAX_ENDSTOP_PRESSED',
    0x00004000: 'ESTOP_REQUESTED',
    0x00020000: 'HOMING_WITHOUT_ENDSTOP',
    0x00040000: 'OVER_TEMP',
}


def axis_error_text(code):
    names = [name for bit, name in AXIS_ERRORS.items() if code & bit]
    rest = code & ~sum(AXIS_ERRORS)
    if rest:
        names.append(f'0x{rest:x}')
    return '|'.join(names) or 'NONE'


def parse_heartbeat(arbitration_id, data):
    """(axis_id, axis_error, axis_state) или None, если это не heartbeat."""
    if arbitration_id & 0x1F != CMD_HEARTBEAT or len(data) < 5:
        return None
    error = struct.unpack_from('<I', bytes(data), 0)[0]
    return arbitration_id >> 5, error, data[4]


class HeartbeatMonitor:
    """Читает heartbeat со всех шин в фоне и помнит последнее состояние каждой оси."""

    def __init__(self, buses, logger):
        self._buses = dict(buses)
        self._logger = logger
        self._lock = threading.Lock()
        self._axes = {}
        self._reported = {}
        self._started = time.monotonic()
        self._running = True
        self._threads = [
            threading.Thread(target=self._read, args=(name, bus), daemon=True)
            for name, bus in self._buses.items()
        ]
        for thread in self._threads:
            thread.start()

    def _read(self, name, bus):
        while self._running:
            try:
                msg = bus.recv(0.1)
            except Exception:
                time.sleep(0.1)
                continue
            if msg is None or msg.is_remote_frame:
                continue
            beat = parse_heartbeat(msg.arbitration_id, msg.data)
            if beat is None:
                continue
            axis, error, state = beat
            key = (name, axis)
            with self._lock:
                self._axes[key] = (error, state, time.monotonic())
                changed = self._reported.get(key) != error
                self._reported[key] = error
            if changed and error:
                self._logger.error(
                    f'ODrive {name} axis{axis}: {axis_error_text(error)} (0x{error:x}), '
                    f'state {state}')
            elif changed:
                self._logger.info(f'ODrive {name} axis{axis}: ошибок нет, state {state}')

    def fault(self, expected):
        """Причина стопа по ожидаемым осям [(шина, axis_id)] или ''."""
        now = time.monotonic()
        with self._lock:
            for key in expected:
                entry = self._axes.get(key)
                if entry is None:
                    if now - self._started > 1.0:
                        return f'нет heartbeat от {key[0]} axis{key[1]}'
                    return 'жду heartbeat ODrive'
                error, _state, seen = entry
                if now - seen > HEARTBEAT_TIMEOUT_S:
                    return f'пропал heartbeat {key[0]} axis{key[1]}'
                if error:
                    return f'ошибка ODrive {key[0]} axis{key[1]}: {axis_error_text(error)}'
        return ''

    def errors(self, expected):
        with self._lock:
            return {key: self._axes[key][0] for key in expected if key in self._axes}

    def stop(self):
        self._running = False
        for thread in self._threads:
            thread.join(timeout=0.5)


def clear_errors(can_mod, bus, axis_id):
    msg = can_mod.Message(
        arbitration_id=(axis_id << 5) | CMD_CLEAR_ERRORS, data=b'', is_extended_id=False)
    bus.send(msg)
