"""Ограничители хода и файлы аварийного стопа.

Пробел выставляет /tmp/odroid_estop. Драйвер не крутит колёса, пока этот
флаг поднят или пока сторож клавиатуры не обновляет watchdog.
"""

import math
import os
import tempfile

ESTOP_FLAG = '/tmp/odroid_estop'
ESTOP_REASON = '/tmp/odroid_estop_reason'
WATCHDOG = '/tmp/odroid_estop_watchdog'
KEYBOARD = '/tmp/odroid_estop_keyboard'
DRIVER_HEARTBEAT = '/tmp/odroid_driver_heartbeat'

WATCHDOG_TIMEOUT_S = 0.5


def _write_atomic(path, text):
    directory = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(prefix='.odroid-', dir=directory)
    try:
        os.write(fd, text.encode())
    finally:
        os.close(fd)
    os.replace(tmp, path)


def estop_latched():
    try:
        with open(ESTOP_FLAG, 'r', encoding='utf-8') as handle:
            return handle.read(1) == '1'
    except OSError:
        return False


def estop_reason():
    try:
        with open(ESTOP_REASON, 'r', encoding='utf-8') as handle:
            return handle.read().strip()
    except OSError:
        return ''


def latch_estop(reason):
    _write_atomic(ESTOP_REASON, reason.strip() + '\n')
    _write_atomic(ESTOP_FLAG, '1\n')


def clear_estop():
    _write_atomic(ESTOP_REASON, '\n')
    _write_atomic(ESTOP_FLAG, '0\n')


def ensure_estop_file():
    if not os.path.exists(ESTOP_FLAG):
        clear_estop()


def touch(path):
    fd = os.open(path, os.O_CREAT | os.O_WRONLY, 0o644)
    os.close(fd)
    os.utime(path, None)


def file_fresh(path, timeout_s, now):
    """now — time.time(), не monotonic: сравнивается с mtime файла."""
    try:
        age = now - os.path.getmtime(path)
    except OSError:
        return False
    return 0.0 <= age <= timeout_s


def watchdog_fresh(now):
    return file_fresh(WATCHDOG, WATCHDOG_TIMEOUT_S, now)


def keyboard_fresh(now):
    return file_fresh(KEYBOARD, WATCHDOG_TIMEOUT_S, now)


def clamp_cmd(vx, vy, wz, max_linear, max_angular):
    linear = math.hypot(vx, vy)
    if linear > max_linear > 0.0:
        scale = max_linear / linear
        vx *= scale
        vy *= scale
    if max_angular > 0.0:
        wz = max(-max_angular, min(max_angular, wz))
    return vx, vy, wz


def ramp_cmd(prev, target, dt, max_linear_accel, max_angular_accel):
    """Шаг от prev к target не быстрее заданных ускорений. 0 = без ограничения.

    (vx, vy) меняется одним вектором, чтобы при разгоне не менялось направление.
    """
    pvx, pvy, pwz = prev
    vx, vy, wz = target
    if max_linear_accel > 0.0:
        dvx = vx - pvx
        dvy = vy - pvy
        delta = math.hypot(dvx, dvy)
        step = max_linear_accel * dt
        if delta > step:
            vx = pvx + dvx * step / delta
            vy = pvy + dvy * step / delta
    if max_angular_accel > 0.0:
        step = max_angular_accel * dt
        wz = pwz + max(-step, min(step, wz - pwz))
    return vx, vy, wz


def scale_wheels(omegas, max_abs):
    peak = max((abs(w) for w in omegas), default=0.0)
    if max_abs > 0.0 and peak > max_abs:
        scale = max_abs / peak
        return tuple(w * scale for w in omegas)
    return tuple(omegas)


def apply_fence(x, y, yaw, vx, vy, wz, dt, radius_limit, yaw_limit, yaw_gain):
    """Не даёт уехать дальше radius_limit от точки старта и накрутить yaw.

    Команда, которая увеличивает нарушение, обнуляется. Возврат к старту проходит.
    yaw копится как wz * yaw_gain — так же, как эта скорость попадает в колёса.
    """
    if dt < 0.0:
        dt = 0.0
    c = math.cos(yaw)
    s = math.sin(yaw)
    nx = x + (c * vx - s * vy) * dt
    ny = y + (s * vx + c * vy) * dt
    nyaw = yaw + wz * yaw_gain * dt
    blocked = False
    radius = math.hypot(x, y)
    next_radius = math.hypot(nx, ny)
    if radius_limit > 0.0 and next_radius > radius_limit and next_radius > radius + 1e-6:
        vx = 0.0
        vy = 0.0
        blocked = True
    else:
        x, y = nx, ny
    if yaw_limit > 0.0 and abs(nyaw) > yaw_limit and abs(nyaw) > abs(yaw) + 1e-6:
        wz = 0.0
        blocked = True
    else:
        yaw = nyaw
    return x, y, yaw, vx, vy, wz, blocked
