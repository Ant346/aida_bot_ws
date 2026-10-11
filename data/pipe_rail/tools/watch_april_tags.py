#!/usr/bin/env python3
"""Сколько тегов AprilGrid сейчас видно. Печатает одну строку в терминал.

Доска та же, что у калибровки: tag36h11, чёрная рамка 2 бита, 3×3,
центр (id 6) пустой, всего 8. Считает libaprilgrid.so рядом со скриптом.
Камера по умолчанию D435. Запуск в контейнере, где уже идёт картинка:

  docker exec -it aida_bot_ws-rviz_novnc-1 bash -lc \\
    'source /opt/ros/jazzy/setup.bash && python3 /ws/pipe_rail/tools/watch_april_tags.py'
"""

import ctypes
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = os.environ.get("CALIB_D435_TOPIC", "/d435/d435/color/image_raw")
EXPECTED = 8
_LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libaprilgrid.so")
_detect = ctypes.CDLL(_LIB)
_detect.detect_april36h11.argtypes = [
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
]
_detect.detect_april36h11.restype = ctypes.c_int


def decode(msg: Image) -> np.ndarray:
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = raw[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(img)
    if enc in ("mono8", "8uc1"):
        return np.ascontiguousarray(raw[:, : msg.width])
    raise RuntimeError(f"неизвестная кодировка {msg.encoding}")


def tag_ids(bgr: np.ndarray) -> list[int]:
    # Kalibr AprilGrid: рамка 2 бита. Словарь OpenCV её не декодирует.
    gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray)
    out = (ctypes.c_int * 16)()
    n = _detect.detect_april36h11(
        gray.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
        gray.shape[1],
        gray.shape[0],
        gray.strides[0],
        out,
        16,
    )
    return [out[i] for i in range(n)]


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("watch_april_tags")
        self.latest = None
        self.create_subscription(Image, TOPIC, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        self.latest = msg


def main() -> None:
    rclpy.init()
    node = Watch()
    last_stamp = None
    frames = 0
    t0 = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            msg = node.latest
            if msg is None:
                print("\rнет кадра с " + TOPIC + "          ", end="", flush=True)
                continue
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            ids = tag_ids(decode(msg))
            frames += 1
            hz = frames / max(time.monotonic() - t0, 1e-3)
            missing = EXPECTED - len(ids)
            line = (
                f"D435  тегов {len(ids)}/{EXPECTED}  "
                f"нет {missing}  id {ids or '-'}  {hz:.1f} Гц"
            )
            print("\r" + line + " " * 8, end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
