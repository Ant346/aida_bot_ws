#!/usr/bin/env python3
"""Поза base_link относительно AprilGrid по D435.

Доска как в калибровке: tag36h11, рамка 2 бита, 3×3, tagSize 0.05 м,
tagSpacing 0.3. Печатает одну строку JSON.
"""

import ctypes
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from watch_april_tags import TOPIC, decode

INFO_TOPIC = os.environ.get("CALIB_D435_INFO", "/d435/d435/color/camera_info")
TAG_SIZE = 0.05
TAG_SPACING = 0.3
COLS = 3
SAMPLES = int(os.environ.get("BOARD_POSE_SAMPLES", "5"))

_LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libaprilgrid.so")
_detect = ctypes.CDLL(_LIB)
_detect.detect_april36h11_corners.argtypes = [
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
]
_detect.detect_april36h11_corners.restype = ctypes.c_int


def rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# CAD: base_link → d435_link → optical.
_T_BASE_OPT = transform(
    rpy(0.0, 0.872664626, 0.0),
    np.array([0.451636729, -0.017499974, 0.691293991]),
) @ transform(rpy(-np.pi / 2, 0.0, -np.pi / 2), np.zeros(3))


def tag_object_points(tag_id: int) -> np.ndarray:
    row, col = divmod(tag_id, COLS)
    pitch = (1.0 + TAG_SPACING) * TAG_SIZE
    x0 = col * pitch
    y0 = row * pitch
    s = TAG_SIZE
    return np.array(
        [
            [x0, y0, 0.0],
            [x0 + s, y0, 0.0],
            [x0 + s, y0 + s, 0.0],
            [x0, y0 + s, 0.0],
        ],
        dtype=np.float64,
    )


def corners(gray: np.ndarray):
    gray = np.ascontiguousarray(gray)
    ids = (ctypes.c_int * 16)()
    xy = (ctypes.c_float * (16 * 8))()
    n = _detect.detect_april36h11_corners(
        gray.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
        gray.shape[1],
        gray.shape[0],
        gray.strides[0],
        ids,
        xy,
        16,
    )
    found = []
    for i in range(n):
        tag_id = int(ids[i])
        if not 0 <= tag_id <= 8:
            continue
        pts = np.array(xy[i * 8 : (i + 1) * 8], dtype=np.float64).reshape(4, 2)
        found.append((tag_id, pts))
    return found


def solve_board(found, camera, dist):
    obj = []
    img = []
    ids = []
    for tag_id, pts in found:
        obj.append(tag_object_points(tag_id))
        img.append(pts)
        ids.append(tag_id)
    obj = np.vstack(obj)
    img = np.vstack(img)
    ok, rvec, tvec = cv2.solvePnP(
        obj, img, camera, dist, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        raise RuntimeError("solvePnP не сошёлся")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera, dist)
    err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)))
    R, _ = cv2.Rodrigues(rvec)
    T_opt_board = transform(R, tvec.reshape(3))
    T_board_base = np.linalg.inv(T_opt_board) @ np.linalg.inv(_T_BASE_OPT)
    t = T_board_base[:3, 3]
    forward = T_board_base[:3, 0]
    yaw = float(np.arctan2(forward[1], forward[0]))
    flat = np.vstack(img)
    margin = float(
        min(
            flat[:, 0].min(),
            flat[:, 1].min(),
            1280 - flat[:, 0].max(),
            720 - flat[:, 1].max(),
        )
    )
    return {
        "ids": ids,
        "reproj_px": err,
        "margin_px": margin,
        "board_xyz": [float(t[0]), float(t[1]), float(t[2])],
        "yaw": yaw,
    }


class Grab(Node):
    def __init__(self):
        super().__init__("board_pose")
        self.image = None
        self.info = None
        self.create_subscription(Image, TOPIC, self._image, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._info, 10)

    def _image(self, msg):
        self.image = msg

    def _info(self, msg):
        self.info = msg


def main():
    rclpy.init()
    node = Grab()
    t0 = time.monotonic()
    while (node.image is None or node.info is None) and time.monotonic() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.image is None or node.info is None:
        raise SystemExit("нет кадра или camera_info")
    K = np.array(node.info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(node.info.d, dtype=np.float64)
    samples = []
    last = None
    while len(samples) < SAMPLES and time.monotonic() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.05)
        msg = node.image
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == last:
            continue
        last = stamp
        bgr = decode(msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        found = corners(gray)
        if len(found) < 6:
            continue
        samples.append(solve_board(found, K, dist))
    if not samples:
        raise SystemExit("мало тегов для позы")
    xyz = np.array([s["board_xyz"] for s in samples])
    yaw = np.array([s["yaw"] for s in samples])
    yaw = np.unwrap(yaw)
    out = {
        "n": len(samples),
        "ids": samples[-1]["ids"],
        "reproj_px": float(np.mean([s["reproj_px"] for s in samples])),
        "margin_px": float(np.min([s["margin_px"] for s in samples])),
        "xyz": xyz.mean(axis=0).tolist(),
        "xyz_std": xyz.std(axis=0).tolist(),
        "yaw": float(yaw.mean()),
        "yaw_std": float(yaw.std()),
        "fx": float(K[0, 0]),
        "dist": dist.tolist(),
    }
    print(json.dumps(out))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
