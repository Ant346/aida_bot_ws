#!/usr/bin/env python3
"""Publish TartanCalib intrinsics on /d435/d435/color/camera_info.

The driver keeps the factory message on color/camera_info_factory. Header
stamp and frame_id are copied from that message so they stay aligned with
the image.
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def load_info(path: Path) -> CameraInfo:
    data = yaml.safe_load(path.read_text())
    msg = CameraInfo()
    msg.width = int(data["image_width"])
    msg.height = int(data["image_height"])
    msg.distortion_model = str(data["distortion_model"])
    msg.d = [float(v) for v in data["distortion_coefficients"]["data"]]
    msg.k = [float(v) for v in data["camera_matrix"]["data"]]
    msg.r = [float(v) for v in data["rectification_matrix"]["data"]]
    msg.p = [float(v) for v in data["projection_matrix"]["data"]]
    return msg


class ColorInfo(Node):
    def __init__(self, info: CameraInfo) -> None:
        super().__init__("d435_color_camera_info")
        self._info = info
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(CameraInfo, "/d435/d435/color/camera_info", qos)
        self.create_subscription(
            CameraInfo,
            "/d435/d435/color/camera_info_factory",
            self._on_factory,
            qos,
        )

    def _on_factory(self, factory: CameraInfo) -> None:
        msg = CameraInfo()
        msg.header = factory.header
        msg.width = self._info.width
        msg.height = self._info.height
        msg.distortion_model = self._info.distortion_model
        msg.d = list(self._info.d)
        msg.k = list(self._info.k)
        msg.r = list(self._info.r)
        msg.p = list(self._info.p)
        self._pub.publish(msg)


def main() -> None:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "/ws/pipe_rail/calib/d435_color_1280x720.yaml")
    info = load_info(path)
    rclpy.init()
    node = ColorInfo(info)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
