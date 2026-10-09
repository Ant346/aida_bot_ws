#!/usr/bin/env python3
"""Save AprilGrid frames from D405 and D435 for TartanCalib.

The board is the small tag36h11 aprilgrid: 3 by 3 with tag id 6
absent (8 tags). Tag edge 50 mm. The gap measured on the frames is
0.30 of the tag, so tagSpacing = 0.3. Ids are Kalibr's row-major order.

The window opens on the same noVNC display as RViz. Click it, then:

  S  save both cameras with one timestamp (intrinsics and the pair)
  1  save D405 only
  2  save D435 only
  Q  quit

Images land in dataset/cam0 (D405) and dataset/cam1 (D435). Filenames are
nanosecond timestamps, which kalibr_bagcreater turns into a ROS1 bag.
"""

import ctypes
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# Маленькая доска 3×3 (тег 50 мм, зазор 0.3) — экстринсики.
# Большая 11×8 (тег 25 мм, зазор 0.3) — интринсики:
#   CALIB_TAG_COLS=11 CALIB_TAG_ROWS=8 CALIB_TAG_SIZE=0.025
TAG_COLS = int(os.environ.get("CALIB_TAG_COLS", "3"))
TAG_ROWS = int(os.environ.get("CALIB_TAG_ROWS", "3"))
TAG_SIZE_M = float(os.environ.get("CALIB_TAG_SIZE", "0.05"))
TAG_SPACING = float(os.environ.get("CALIB_TAG_SPACING", "0.30"))
FAMILY = "tag36h11"
EXPECTED_TAGS = 8

# cam0 is the TartanCalib reference camera.
CAMERAS = (
    {
        "name": "d405",
        "cam": "cam0",
        "topic": os.environ.get("CALIB_D405_TOPIC", "/d405/d405/color/image_raw"),
        "key": ord("1"),
    },
    {
        "name": "d435",
        "cam": "cam1",
        "topic": os.environ.get("CALIB_D435_TOPIC", "/d435/d435/color/image_raw"),
        "key": ord("2"),
    },
)

OUT = Path(os.environ.get("CALIB_OUT", "/ws/pipe_rail/calib_kalibr"))
STALE_SEC = 1.0


def target_yaml() -> dict:
    return {
        "target_type": "aprilgrid",
        "tagCols": TAG_COLS,
        "tagRows": TAG_ROWS,
        "tagSize": TAG_SIZE_M,
        "tagSpacing": round(TAG_SPACING, 4),
    }


def make_detector():
    """Детектор Kalibr: у доски чёрная рамка в 2 бита, pupil apriltag её не читает."""
    lib_path = os.environ.get("APRILGRID_LIB", "/ws/pipe_rail/tools/libaprilgrid.so")
    if not os.path.exists(lib_path):
        raise SystemExit(f"Нет {lib_path}. Это детектор доски Kalibr.")
    lib = ctypes.CDLL(lib_path)
    fn = lib.detect_april36h11_corners
    fn.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    fn.restype = ctypes.c_int
    return fn


def detect_tags(detector, gray: np.ndarray) -> list:
    max_tags = TAG_COLS * TAG_ROWS
    ids = (ctypes.c_int * max_tags)()
    xy = (ctypes.c_float * (max_tags * 8))()
    n = detector(
        gray.ctypes.data,
        gray.shape[1],
        gray.shape[0],
        gray.strides[0],
        ids,
        xy,
        max_tags,
    )
    hits = []
    for i in range(n):
        pts = np.array(xy[i * 8 : (i + 1) * 8], dtype=np.float32).reshape(4, 2)
        hits.append({"id": int(ids[i]), "center": pts.mean(axis=0), "lb-rb-rt-lt": pts})
    return hits


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
    if enc in ("yuv422_yuy2", "yuyv", "yuy2"):
        img = raw[:, : msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    raise RuntimeError(f"unsupported encoding {msg.encoding}")


def draw_tags(bgr: np.ndarray, hits: list) -> np.ndarray:
    view = bgr if bgr.ndim == 3 else cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
    view = view.copy()
    for hit in hits:
        pts = np.round(hit["lb-rb-rt-lt"]).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(view, [pts], True, (0, 220, 0), 1, cv2.LINE_AA)
        center = tuple(np.round(hit["center"]).astype(int))
        cv2.putText(
            view,
            str(hit["id"]),
            center,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 220, 0),
            1,
            cv2.LINE_AA,
        )
    return view


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("capture_kalibr_frames")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.detector = make_detector()
        self.latest: dict[str, Image | None] = {cam["name"]: None for cam in CAMERAS}
        self.arrived: dict[str, float] = {cam["name"]: 0.0 for cam in CAMERAS}
        for cam in CAMERAS:
            self.create_subscription(
                Image,
                cam["topic"],
                lambda msg, name=cam["name"]: self._store(name, msg),
                qos,
            )
        self.session = OUT / time.strftime("%Y%m%d_%H%M%S")
        self.session.mkdir(parents=True, exist_ok=True)
        (self.session / "target.yaml").write_text(
            yaml.safe_dump(target_yaml(), sort_keys=False),
            encoding="utf-8",
        )
        (self.session / "cameras.yaml").write_text(
            yaml.safe_dump(
                {
                    "model": "pinhole-radtan",
                    "cameras": [
                        {
                            "name": cam["name"],
                            "folder": cam["cam"],
                            "topic": f"/{cam['cam']}/image_raw",
                            "source": cam["topic"],
                        }
                        for cam in CAMERAS
                    ],
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        for cam in CAMERAS:
            (self.session / "dataset" / cam["cam"]).mkdir(parents=True, exist_ok=True)
        (self.session / "frames").mkdir(exist_ok=True)
        self.saved = 0
        self.flash = ""
        self.flash_until = 0.0

    def _store(self, name: str, msg: Image) -> None:
        self.latest[name] = msg
        self.arrived[name] = time.time()

    def _fresh(self, name: str) -> Image | None:
        msg = self.latest[name]
        if msg is None or time.time() - self.arrived[name] > STALE_SEC:
            return None
        return msg

    def inspect(self, name: str):
        msg = self._fresh(name)
        if msg is None:
            return None
        bgr = decode(msg)
        gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        hits = detect_tags(self.detector, np.ascontiguousarray(gray))
        return bgr, hits

    def save(self, names: list[str]) -> None:
        shots = []
        for cam in CAMERAS:
            if cam["name"] not in names:
                continue
            viewed = self.inspect(cam["name"])
            if viewed is None:
                continue
            shots.append((cam, viewed))
        if not shots:
            self.flash = "нет свежего кадра"
            self.flash_until = time.time() + 1.2
            return

        stamp_ns = time.time_ns()
        index = f"{self.saved:03d}"
        parts = []
        for cam, (bgr, hits) in shots:
            folder = self.session / "dataset" / cam["cam"]
            cv2.imwrite(str(folder / f"{stamp_ns}.png"), bgr)
            cv2.imwrite(str(self.session / "frames" / f"{index}_{cam['name']}.png"), bgr)
            parts.append(f"{cam['name']} {len(hits)}")
        line = f"{index} {stamp_ns} " + " ".join(parts)
        with (self.session / "shots.txt").open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
        self.saved += 1
        self.flash = "saved " + line
        self.flash_until = time.time() + 1.4
        self.get_logger().info(self.flash)

    def canvas(self) -> np.ndarray:
        width, height = 1560, 860
        body = np.zeros((height, width, 3), np.uint8)
        panel_w = width // len(CAMERAS)
        for i, cam in enumerate(CAMERAS):
            x0 = i * panel_w
            viewed = self.inspect(cam["name"])
            if viewed is None:
                label = f"{cam['name']}  нет кадра"
                color = (80, 80, 255)
            else:
                bgr, hits = viewed
                thumb = draw_tags(bgr, hits)
                scale = min((panel_w - 16) / thumb.shape[1], (height - 64) / thumb.shape[0])
                size = (max(1, int(thumb.shape[1] * scale)), max(1, int(thumb.shape[0] * scale)))
                thumb = cv2.resize(thumb, size, interpolation=cv2.INTER_AREA)
                y = 40 + (height - 64 - thumb.shape[0]) // 2
                x = x0 + (panel_w - thumb.shape[1]) // 2
                body[y : y + thumb.shape[0], x : x + thumb.shape[1]] = thumb
                label = f"{cam['name']}  {len(hits)} меток"
                color = (0, 200, 0)
            cv2.putText(
                body, label, (x0 + 12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA
            )
        note = "Интринсики D435: клавиша 2, доска в разных местах кадра. Q выход"
        if time.time() < self.flash_until:
            note = self.flash
        cv2.putText(
            body,
            f"saved {self.saved}    {note}",
            (12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        return body


def main() -> None:
    if not os.environ.get("DISPLAY") and os.path.exists("/tmp/.X11-unix/X99"):
        os.environ["DISPLAY"] = ":99"
    rclpy.init()
    node = Capture()
    print(f"dataset: {node.session}", flush=True)
    print("Окно kalibr в noVNC. S сохраняет обе камеры, 1 только D405, 2 только D435, Q выход.", flush=True)
    cv2.namedWindow("kalibr", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("kalibr", 1560, 860)
    last_save = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow("kalibr", node.canvas())
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            now = time.time()
            if now - last_save <= 0.4:
                continue
            if key in (ord("s"), ord("S"), 32):
                node.save([cam["name"] for cam in CAMERAS])
                last_save = now
            elif key in (ord("1"),):
                node.save(["d405"])
                last_save = now
            elif key in (ord("2"),):
                node.save(["d435"])
                last_save = now
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
