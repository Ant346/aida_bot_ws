from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__("pipe_rail_video_image_publisher")
        self.declare_parameter("video_file", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("loop", True)
        self.declare_parameter("fps", 30.0)

        self.video_file = str(self.get_parameter("video_file").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop = bool(self.get_parameter("loop").value)
        fps = float(self.get_parameter("fps").value)

        if not self.video_file:
            raise RuntimeError("video_file parameter is required")
        self.cap = cv2.VideoCapture(self.video_file)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video: {self.video_file}")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, 10)
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self._tick)
        self.get_logger().info(f"Publishing {self.video_file} to {self.image_topic}")

    def _tick(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            if not self.loop:
                return
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VideoImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
