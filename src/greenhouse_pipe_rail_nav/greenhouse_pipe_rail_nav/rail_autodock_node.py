from __future__ import annotations

import math
import time
from enum import Enum
from typing import Any, Dict

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .detector import PipeRailDetector, RailDetection
from .visualization import compose_source_and_bird, draw_motion_command


class DockState(str, Enum):
    IDLE = "idle"
    ALIGN = "align"
    DRIVE_ON_RAIL = "drive_on_rail"
    LOST = "lost"


class PidController:
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float, integral_limit: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        dt = max(dt, 1.0e-3)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral + error * dt))
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


class RailAutodockNode(Node):
    def __init__(self) -> None:
        super().__init__("greenhouse_pipe_rail_autodock")

        self._declare_parameters()
        detector_cfg = self._detector_config()
        self.detector = PipeRailDetector(detector_cfg)
        self.bridge = CvBridge()

        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.bird_view_topic = self.get_parameter("bird_view_topic").value

        self.control_enabled = bool(self.get_parameter("control_enabled").value)
        self.visualize = bool(self.get_parameter("visualize").value)
        self.publish_debug_images = bool(self.get_parameter("publish_debug_images").value)

        self.kp_lateral = float(self.get_parameter("kp_lateral").value)
        self.ki_lateral = float(self.get_parameter("ki_lateral").value)
        self.kd_lateral = float(self.get_parameter("kd_lateral").value)
        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.ki_heading = float(self.get_parameter("ki_heading").value)
        self.kd_heading = float(self.get_parameter("kd_heading").value)
        self.approach_speed = float(self.get_parameter("approach_speed").value)
        self.rail_drive_speed = float(self.get_parameter("rail_drive_speed").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.max_lateral_integral = float(self.get_parameter("max_lateral_integral").value)
        self.max_heading_integral = float(self.get_parameter("max_heading_integral").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.lock_confidence = float(self.get_parameter("lock_confidence").value)
        self.lateral_lock_threshold_m = float(self.get_parameter("lateral_lock_threshold_m").value)
        self.heading_lock_threshold_deg = float(self.get_parameter("heading_lock_threshold_deg").value)
        self.stable_frames_to_lock = int(self.get_parameter("stable_frames_to_lock").value)
        self.drive_distance_m = float(self.get_parameter("drive_distance_m").value)
        self.drive_duration_s = float(self.get_parameter("drive_duration_s").value)
        self.lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self.start_lock_distance_m = float(self.get_parameter("start_lock_distance_m").value)
        self.start_slow_distance_m = float(self.get_parameter("start_slow_distance_m").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 1)
        self.bird_pub = self.create_publisher(Image, self.bird_view_topic, 1)
        self.sub = self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self.state = DockState.IDLE
        self.stable_count = 0
        self.last_seen_time = 0.0
        self.drive_start_time = 0.0
        self.last_control_time = time.monotonic()
        self._last_log_time = 0.0
        self._last_perf_log_time = 0.0
        self.lateral_pid = PidController(
            self.kp_lateral,
            self.ki_lateral,
            self.kd_lateral,
            self.max_lateral_speed,
            self.max_lateral_integral,
        )
        self.heading_pid = PidController(
            self.kp_heading,
            self.ki_heading,
            self.kd_heading,
            self.max_yaw_rate,
            self.max_heading_integral,
        )

        self.get_logger().info(
            f"Listening on {self.image_topic}, publishing {self.cmd_vel_topic}; "
            f"control_enabled={self.control_enabled}, visualize={self.visualize}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_nav")
        self.declare_parameter("annotated_topic", "/pipe_rail/debug/annotated")
        self.declare_parameter("bird_view_topic", "/pipe_rail/debug/bird_view")
        self.declare_parameter("control_enabled", True)
        self.declare_parameter("visualize", True)
        self.declare_parameter("publish_debug_images", True)

        self.declare_parameter("kp_lateral", 0.95)
        self.declare_parameter("ki_lateral", 0.0)
        self.declare_parameter("kd_lateral", 0.08)
        self.declare_parameter("kp_heading", 1.4)
        self.declare_parameter("ki_heading", 0.0)
        self.declare_parameter("kd_heading", 0.10)
        self.declare_parameter("approach_speed", 0.10)
        self.declare_parameter("rail_drive_speed", 0.12)
        self.declare_parameter("max_lateral_speed", 0.16)
        self.declare_parameter("max_yaw_rate", 0.35)
        self.declare_parameter("max_lateral_integral", 0.08)
        self.declare_parameter("max_heading_integral", 0.20)
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("lock_confidence", 0.48)
        self.declare_parameter("lateral_lock_threshold_m", 0.035)
        self.declare_parameter("heading_lock_threshold_deg", 5.0)
        self.declare_parameter("stable_frames_to_lock", 12)
        self.declare_parameter("drive_distance_m", 2.0)
        self.declare_parameter("drive_duration_s", 0.0)
        self.declare_parameter("lost_timeout_s", 0.45)
        self.declare_parameter("start_lock_distance_m", 0.12)
        self.declare_parameter("start_slow_distance_m", 0.65)

        self.declare_parameter("detector.bev_width", 720)
        self.declare_parameter("detector.bev_height", 720)
        self.declare_parameter("detector.source_points", [0.12, 0.04, 0.88, 0.04, 0.98, 0.98, 0.02, 0.98])
        self.declare_parameter("detector.destination_margin_px", 24.0)
        self.declare_parameter("detector.meters_per_pixel", 0.004)
        self.declare_parameter("detector.expected_spacing_px", 0.0)
        self.declare_parameter("detector.expected_spacing_m", 0.55)
        self.declare_parameter("detector.min_spacing_fraction", 0.16)
        self.declare_parameter("detector.max_spacing_fraction", 0.72)
        self.declare_parameter("detector.hard_spacing_tolerance_fraction", 0.65)
        self.declare_parameter("detector.max_line_angle_deg", 36.0)
        self.declare_parameter("detector.max_crossbar_angle_deg", 42.0)
        self.declare_parameter("detector.require_crossbar", True)
        self.declare_parameter("detector.min_segment_length_px", 70)
        self.declare_parameter("detector.hough_threshold", 42)
        self.declare_parameter("detector.max_line_gap_px", 42)
        self.declare_parameter("detector.cluster_window_px", 46.0)
        self.declare_parameter("detector.dark_delta", 26.0)
        self.declare_parameter("detector.blackhat_threshold", 11.0)
        self.declare_parameter("detector.source_processing_width", 640)
        self.declare_parameter("detector.u_shape_processing_width", 360)
        self.declare_parameter("detector.source_min_segment_length_px", 48)
        self.declare_parameter("detector.source_hough_threshold", 34)
        self.declare_parameter("detector.source_max_line_gap_px", 28)

    def _detector_config(self) -> Dict[str, Any]:
        flat_points = list(self.get_parameter("detector.source_points").value)
        source_points = [flat_points[i : i + 2] for i in range(0, len(flat_points), 2)]
        names = [
            "bev_width",
            "bev_height",
            "destination_margin_px",
            "meters_per_pixel",
            "expected_spacing_px",
            "expected_spacing_m",
            "min_spacing_fraction",
            "max_spacing_fraction",
            "hard_spacing_tolerance_fraction",
            "max_line_angle_deg",
            "max_crossbar_angle_deg",
            "require_crossbar",
            "min_segment_length_px",
            "hough_threshold",
            "max_line_gap_px",
            "cluster_window_px",
            "dark_delta",
            "blackhat_threshold",
            "source_processing_width",
            "u_shape_processing_width",
            "source_min_segment_length_px",
            "source_hough_threshold",
            "source_max_line_gap_px",
        ]
        cfg: Dict[str, Any] = {"source_points": source_points}
        for name in names:
            cfg[name] = self.get_parameter(f"detector.{name}").value
        return cfg

    def _on_image(self, msg: Image) -> None:
        t0 = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        want_debug_images = self.publish_debug_images or self.visualize
        detection, debug = self.detector.detect(bgr, debug_images=want_debug_images)
        cmd = self._update_state_and_publish(detection)
        if want_debug_images:
            overlay = draw_motion_command(
                debug.overlay,
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z,
                self.state.value,
            )
            annotated_view = compose_source_and_bird(debug.source_overlay, overlay)

            if self.publish_debug_images:
                annotated = self.bridge.cv2_to_imgmsg(annotated_view, encoding="bgr8")
                annotated.header = msg.header
                bird = self.bridge.cv2_to_imgmsg(debug.bird_view, encoding="bgr8")
                bird.header = msg.header
                self.annotated_pub.publish(annotated)
                self.bird_pub.publish(bird)

            if self.visualize:
                cv2.imshow("greenhouse_pipe_rail_autodock", annotated_view)
                cv2.waitKey(1)

        self._log_processing_time(time.perf_counter() - t0)

    def _update_state_and_publish(self, detection: RailDetection) -> Twist:
        now = time.monotonic()
        dt = now - self.last_control_time
        self.last_control_time = now
        seen = detection.ok and detection.confidence >= self.min_confidence
        if seen:
            self.last_seen_time = now

        if not seen and (now - self.last_seen_time) > self.lost_timeout_s:
            self.state = DockState.LOST
            self.stable_count = 0
            self._reset_pid()

        cmd = Twist()
        if self.state in (DockState.IDLE, DockState.LOST):
            if seen:
                self.state = DockState.ALIGN
            else:
                self._publish_cmd(cmd)
                self._throttled_log("Rail pair not locked; publishing zero cmd_vel")
                return cmd

        if self.state == DockState.ALIGN:
            cmd = self._alignment_command(detection, dt)
            if self._is_locked(detection):
                self.stable_count += 1
            else:
                self.stable_count = 0

            if self.stable_count >= self.stable_frames_to_lock:
                self.state = DockState.DRIVE_ON_RAIL
                self.drive_start_time = now
                self._reset_pid()
                self.get_logger().info("Rail center locked; switching to forward-only rail drive")

        elif self.state == DockState.DRIVE_ON_RAIL:
            duration = self._rail_drive_duration()
            if now - self.drive_start_time <= duration:
                cmd.linear.x = self.rail_drive_speed
            else:
                self.state = DockState.IDLE
                self.stable_count = 0
                self._reset_pid()
                self.get_logger().info("Timed rail drive finished; stopping")

        self._publish_cmd(cmd)
        return cmd

    def _alignment_command(self, detection: RailDetection, dt: float) -> Twist:
        cmd = Twist()
        if not detection.ok:
            return cmd

        abs_error = abs(detection.center_error_m)
        abs_heading = abs(detection.heading_error_rad)
        slowdown = 1.0 - min(0.65, 4.0 * abs_error + 0.8 * abs_heading)
        if detection.start_segment is not None and detection.start_distance_m <= self.start_lock_distance_m:
            distance_scale = 0.0
        elif detection.start_segment is not None:
            distance_scale = self._clamp(detection.start_distance_m / max(self.start_slow_distance_m, 0.01), 0.25, 1.0)
        else:
            distance_scale = 0.45
        cmd.linear.x = self.approach_speed * slowdown * distance_scale
        cmd.linear.y = self.lateral_pid.update(-detection.center_error_m, dt)
        cmd.angular.z = self.heading_pid.update(detection.heading_error_rad, dt)
        return cmd

    def _is_locked(self, detection: RailDetection) -> bool:
        return (
            detection.ok
            and detection.confidence >= self.lock_confidence
            and abs(detection.center_error_m) <= self.lateral_lock_threshold_m
            and abs(math.degrees(detection.heading_error_rad)) <= self.heading_lock_threshold_deg
            and detection.start_segment is not None
            and detection.start_distance_m <= self.start_lock_distance_m
        )

    def _rail_drive_duration(self) -> float:
        if self.drive_duration_s > 0.0:
            return self.drive_duration_s
        speed = max(abs(self.rail_drive_speed), 0.01)
        return self.drive_distance_m / speed

    def _reset_pid(self) -> None:
        self.lateral_pid.reset()
        self.heading_pid.reset()

    def _publish_cmd(self, cmd: Twist) -> None:
        if self.control_enabled:
            self.cmd_pub.publish(cmd)

    def _throttled_log(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_log_time > 2.0:
            self.get_logger().warn(text)
            self._last_log_time = now

    def _log_processing_time(self, elapsed_s: float) -> None:
        now = time.monotonic()
        if now - self._last_perf_log_time > 2.0:
            fps = 1.0 / max(elapsed_s, 1.0e-6)
            self.get_logger().info(f"RGB pipeline {elapsed_s * 1000.0:.1f} ms/frame ({fps:.1f} FPS instantaneous)")
            self._last_perf_log_time = now

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main() -> None:
    import rclpy

    rclpy.init()
    node = RailAutodockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
