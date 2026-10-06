from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from .detector import PipeRailDetector
from .config_utils import load_detector_config
from .visualization import draw_motion_command


def _sample_alignment_command(center_error_m: float, heading_error_rad: float, start_distance_m: float = 0.0) -> tuple[float, float, float]:
    approach_speed = 0.10
    kp_lateral = 0.95
    kp_heading = 1.40
    max_lateral_speed = 0.16
    max_yaw_rate = 0.35

    slowdown = 1.0 - min(0.65, 4.0 * abs(center_error_m) + 0.8 * abs(heading_error_rad))
    distance_scale = 0.0 if start_distance_m <= 0.12 else max(0.25, min(1.0, start_distance_m / 0.65))
    vx = approach_speed * slowdown * distance_scale
    vy = max(-max_lateral_speed, min(max_lateral_speed, -kp_lateral * center_error_m))
    wz = max(-max_yaw_rate, min(max_yaw_rate, kp_heading * heading_error_rad))
    return vx, vy, wz


def main() -> None:
    parser = argparse.ArgumentParser(description="Run pipe-rail detection on one image.")
    parser.add_argument("image", help="Input image path")
    parser.add_argument("--output", default="", help="Output debug image path")
    parser.add_argument("--detector-config", default="", help="Detector calibration YAML")
    parser.add_argument("--show", action="store_true", help="Show OpenCV window")
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Could not read {args.image}")

    detector = PipeRailDetector(load_detector_config(args.detector_config))
    detection, debug = detector.detect(image)
    vx, vy, wz = _sample_alignment_command(detection.center_error_m, detection.heading_error_rad, detection.start_distance_m) if detection.ok else (0.0, 0.0, 0.0)
    overlay = draw_motion_command(debug.overlay, vx, vy, wz, "sample_align" if detection.ok else "lost")
    print(
        "ok={ok} confidence={conf:.3f} lateral_error_m={err:+.3f} "
        "heading_deg={head:+.2f} spacing_px={spacing:.1f} cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f}) message={msg}".format(
            ok=detection.ok,
            conf=detection.confidence,
            err=detection.center_error_m,
            head=detection.heading_error_rad * 57.295779513,
            spacing=detection.rail_spacing_px,
            vx=vx,
            vy=vy,
            wz=wz,
            msg=detection.message,
        )
    )

    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(args.output, overlay)
    if args.show:
        cv2.imshow("pipe rail detector", overlay)
        cv2.waitKey(0)


if __name__ == "__main__":
    main()
