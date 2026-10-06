from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

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


def _fit_height(image: np.ndarray, height: int) -> np.ndarray:
    src_h, src_w = image.shape[:2]
    width = int(round(src_w * height / max(src_h, 1)))
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def main() -> None:
    parser = argparse.ArgumentParser(description="Render a full RGB-only pipe-rail debug video.")
    parser.add_argument("video", help="Input RGB video")
    parser.add_argument("output", help="Output mp4 path")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fourcc", default="mp4v")
    parser.add_argument("--detector-config", default="", help="Detector calibration YAML")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {args.video}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    detector_config = load_detector_config(args.detector_config)
    detector_config["bev_height"] = args.height
    detector_config["bev_width"] = args.height
    detector = PipeRailDetector(detector_config)

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Input video is empty")

    _, debug = detector.detect(frame)
    original = _fit_height(debug.source_overlay if debug.source_overlay is not None else frame, args.height)
    annotated = draw_motion_command(debug.overlay, 0.0, 0.0, 0.0, "warmup")
    output_size = (original.shape[1] + annotated.shape[1], args.height)

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        args.output,
        cv2.VideoWriter_fourcc(*args.fourcc),
        fps,
        output_size,
    )
    if not writer.isOpened():
        raise SystemExit(f"Could not open writer for {args.output}")

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    processed = 0
    detections = 0
    while True:
        if args.max_frames > 0 and processed >= args.max_frames:
            break
        ok, frame = cap.read()
        if not ok:
            break

        detection, debug = detector.detect(frame)
        detections += int(detection.ok)
        if detection.ok:
            vx, vy, wz = _sample_alignment_command(detection.center_error_m, detection.heading_error_rad, detection.start_distance_m)
            state = "sample_align"
        else:
            vx, vy, wz = 0.0, 0.0, 0.0
            state = "lost"

        original = _fit_height(debug.source_overlay if debug.source_overlay is not None else frame, args.height)
        annotated = draw_motion_command(debug.overlay, vx, vy, wz, state)
        composite = np.hstack((original, annotated))
        writer.write(composite)
        processed += 1

    writer.release()
    cap.release()
    print(f"wrote={args.output}")
    print(f"frames={processed} detections={detections} fps={fps:.2f} size={output_size[0]}x{output_size[1]}")


if __name__ == "__main__":
    main()
