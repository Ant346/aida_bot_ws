from __future__ import annotations

import argparse
import statistics
import time
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
    parser = argparse.ArgumentParser(description="Benchmark RGB-only pipe-rail detector on a video.")
    parser.add_argument("video", help="Input RGB video")
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--snapshot", default="", help="Write one debug overlay image")
    parser.add_argument("--detector-config", default="", help="Detector calibration YAML")
    parser.add_argument("--no-debug", action="store_true", help="Skip debug image rendering to measure the real control hot path")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {args.video}")

    detector = PipeRailDetector(load_detector_config(args.detector_config))
    source_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    frame_count = 0
    processed = 0
    detections = 0
    times_ms = []
    snapshot_written = False

    while processed < args.max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        frame_count += 1
        if args.stride > 1 and (frame_count - 1) % args.stride != 0:
            continue

        t0 = time.perf_counter()
        detection, debug = detector.detect(frame, debug_images=not args.no_debug)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        times_ms.append(elapsed_ms)
        processed += 1
        detections += int(detection.ok)

        if args.snapshot and detection.ok and not snapshot_written and not args.no_debug:
            vx, vy, wz = _sample_alignment_command(detection.center_error_m, detection.heading_error_rad, detection.start_distance_m)
            overlay = draw_motion_command(debug.overlay, vx, vy, wz, "sample_align")
            Path(args.snapshot).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(args.snapshot, overlay)
            snapshot_written = True

    if not times_ms:
        raise SystemExit("No frames processed")

    mean_ms = statistics.fmean(times_ms)
    p95_ms = sorted(times_ms)[int(0.95 * (len(times_ms) - 1))]
    print(f"video={args.video}")
    print(f"source_fps={source_fps:.2f} processed_frames={processed} detections={detections}")
    print(f"mean_ms={mean_ms:.2f} p95_ms={p95_ms:.2f} max_ms={max(times_ms):.2f}")
    print(f"mean_fps={1000.0 / mean_ms:.1f}")


if __name__ == "__main__":
    main()
