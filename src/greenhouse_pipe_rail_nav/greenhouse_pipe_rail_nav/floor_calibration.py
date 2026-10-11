from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class GroundModel:
    normal: np.ndarray
    d: float
    origin: np.ndarray
    right: np.ndarray
    forward: np.ndarray
    inlier_ratio: float
    mean_error_m: float


def estimate_intrinsics(width: int, height: int, fov_x_deg: float, fov_y_deg: float) -> Intrinsics:
    fx = (0.5 * width) / math.tan(math.radians(fov_x_deg) * 0.5)
    fy = (0.5 * height) / math.tan(math.radians(fov_y_deg) * 0.5)
    return Intrinsics(fx=fx, fy=fy, cx=(width - 1) * 0.5, cy=(height - 1) * 0.5)


def unproject_depth(depth_mm: np.ndarray, intr: Intrinsics, stride: int, roi: Tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    height, width = depth_mm.shape[:2]
    x0 = int(width * roi[0])
    y0 = int(height * roi[1])
    x1 = int(width * roi[2])
    y1 = int(height * roi[3])

    ys, xs = np.mgrid[y0:y1:stride, x0:x1:stride]
    z = depth_mm[y0:y1:stride, x0:x1:stride].astype(np.float32) * 0.001
    valid = (z > 0.20) & (z < 5.0)
    xs = xs[valid].astype(np.float32)
    ys = ys[valid].astype(np.float32)
    z = z[valid]

    x = (xs - intr.cx) * z / intr.fx
    y = (ys - intr.cy) * z / intr.fy
    points = np.column_stack((x, y, z)).astype(np.float32)
    pixels = np.column_stack((xs, ys)).astype(np.float32)
    return points, pixels


def fit_ground_plane(points: np.ndarray, iterations: int, threshold_m: float, seed: int) -> GroundModel:
    if len(points) < 300:
        raise RuntimeError("Not enough valid depth points to fit the floor plane")

    rng = np.random.default_rng(seed)
    best_inliers = None
    best_count = -1

    for _ in range(iterations):
        sample_idx = rng.choice(len(points), size=3, replace=False)
        p0, p1, p2 = points[sample_idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1.0e-6:
            continue
        normal = normal / norm
        if normal[1] > 0.0:
            normal = -normal
        d = -float(np.dot(normal, p0))
        distances = np.abs(points @ normal + d)
        inliers = distances < threshold_m
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 300:
        raise RuntimeError("Could not find a dominant floor plane in depth data")

    plane_points = points[best_inliers]
    centroid = np.mean(plane_points, axis=0)
    _, _, vh = np.linalg.svd(plane_points - centroid, full_matrices=False)
    normal = vh[-1].astype(np.float64)
    normal = normal / np.linalg.norm(normal)
    if normal[1] > 0.0:
        normal = -normal
    d = -float(np.dot(normal, centroid))
    distances = np.abs(points @ normal + d)
    refined_inliers = distances < threshold_m
    mean_error = float(np.mean(distances[refined_inliers]))

    origin = -d * normal
    optical_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    forward = optical_axis - np.dot(optical_axis, normal) * normal
    forward_norm = np.linalg.norm(forward)
    if forward_norm < 1.0e-6:
        raise RuntimeError("Camera optical axis is almost perpendicular to the floor plane")
    forward = forward / forward_norm
    if forward[2] < 0.0:
        forward = -forward
    right = np.cross(forward, normal)
    right = right / np.linalg.norm(right)
    if right[0] < 0.0:
        right = -right

    return GroundModel(
        normal=normal,
        d=d,
        origin=origin,
        right=right,
        forward=forward,
        inlier_ratio=float(np.count_nonzero(refined_inliers) / len(points)),
        mean_error_m=mean_error,
    )


def project_ground_points(points_xy: np.ndarray, ground: GroundModel, intr: Intrinsics) -> np.ndarray:
    points_3d = ground.origin + points_xy[:, 0:1] * ground.right + points_xy[:, 1:2] * ground.forward
    z = points_3d[:, 2]
    u = intr.fx * points_3d[:, 0] / z + intr.cx
    v = intr.fy * points_3d[:, 1] / z + intr.cy
    return np.column_stack((u, v)).astype(np.float32)


def write_ros_yaml(path: str, source_points_norm: np.ndarray, meters_per_pixel: float, expected_spacing_m: float, bev_size: int) -> None:
    import yaml

    data = {
        "greenhouse_pipe_rail_autodock": {
            "ros__parameters": {
                "detector": {
                    "bev_width": int(bev_size),
                    "bev_height": int(bev_size),
                    "source_points": [round(float(v), 6) for v in source_points_norm.reshape(-1)],
                    "meters_per_pixel": round(float(meters_per_pixel), 7),
                    "expected_spacing_m": float(expected_spacing_m),
                    "expected_spacing_px": 0.0,
                    "hard_spacing_tolerance_fraction": 0.34,
                    "require_crossbar": True,
                    "source_processing_width": 640,
                    "u_shape_processing_width": 360,
                }
            }
        }
    }
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)


def draw_debug(
    color: np.ndarray,
    depth_mm: np.ndarray,
    ground: GroundModel,
    source_points: np.ndarray,
    output_path: str,
) -> None:
    vis = color.copy()
    quad = np.round(source_points).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(vis, [quad], isClosed=True, color=(0, 255, 255), thickness=4, lineType=cv2.LINE_AA)
    for idx, point in enumerate(source_points):
        cv2.circle(vis, tuple(np.round(point).astype(int)), 10, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(vis, str(idx), tuple(np.round(point + 14).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    text = f"floor inliers={ground.inlier_ratio:.2f} mean_err={ground.mean_error_m * 1000.0:.1f}mm"
    cv2.rectangle(vis, (8, 8), (720, 50), (0, 0, 0), -1)
    cv2.putText(vis, text, (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (255, 255, 255), 2, cv2.LINE_AA)

    depth_vis = cv2.normalize(depth_mm, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
    combo = np.vstack((cv2.resize(vis, (960, 540)), cv2.resize(depth_vis, (960, 540))))
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(output_path, combo)


def main() -> None:
    parser = argparse.ArgumentParser(description="Estimate camera-floor extrinsics from aligned RGB + 16-bit depth.")
    parser.add_argument("--color", required=True, help="Aligned RGB image")
    parser.add_argument("--depth", required=True, help="Aligned 16-bit depth PNG, millimeters")
    parser.add_argument("--output", required=True, help="Output ROS parameter YAML")
    parser.add_argument("--debug", default="", help="Debug image with projected floor rectangle")
    parser.add_argument("--fx", type=float, default=0.0)
    parser.add_argument("--fy", type=float, default=0.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument("--fov-x-deg", type=float, default=69.4)
    parser.add_argument("--fov-y-deg", type=float, default=42.5)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--ransac-iters", type=int, default=900)
    parser.add_argument("--threshold-m", type=float, default=0.018)
    parser.add_argument("--roi", nargs=4, type=float, default=[0.04, 0.28, 0.96, 0.96], metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("--bev-size", type=int, default=720)
    parser.add_argument("--x-half-width-m", type=float, default=1.25)
    parser.add_argument("--y-near-m", type=float, default=0.25)
    parser.add_argument("--y-far-m", type=float, default=2.75)
    parser.add_argument("--expected-spacing-m", type=float, default=0.55)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    color = cv2.imread(args.color, cv2.IMREAD_COLOR)
    depth = cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
    if color is None:
        raise SystemExit(f"Could not read color image: {args.color}")
    if depth is None or depth.dtype != np.uint16:
        raise SystemExit(f"Could not read 16-bit depth PNG: {args.depth}")

    height, width = depth.shape[:2]
    if args.fx > 0.0 and args.fy > 0.0:
        intr = Intrinsics(
            fx=args.fx,
            fy=args.fy,
            cx=args.cx if args.cx >= 0.0 else (width - 1) * 0.5,
            cy=args.cy if args.cy >= 0.0 else (height - 1) * 0.5,
        )
    else:
        intr = estimate_intrinsics(width, height, args.fov_x_deg, args.fov_y_deg)

    points, _ = unproject_depth(depth, intr, args.stride, tuple(args.roi))
    ground = fit_ground_plane(points, args.ransac_iters, args.threshold_m, args.seed)

    ground_corners = np.array(
        [
            [-args.x_half_width_m, args.y_far_m],
            [args.x_half_width_m, args.y_far_m],
            [args.x_half_width_m, args.y_near_m],
            [-args.x_half_width_m, args.y_near_m],
        ],
        dtype=np.float32,
    )
    source_points = project_ground_points(ground_corners, ground, intr)
    source_points_norm = source_points / np.array([[width, height]], dtype=np.float32)
    meters_per_pixel = (2.0 * args.x_half_width_m) / args.bev_size
    write_ros_yaml(args.output, source_points_norm, meters_per_pixel, args.expected_spacing_m, args.bev_size)

    if args.debug:
        draw_debug(color, depth, ground, source_points, args.debug)

    print(f"wrote={args.output}")
    print(f"inlier_ratio={ground.inlier_ratio:.3f} mean_error_mm={ground.mean_error_m * 1000.0:.2f}")
    print(f"normal={ground.normal.tolist()} d={ground.d:.4f}")
    print(f"source_points_norm={source_points_norm.reshape(-1).tolist()}")


if __name__ == "__main__":
    main()
