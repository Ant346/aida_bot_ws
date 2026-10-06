from __future__ import annotations

import math
from typing import Tuple

import cv2
import numpy as np


def draw_motion_command(
    image: np.ndarray,
    vx: float,
    vy: float,
    wz: float,
    state: str,
    scale_px_per_mps: float = 560.0,
    scale_px_per_radps: float = 170.0,
) -> np.ndarray:
    """Draw the actual robot motion command in bird-view coordinates."""

    out = image.copy()
    height, width = out.shape[:2]
    origin = (width // 2, int(height * 0.88))

    cv2.circle(out, origin, 11, (20, 20, 20), -1, cv2.LINE_AA)
    cv2.circle(out, origin, 11, (255, 255, 255), 2, cv2.LINE_AA)

    if math.hypot(vx, vy) > 1.0e-3:
        end_x = int(round(origin[0] - vy * scale_px_per_mps))
        end_y = int(round(origin[1] - vx * scale_px_per_mps))
        end_x = max(18, min(width - 18, end_x))
        end_y = max(58, min(height - 18, end_y))
        cv2.arrowedLine(out, origin, (end_x, end_y), (50, 255, 90), 6, cv2.LINE_AA, tipLength=0.22)
    else:
        cv2.circle(out, origin, 18, (50, 255, 90), 2, cv2.LINE_AA)

    if abs(wz) > 1.0e-3:
        _draw_yaw_arc(out, origin, wz, scale_px_per_radps)

    text = f"{state}  vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}"
    cv2.rectangle(out, (8, height - 42), (min(width - 8, 520), height - 8), (0, 0, 0), -1)
    cv2.putText(out, text, (18, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def compose_source_and_bird(source_overlay: np.ndarray | None, bird_overlay: np.ndarray) -> np.ndarray:
    if source_overlay is None:
        return bird_overlay
    height = bird_overlay.shape[0]
    src_h, src_w = source_overlay.shape[:2]
    width = int(round(src_w * height / max(src_h, 1)))
    source_resized = cv2.resize(source_overlay, (width, height), interpolation=cv2.INTER_AREA)
    return np.hstack((source_resized, bird_overlay))


def _draw_yaw_arc(image: np.ndarray, origin: Tuple[int, int], wz: float, scale_px_per_radps: float) -> None:
    radius = int(max(38, min(92, abs(wz) * scale_px_per_radps)))
    color = (40, 210, 255)
    start_deg, end_deg = (205, 335) if wz > 0.0 else (335, 205)

    cv2.ellipse(image, origin, (radius, radius), 0.0, start_deg, end_deg, color, 4, cv2.LINE_AA)

    tip_angle = math.radians(end_deg)
    tip = (
        int(round(origin[0] + radius * math.cos(tip_angle))),
        int(round(origin[1] + radius * math.sin(tip_angle))),
    )
    tangent = tip_angle + (math.pi * 0.5 if wz > 0.0 else -math.pi * 0.5)
    left = (
        int(round(tip[0] - 13 * math.cos(tangent - 0.55))),
        int(round(tip[1] - 13 * math.sin(tangent - 0.55))),
    )
    right = (
        int(round(tip[0] - 13 * math.cos(tangent + 0.55))),
        int(round(tip[1] - 13 * math.sin(tangent + 0.55))),
    )
    cv2.line(image, tip, left, color, 4, cv2.LINE_AA)
    cv2.line(image, tip, right, color, 4, cv2.LINE_AA)
