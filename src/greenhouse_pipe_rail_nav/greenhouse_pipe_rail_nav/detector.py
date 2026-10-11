from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


Line = Tuple[float, float]


@dataclass
class RailDetection:
    ok: bool
    confidence: float
    center_error_px: float = 0.0
    center_error_m: float = 0.0
    heading_error_rad: float = 0.0
    rail_spacing_px: float = 0.0
    left_line: Optional[Line] = None
    right_line: Optional[Line] = None
    start_segment: Optional[Tuple[float, float, float, float]] = None
    start_y_px: float = 0.0
    start_distance_m: float = 0.0
    message: str = ""


@dataclass
class DetectorDebug:
    bird_view: np.ndarray
    mask: np.ndarray
    overlay: np.ndarray
    source_overlay: Optional[np.ndarray] = None


def _as_points(values: Sequence[Sequence[float]], width: int, height: int) -> np.ndarray:
    pts = np.asarray(values, dtype=np.float32)
    if pts.shape != (4, 2):
        raise ValueError("Expected four 2D points")
    if np.max(pts) <= 1.5:
        scale = np.asarray([width, height], dtype=np.float32)
        pts = pts * scale
    return pts.astype(np.float32)


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PipeRailDetector:
    """Detects a greenhouse pipe rail pair in the original RGB image.

    Dark line segments are extracted in source-image coordinates first. Their
    endpoints are then projected onto the calibrated floor/bird-view plane where
    pair selection and control errors are computed.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = config or {}
        self.bev_width = int(cfg.get("bev_width", 720))
        self.bev_height = int(cfg.get("bev_height", 720))
        self.source_points = cfg.get(
            "source_points",
            [[0.12, 0.04], [0.88, 0.04], [0.98, 0.98], [0.02, 0.98]],
        )
        self.destination_margin_px = float(cfg.get("destination_margin_px", 24.0))
        self.roi_top_fraction = float(cfg.get("roi_top_fraction", 0.05))
        self.roi_bottom_fraction = float(cfg.get("roi_bottom_fraction", 0.95))
        self.reference_y_fraction = float(cfg.get("reference_y_fraction", 0.78))
        self.lookahead_y_fraction = float(cfg.get("lookahead_y_fraction", 0.35))
        self.meters_per_pixel = float(cfg.get("meters_per_pixel", 0.004))
        self.expected_spacing_px = float(cfg.get("expected_spacing_px", 0.0))
        self.expected_spacing_m = float(cfg.get("expected_spacing_m", 0.55))
        self.min_spacing_fraction = float(cfg.get("min_spacing_fraction", 0.16))
        self.max_spacing_fraction = float(cfg.get("max_spacing_fraction", 0.72))
        self.spacing_tolerance_fraction = float(cfg.get("spacing_tolerance_fraction", 0.45))
        self.hard_spacing_tolerance_fraction = float(cfg.get("hard_spacing_tolerance_fraction", 0.65))
        self.max_line_angle_deg = float(cfg.get("max_line_angle_deg", 36.0))
        self.max_parallel_angle_deg = float(cfg.get("max_parallel_angle_deg", 16.0))
        self.max_crossbar_angle_deg = float(cfg.get("max_crossbar_angle_deg", 42.0))
        self.require_crossbar = bool(cfg.get("require_crossbar", True))
        self.min_segment_length_px = int(cfg.get("min_segment_length_px", 70))
        self.hough_threshold = int(cfg.get("hough_threshold", 42))
        self.max_line_gap_px = int(cfg.get("max_line_gap_px", 42))
        self.cluster_window_px = float(cfg.get("cluster_window_px", 46.0))
        self.dark_delta = float(cfg.get("dark_delta", 26.0))
        self.blackhat_threshold = float(cfg.get("blackhat_threshold", 11.0))
        self.source_processing_width = int(cfg.get("source_processing_width", 800))
        self.u_shape_processing_width = int(cfg.get("u_shape_processing_width", 480))
        self.source_min_segment_length_px = int(cfg.get("source_min_segment_length_px", 48))
        self.source_hough_threshold = int(cfg.get("source_hough_threshold", 34))
        self.source_max_line_gap_px = int(cfg.get("source_max_line_gap_px", 28))
        self._homography: Optional[np.ndarray] = None
        self._inverse_homography: Optional[np.ndarray] = None
        self._homography_shape: Optional[Tuple[int, int]] = None
        self._last_detection: Optional[RailDetection] = None
        self._missed_frames = 9999

    def detect(self, bgr: np.ndarray, debug_images: bool = True) -> Tuple[RailDetection, DetectorDebug]:
        if bgr is None or bgr.size == 0:
            empty = np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8)
            return RailDetection(False, 0.0, message="empty image"), DetectorDebug(empty, empty[..., 0], empty)

        source = self._resize_for_detection(bgr)
        self._ensure_homography(source.shape[1], source.shape[0])
        bird = self._warp_to_bird_view(source) if debug_images else np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8)
        source_mask = self._segment_dark_rails_source(source)
        source_candidates = self._source_line_candidates(source_mask)
        candidates = self._project_source_candidates_to_bird(source_candidates)
        bird_mask = cv2.warpPerspective(source_mask, self._require_homography(source), (self.bev_width, self.bev_height))
        detection = self._select_pair(candidates)
        if detection.ok:
            refined = self._refine_detection_from_mask(detection, bird_mask, "ok-refined", confidence_scale=0.98)
            if refined.ok:
                detection = refined
        if not detection.ok and self._last_detection is not None and self._missed_frames <= 6:
            tracked = self._refine_detection_from_mask(
                self._last_detection,
                bird_mask,
                "tracked-refined",
                confidence_scale=0.72,
            )
            if tracked.ok:
                detection = tracked
        if not detection.ok or detection.confidence < 0.55:
            u_detection = self._select_u_shape_from_mask(bird_mask)
            if u_detection.ok and (not detection.ok or u_detection.confidence > detection.confidence):
                detection = u_detection
        if detection.ok:
            self._last_detection = detection
            self._missed_frames = 0
        else:
            self._missed_frames += 1
        if debug_images:
            overlay = self._draw_debug(bird, bird_mask, candidates, detection)
            source_overlay = self._draw_source_debug(source, source_mask, source_candidates, detection)
        else:
            overlay = bird
            source_overlay = None
        return detection, DetectorDebug(bird, bird_mask, overlay, source_overlay)

    def _ensure_homography(self, width: int, height: int) -> None:
        shape = (width, height)
        if self._homography is not None and self._homography_shape == shape:
            return
        src = _as_points(self.source_points, width, height)
        margin = self.destination_margin_px
        dst = np.asarray(
            [
                [margin, margin],
                [self.bev_width - margin, margin],
                [self.bev_width - margin, self.bev_height - margin],
                [margin, self.bev_height - margin],
            ],
            dtype=np.float32,
        )
        self._homography = cv2.getPerspectiveTransform(src, dst)
        self._inverse_homography = np.linalg.inv(self._homography)
        self._homography_shape = shape

    def _resize_for_detection(self, bgr: np.ndarray) -> np.ndarray:
        height, width = bgr.shape[:2]
        if self.source_processing_width <= 0 or width <= self.source_processing_width:
            return bgr
        scale = self.source_processing_width / float(width)
        return cv2.resize(bgr, (self.source_processing_width, int(round(height * scale))), interpolation=cv2.INTER_AREA)

    def _warp_to_bird_view(self, bgr: np.ndarray) -> np.ndarray:
        height, width = bgr.shape[:2]
        self._ensure_homography(width, height)
        assert self._homography is not None
        return cv2.warpPerspective(
            bgr,
            self._homography,
            (self.bev_width, self.bev_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )

    def _require_homography(self, bgr: np.ndarray) -> np.ndarray:
        if self._homography is None or self._homography_shape != (bgr.shape[1], bgr.shape[0]):
            self._warp_to_bird_view(bgr)
        assert self._homography is not None
        return self._homography

    def _require_inverse_homography(self, bgr: np.ndarray) -> np.ndarray:
        if self._inverse_homography is None or self._homography_shape != (bgr.shape[1], bgr.shape[0]):
            self._warp_to_bird_view(bgr)
        assert self._inverse_homography is not None
        return self._inverse_homography

    def _segment_dark_rails_source(self, source: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        kernel_size = max(17, int(round(source.shape[1] * 0.035)) | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)

        floor_level = float(np.percentile(gray, 58.0))
        dark_mask = gray < (floor_level - self.dark_delta)
        response_mask = blackhat > self.blackhat_threshold
        mask = np.logical_or(dark_mask, response_mask).astype(np.uint8) * 255

        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 11))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    def _segment_dark_rails(self, bird: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        kernel_size = max(19, int(round(self.bev_width * 0.055)) | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)

        floor_level = float(np.percentile(gray, 62.0))
        dark_mask = gray < (floor_level - self.dark_delta)
        response_mask = blackhat > self.blackhat_threshold
        mask = np.logical_or(dark_mask, response_mask).astype(np.uint8) * 255

        top = int(self.roi_top_fraction * self.bev_height)
        bottom = int(self.roi_bottom_fraction * self.bev_height)
        roi = np.zeros_like(mask)
        roi[top:bottom, :] = 255
        mask = cv2.bitwise_and(mask, roi)

        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 17))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    def _source_line_candidates(self, mask: np.ndarray) -> List[Dict[str, Any]]:
        edges = cv2.Canny(mask, 45, 145)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.source_hough_threshold,
            minLineLength=self.source_min_segment_length_px,
            maxLineGap=self.source_max_line_gap_px,
        )
        if lines is None:
            return []

        candidates: List[Dict[str, Any]] = []
        for item in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(v) for v in item]
            dx = x2 - x1
            dy = y2 - y1
            length = float(np.hypot(dx, dy))
            if length < self.source_min_segment_length_px:
                continue
            candidates.append({"source_segment": (x1, y1, x2, y2), "source_length": length})
        return candidates

    def _project_source_candidates_to_bird(self, source_candidates: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not source_candidates:
            return []
        points = []
        for cand in source_candidates:
            x1, y1, x2, y2 = cand["source_segment"]
            points.append([x1, y1])
            points.append([x2, y2])
        pts = np.asarray(points, dtype=np.float32).reshape((-1, 1, 2))
        if self._homography is None:
            raise RuntimeError("homography is not initialized")
        warped = cv2.perspectiveTransform(pts, self._homography).reshape((-1, 2))

        y_ref = self.reference_y_fraction * self.bev_height
        candidates: List[Dict[str, Any]] = []
        for idx, cand in enumerate(source_candidates):
            (x1, y1), (x2, y2) = warped[2 * idx], warped[2 * idx + 1]
            if not np.all(np.isfinite([x1, y1, x2, y2])):
                continue
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = float(np.hypot(dx, dy))
            if length < self.min_segment_length_px:
                continue
            angle = float(np.arctan2(dx, dy))
            if abs(dy) > 8.0:
                slope = dx / dy
                intercept = float(x1 - slope * y1)
                x_ref = slope * y_ref + intercept
            else:
                slope = 1.0e6
                intercept = 0.0
                x_ref = 0.5 * (x1 + x2)
            candidates.append(
                {
                    "segment": (float(x1), float(y1), float(x2), float(y2)),
                    "source_segment": cand["source_segment"],
                    "line": (float(slope), float(intercept)),
                    "x_ref": float(x_ref),
                    "angle": angle,
                    "length": length,
                    "y_min": float(min(y1, y2)),
                    "y_max": float(max(y1, y2)),
                }
            )
        return candidates

    def _line_candidates(self, mask: np.ndarray) -> List[Dict[str, Any]]:
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=self.min_segment_length_px,
            maxLineGap=self.max_line_gap_px,
        )
        if lines is None:
            return []

        max_angle = np.deg2rad(self.max_line_angle_deg)
        y_ref = self.reference_y_fraction * self.bev_height
        candidates: List[Dict[str, Any]] = []
        for item in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(v) for v in item]
            dx = x2 - x1
            dy = y2 - y1
            length = float(np.hypot(dx, dy))
            if length < self.min_segment_length_px or abs(dy) < 8.0:
                continue
            angle = float(np.arctan2(dx, dy))
            if abs(angle) > max_angle:
                continue
            slope = dx / dy
            intercept = x1 - slope * y1
            x_ref = slope * y_ref + intercept
            candidates.append(
                {
                    "segment": (x1, y1, x2, y2),
                    "line": (slope, intercept),
                    "x_ref": x_ref,
                    "angle": angle,
                    "length": length,
                    "y_min": min(y1, y2),
                    "y_max": max(y1, y2),
                }
            )
        return candidates

    def _select_pair(self, candidates: List[Dict[str, Any]]) -> RailDetection:
        if len(candidates) < 2:
            return RailDetection(False, 0.0, message="not enough line candidates")

        center_ref = self.bev_width * 0.5
        expected = self._expected_spacing_px()
        min_spacing = self.min_spacing_fraction * self.bev_width
        max_spacing = self.max_spacing_fraction * self.bev_width
        max_angle_diff = np.deg2rad(self.max_parallel_angle_deg)
        max_side_angle = np.deg2rad(self.max_line_angle_deg)
        max_crossbar_angle = np.deg2rad(self.max_crossbar_angle_deg)

        side_candidates = [cand for cand in candidates if abs(cand["angle"]) <= max_side_angle]
        crossbar_candidates = [
            cand
            for cand in candidates
            if abs(abs(cand["angle"]) - np.pi * 0.5) <= max_crossbar_angle
            and 0.30 * expected <= cand["length"] <= 1.85 * expected
        ]
        if len(side_candidates) < 2:
            return RailDetection(False, 0.0, message="not enough side rail candidates")
        if self.require_crossbar and not crossbar_candidates:
            return RailDetection(False, 0.0, message="no start crossbar candidates")

        best_score = -1.0e9
        best_pair: Optional[Tuple[Line, Line, float, float, Optional[Dict[str, Any]]]] = None

        for i, left_seed in enumerate(side_candidates):
            for right_seed in side_candidates[i + 1 :]:
                y_seed = self.reference_y_fraction * self.bev_height
                left_x = self._x_at(left_seed["line"], y_seed)
                right_x = self._x_at(right_seed["line"], y_seed)
                if left_x > right_x:
                    left_seed, right_seed = right_seed, left_seed
                    left_x, right_x = right_x, left_x

                left_x = left_seed["x_ref"]
                right_x = right_seed["x_ref"]
                if left_x > right_x:
                    left_seed, right_seed = right_seed, left_seed
                    left_x, right_x = right_x, left_x

                spacing = right_x - left_x
                if spacing < min_spacing or spacing > max_spacing:
                    continue
                if abs(left_seed["angle"] - right_seed["angle"]) > max_angle_diff:
                    continue

                left_line, left_weight = self._fit_cluster(side_candidates, left_seed)
                right_line, right_weight = self._fit_cluster(side_candidates, right_seed)
                if left_line is None or right_line is None:
                    continue

                crossbar, crossbar_quality = self._best_crossbar(crossbar_candidates, left_line, right_line, expected)
                if self.require_crossbar and crossbar is None:
                    continue

                y_ref = self._target_y_from_crossbar(crossbar)
                left_ref = self._x_at(left_line, y_ref)
                right_ref = self._x_at(right_line, y_ref)
                if left_ref > right_ref:
                    left_line, right_line = right_line, left_line
                    left_ref, right_ref = right_ref, left_ref

                spacing = right_ref - left_ref
                if spacing < min_spacing or spacing > max_spacing:
                    continue
                if abs(spacing - expected) > expected * self.hard_spacing_tolerance_fraction:
                    continue

                center = 0.5 * (left_ref + right_ref)
                spacing_penalty = abs(spacing - expected) / max(expected, 1.0)
                center_penalty = abs(center - center_ref) / max(center_ref, 1.0)
                angle_penalty = abs(np.arctan(left_line[0]) - np.arctan(right_line[0])) / max_angle_diff
                coverage = _clip((left_weight + right_weight) / max(2.0 * self.bev_height, 1.0), 0.0, 1.0)
                score = (
                    1.8 * coverage
                    + 1.3 * crossbar_quality
                    - 0.9 * spacing_penalty
                    - 0.35 * center_penalty
                    - 0.7 * angle_penalty
                )
                if score > best_score:
                    best_score = score
                    best_pair = (left_line, right_line, spacing, coverage, crossbar)

        if best_pair is None:
            return RailDetection(False, 0.0, message="no plausible parallel pair")

        left_line, right_line, spacing, coverage, crossbar = best_pair
        y_ref = self._target_y_from_crossbar(crossbar)
        y_look = max(self.lookahead_y_fraction * self.bev_height, y_ref - 0.38 * self.bev_height)
        center_near = 0.5 * (self._x_at(left_line, y_ref) + self._x_at(right_line, y_ref))
        center_far = 0.5 * (self._x_at(left_line, y_look) + self._x_at(right_line, y_look))
        center_error_px = center_near - center_ref
        heading_error = float(np.arctan2(center_far - center_near, y_ref - y_look))

        spacing_quality = 1.0 - _clip(abs(spacing - expected) / max(expected * self.spacing_tolerance_fraction, 1.0), 0.0, 1.0)
        heading_quality = 1.0 - _clip(abs(heading_error) / np.deg2rad(35.0), 0.0, 1.0)
        center_quality = 1.0 - _clip(abs(center_error_px) / (0.5 * self.bev_width), 0.0, 1.0)
        crossbar_quality = float(crossbar["quality"]) if crossbar is not None else 0.0
        confidence = _clip(
            0.30 * coverage
            + 0.24 * spacing_quality
            + 0.20 * crossbar_quality
            + 0.16 * heading_quality
            + 0.10 * center_quality,
            0.0,
            1.0,
        )

        start_segment = tuple(float(v) for v in crossbar["segment"]) if crossbar is not None else None
        start_y = float(crossbar["y_mid"]) if crossbar is not None else 0.0
        start_distance = max(0.0, (self.bev_height - start_y) * self.meters_per_pixel) if crossbar is not None else 0.0

        return RailDetection(
            ok=confidence > 0.2,
            confidence=confidence,
            center_error_px=float(center_error_px),
            center_error_m=float(center_error_px * self.meters_per_pixel),
            heading_error_rad=heading_error,
            rail_spacing_px=float(spacing),
            left_line=left_line,
            right_line=right_line,
            start_segment=start_segment,
            start_y_px=start_y,
            start_distance_m=float(start_distance),
            message="ok",
        )

    def _best_crossbar(
        self,
        crossbar_candidates: List[Dict[str, Any]],
        left_line: Line,
        right_line: Line,
        expected_spacing: float,
    ) -> Tuple[Optional[Dict[str, Any]], float]:
        best: Optional[Dict[str, Any]] = None
        best_quality = 0.0
        for cand in crossbar_candidates:
            x1, y1, x2, y2 = cand["segment"]
            y_mid = 0.5 * (y1 + y2)
            if y_mid < 0.18 * self.bev_height or y_mid > 1.08 * self.bev_height:
                continue
            left_x = self._x_at(left_line, y_mid)
            right_x = self._x_at(right_line, y_mid)
            if left_x > right_x:
                left_x, right_x = right_x, left_x
            spacing = right_x - left_x
            if spacing <= 1.0:
                continue
            seg_min = min(x1, x2)
            seg_max = max(x1, x2)
            overlap = max(0.0, min(seg_max, right_x) - max(seg_min, left_x))
            overlap_ratio = overlap / max(spacing, 1.0)
            center_error = abs(0.5 * (seg_min + seg_max) - 0.5 * (left_x + right_x)) / max(spacing, 1.0)
            spacing_quality = 1.0 - _clip(abs(spacing - expected_spacing) / max(expected_spacing * self.spacing_tolerance_fraction, 1.0), 0.0, 1.0)
            angle_quality = 1.0 - _clip(abs(abs(cand["angle"]) - np.pi * 0.5) / max(np.deg2rad(self.max_crossbar_angle_deg), 1.0e-6), 0.0, 1.0)
            start_priority = _clip(y_mid / self.bev_height, 0.0, 1.0)
            quality = (
                0.34 * _clip(overlap_ratio, 0.0, 1.0)
                + 0.24 * angle_quality
                + 0.22 * spacing_quality
                + 0.12 * (1.0 - _clip(center_error, 0.0, 1.0))
                + 0.08 * start_priority
            )
            if quality > best_quality and overlap_ratio > 0.22:
                best_quality = quality
                best = dict(cand)
                best["quality"] = float(quality)
                best["y_mid"] = float(y_mid)
        return best, best_quality

    def _target_y_from_crossbar(self, crossbar: Optional[Dict[str, Any]]) -> float:
        if crossbar is None:
            return self.reference_y_fraction * self.bev_height
        y = float(crossbar["y_mid"])
        return _clip(y - 0.04 * self.bev_height, 0.35 * self.bev_height, 0.90 * self.bev_height)

    def _select_u_shape_from_mask(self, bird_mask: np.ndarray) -> RailDetection:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))
        closed = cv2.morphologyEx(bird_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        closed = cv2.morphologyEx(closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        scale = 1.0
        if self.u_shape_processing_width > 0 and closed.shape[1] > self.u_shape_processing_width:
            scale = closed.shape[1] / float(self.u_shape_processing_width)
            small_h = int(round(closed.shape[0] / scale))
            closed = cv2.resize(closed, (self.u_shape_processing_width, small_h), interpolation=cv2.INTER_NEAREST)
        contours, _ = cv2.findContours((closed > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return RailDetection(False, 0.0, message="no u-shape components")

        expected = self._expected_spacing_px()
        best: Optional[RailDetection] = None
        best_conf = 0.0
        for contour in contours:
            area = int(cv2.contourArea(contour))
            area_full = int(round(area * scale * scale))
            if area_full < 1800 or area_full > 0.38 * self.bev_width * self.bev_height:
                continue
            comp_mask = np.zeros_like(closed, dtype=np.uint8)
            cv2.drawContours(comp_mask, [contour], -1, 255, thickness=-1)
            stroke_mask = cv2.bitwise_and((closed > 0).astype(np.uint8) * 255, comp_mask)
            ys, xs = np.where(stroke_mask > 0)
            xs = xs.astype(np.float32) * scale
            ys = ys.astype(np.float32) * scale
            if len(xs) < 800:
                continue
            stroke_area_full = int(round(len(xs) * scale * scale))
            det = self._fit_u_component(xs, ys, stroke_area_full, expected)
            if det.ok and det.confidence > best_conf:
                best = det
                best_conf = det.confidence

        if best is None:
            return RailDetection(False, 0.0, message="no plausible u-shape")
        return best

    def _refine_detection_from_mask(
        self,
        seed: RailDetection,
        bird_mask: np.ndarray,
        message: str,
        confidence_scale: float = 1.0,
    ) -> RailDetection:
        if seed.left_line is None or seed.right_line is None:
            return RailDetection(False, 0.0, message=f"{message}: no seed lines")

        mask = cv2.morphologyEx(
            bird_mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
            iterations=1,
        )
        ys, xs = np.where(mask > 0)
        if len(xs) < 600:
            return RailDetection(False, 0.0, message=f"{message}: not enough mask pixels")

        xs_f = xs.astype(np.float32)
        ys_f = ys.astype(np.float32)
        pts = np.column_stack((xs_f, ys_f)).astype(np.float32)
        expected = self._expected_spacing_px()
        gate = max(18.0, 0.072 * expected)

        left_x = seed.left_line[0] * ys_f + seed.left_line[1]
        right_x = seed.right_line[0] * ys_f + seed.right_line[1]
        if np.median(left_x - right_x) > 0.0:
            left_x, right_x = right_x, left_x
        dist_left = np.abs(xs_f - left_x)
        dist_right = np.abs(xs_f - right_x)

        start_y = seed.start_y_px if seed.start_y_px > 1.0 else self.reference_y_fraction * self.bev_height
        side_y = ys_f < start_y - max(12.0, 0.018 * self.bev_height)
        side_y &= ys_f > self.roi_top_fraction * self.bev_height
        left_points = pts[(dist_left < gate) & (dist_left <= dist_right + 4.0) & side_y]
        right_points = pts[(dist_right < gate) & (dist_right < dist_left + 4.0) & side_y]
        min_side_pixels = 90
        if len(left_points) < min_side_pixels or len(right_points) < min_side_pixels:
            return RailDetection(False, 0.0, message=f"{message}: weak side support")

        left_line = self._fit_huber_line(left_points)
        right_line = self._fit_huber_line(right_points)
        if left_line is None or right_line is None:
            return RailDetection(False, 0.0, message=f"{message}: side fit failed")

        cross_points = self._crossbar_points_from_seed(pts, seed, expected)
        if len(cross_points) < 70:
            y_gate = max(20.0, 0.065 * expected)
            y_band = np.abs(ys_f - start_y) < y_gate
            left_band = np.minimum(left_x, right_x) - gate
            right_band = np.maximum(left_x, right_x) + gate
            cross_points = pts[y_band & (xs_f > left_band) & (xs_f < right_band)]
        start_segment = self._fit_huber_segment(cross_points, min_length=0.38 * expected)
        if start_segment is None:
            return RailDetection(False, 0.0, message=f"{message}: crossbar fit failed")

        return self._make_detection_from_geometry(
            left_line,
            right_line,
            start_segment,
            expected,
            len(left_points),
            len(right_points),
            len(cross_points),
            len(pts),
            message,
            confidence_scale,
        )

    @staticmethod
    def _crossbar_points_from_seed(pts: np.ndarray, seed: RailDetection, expected: float) -> np.ndarray:
        if seed.start_segment is None:
            return pts[:0]
        x1, y1, x2, y2 = seed.start_segment
        p1 = np.asarray([x1, y1], dtype=np.float32)
        p2 = np.asarray([x2, y2], dtype=np.float32)
        direction = p2 - p1
        length = float(np.linalg.norm(direction))
        if length < 1.0e-6:
            return pts[:0]
        direction /= length
        rel = pts - p1
        along = rel @ direction
        dist = np.abs(rel[:, 0] * direction[1] - rel[:, 1] * direction[0])
        gate = max(18.0, 0.07 * expected)
        region = dist < gate
        region &= along > -0.18 * expected
        region &= along < length + 0.18 * expected
        return pts[region]

    def _make_detection_from_geometry(
        self,
        left_line: Line,
        right_line: Line,
        start_segment: Tuple[float, float, float, float],
        expected: float,
        left_support: int,
        right_support: int,
        cross_support: int,
        total_support: int,
        message: str,
        confidence_scale: float = 1.0,
    ) -> RailDetection:
        start_x1, start_y1, start_x2, start_y2 = start_segment
        start_mid_y = 0.5 * (start_y1 + start_y2)
        y_ref = _clip(start_mid_y - 0.04 * self.bev_height, 0.35 * self.bev_height, 0.90 * self.bev_height)
        if self._x_at(left_line, y_ref) > self._x_at(right_line, y_ref):
            left_line, right_line = right_line, left_line

        left_ref = self._x_at(left_line, y_ref)
        right_ref = self._x_at(right_line, y_ref)
        spacing = right_ref - left_ref
        if spacing < self.min_spacing_fraction * self.bev_width or spacing > self.max_spacing_fraction * self.bev_width:
            return RailDetection(False, 0.0, message=f"{message}: spacing outside bounds")
        if abs(spacing - expected) > expected * max(self.hard_spacing_tolerance_fraction, 0.42):
            return RailDetection(False, 0.0, message=f"{message}: spacing rejected")

        cross_span = float(np.hypot(start_x2 - start_x1, start_y2 - start_y1))
        if cross_span < 0.58 * spacing:
            return RailDetection(False, 0.0, message=f"{message}: short crossbar")
        seg_min_x = min(start_x1, start_x2)
        seg_max_x = max(start_x1, start_x2)
        endpoint_error = (
            abs(self._x_at(left_line, start_mid_y) - seg_min_x)
            + abs(self._x_at(right_line, start_mid_y) - seg_max_x)
        ) / max(2.0 * spacing, 1.0)
        if endpoint_error > 0.34:
            return RailDetection(False, 0.0, message=f"{message}: crossbar endpoint mismatch")
        crossbar_quality = 1.0 - _clip(abs(cross_span - spacing) / max(expected * 0.65, 1.0), 0.0, 1.0)
        if crossbar_quality < 0.15:
            return RailDetection(False, 0.0, message=f"{message}: weak crossbar")

        y_look = max(self.lookahead_y_fraction * self.bev_height, y_ref - 0.38 * self.bev_height)
        center_near = 0.5 * (left_ref + right_ref)
        center_far = 0.5 * (self._x_at(left_line, y_look) + self._x_at(right_line, y_look))
        heading_error = float(np.arctan2(center_far - center_near, y_ref - y_look))
        center_error_px = float(center_near - 0.5 * self.bev_width)
        start_y = max(start_y1, start_y2)
        start_distance = max(0.0, (self.bev_height - start_y) * self.meters_per_pixel)

        spacing_quality = 1.0 - _clip(abs(spacing - expected) / max(expected * self.spacing_tolerance_fraction, 1.0), 0.0, 1.0)
        side_balance = 1.0 - _clip(abs(left_support - right_support) / max(left_support + right_support, 1.0), 0.0, 1.0)
        support_quality = _clip((left_support + right_support + cross_support) / max(0.40 * total_support, 1.0), 0.0, 1.0)
        endpoint_quality = 1.0 - _clip(endpoint_error / 0.34, 0.0, 1.0)
        heading_quality = 1.0 - _clip(abs(heading_error) / np.deg2rad(38.0), 0.0, 1.0)
        center_quality = 1.0 - _clip(abs(center_error_px) / (0.5 * self.bev_width), 0.0, 1.0)
        confidence = confidence_scale * _clip(
            0.24 * spacing_quality
            + 0.22 * crossbar_quality
            + 0.18 * support_quality
            + 0.12 * side_balance
            + 0.10 * endpoint_quality
            + 0.09 * heading_quality
            + 0.05 * center_quality,
            0.0,
            1.0,
        )
        return RailDetection(
            ok=confidence >= 0.31,
            confidence=float(confidence),
            center_error_px=center_error_px,
            center_error_m=float(center_error_px * self.meters_per_pixel),
            heading_error_rad=heading_error,
            rail_spacing_px=float(spacing),
            left_line=left_line,
            right_line=right_line,
            start_segment=tuple(float(v) for v in start_segment),
            start_y_px=float(start_y),
            start_distance_m=float(start_distance),
            message=message,
        )

    def _fit_u_component(self, xs: np.ndarray, ys: np.ndarray, area: int, expected: float) -> RailDetection:
        pts = np.column_stack((xs, ys)).astype(np.float32)
        center = np.mean(pts, axis=0)
        centered = pts - center
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        long_axis = eigvecs[:, int(np.argmax(eigvals))].astype(np.float32)
        long_axis /= max(float(np.linalg.norm(long_axis)), 1.0e-6)
        lat_axis = np.array([-long_axis[1], long_axis[0]], dtype=np.float32)

        s = centered @ long_axis
        l = centered @ lat_axis
        s_min, s_max = float(np.percentile(s, 1.0)), float(np.percentile(s, 99.0))
        l_min, l_max = float(np.percentile(l, 2.0)), float(np.percentile(l, 98.0))
        length = s_max - s_min
        rough_spacing = l_max - l_min
        if length < 0.45 * self.bev_height or rough_spacing < 0.45 * expected:
            return RailDetection(False, 0.0, message="u-shape too small")
        if abs(rough_spacing - expected) > expected * max(self.hard_spacing_tolerance_fraction, 0.42):
            return RailDetection(False, 0.0, message="u-shape spacing rejected")

        # The rail beginning is the end of the U closest to the robot, i.e. the
        # end with larger image y in bird view.
        end_a = center + s_min * long_axis
        end_b = center + s_max * long_axis
        start_s = s_min if end_a[1] > end_b[1] else s_max
        far_s = s_max if start_s == s_min else s_min
        away_sign = 1.0 if far_s > start_s else -1.0

        side_region = (s - start_s) * away_sign > 0.22 * length
        side_region &= (s - start_s) * away_sign < 0.88 * length
        if np.count_nonzero(side_region) < 300:
            return RailDetection(False, 0.0, message="not enough u side pixels")
        side_l = l[side_region]
        side_offsets = self._estimate_u_side_offsets(side_l, expected)
        if side_offsets is None:
            return RailDetection(False, 0.0, message="u side peaks rejected")
        left_l, right_l = side_offsets
        spacing = right_l - left_l
        if abs(spacing - expected) > expected * max(self.hard_spacing_tolerance_fraction, 0.42):
            return RailDetection(False, 0.0, message="u side spacing rejected")

        start_region = np.abs(s - start_s) < max(0.12 * length, 35.0)
        start_l = l[start_region]
        if start_l.size < 80:
            return RailDetection(False, 0.0, message="not enough crossbar pixels")
        start_width = float(np.percentile(start_l, 96.0) - np.percentile(start_l, 4.0))
        crossbar_quality = 1.0 - _clip(abs(start_width - spacing) / max(expected * 0.65, 1.0), 0.0, 1.0)
        if crossbar_quality < 0.18:
            return RailDetection(False, 0.0, message="weak u crossbar")

        rail_gate = max(16.0, 0.075 * expected)
        side_left_region = side_region & (np.abs(l - left_l) < rail_gate)
        side_right_region = side_region & (np.abs(l - right_l) < rail_gate)
        left_points = pts[side_left_region]
        right_points = pts[side_right_region]
        min_side_pixels = max(90, int(0.035 * len(pts)))
        if len(left_points) < min_side_pixels or len(right_points) < min_side_pixels:
            return RailDetection(False, 0.0, message="not enough refined side pixels")

        left_line = self._fit_huber_line(left_points)
        right_line = self._fit_huber_line(right_points)
        if left_line is None or right_line is None:
            return RailDetection(False, 0.0, message="refined u side lines degenerate")

        cross_gate = max(22.0, 0.08 * length)
        cross_region = np.abs(s - start_s) < cross_gate
        cross_region &= l > left_l - 1.15 * rail_gate
        cross_region &= l < right_l + 1.15 * rail_gate
        cross_points = pts[cross_region]
        if len(cross_points) < max(80, int(0.025 * len(pts))):
            return RailDetection(False, 0.0, message="not enough refined crossbar pixels")
        start_segment = self._fit_huber_segment(cross_points, min_length=0.42 * expected)
        if start_segment is None:
            return RailDetection(False, 0.0, message="refined crossbar degenerate")

        start_x1, start_y1, start_x2, start_y2 = start_segment
        start_mid_y = 0.5 * (start_y1 + start_y2)
        y_ref = _clip(start_mid_y - 0.04 * self.bev_height, 0.35 * self.bev_height, 0.90 * self.bev_height)
        if self._x_at(left_line, y_ref) > self._x_at(right_line, y_ref):
            left_line, right_line = right_line, left_line

        left_ref = self._x_at(left_line, y_ref)
        right_ref = self._x_at(right_line, y_ref)
        spacing = right_ref - left_ref
        if spacing < self.min_spacing_fraction * self.bev_width or spacing > self.max_spacing_fraction * self.bev_width:
            return RailDetection(False, 0.0, message="refined u spacing outside bounds")
        if abs(spacing - expected) > expected * max(self.hard_spacing_tolerance_fraction, 0.42):
            return RailDetection(False, 0.0, message="refined u spacing rejected")

        cross_span = float(np.hypot(start_x2 - start_x1, start_y2 - start_y1))
        if cross_span < 0.58 * spacing:
            return RailDetection(False, 0.0, message="short refined u crossbar")
        seg_min_x = min(start_x1, start_x2)
        seg_max_x = max(start_x1, start_x2)
        endpoint_error = (
            abs(self._x_at(left_line, start_mid_y) - seg_min_x)
            + abs(self._x_at(right_line, start_mid_y) - seg_max_x)
        ) / max(2.0 * spacing, 1.0)
        if endpoint_error > 0.34:
            return RailDetection(False, 0.0, message="refined u crossbar endpoint mismatch")
        crossbar_quality = 1.0 - _clip(abs(cross_span - spacing) / max(expected * 0.65, 1.0), 0.0, 1.0)
        if crossbar_quality < 0.18:
            return RailDetection(False, 0.0, message="weak refined u crossbar")

        y_look = max(self.lookahead_y_fraction * self.bev_height, y_ref - 0.38 * self.bev_height)
        center_near = 0.5 * (left_ref + right_ref)
        center_far = 0.5 * (self._x_at(left_line, y_look) + self._x_at(right_line, y_look))
        heading_error = float(np.arctan2(center_far - center_near, y_ref - y_look))
        center_error_px = float(center_near - 0.5 * self.bev_width)
        start_y = max(start_y1, start_y2)
        start_distance = max(0.0, (self.bev_height - start_y) * self.meters_per_pixel)

        spacing_quality = 1.0 - _clip(abs(spacing - expected) / max(expected * self.spacing_tolerance_fraction, 1.0), 0.0, 1.0)
        length_quality = _clip(length / (0.88 * self.bev_height), 0.0, 1.0)
        area_quality = _clip(area / 26000.0, 0.0, 1.0)
        heading_quality = 1.0 - _clip(abs(heading_error) / np.deg2rad(38.0), 0.0, 1.0)
        center_quality = 1.0 - _clip(abs(center_error_px) / (0.5 * self.bev_width), 0.0, 1.0)
        support_quality = _clip((len(left_points) + len(right_points) + len(cross_points)) / max(0.62 * len(pts), 1.0), 0.0, 1.0)
        endpoint_quality = 1.0 - _clip(endpoint_error / 0.34, 0.0, 1.0)
        confidence = _clip(
            0.22 * spacing_quality
            + 0.22 * crossbar_quality
            + 0.15 * support_quality
            + 0.14 * length_quality
            + 0.08 * endpoint_quality
            + 0.11 * area_quality
            + 0.05 * heading_quality
            + 0.03 * center_quality,
            0.0,
            1.0,
        )
        return RailDetection(
            ok=confidence >= 0.34,
            confidence=float(confidence),
            center_error_px=center_error_px,
            center_error_m=float(center_error_px * self.meters_per_pixel),
            heading_error_rad=heading_error,
            rail_spacing_px=float(spacing),
            left_line=left_line,
            right_line=right_line,
            start_segment=tuple(float(v) for v in start_segment),
            start_y_px=float(start_y),
            start_distance_m=float(start_distance),
            message="u-shape-refined",
        )

    @staticmethod
    def _fit_huber_line(points: np.ndarray) -> Optional[Line]:
        if len(points) < 2:
            return None
        pts_src = PipeRailDetector._sample_fit_points(points)
        pts = pts_src.astype(np.float32).reshape((-1, 1, 2))
        vx, vy, x0, y0 = [float(v) for v in cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)]
        if abs(vy) < 1.0e-3:
            return None
        slope = vx / vy
        intercept = x0 - slope * y0
        return float(slope), float(intercept)

    @staticmethod
    def _sample_fit_points(points: np.ndarray, max_points: int = 2600) -> np.ndarray:
        if len(points) <= max_points:
            return points
        indices = np.linspace(0, len(points) - 1, max_points).astype(np.int32)
        return points[indices]

    def _estimate_u_side_offsets(self, lateral_values: np.ndarray, expected: float) -> Optional[Tuple[float, float]]:
        if lateral_values.size < 80:
            return None
        lo, hi = np.percentile(lateral_values, [1.0, 99.0])
        if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 0.35 * expected:
            return None

        bin_count = int(_clip((hi - lo) / 4.5, 48.0, 132.0))
        hist, edges = np.histogram(lateral_values, bins=bin_count, range=(float(lo), float(hi)))
        hist = hist.astype(np.float32)
        if float(np.max(hist)) <= 0.0:
            return None
        smooth = np.convolve(hist, np.asarray([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float32), mode="same")
        centers = 0.5 * (edges[:-1] + edges[1:])

        peak_indices: List[int] = []
        for idx in range(1, len(smooth) - 1):
            if smooth[idx] >= smooth[idx - 1] and smooth[idx] >= smooth[idx + 1]:
                peak_indices.append(idx)
        if len(peak_indices) < 2:
            left = float(np.percentile(lateral_values, 12.0))
            right = float(np.percentile(lateral_values, 88.0))
            return (left, right) if right > left else None

        peak_indices = sorted(peak_indices, key=lambda i: float(smooth[i]), reverse=True)[:14]
        best: Optional[Tuple[float, float]] = None
        best_score = -1.0e9
        tolerance = max(self.hard_spacing_tolerance_fraction, 0.42)
        for i, idx_a in enumerate(peak_indices):
            for idx_b in peak_indices[i + 1 :]:
                left = float(min(centers[idx_a], centers[idx_b]))
                right = float(max(centers[idx_a], centers[idx_b]))
                spacing = right - left
                spacing_error = abs(spacing - expected) / max(expected, 1.0)
                if spacing_error > tolerance:
                    continue
                support = float(smooth[idx_a] + smooth[idx_b]) / max(float(np.max(smooth)), 1.0)
                balance = 1.0 - abs(float(smooth[idx_a] - smooth[idx_b])) / max(float(smooth[idx_a] + smooth[idx_b]), 1.0)
                score = support + 0.35 * balance - 1.8 * spacing_error
                if score > best_score:
                    best_score = score
                    best = (left, right)
        if best is not None:
            return best

        left = float(np.percentile(lateral_values, 12.0))
        right = float(np.percentile(lateral_values, 88.0))
        if right <= left:
            return None
        if abs((right - left) - expected) > expected * tolerance:
            return None
        return left, right

    @staticmethod
    def _fit_huber_segment(points: np.ndarray, min_length: float) -> Optional[Tuple[float, float, float, float]]:
        if len(points) < 2:
            return None
        pts = PipeRailDetector._sample_fit_points(points).astype(np.float32)
        fit_pts = pts.reshape((-1, 1, 2))
        vx, vy, x0, y0 = [float(v) for v in cv2.fitLine(fit_pts, cv2.DIST_L2, 0, 0.01, 0.01)]
        direction = np.asarray([vx, vy], dtype=np.float32)
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-6:
            return None
        direction /= norm
        origin = np.asarray([x0, y0], dtype=np.float32)
        coord = (pts - origin) @ direction
        q1, q2 = np.percentile(coord, [4.0, 96.0])
        if float(q2 - q1) < min_length:
            return None
        p1 = origin + float(q1) * direction
        p2 = origin + float(q2) * direction
        return float(p1[0]), float(p1[1]), float(p2[0]), float(p2[1])

    @staticmethod
    def _line_from_points(p1: np.ndarray, p2: np.ndarray) -> Optional[Line]:
        dy = float(p2[1] - p1[1])
        if abs(dy) < 1.0e-3:
            return None
        slope = float((p2[0] - p1[0]) / dy)
        intercept = float(p1[0] - slope * p1[1])
        return slope, intercept

    def _expected_spacing_px(self) -> float:
        if self.expected_spacing_px > 0.0:
            return self.expected_spacing_px
        if self.expected_spacing_m > 0.0 and self.meters_per_pixel > 0.0:
            return self.expected_spacing_m / self.meters_per_pixel
        return 0.34 * self.bev_width

    def _fit_cluster(self, candidates: Iterable[Dict[str, Any]], seed: Dict[str, Any]) -> Tuple[Optional[Line], float]:
        seed_x = seed["x_ref"]
        seed_angle = seed["angle"]
        max_angle_diff = np.deg2rad(self.max_parallel_angle_deg)
        points: List[Tuple[float, float]] = []
        weight = 0.0
        for cand in candidates:
            if abs(cand["x_ref"] - seed_x) > self.cluster_window_px:
                continue
            if abs(cand["angle"] - seed_angle) > max_angle_diff:
                continue
            x1, y1, x2, y2 = cand["segment"]
            points.append((x1, y1))
            points.append((x2, y2))
            weight += cand["length"]

        if len(points) < 4:
            return seed["line"], seed["length"]

        pts = np.asarray(points, dtype=np.float32).reshape((-1, 1, 2))
        vx, vy, x0, y0 = [float(v) for v in cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)]
        if abs(vy) < 1.0e-3:
            return None, 0.0
        slope = vx / vy
        intercept = x0 - slope * y0
        return (float(slope), float(intercept)), weight

    @staticmethod
    def _x_at(line: Line, y: float) -> float:
        slope, intercept = line
        return slope * y + intercept

    def _draw_debug(
        self,
        bird: np.ndarray,
        mask: np.ndarray,
        candidates: List[Dict[str, Any]],
        detection: RailDetection,
    ) -> np.ndarray:
        overlay = bird.copy()
        mask_color = np.zeros_like(overlay)
        mask_color[:, :, 2] = mask
        overlay = cv2.addWeighted(overlay, 0.82, mask_color, 0.28, 0.0)

        for cand in candidates:
            x1, y1, x2, y2 = [int(round(v)) for v in cand["segment"]]
            cv2.line(overlay, (x1, y1), (x2, y2), (60, 160, 255), 1, cv2.LINE_AA)

        y_ref = self.reference_y_fraction * self.bev_height
        cv2.line(overlay, (self.bev_width // 2, 0), (self.bev_width // 2, self.bev_height), (255, 255, 255), 1)

        if detection.left_line is not None and detection.right_line is not None:
            for line, color in ((detection.left_line, (0, 255, 0)), (detection.right_line, (0, 255, 0))):
                p1 = (int(round(self._x_at(line, self.roi_top_fraction * self.bev_height))), int(round(self.roi_top_fraction * self.bev_height)))
                p2 = (int(round(self._x_at(line, self.roi_bottom_fraction * self.bev_height))), int(round(self.roi_bottom_fraction * self.bev_height)))
                cv2.line(overlay, p1, p2, color, 4, cv2.LINE_AA)

            center_top = int(round(0.5 * (
                self._x_at(detection.left_line, self.roi_top_fraction * self.bev_height)
                + self._x_at(detection.right_line, self.roi_top_fraction * self.bev_height)
            )))
            center_bottom = int(round(0.5 * (
                self._x_at(detection.left_line, self.roi_bottom_fraction * self.bev_height)
                + self._x_at(detection.right_line, self.roi_bottom_fraction * self.bev_height)
            )))
            cv2.line(
                overlay,
                (center_top, int(round(self.roi_top_fraction * self.bev_height))),
                (center_bottom, int(round(self.roi_bottom_fraction * self.bev_height))),
                (255, 220, 80),
                2,
                cv2.LINE_AA,
            )
            center_ref = int(round(0.5 * (self._x_at(detection.left_line, y_ref) + self._x_at(detection.right_line, y_ref))))
            cv2.circle(overlay, (center_ref, int(round(y_ref))), 6, (255, 220, 80), -1, cv2.LINE_AA)

        if detection.start_segment is not None:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.start_segment]
            cv2.line(overlay, (x1, y1), (x2, y2), (255, 0, 255), 5, cv2.LINE_AA)
            cv2.circle(overlay, (int(round(0.5 * (x1 + x2))), int(round(0.5 * (y1 + y2)))), 7, (255, 0, 255), -1, cv2.LINE_AA)

        status = (
            f"ok={int(detection.ok)} conf={detection.confidence:.2f} "
            f"err={detection.center_error_m:+.3f}m head={np.rad2deg(detection.heading_error_rad):+.1f}deg "
            f"start={detection.start_distance_m:.2f}m"
        )
        cv2.rectangle(overlay, (8, 8), (min(self.bev_width - 8, 610), 46), (0, 0, 0), -1)
        cv2.putText(overlay, status, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA)
        return overlay

    def _draw_source_debug(
        self,
        source: np.ndarray,
        mask: np.ndarray,
        source_candidates: List[Dict[str, Any]],
        detection: RailDetection,
    ) -> np.ndarray:
        overlay = source.copy()
        mask_color = np.zeros_like(overlay)
        mask_color[:, :, 2] = mask
        overlay = cv2.addWeighted(overlay, 0.84, mask_color, 0.24, 0.0)

        for cand in source_candidates:
            x1, y1, x2, y2 = [int(round(v)) for v in cand["source_segment"]]
            cv2.line(overlay, (x1, y1), (x2, y2), (60, 160, 255), 1, cv2.LINE_AA)

        if detection.left_line is not None and detection.right_line is not None:
            inv_h = self._require_inverse_homography(source)
            for line in (detection.left_line, detection.right_line):
                y1 = self.roi_top_fraction * self.bev_height
                y2 = self.roi_bottom_fraction * self.bev_height
                pts = np.asarray(
                    [[[self._x_at(line, y1), y1]], [[self._x_at(line, y2), y2]]],
                    dtype=np.float32,
                )
                src_pts = cv2.perspectiveTransform(pts, inv_h).reshape((-1, 2))
                p1 = tuple(np.round(src_pts[0]).astype(int))
                p2 = tuple(np.round(src_pts[1]).astype(int))
                cv2.line(overlay, p1, p2, (0, 255, 0), 4, cv2.LINE_AA)

        if detection.start_segment is not None:
            inv_h = self._require_inverse_homography(source)
            x1, y1, x2, y2 = detection.start_segment
            pts = np.asarray([[[x1, y1]], [[x2, y2]]], dtype=np.float32)
            src_pts = cv2.perspectiveTransform(pts, inv_h).reshape((-1, 2))
            p1 = tuple(np.round(src_pts[0]).astype(int))
            p2 = tuple(np.round(src_pts[1]).astype(int))
            cv2.line(overlay, p1, p2, (255, 0, 255), 5, cv2.LINE_AA)

        status = (
            f"source detect  ok={int(detection.ok)} conf={detection.confidence:.2f} "
            f"err={detection.center_error_m:+.3f}m start={detection.start_distance_m:.2f}m"
        )
        cv2.rectangle(overlay, (8, 8), (min(source.shape[1] - 8, 720), 46), (0, 0, 0), -1)
        cv2.putText(overlay, status, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA)
        return overlay
