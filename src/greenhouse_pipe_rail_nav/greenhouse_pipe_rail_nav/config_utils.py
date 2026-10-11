from __future__ import annotations

from typing import Any, Dict


def load_detector_config(path: str) -> Dict[str, Any]:
    if not path:
        return {}

    import yaml

    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if "detector" in data:
        return _normalize_detector_config(data["detector"] or {})

    node = data.get("greenhouse_pipe_rail_autodock", {})
    ros_params = node.get("ros__parameters", {})
    return _normalize_detector_config(ros_params.get("detector", {}) or {})


def _normalize_detector_config(config: Dict[str, Any]) -> Dict[str, Any]:
    cfg = dict(config)
    points = cfg.get("source_points")
    if isinstance(points, list) and len(points) == 8 and not isinstance(points[0], list):
        cfg["source_points"] = [points[i : i + 2] for i in range(0, 8, 2)]
    return cfg
