"""Собрать фиксированные TF в base_link из выгрузки СК Fusion.

Родитель — СК agrobot main (геометрический центр).
Оси этой СК уже ROS: X вперёд, Y влево, Z вверх.
Кадры камер в файле — те же оси, начало в оптическом центре.
Оптический кадр (X вправо, Y вниз, Z вперёд) — ребёнок кадра камеры.
"""

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "framesd435.json"
URDF = ROOT / "agrobot_frames.urdf"

WHEELS = {
    "upper left": "wheel_fl",
    "upper right": "wheel_fr",
    "bottom left": "wheel_rl",
    "bottom right": "wheel_rr",
}


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def mul_t(axes, v):
    """axes: три оси кадра в координатах Fusion, по строкам x, y, z."""
    return [dot(axes[0], v), dot(axes[1], v), dot(axes[2], v)]


def rpy_from_axes(child_axes_in_parent):
    """child_axes: строки — X, Y, Z ребёнка в координатах родителя. ROS rpy, фикс. XYZ."""
    r = [
        [child_axes_in_parent[0][0], child_axes_in_parent[1][0], child_axes_in_parent[2][0]],
        [child_axes_in_parent[0][1], child_axes_in_parent[1][1], child_axes_in_parent[2][1]],
        [child_axes_in_parent[0][2], child_axes_in_parent[1][2], child_axes_in_parent[2][2]],
    ]
    sy = math.sqrt(r[0][0] ** 2 + r[1][0] ** 2)
    if sy > 1e-9:
        roll = math.atan2(r[2][1], r[2][2])
        pitch = math.atan2(-r[2][0], sy)
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = math.atan2(-r[1][2], r[1][1])
        pitch = math.atan2(-r[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def link_name(frame):
    name = frame["name"]
    component = frame.get("component", "")
    path = frame.get("path", "")
    if name == "agrobot main":
        return "base_link"
    if name == "agrobot forward":
        return "front_hit"
    if name in WHEELS:
        return WHEELS[name]
    if component == "RealSense_D405" or "RealSense_D405" in path:
        return "d405_link"
    if "D435" in component or "D435" in path:
        return "d435_link"
    safe = "".join(ch if ch.isalnum() else "_" for ch in name)
    return safe


def is_camera(child):
    return child in ("d405_link", "d435_link")


def in_root_space(payload, frame):
    if frame.get("space") == "root" or payload.get("space") == "root":
        return True
    return frame["component"] == payload["root_component"]


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else SRC
    payload = json.loads(src.read_text(encoding="utf-8"))
    frames = payload["frames"]
    base = next(f for f in frames if f["name"] == "agrobot main")
    base_axes = [base["x_axis"], base["y_axis"], base["z_axis"]]

    trusted = []
    skipped = []
    for frame in frames:
        if frame is base:
            continue
        if frame["name"].startswith("ZEDm") or frame.get("component") == "ZEDM":
            skipped.append(frame)
            continue
        if in_root_space(payload, frame):
            trusted.append(frame)
        else:
            skipped.append(frame)

    joints = []
    for frame in trusted:
        child = link_name(frame)
        delta_mm = sub(frame["origin_mm"], base["origin_mm"])
        xyz_m = [c / 1000.0 for c in mul_t(base_axes, delta_mm)]
        child_axes = [mul_t(base_axes, frame["x_axis"]), mul_t(base_axes, frame["y_axis"]), mul_t(base_axes, frame["z_axis"])]
        roll, pitch, yaw = rpy_from_axes(child_axes)

        def snap(v):
            return 0.0 if abs(v) < 1e-9 else v

        xyz_m = [snap(v) for v in xyz_m]
        joints.append((child, frame, xyz_m, (snap(roll), snap(pitch), snap(yaw))))

    optical_rpy = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
    lines = [
        '<?xml version="1.0"?>',
        '<robot name="agrobot">',
        "  <!-- base_link: геометрический центр, X вперёд, Y влево, Z вверх -->",
        '  <link name="base_link"/>',
    ]
    for child, frame, xyz, rpy in joints:
        lines.append(f'  <!-- {frame["path"]} -->')
        lines.append(f'  <link name="{child}"/>')
        lines.append(f'  <joint name="{child}_joint" type="fixed">')
        lines.append('    <parent link="base_link"/>')
        lines.append(f'    <child link="{child}"/>')
        lines.append(
            '    <origin xyz="{:.9f} {:.9f} {:.9f}" rpy="{:.9f} {:.9f} {:.9f}"/>'.format(
                *xyz, *rpy
            )
        )
        lines.append("  </joint>")
        if is_camera(child):
            optical = child + "_optical"
            lines.append(f'  <link name="{optical}"/>')
            lines.append(f'  <joint name="{optical}_joint" type="fixed">')
            lines.append(f'    <parent link="{child}"/>')
            lines.append(f'    <child link="{optical}"/>')
            lines.append(
                '    <origin xyz="0 0 0" rpy="{:.9f} {:.9f} {:.9f}"/>'.format(*optical_rpy)
            )
            lines.append("  </joint>")
    lines.append("</robot>")
    URDF.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"wrote {URDF}")
    print(f"root space frames used: {len(joints)}")
    for child, frame, xyz, rpy in joints:
        mm = [c * 1000.0 for c in xyz]
        deg = [n * 180.0 / math.pi for n in rpy]
        print(f"  {child}")
        print(f"    path {frame['path']}")
        print(f"    xyz_mm  {mm[0]:.4f} {mm[1]:.4f} {mm[2]:.4f}")
        print(f"    rpy_deg {deg[0]:.6f} {deg[1]:.6f} {deg[2]:.6f}")
    if skipped:
        print(f"skipped: {len(skipped)}")
        for frame in skipped:
            print(f"  {frame['name']}  {frame['path']}")

    by_name = {child: (xyz, rpy) for child, frame, xyz, rpy in joints}

    def dist_mm(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3))) * 1000.0

    print("baselines mm")
    if "d405_link" in by_name and "d435_link" in by_name:
        print(
            "  d405_link <-> d435_link: "
            f"{dist_mm(by_name['d405_link'][0], by_name['d435_link'][0]):.3f}"
        )


if __name__ == "__main__":
    main()
