"""Собрать URDF из STEP: один корпус, четыре колеса, камеры.

Колесо — сборка «Сдвоенное мотор-колесо» (омни и ролик на одной оси).
Подвеска остаётся в корпусе. СК колеса — кадр из frames.json.
"""

import json
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
GLB = ROOT / "robot.glb"
FRAMES = ROOT / "frames.json"
OUT = ROOT / "agrobot_description"
MESH = OUT / "meshes"

OMNI_Y_MM = -77.75
ROLLER_Y_MM = 65.0


def load_glb(path):
    data = path.read_bytes()
    off = 12
    doc = None
    blob = None
    while off < len(data):
        clen, ctype = struct.unpack_from("<I4s", data, off)
        off += 8
        chunk = data[off : off + clen]
        off += clen
        if ctype == b"JSON":
            doc = json.loads(chunk)
        elif ctype[:3] == b"BIN":
            blob = chunk
    return doc, blob


def quat_to_mat(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def local_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T
    m = np.eye(4)
    if "scale" in node:
        m[0, 0], m[1, 1], m[2, 2] = node["scale"]
    if "rotation" in node:
        rot = np.eye(4)
        rot[:3, :3] = quat_to_mat(node["rotation"])
        m = rot @ m
    if "translation" in node:
        m[:3, 3] = node["translation"]
    return m


COMP = {5120: "b", 5121: "B", 5122: "h", 5123: "H", 5125: "I", 5126: "f"}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_accessor(doc, blob, index):
    acc = doc["accessors"][index]
    view = doc["bufferViews"][acc["bufferView"]]
    ctype = COMP[acc["componentType"]]
    ncomp = NCOMP[acc["type"]]
    count = acc["count"]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", 0)
    item = np.dtype(ctype).itemsize * ncomp
    if stride and stride != item:
        out = np.empty((count, ncomp), dtype=np.float64)
        for i in range(count):
            out[i] = np.frombuffer(blob, dtype=ctype, count=ncomp, offset=start + i * stride)
        return out
    raw = np.frombuffer(blob, dtype=ctype, count=count * ncomp, offset=start)
    return raw.reshape(count, ncomp).astype(np.float64)


def mesh_triangles(doc, blob, mesh_index, world):
    tris = []
    for prim in doc["meshes"][mesh_index]["primitives"]:
        pos = read_accessor(doc, blob, prim["attributes"]["POSITION"])
        homo = np.concatenate([pos, np.ones((len(pos), 1))], axis=1)
        xyz = (homo @ world.T)[:, :3]
        if "indices" in prim:
            idx = read_accessor(doc, blob, prim["indices"]).astype(np.int64).reshape(-1)
        else:
            idx = np.arange(len(xyz), dtype=np.int64)
        tri = xyz[idx].reshape(-1, 3, 3)
        tris.append(tri)
    if not tris:
        return np.zeros((0, 3, 3))
    return np.concatenate(tris, axis=0)


def write_stl(path, tris_m):
    path.parent.mkdir(parents=True, exist_ok=True)
    n = len(tris_m)
    with path.open("wb") as f:
        f.write(b"agrobot".ljust(80, b"\0"))
        f.write(struct.pack("<I", n))
        for tri in tris_m:
            a, b, c = tri
            nrm = np.cross(b - a, c - a)
            ln = np.linalg.norm(nrm)
            if ln > 0:
                nrm = nrm / ln
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))


def _component_ids(tris, quant=1e4):
    """Одинаковый id у треугольников одной оболочки. Стык ближе 0.1 мм склеивается."""
    flat = np.ascontiguousarray(tris.reshape(-1, 3))
    keys = np.round(flat * quant).astype(np.int64)
    keys -= keys.min(axis=0)
    packed = keys.view(np.dtype((np.void, keys.dtype.itemsize * 3))).ravel()
    _, inv = np.unique(packed, return_inverse=True)
    ntri = len(tris)
    parent = np.arange(ntri, dtype=np.int32)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    order = np.argsort(inv, kind="mergesort")
    tri_idx = order // 3
    vids = inv[order]
    starts = np.flatnonzero(np.diff(vids, prepend=vids[0] - 1))
    ends = np.append(starts[1:], len(vids))
    for s, e in zip(starts, ends):
        if e - s < 2:
            continue
        base = int(tri_idx[s])
        for k in range(s + 1, e):
            other = int(tri_idx[k])
            a, b = find(base), find(other)
            if a != b:
                parent[b] = a
    roots = np.fromiter((find(i) for i in range(ntri)), dtype=np.int32, count=ntri)
    _, comp = np.unique(roots, return_inverse=True)
    return comp


def _bbox_mm(tris, comp):
    ncomp = int(comp.max()) + 1
    pts = tris.reshape(-1, 3)
    labels = np.repeat(comp, 3)
    order = np.argsort(labels, kind="mergesort")
    pts_s = pts[order]
    labels_s = labels[order]
    starts = np.flatnonzero(np.diff(labels_s, prepend=-1))
    ends = np.append(starts[1:], len(labels_s))
    dims = np.zeros((ncomp, 3))
    for s, e in zip(starts, ends):
        chunk = pts_s[s:e]
        dims[int(labels_s[s])] = np.sort((chunk.max(0) - chunk.min(0)) * 1000.0)
    return dims


def is_fastener(dims_mm):
    """Винт, гайка или шайба: узкое сечение, не ролик и не подшипник."""
    small, mid, long = dims_mm
    if mid <= 14.0 and long <= 80.0:
        return True
    if small <= 2.5 and long <= 24.0:
        return True
    return False


def drop_fasteners(tris):
    if len(tris) == 0:
        return tris
    comp = _component_ids(tris)
    dims = _bbox_mm(tris, comp)
    drop = np.fromiter((is_fastener(dims[i]) for i in comp), dtype=bool, count=len(tris))
    if not drop.any():
        return tris
    return tris[~drop]


def to_frame(points, origin, axes):
    d = points - origin
    return np.column_stack([d @ axes[0], d @ axes[1], d @ axes[2]])


def rpy_from_axes(axes_in_parent):
    r = np.column_stack(axes_in_parent)
    sy = np.hypot(r[0, 0], r[1, 0])
    if sy > 1e-9:
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = np.arctan2(r[1, 0], r[0, 0])
    else:
        roll = np.arctan2(-r[1, 2], r[1, 1])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def snap(v):
    return 0.0 if abs(v) < 1e-9 else float(v)


def main():
    doc, blob = load_glb(GLB)
    nodes = doc["nodes"]
    parents = {}
    children = {}
    for i, node in enumerate(nodes):
        for c in node.get("children", []):
            parents[c] = i
            children.setdefault(i, []).append(c)

    local = [local_matrix(n) for n in nodes]
    world = [None] * len(nodes)

    def world_of(i):
        if world[i] is None:
            p = parents.get(i)
            world[i] = local[i] if p is None else world_of(p) @ local[i]
        return world[i]

    for i in range(len(nodes)):
        world_of(i)

    def descendants(i):
        out = []
        stack = [i]
        while stack:
            cur = stack.pop()
            out.append(cur)
            stack.extend(children.get(cur, []))
        return out

    def ancestor_names(i):
        names = []
        while i is not None:
            names.append(nodes[i].get("name", ""))
            i = parents.get(i)
        return names

    frames = json.loads(FRAMES.read_text(encoding="utf-8"))["frames"]
    by_name = {}
    for fr in frames:
        by_name.setdefault(fr["name"], []).append(fr)

    base = by_name["agrobot main"][0]
    base_origin = np.array(base["origin_mm"], dtype=float)
    base_axes = [np.array(v, dtype=float) for v in (base["x_axis"], base["y_axis"], base["z_axis"])]

    wheel_defs = {
        "wheel_fl": by_name["upper left"][0],
        "wheel_fr": by_name["upper right"][0],
        "wheel_rl": by_name["bottom left"][0],
        "wheel_rr": by_name["bottom right"][0],
    }
    wheel_origin = {k: np.array(v["origin_mm"], dtype=float) for k, v in wheel_defs.items()}
    wheel_axes = {
        k: [np.array(a, dtype=float) for a in (v["x_axis"], v["y_axis"], v["z_axis"])]
        for k, v in wheel_defs.items()
    }

    zed = next(fr for fr in by_name["ZEDm center"] if "ZEDM:2" in fr["path"])
    d405 = by_name["User Coordinate System1"][0]
    front = by_name["agrobot forward"][0]

    wheel_roots = [
        i for i, n in enumerate(nodes) if n.get("name", "").startswith("Сдвоенное мотор-колесо")
    ]
    buckets = {name: [] for name in wheel_defs}
    buckets.update({"body": [], "d405": [], "zed": []})
    extra = []

    def subtree_centroid(root_i):
        pts = []
        for i in descendants(root_i):
            mesh = nodes[i].get("mesh")
            if mesh is None:
                continue
            tris = mesh_triangles(doc, blob, mesh, world_of(i))
            if len(tris):
                pts.append(tris.reshape(-1, 3))
        if not pts:
            return None
        return np.concatenate(pts, axis=0).mean(axis=0)

    probe = None
    for root_i in wheel_roots:
        probe = subtree_centroid(root_i)
        if probe is not None:
            break
    # STEP в миллиметрах, а сетка из конвертера уже в метрах.
    mm_to_mesh = 0.001 if probe is not None and np.linalg.norm(probe) < 20 else 1.0
    print("mm_to_mesh", mm_to_mesh)

    assigned = {}
    print("wheel modules")
    for root_i in wheel_roots:
        cen = subtree_centroid(root_i)
        if cen is None:
            print("  empty", nodes[root_i].get("name"))
            continue
        best = min(
            wheel_defs,
            key=lambda k: np.linalg.norm(cen - wheel_origin[k] * mm_to_mesh),
        )
        assigned[root_i] = best
        dist_mm = np.linalg.norm(cen - wheel_origin[best] * mm_to_mesh) / mm_to_mesh
        print(
            f"  {nodes[root_i].get('name')} -> {best}  "
            f"mesh {cen[0]:.3f} {cen[1]:.3f} {cen[2]:.3f}  dist {dist_mm:.1f} mm"
        )

    mast_roots = [i for i, n in enumerate(nodes) if n.get("name", "").startswith("Мачта:")]
    print("masts", [nodes[i].get("name") for i in mast_roots])

    def bucket_of(i):
        node_i = i
        mast_hit = None
        while node_i is not None:
            name = nodes[node_i].get("name", "")
            if name.startswith("Горизонтальная направляющая"):
                return "body"
            if node_i in assigned:
                return assigned[node_i]
            if node_i in mast_roots:
                mast_hit = node_i
                break
            node_i = parents.get(node_i)
        if mast_hit is not None:
            return f"mast:{mast_hit}"
        names = ancestor_names(i)
        joined = " / ".join(names)
        if "zedm_universal_box_forward" in joined or "ZEDM" in joined or "ZED M" in joined:
            return "zed"
        if any(n.startswith("405:") or "RealSense_D405" in n for n in names):
            return "d405"
        if "camera supplier" in joined or "camera assembly" in joined:
            return "extra"
        return "body"

    extra_roots = [
        i for i, n in enumerate(nodes) if n.get("name", "").startswith("camera supplier:")
    ]
    extra_ids = {}
    for n, root_i in enumerate(extra_roots, start=1):
        for i in descendants(root_i):
            extra_ids[i] = f"cam_{n}"
        buckets[f"cam_{n}"] = []

    guide_boxes = {}
    mast_parts = []
    for i, node in enumerate(nodes):
        mesh = node.get("mesh")
        if mesh is None:
            continue
        if i in extra_ids:
            key = extra_ids[i]
        else:
            key = bucket_of(i)
            if key == "extra":
                continue
        tris = mesh_triangles(doc, blob, mesh, world_of(i))
        if len(tris) == 0:
            continue
        name = node.get("name", "")
        if name.startswith("Горизонтальная направляющая"):
            walk = i
            while walk is not None and walk not in mast_roots:
                walk = parents.get(walk)
            if walk is not None:
                pts = tris.reshape(-1, 3)
                guide_boxes.setdefault(walk, []).append((pts.min(axis=0), pts.max(axis=0)))
        if key.startswith("mast:"):
            mast_parts.append((key, tris))
        else:
            buckets.setdefault(key, []).append(tris)

    # Заглушки и крепёж, сидящие на направляющей, остаются в корпусе вместе с ней.
    pad = 0.03
    kept = 0
    for key, tris in mast_parts:
        mid = int(key.split(":", 1)[1])
        cen = tris.reshape(-1, 3).mean(axis=0)
        on_rail = False
        for mn, mx in guide_boxes.get(mid, []):
            if np.all(cen >= mn - pad) and np.all(cen <= mx + pad):
                on_rail = True
                break
        if on_rail:
            buckets["body"].append(tris)
            kept += len(tris)
        else:
            buckets.setdefault(key, []).append(tris)
    print(f"rail hardware triangles kept in body {kept}")

    def cat(key):
        parts = buckets.get(key) or []
        if not parts:
            return np.zeros((0, 3, 3))
        return np.concatenate(parts, axis=0)

    def frame_mesh(tris, origin_mm, axes):
        if len(tris) == 0:
            return tris
        origin = np.asarray(origin_mm, dtype=float) * mm_to_mesh
        flat = to_frame(tris.reshape(-1, 3), origin, axes).reshape(-1, 3, 3)
        if mm_to_mesh != 0.001:
            flat = flat * 0.001
        return drop_fasteners(flat)

    MESH.mkdir(parents=True, exist_ok=True)
    body = frame_mesh(cat("body"), base_origin, base_axes)
    write_stl(MESH / "body.stl", body)
    print(f"body triangles {len(body)}")

    wheel_tris = {}
    for key in wheel_defs:
        tri = frame_mesh(cat(key), wheel_origin[key], wheel_axes[key])
        wheel_tris[key] = tri
        write_stl(MESH / f"{key}.stl", tri)
        print(f"{key} triangles {len(tri)}")

    mast_ids = sorted({key.split(":", 1)[1] for key in buckets if key.startswith("mast:")})
    mast_named = []
    for mid in mast_ids:
        raw = cat(f"mast:{mid}")
        if len(raw) == 0:
            continue
        cen_mm = raw.reshape(-1, 3).mean(axis=0) / mm_to_mesh
        xyz = to_frame(cen_mm.reshape(1, 3), base_origin, base_axes)[0] * 0.001
        side = "left" if xyz[1] >= 0 else "right"
        link = f"mast_{side}"
        if any(item[0] == link for item in mast_named):
            link = f"{link}_{mid}"
        tri = frame_mesh(raw, base_origin, base_axes)
        write_stl(MESH / f"{link}.stl", tri)
        mast_named.append((link, xyz, len(tri)))
        print(f"{link} triangles {len(tri)} centroid y {xyz[1]:.3f} m")

    d405_origin = np.array(d405["origin_mm"], dtype=float)
    d405_axes = [np.array(v, dtype=float) for v in (d405["x_axis"], d405["y_axis"], d405["z_axis"])]
    zed_origin = np.array(zed["origin_mm"], dtype=float)
    zed_axes = [np.array(v, dtype=float) for v in (zed["x_axis"], zed["y_axis"], zed["z_axis"])]
    d405_mesh = frame_mesh(cat("d405"), d405_origin, d405_axes)
    zed_mesh = frame_mesh(cat("zed"), zed_origin, zed_axes)
    write_stl(MESH / "d405.stl", d405_mesh)
    write_stl(MESH / "zedm.stl", zed_mesh)
    print(f"d405 triangles {len(d405_mesh)}  zed triangles {len(zed_mesh)}")

    cam_keys = []
    for n in range(1, len(extra_roots) + 1):
        key = f"cam_{n}"
        raw = cat(key)
        if len(raw) == 0:
            continue
        cen = raw.reshape(-1, 3).mean(axis=0)
        cen_mm = cen / mm_to_mesh
        tri = frame_mesh(raw, cen_mm, base_axes)
        write_stl(MESH / f"{key}.stl", tri)
        cam_keys.append((key, cen_mm))
        cen_mm = cen / mm_to_mesh
        xyz = to_frame(cen_mm.reshape(1, 3), base_origin, base_axes)[0] * 0.001
        print(f"{key} triangles {len(tri)} base xyz m {xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f}")

    def pose_in_base(frame):
        origin = np.array(frame["origin_mm"], dtype=float)
        axes = [np.array(v, dtype=float) for v in (frame["x_axis"], frame["y_axis"], frame["z_axis"])]
        xyz = to_frame(origin.reshape(1, 3), base_origin, base_axes)[0] * 0.001
        child = [np.array([float(np.dot(b, ax)) for b in base_axes]) for ax in axes]
        rpy = rpy_from_axes(child)
        return [snap(float(v)) for v in xyz], tuple(snap(float(v)) for v in rpy)

    optical = (-np.pi / 2, 0.0, -np.pi / 2)
    lines = [
        '<?xml version="1.0"?>',
        '<robot name="agrobot">',
        "  <!-- base_link: геометрический центр. Корпус, направляющие опор и подвеска. Мачты — отдельные звенья. -->",
        '  <link name="base_link">',
        "    <visual>",
        '      <geometry><mesh filename="package://agrobot_description/meshes/body.stl"/></geometry>',
        "    </visual>",
        "  </link>",
    ]

    def add_fixed_into(buf, parent, child, xyz, rpy, mesh=None):
        buf.append(f'  <link name="{child}">')
        if mesh:
            buf.append("    <visual>")
            buf.append(
                f'      <geometry><mesh filename="package://agrobot_description/meshes/{mesh}"/></geometry>'
            )
            buf.append("    </visual>")
        buf.append("  </link>")
        buf.append(f'  <joint name="{child}_joint" type="fixed">')
        buf.append(f'    <parent link="{parent}"/>')
        buf.append(f'    <child link="{child}"/>')
        buf.append(
            '    <origin xyz="{:.9f} {:.9f} {:.9f}" rpy="{:.9f} {:.9f} {:.9f}"/>'.format(*xyz, *rpy)
        )
        buf.append("  </joint>")

    def add_fixed(parent, child, xyz, rpy, mesh=None):
        add_fixed_into(lines, parent, child, xyz, rpy, mesh)

    xyz, rpy = pose_in_base(front)
    add_fixed("base_link", "front_hit", xyz, rpy)

    for key in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        xyz, rpy = pose_in_base(wheel_defs[key])
        lines.append(f'  <link name="{key}">')
        lines.append("    <visual>")
        lines.append(
            f'      <geometry><mesh filename="package://agrobot_description/meshes/{key}.stl"/></geometry>'
        )
        lines.append("    </visual>")
        lines.append("  </link>")
        lines.append(f'  <joint name="{key}_joint" type="continuous">')
        lines.append('    <parent link="base_link"/>')
        lines.append(f'    <child link="{key}"/>')
        lines.append(
            '    <origin xyz="{:.9f} {:.9f} {:.9f}" rpy="{:.9f} {:.9f} {:.9f}"/>'.format(*xyz, *rpy)
        )
        lines.append('    <axis xyz="0 1 0"/>')
        lines.append("  </joint>")

    xyz, rpy = pose_in_base(d405)
    add_fixed("base_link", "d405_link", xyz, rpy, "d405.stl")
    add_fixed("d405_link", "d405_link_optical", (0, 0, 0), optical)

    xyz, rpy = pose_in_base(zed)
    add_fixed("base_link", "zedm_center", xyz, rpy, "zedm.stl")
    add_fixed("zedm_center", "zedm_left", (0, 0.0315, 0), (0, 0, 0))
    add_fixed("zedm_center", "zedm_right", (0, -0.0315, 0), (0, 0, 0))
    for name in ("zedm_center", "zedm_left", "zedm_right"):
        add_fixed(name, name + "_optical", (0, 0, 0), optical)

    for key, cen in cam_keys:
        xyz = to_frame(cen.reshape(1, 3), base_origin, base_axes)[0] * 0.001
        xyz = [snap(float(v)) for v in xyz]
        print(f"{key} parent base_link")
        add_fixed("base_link", key, xyz, (0, 0, 0), f"{key}.stl")

    urdf_lines = lines + ["</robot>"]
    xacro_lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="agrobot">',
    ]
    # lines[0] is xml header, lines[1] is robot tag
    xacro_lines.extend(lines[2:])
    xacro_lines.append("</robot>")
    lines = urdf_lines
    urdf = OUT / "urdf" / "agrobot.urdf"
    xacro = OUT / "urdf" / "agrobot.urdf.xacro"
    urdf.parent.mkdir(parents=True, exist_ok=True)
    urdf.write_text("\n".join(lines) + "\n", encoding="utf-8")
    xacro.write_text("\n".join(xacro_lines) + "\n", encoding="utf-8")
    print("wrote", xacro)
    (OUT / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3">
  <name>agrobot_description</name>
  <version>0.1.0</version>
  <description>Agrobot rigid body, wheels and cameras.</description>
  <maintainer email="dev@todo.local">dev</maintainer>
  <license>BSD</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
""",
        encoding="utf-8",
    )
    (OUT / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.5)
project(agrobot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
""",
        encoding="utf-8",
    )
    print("wrote", urdf)


if __name__ == "__main__":
    main()
