#!/bin/bash
set -e

# X11 cookie in ~/.Xauthority is keyed by the host machine name; the container default
# hostname is a Docker id, so libX11 often fails with "Authorization required..."
# when DISPLAY=:0. Compose mounts host /etc/hostname here only for realsense/rviz-like services.
if [[ -r /run/host-etc-hostname ]]; then
    _hn=$(head -n1 /run/host-etc-hostname | tr -d ' \t\r')
    [[ -n "$_hn" ]] && hostname "$_hn" 2>/dev/null || true
    unset _hn
fi

# shellcheck source=/entrypoint-x11.bash
[[ -f /entrypoint-x11.bash ]] && source /entrypoint-x11.bash

if [[ -f /opt/ros/${ROS_DISTRO:-humble}/setup.bash ]]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

# Drop colcon build dirs for packages whose rosidl_adapter json was left corrupt
# (e.g. two containers interrupted a shared build on ./build).
_sanitize_rosidl_build_artifacts() {
    local f pkg
    shopt -s nullglob
    for f in /workspace/build/*/rosidl_adapter__arguments__*.json; do
        [[ -f "$f" ]] || continue
        if python3 -c 'import json,sys; json.load(open(sys.argv[1],"r",encoding="utf-8"))' "$f" 2>/dev/null; then
            continue
        fi
        pkg=$(basename "$(dirname "$f")")
        echo "⚠️  Invalid rosidl_adapter json in build/${pkg} — removing build+install for clean reconfigure"
        rm -rf "/workspace/build/${pkg}" "/workspace/install/${pkg}"
    done
    shopt -u nullglob
}

setup_workspace_structure() {
    cd /workspace
    mkdir -p src
    if [[ -d "ds4_driver_submodule/ds4_driver_msgs" ]] && [[ ! -e "src/ds4_driver_msgs" ]]; then
        ln -sf ../ds4_driver_submodule/ds4_driver_msgs src/ds4_driver_msgs
    fi
    if [[ -d "ds4_driver_submodule/ds4_driver" ]] && [[ ! -e "src/ds4_driver" ]]; then
        ln -sf ../ds4_driver_submodule/ds4_driver src/ds4_driver
    fi
    if [[ -d "src/odroid_node" ]]; then
        echo "✅ odroid_node in src/"
    fi
    if [[ -d "src/realsense_bringup" ]]; then
        echo "✅ realsense_bringup in src/"
    fi
    if [[ -d "src/realsense-ros" ]]; then
        echo "✅ realsense-ros in src/"
    fi
}

build_workspace() {
    cd /workspace
    setup_workspace_structure
    _sanitize_rosidl_build_artifacts
    if [[ -d "src" ]] && [[ -n "$(ls -A src/ 2>/dev/null)" ]]; then
        rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2 2>/dev/null || true
    fi
    # One invocation: colcon orders packages by dependency (msgs before nodes).
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
}

setup_workspace_structure

NEED_BUILD=false
if [[ ! -f /workspace/install/setup.bash ]]; then
    NEED_BUILD=true
else
    LATEST_SRC=$(find src/ -type f \( \
        -name "*.py" -o -name "*.xml" -o -name "*.msg" -o -name "*.srv" -o \
        -name "CMakeLists.txt" \) 2>/dev/null | xargs stat -c %Y 2>/dev/null | sort -n | tail -1 || echo 0)
    LATEST_INSTALL=$(stat -c %Y /workspace/install/setup.bash 2>/dev/null || echo 0)
    if [[ -n "$LATEST_SRC" ]] && [[ "$LATEST_SRC" =~ ^[0-9]+$ ]] && [[ "$LATEST_INSTALL" =~ ^[0-9]+$ ]] && [[ "$LATEST_SRC" -gt "$LATEST_INSTALL" ]]; then
        NEED_BUILD=true
    fi
fi

# symlink-install: install/*/local_setup.bash points into build/. If compose omits the
# build/ volume, symlinks are broken and "not found: ... local_setup.bash" appears.
if [[ -f /workspace/install/setup.bash ]] && [[ "$NEED_BUILD" != "true" ]]; then
    if find /workspace/install -maxdepth 6 -type l ! -exec test -e {} \; -print -quit 2>/dev/null | grep -q .; then
        echo "⚠️  install/ has broken symlinks (mount ./build:/workspace/build or rebuild). Rebuilding..."
        NEED_BUILD=true
    fi
fi

# Partial colcon: empty or stale install/realsense2_camera/ — directory may exist though build failed.
if [[ -f /workspace/install/setup.bash ]] && [[ "$NEED_BUILD" != "true" ]]; then
    rs_src_pkg="/workspace/src/realsense-ros/realsense2_camera/package.xml"
    rs_inst_pkg="/workspace/install/realsense2_camera/share/realsense2_camera/package.xml"
    if [[ -f "$rs_src_pkg" ]] && [[ ! -f "$rs_inst_pkg" ]]; then
        echo "⚠️  realsense2_camera not installed (no share/*/package.xml; partial colcon or failed CMake). Rebuilding..."
        NEED_BUILD=true
    fi
fi

# Several compose services share ./build and ./install; multiple colcon must not run in parallel.
# Use append (>>): opening with > truncates and can undermine advisory locking across containers.
if [[ "$NEED_BUILD" == "true" ]]; then
    mkdir -p /workspace
    touch /workspace/.colcon_build.lock
    echo "colcon: acquiring shared lock (other containers wait up to 30 min)..."
    (
        flock -w 1800 9 || { echo "colcon: flock timed out after 30min"; exit 1; }
        build_workspace
    ) 9>>/workspace/.colcon_build.lock
fi

if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  Workspace=/workspace"
echo "Examples:"
echo "  ros2 launch ds4_driver ds4_twist.launch.xml"
echo "  ros2 launch odroid_node odroid_driver.launch.py"
echo "  ros2 launch realsense_bringup d435_default.launch.py"

if [[ $# -eq 0 ]]; then
    exec /bin/bash
else
    exec "$@"
fi
