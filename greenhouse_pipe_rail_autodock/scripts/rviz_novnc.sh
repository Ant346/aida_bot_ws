#!/usr/bin/env bash
# RViz on a virtual X display, viewed in the browser via noVNC.
set -eo pipefail

export DISPLAY="${NOVNC_DISPLAY:-:99}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export NO_AT_BRIDGE=1

NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5900}"
disp_num="${DISPLAY#:}"
rm -f "/tmp/.X${disp_num}-lock" "/tmp/.X11-unix/X${disp_num}"
mkdir -p /tmp/novnc /tmp/.X11-unix

Xvfb "$DISPLAY" -screen 0 1600x900x24 -ac +extension GLX +render -noreset \
  >/tmp/novnc/xvfb.log 2>&1 &

ready=0
for _ in $(seq 1 50); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.1
done
if [[ "$ready" -ne 1 ]]; then
  echo "Xvfb did not start" >&2
  cat /tmp/novnc/xvfb.log >&2 || true
  exit 1
fi

openbox >/tmp/novnc/openbox.log 2>&1 &
x11vnc -display "$DISPLAY" -localhost -nopw -forever -shared \
  -rfbport "$VNC_PORT" -bg -o /tmp/novnc/x11vnc.log
/usr/share/novnc/utils/novnc_proxy --vnc "localhost:${VNC_PORT}" --listen "$NOVNC_PORT" \
  >/tmp/novnc/novnc.log 2>&1 &

(
  for _ in $(seq 1 90); do
    if wmctrl -l 2>/dev/null | grep -qi rviz; then
      wmctrl -l | grep -i rviz | awk '{print $1}' | while read -r wid; do
        wmctrl -i -r "$wid" -b add,maximized_vert,maximized_horz || true
      done
      exit 0
    fi
    sleep 1
  done
) &

ip=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "RViz in the browser: http://127.0.0.1:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"
if [[ -n "${ip}" ]]; then
  echo "From another computer: http://${ip}:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale"
fi

source /opt/ros/jazzy/setup.bash

if ! python3 -c "from apriltag import apriltag" >/dev/null 2>&1; then
  apt-get update
  apt-get install -y --no-install-recommends python3-apriltag
fi

# Same process graph as RViz. The Humble camera services are a different distro
# and this Jazzy RViz will not receive their images.
# Color only: depth on the same USB link drops the calibration frames.
# D435i on USB 2 fits 1280x720x15. D405 uses the same profile; override with
# REALSENSE_D405_COLOR_PROFILE if that camera is on USB 3.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-LARGE_DATA}"
d435_color="${REALSENSE_D435_COLOR_PROFILE:-1280x720x15}"
d405_color="${REALSENSE_D405_COLOR_PROFILE:-1280x720x15}"
d405_serial="${REALSENSE_D405_SERIAL:-218622278337}"
d405_serial="${d405_serial#_}"

# Factory color camera_info is remapped aside. TartanCalib intrinsics are
# published on the original topic. Backup: /ws/calib/d435_factory_color_1280x720.yaml
ros2 launch /ws/calib/d435_color.launch.py \
  camera_namespace:=d435 \
  camera_name:=d435 \
  device_type:=d435 \
  enable_color:=true \
  enable_depth:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  rgb_camera.color_profile:="$d435_color" \
  >/tmp/novnc/d435.log 2>&1 &
python3 /ws/calib/publish_d435_camera_info.py \
  >/tmp/novnc/d435_camera_info.log 2>&1 &

# D405 is pinned by serial so it cannot take the D435.
(
  sleep 2
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=d405 \
    camera_name:=d405 \
    device_type:=d405 \
    serial_no:="_${d405_serial}" \
    enable_color:=true \
    enable_depth:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    pointcloud.enable:=false \
    align_depth.enable:=false \
    depth_module.color_profile:="$d405_color"
) >/tmp/novnc/d405.log 2>&1 &

echo "Calibration frames, in this container: python3 /ws/capture_kalibr_frames.py"

cd /ws
colcon build --symlink-install
source install/setup.bash
exec ros2 launch greenhouse_pipe_rail_nav pipe_rail_rviz.launch.py
