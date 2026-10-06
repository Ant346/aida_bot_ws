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

# Same process graph as RViz. The Humble camera services are a different distro
# and this Jazzy RViz will not receive their images.
serial="${REALSENSE_D405_SERIAL:-218622278337}"
serial="${serial#_}"
# D405 is on USB 2. 848x480 color+depth overflows the link and no frames arrive.
# 480x270x15 for both streams fits. initial_reset clears a stuck USB stream.
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=d405 \
  camera_name:=d405 \
  device_type:=d405 \
  serial_no:=_"${serial}" \
  initial_reset:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  depth_module.color_profile:=480x270x15 \
  depth_module.depth_profile:=480x270x15 \
  >/tmp/novnc/d405.log 2>&1 &
# ZED-M UVC frames are torn unless uvcvideo is loaded with quirks=128
# (UVC_QUIRK_FIX_BANDWIDTH). 1344x376 is the VGA side-by-side mode.
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/zedm \
  -r __node:=zedm_camera \
  -p video_device:=/dev/v4l/by-id/usb-Technologies__Inc._ZED-M-video-index0 \
  -p pixel_format:=YUYV \
  -p image_size:=[1344,376] \
  -p camera_frame_id:=zedm_camera_optical_frame \
  >/tmp/novnc/zedm.log 2>&1 &

cd /ws
colcon build --symlink-install
source install/setup.bash
exec ros2 launch greenhouse_pipe_rail_nav pipe_rail_rviz.launch.py
