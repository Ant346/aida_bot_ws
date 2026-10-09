# Greenhouse Pipe Rail Autodock

ROS 2 Jazzy package for visually aligning a mecanum robot with greenhouse pipe
rails and driving forward onto the rails. Rails are detected in the original RGB
image. The calibrated bird view is used after detection to put the detected
rails into the floor plane and compute metric control errors.

## What I saw in the sample frame

Extracted frame:

`analysis/screenshots/rgb_video_015_11s_greenhouse_pipe_rail.jpg`

The useful visual features are:

- The pipe rail is much darker than the gray floor.
- The two long side pipes are almost parallel in floor coordinates.
- The front cross pipe is strong but should not be used for steering, because
  it is perpendicular to the driving direction.
- The floor has tape, scuffs, glare, and shadows, so thresholding alone is not
  stable enough.

## Algorithm

1. Warp the RGB image into a bird-view image with four calibrated floor points.
2. In the full original image, enhance dark tubular objects with black-hat
   morphology and a local dark-pixel mask. There is no source-image trapezoid
   crop in the detector.
3. Run probabilistic Hough on the full original-image mask.
4. Project each source-image line segment into the calibrated floor/bird plane.
5. In bird coordinates, evaluate two classical shape models:
   - fast line model: two side pipes plus a start crossbar from Hough segments;
   - fallback U-component model: one connected dark U/rectangle-like component,
     using PCA and lateral projection peaks to recover the two side pipes and
     the start crossbar.
6. Estimate lateral error from the center between the side pipes near the start.
7. Estimate heading error from the rail axis.
8. Use the detected crossbar as the beginning of the rail; forward speed is
   reduced as that start approaches.
9. Run a PID visual servo controller:
   - while on the floor: `vx`, PID `vy`, and PID `wz` correct alignment;
   - after stable lock: publish forward-only `vx` for a timed distance.

Because there is no odometry, the last "drive a couple meters" stage is timed:
`duration = drive_distance_m / rail_drive_speed`. Keep the speed conservative
and tune it on the real robot.

The ROS image subscriber uses sensor-data QoS so stale camera frames are dropped
instead of queued. Debug publishers use depth 1 for the same reason. On the
sample `rgb_video_015.mp4`, the RGB-only detector processes the full 1920x1080
video at about 26-36 FPS with full debug visualization enabled. The final
U-shape model detects 478/587 frames on `rgb_video_015.mp4` (81.4%).

## Depth-Based Floor Calibration

Use an aligned RGB frame and aligned 16-bit depth PNG to estimate the floor
plane and write detector homography parameters:

```bash
source install/setup.bash
ros2 run greenhouse_pipe_rail_nav rail_calibrate_floor \
  --color /data/frames_output_001/color_0000.jpg \
  --depth /data/frames_output_001/depth_0000.png \
  --output /ws/pipe_rail/calibration/floor_from_depth.yaml \
  --debug /ws/pipe_rail/analysis/calibration/floor_from_depth_debug.jpg \
  --x-half-width-m 0.62 \
  --y-near-m 0.70 \
  --y-far-m 1.80 \
  --expected-spacing-m 0.48
```

If you have exact `CameraInfo`, pass `--fx --fy --cx --cy`. Without that, the
tool uses RealSense-like RGB FOV defaults and is good enough for a first pass,
but exact intrinsics are better.

## Run with Docker Compose

From the repo root (`aida_bot_ws`), not from this directory:

```bash
docker compose up pipe_rail_autodock
```

By default it subscribes to `/camera/color/image_raw` and publishes `/cmd_nav`
(merge with joystick via `cmd_vel_mux`: teleop `/cmd_vel_teleop`, nav `/cmd_nav` → `/cmd_vel`).
Pass a calibrated config with:

```bash
docker compose run --rm pipe_rail_autodock bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install --packages-select greenhouse_pipe_rail_nav && source install/setup.bash && ros2 launch greenhouse_pipe_rail_nav pipe_rail_autodock.launch.py config_file:=/ws/pipe_rail/calibration/floor_from_depth.yaml"
```

To replay the included sample video through ROS:

```bash
USE_VIDEO=true VIDEO_FILE=/ws/pipe_rail/analysis/videos/rgb_video_015.mp4 CONTROL_ENABLED=false docker compose up pipe_rail_autodock
```

Set `CONTROL_ENABLED=false` for dry runs. The debug window is opened with OpenCV
when `VISUALIZE=true`. The live debug view shows source-image detection on the
left and bird-view control geometry on the right. In the bird view, the green
arrow near the bottom is the commanded `linear.x/linear.y` vector. The yellow
arc is the commanded `angular.z` rotation.

If X11 blocks the OpenCV window on the host, allow local Docker clients first:

```bash
xhost +local:docker
```

## RViz in the browser (noVNC)

RViz loads `agro_cad/agrobot_description` (body, wheels, D405, ZED-M, mast cameras).

RViz runs on a virtual display inside the container and is served with noVNC.
This does not use the host `DISPLAY` or `xhost`.

```bash
docker compose --profile novnc up rviz_novnc
```

On this machine open:

http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale

From another computer on the same network, use the robot IP instead of
`127.0.0.1`.

The RViz preset (`rviz/pipe_rail.rviz`) shows RobotModel, TF, camera image, and
pipe-rail debug image topics. Run autodock in another terminal so those topics
exist:

```bash
docker compose up pipe_rail_autodock
```

## Important calibration

Edit `src/greenhouse_pipe_rail_nav/config/default.yaml`.

The most important fields are:

- `detector.source_points`: four image points on the floor plane, ordered as
  top-left, top-right, bottom-right, bottom-left. Normalized coordinates are
  accepted.
- `detector.meters_per_pixel`: bird-view metric scale.
- `detector.expected_spacing_m`: measured distance between the two pipe centers.
- `approach_speed`, `rail_drive_speed`, and controller gains.

For production, put four visible floor markers around the rail approach area,
measure their real rectangle, and tune `source_points` until the two pipes are
parallel and have the expected spacing in the debug bird view.

## ROS interfaces

Subscribed:

- `/camera/color/image_raw` (`sensor_msgs/Image`)

Published:

- `/cmd_nav` (`geometry_msgs/Twist`) — wire to odroid `cmd_vel_mux` `nav_cmd_vel_topic`
- `/pipe_rail/debug/annotated` (`sensor_msgs/Image`)
- `/pipe_rail/debug/bird_view` (`sensor_msgs/Image`)

## Offline detector check

After building the workspace in the container:

```bash
source install/setup.bash
ros2 run greenhouse_pipe_rail_nav rail_offline_demo /ws/pipe_rail/analysis/screenshots/rgb_video_015_11s_greenhouse_pipe_rail.jpg --output /tmp/rail_debug.jpg
```

## Video Benchmark

```bash
source install/setup.bash
ros2 run greenhouse_pipe_rail_nav rail_benchmark_video /data/rgb_video_015.mp4 --max-frames 587 --snapshot /tmp/rail_motion_overlay.jpg
```

Render a full comparison video with the calibrated bird view:

```bash
ros2 run greenhouse_pipe_rail_nav rail_render_video /ws/pipe_rail/analysis/videos/rgb_video_015.mp4 /tmp/rgb_video_015_source_detect_debug.mp4 --detector-config /ws/pipe_rail/calibration/floor_from_depth.yaml
```
