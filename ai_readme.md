# AI context: agrobot, 2026-10-09

Workspace: `/home/aida/projects/aida_bot_ws`, branch `maks_refactor`.
Human write-up of the same day: `docs/2026-10-09-odrive-calibration.md`.
This file is the state to continue from. Facts below are measured or written in the tree. Items under "not done" are agreed and not finished.

## Hard rules

- The operator is at the robot and stops every motor with space. Do not command motion unless `estop_ssh.py` is running in their SSH window. A driver restart kills that process; they must start it again.
- Space latches `/tmp/odroid_estop` inside the driver container. `docker start` / `docker restart` keeps container `/tmp`, so a latched stop survives a restart. Clear only with `ros2 service call /odroid_estop/clear std_srvs/srv/Empty`.
- Calibrate profile travel limit is 2 m from the pose at driver start (`travel_radius_limit_m: 2.0`). Do not raise it.
- Do not run `odroid_node` and `odroid_calibrate` together. Both write the same CAN.
- Do not `sudo odrivetool`. Host sudo needs a password. System package 0.6.10 was removed because it does not speak to firmware 0.5.1. Use `/home/aida/micromamba/envs/odroid/bin/odrivetool` (0.5.1) without sudo.
- Do not re-enable `dev.reset()` in fibre. On this machine a USB reset makes the ODrive 3.6 re-enumerate with 0 interfaces.
- Root on the host is `docker run --rm -i --privileged --pid host --entrypoint nsenter aida_bot_ws-odroid_node:latest -t 1 -m -u -i -n -- <cmd>`.
- Raising `gear_ratio` in `odroid_driver.yaml` makes the joystick faster by the same factor. Scales in `docker_ros/ds4_twist/twist_aida.yaml` are linear.x 2.0, linear.y 1.0, angular.z 1.0. Shrink those in the same change. Put the measured ratio in `odroid_driver_calibrate.yaml` first.

## Hardware

| Item | Value |
|---|---|
| Front ODrive | serial `336B356C3033`, USB `1209:0d32`, CAN `can0` |
| Rear ODrive | serial `335535593033`, USB `1209:0d32`, CAN `can1` |
| CAN adapters | CANable2. Host symlinks `/dev/can_front` → can0, `/dev/can_rear` → can1. Bitrate 250000. Held by privileged container `can-keeper` (`slcand`). |
| Axis map | can0 axis0 = FL, axis1 = FR. can1 axis0 = RR, axis1 = RL. |
| Motors | Geared DDK hub, halls count motor shaft not wheel. Ratio ≈ 5, not measured exactly. |
| Wheel | radius 0.095 m, base 0.635 m, track 0.72 m. Rollers layout `O`. `yaw_gain` 10. |
| Cameras | D435 color `/d435/d435/color/image_raw`, info `/d435/d435/color/camera_info`. D405 is the other RealSense (`8086:0b5b`). |
| Host | ThinkBook. Operator also SSHes from a Mac; the Mac keyboard is not in `/dev/input`. |

If a CANable re-enumerates, can0/can1 disappear. Reattach slcand inside `can-keeper`, then restart the driver. USB log line to expect: "disabled by hub (EMI?)".

## Containers

- Drive: `aida_bot_ws-odroid_calibrate-1` (profile `calibrate`, `ODRIVE_CALIBRATE=true`). Image Jazzy. Source is bind-mounted; Python edits apply after `docker restart aida_bot_ws-odroid_calibrate-1`.
- CAN: `can-keeper`.
- AprilGrid / D435 tools: `aida_bot_ws-rviz_novnc-1` (Jazzy, `docker compose --profile novnc up rviz_novnc`). Scripts are in `data/pipe_rail/tools/`, mounted at `/ws/pipe_rail`.
- `aida_bot_ws-ds4driver-1` and `aida_bot_ws-odroid_node-1` were stopped. Do not start the normal driver while calibrate is up.

```bash
docker compose --profile calibrate up -d odroid_calibrate
docker compose stop odroid_calibrate && docker compose up -d odroid_node
```

Driver log when `docker logs` is stale: `/root/.ros/log/python3_*.log` inside the calibrate container.

## ODrive: what was fixed

Native USB (`fibre` protocol of 0.5.1) failed in three stacked ways:

1. `fibre` calls `dev.reset()` on non-Windows. That reset leaves this 3.6 with an empty configuration. The call is commented out in `/home/aida/micromamba/envs/odroid/lib/python3.11/site-packages/fibre/usbbulk_transport.py`. This edit is outside the git repo.
2. `sudo odrivetool` resolved to system 0.6.10. That package is uninstalled.
3. ASCII-over-USB occupied the CDC interface and hid the native one. Front board: ASCII flag false and saved to EEPROM. Rear board: ASCII flag false in RAM only, not saved.

Board config already on the axes (treat as the working set; bandwidth and rear ASCII are RAM-only):

- Encoder hall, mode 1, cpr 60. `pole_pairs` 10. `torque_constant` 0.04. `current_lim` 20, margin 8.
- `controller.config.control_mode` 2 (velocity), `input_mode` 1 (passthrough), `vel_gain` 1.0, `vel_integrator_gain` 0, `vel_limit` 30, `vel_ramp_rate` 1.
- `dc_max_negative_current` −5. `brake_resistance` 0.
- `startup_closed_loop_control` true. ODrive watchdog disabled.
- `current_control_bandwidth` was 5 (torque could not follow). Set to 100 on all four axes in RAM. Not saved. Saving waits for a drive test.

## CAN errors and the clear service

Implemented in `src/odroid_node/odroid_driver/odrive_can.py`, wired from `odroid_driver.py`.

- Heartbeat cmd `0x001`. Frame id `(axis << 5) | cmd`. Bytes 0–3 axis_error uint32 LE, byte 4 axis state (1 IDLE, 8 CLOSED_LOOP). ~10 Hz.
- `HeartbeatMonitor` reads both buses. A nonzero axis error, or no heartbeat for 0.5 s, is a safety block: all axes go IDLE. Same path as space.
- At startup the driver waits ~0.6 s, logs any errors, sends Clear_Errors once.
- Service `~/clear_errors` (`std_srvs/Empty`), full name `/odroid_driver/clear_errors`. Sends cmd `0x018`, DLC 0, to every configured axis. Verified: injected ODrive estop on front axis0, log showed `ESTOP_REQUESTED (0x4000)`, after the service `ошибок нет`.

```bash
docker exec aida_bot_ws-odroid_calibrate-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 service call /odroid_driver/clear_errors std_srvs/srv/Empty'
```

Command ramp is in `motion_guard.ramp_cmd`, applied in `_guarded_cmd` after the speed clamp. Vector limit on `(vx, vy)`, separate limit on `wz`. Params `max_linear_accel_mps2: 0.6`, `max_angular_accel_rps2: 2.0` in `odroid_driver.yaml`. E-stop skips the ramp and idles the axes. Ramp state resets when safety blocks.

## Safety files (inside the calibrate container)

| Path | Role |
|---|---|
| `/tmp/odroid_estop` | `1` = latched stop |
| `/tmp/odroid_estop_reason` | text, e.g. `space from ssh` or `space from AT Translated Set 2 keyboard` |
| `/tmp/odroid_estop_watchdog` | `estop_space` heartbeat |
| `/tmp/odroid_estop_keyboard` | at least one keyboard seen |
| driver heartbeat | driver loop is alive |

`estop_space` is launched with the driver and reads `/dev/input` (ThinkBook keyboard, focus does not matter). `estop_ssh.py` reads the SSH tty so space from the Mac works. Start it only in the operator's terminal:

```bash
docker exec -it aida_bot_ws-odroid_calibrate-1 python3 /workspace/src/odroid_node/odroid_driver/estop_ssh.py
```

## Camera calibration

Solver: TartanCalib (Kalibr), model `pinhole-radtan`, AprilGrid `tag36h11`, color 1280×720.

Intrinsics, session `data/pipe_rail/calib_kalibr/20261008_225113`, take the `*-1-*` files (every frame used):

| | D405 | D435 |
|---|---|---|
| frames | 22/22 | 25/25 |
| fx, fy | 648.17, 648.38 | 891.91, 892.05 |
| cx, cy | 645.60, 362.84 | 642.80, 360.41 |
| k1, k2, p1, p2 | −0.0436, 0.0305, −0.00033, 0.00100 | 0.1047, −0.1947, 0.00032, 0.00192 |
| reprojection std, px | 0.207 × 0.189 | 0.171 × 0.198 |

Factory D435 color (serial 044322072287, fw 5.15.1) is backed up at `data/pipe_rail/calib/d435_factory_color_1280x720.yaml`. Published factory fx was 911.33 (2.1% higher than custom) and distortion coefficients were 0. Custom values replaced it.

Custom intrinsics file: `data/pipe_rail/calib/d435_color_1280x720.yaml`. Publisher: `publish_d435_camera_info.py`. It overwrites `/d435/d435/color/camera_info`. The driver keeps the factory message on `color/camera_info_factory`. ROS distortion model in the message is `plumb_bob` with k3 = 0 so it matches radtan's four coefficients.

Extrinsics, session `calib_kalibr/20261009_001947`, file `tartan/log1-camchain.yaml`. This is the result to use.

- Board: 3×3, tag edge 0.05 m, `tagSpacing` 0.30 (gap 15 mm). Tag id 6 is absent on the print. Ids are Kalibr row-major. Do not set `id_order: column`.
- A run with `tagSpacing` 0.2 reproduced at about 3 px. Discard it.
- Reprojection of `log1`: cam0 (D405, `/cam0/image_raw`) std `[0.179230, 0.124193]` px; cam1 (D435, `/cam1/image_raw`) std `[0.180277, 0.127041]` px. Quote as 0.18 px.
- `T_cn_cnm1` maps a point in the D405 camera frame into the D435 camera frame. Translation metres: tx 0.030634 ± 0.000276, ty 0.257610 ± 0.000429, tz 0.422563 ± 0.002557. Baseline 0.496 m. Rotation is mostly about x, about 35°.
- Scale of that translation assumes the printed tag is 50 mm. A different ruler measurement scales all three components by the same factor.

CAD vs this extrinsic, after moving CAD origins onto the color sensors (D435 color is 65 mm left of the right lens, which is the CAD origin; D405 color is the left lens): difference 15 mm and 1.0°. In the D435 image axes: right 10 mm, down 9 mm, forward 5 mm.

URDF `d435_link` is still the CAD pose, not the calibrated extrinsic. From `base_link`: xyz `0.451636729 -0.017499974 0.691293991` m, rpy pitch `0.872664626`. Source frames: `agro_cad/framesd435.json`, generated by `agro_cad/build_robot_urdf.py`.

AprilGrid on the live image is not OpenCV aruco. The Kalibr board has a 2-bit black border; `DICT_APRILTAG_36h11` expects 1 bit and reads nothing. Detector is `data/pipe_rail/tools/aprilgrid_detect.cpp` plus ethz apriltag2, built in the rviz container to `/ws/pipe_rail/tools/libaprilgrid.so`. `TagDetector.h` must be included before `Tag36h11.h`. Live check once saw 8/8 tags, ids 0 1 2 3 4 5 7 8.

Pose of the robot against the board: `data/pipe_rail/tools/board_pose.py`. `solvePnP` on all tag corners, tag pitch `(1+0.3)*0.05`. Prints JSON `n, ids, reproj_px, margin_px, xyz, xyz_std, yaw, yaw_std`. A good frame had `reproj_px` ≈ 0.12 and `xyz_std` ≈ 0.1 mm.

```bash
docker exec --workdir /ws aida_bot_ws-rviz_novnc-1 bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 board_pose.py'
```

## Omni wheels: the two separate errors

Roller layout is `O` (`odroid_driver.yaml`). Roller axes that touch the floor point at the robot center. Strafe sign flips versus layout `X`. Yaw arm is about `|W-L|` ≈ 0, so yaw is roller slip, scaled by `yaw_gain` (still 10, not identified).

Friction. Below about 0.05 m/s the wheels do not break roller friction. Turn-in-place at `max_wheel_turns_s` 0.8 did not start until the chassis was nudged by hand; the cap is 2.0 in the calibrate profile. A turn that only buzzes is this, not a dead axis.

Scale. With `gear_ratio: 1` the driver sends wheel turns as motor turns. Halls count the motor shaft, so the robot moves at about 1/5 of the command. Camera runs, gear 1, board held in view:

| command | measured | scale |
|---|---|---|
| +0.25 m/s for 1 s | 4.6 cm | 0.18 |
| −0.25 m/s for 1 s | 5.3 cm | 0.21 |
| strafe 0.20 m | 3.1 cm and 2.8 cm | 0.14–0.16 |
| yaw +0.30 rad | +0.187 rad, ~2 cm drift | |
| yaw −0.30 rad | −0.154 rad | |
| 0.4 s pulse | same ≈ 0.2 scale | not accel |

`gear_ratio` multiplies wheel turns after the kinematics cap, so `max_wheel_turns_s` stays in wheel turns. It is still 1.0 in both yamls. Do not guess 5.0 into the main file. Measure `mean(motor pos_estimate delta) * 2π * 0.095 / camera_distance` over 4 wheels, forward and back, then write that number.

How a scripted move was done without the joystick fighting it: `SIGSTOP` the `cmd_vel_mux` pid inside the calibrate container, publish `/cmd_vel` with rclpy (wait until a subscriber exists), publish zeros, `SIGCONT`. Stopping only `ds4_twist` is not enough: the mux then publishes zeros from its joystick watchdog.

## Not done (agreed order)

1. Measure `gear_ratio` under the D435 with the board in view. Needs the D435 on USB (id is not `8086:0b5b`; that id is the D405) and `estop_ssh.py` running. Then clear the latched estop.
2. Save to both boards: bandwidth 100, rear ASCII false, and the measured controller gains.
3. Tune `vel_gain` / `vel_integrator_gain` from USB telemetry (`Iq`, `vel_estimate`) while driving.
4. Re-run straight, strafe, and turn-in-place under the camera. Adjust `yaw_gain` so commanded yaw matches measured yaw.
5. Enable the ODrive watchdog so axes fall to IDLE if CAN commands stop while the driver is down. Confirmed with the operator, do it after the gear measurement. On 0.5.1, confirm that `Set_Input_Vel` feeds the watchdog before saving: the driver already sends it at 50 Hz in every state, including zero speed. Test with wheels unloaded. If a watchdog trip happens while the driver is up, the heartbeat path idles all axes and stays there until `/odroid_driver/clear_errors`. Startup already clears a leftover `WATCHDOG_TIMER_EXPIRED` once.

## File map

| Path | What |
|---|---|
| `src/odroid_node/odroid_driver/odroid_driver.py` | CAN loop, gear_ratio, ramp, clear service, heartbeat fault |
| `src/odroid_node/odroid_driver/odrive_can.py` | heartbeat parse, error names, Clear_Errors |
| `src/odroid_node/odroid_driver/motion_guard.py` | clamp, fence, ramp, estop files |
| `src/odroid_node/odroid_driver/estop_space.py` | keyboard estop |
| `src/odroid_node/odroid_driver/estop_ssh.py` | SSH-tty estop |
| `src/odroid_node/config/odroid_driver.yaml` | normal limits 0, accel 0.6 / 2.0, gear 1, layout O |
| `src/odroid_node/config/odroid_driver_calibrate.yaml` | 0.5 m/s, 2 rad/s, 2 wheel rev/s, 2 m radius |
| `agro_cad/framesd435.json` | CAD frames including D435 |
| `data/pipe_rail/calib/` | factory backup + custom D435 yaml + publisher |
| `data/pipe_rail/calib_kalibr/20261009_001947/tartan/log1-camchain.yaml` | extrinsics to use |
| `data/pipe_rail/tools/board_pose.py` | board pose JSON |
| `.gitmodules` | `ds4_driver_submodule`, `src/realsense-ros` |
