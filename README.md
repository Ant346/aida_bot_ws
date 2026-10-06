# aida_bot_ws

ROS 2 workspace. This repository uses **Git submodules** for vendored packages; clone them so paths like `ds4_driver_submodule/` and `src/realsense-ros/` are populated.

## Clone (recommended)

Clone the repository and initialize submodules in one step:

```bash
git clone --recurse-submodules https://github.com/Ant346/aida_bot_ws.git
cd aida_bot_ws
```

Shallow clones are fine if you only need current commits:

```bash
git clone --recurse-submodules --depth 1 https://github.com/Ant346/aida_bot_ws.git
cd aida_bot_ws
```

## Already cloned without submodules

From the repo root:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

After `git pull` on the parent repo, submodule pointers may move; refresh checkouts:

```bash
git submodule update --init --recursive
```

## Submodules

| Path | Repository | Configured branch (for `update --remote`) |
|------|------------|-------------------------------------------|
| `ds4_driver_submodule` | [naoki-mizuno/ds4_driver](https://github.com/naoki-mizuno/ds4_driver) | `humble` |
| `src/realsense-ros` | [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros) | `ros2-master` |

Commits are pinned by this repo’s superproject; **`git submodule update`** checks out those pins. Use **`git submodule update --remote`** only if you intentionally want to move to the latest commit on the branch above (then commit the new submodule SHA in the parent repo).

## Verify

```bash
git submodule status
```

Each line should start with a space (submodule checked out at the expected commit), not `-` (missing) or `+` (different commit than recorded).

## Cameras

D435, D405, and ZED-M each have their own Compose service (Humble, `network_mode: host`).

```bash
docker compose up realsense          # D435, topics /camera/camera/...
docker compose up realsense_d405     # D405, topics /d405/d405/...
docker compose up zedm               # ZED-M UVC, topic /zedm/image_raw
```

`realsense` is pinned to `device_type:=d435` and `realsense_d405` to the D405 serial (`REALSENSE_D405_SERIAL`, default `218622278337`), so the two RealSense nodes do not grab the same USB device.

The D405 on this machine is on a USB 2.0 port, so the default depth/color profile is `848x480x15`. On a USB 3 port:

```bash
REALSENSE_D405_DEPTH_PROFILE=848x480x30 docker compose up realsense_d405
```

ZED-M is published as a side-by-side UVC image (`2560x720` YUYV by default) via `v4l2_camera`. The Stereolabs ROS wrapper needs an NVIDIA GPU and the nvidia container runtime, which this host does not have.

## RViz

The `viz` profile loads `agro_cad/agrobot_description` (`base_link`, wheels, D405, ZED-M, masts) and opens RViz with that model:

```bash
xhost +local:docker
docker compose --profile viz up rviz
```

Rebuild the image once after this change so it contains `robot_state_publisher` and `joint_state_publisher`:

```bash
docker compose --profile viz build rviz
```
