FROM ros:humble-ros-base

# ZED-M UVC driver. The Stereolabs SDK / zed-ros2-wrapper needs an NVIDIA GPU
# and the nvidia container runtime; this image publishes the side-by-side
# stereo frames the camera exposes as a USB video device.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-v4l2-camera \
        ros-humble-image-transport \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "run", "v4l2_camera", "v4l2_camera_node"]
