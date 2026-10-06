FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-realsense2-camera \
    ros-jazzy-v4l2-camera \
    ros-jazzy-rqt \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rqt-common-plugins \
    xvfb \
    x11vnc \
    novnc \
    python3-websockify \
    x11-utils \
    openbox \
    wmctrl \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /ws

CMD ["bash"]
