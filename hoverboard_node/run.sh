#!/bin/bash

# Enable X11 forwarding for Docker
xhost +local:docker

# Run the hoverboard_node container with TTY support
docker run -it --rm \
  --name hoverboard_node \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/..:/workspace \
  -v /dev:/dev \
  -e ROS_DISTRO=humble \
  --privileged \
  ros_humble_zsh

  