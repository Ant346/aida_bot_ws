#!/bin/bash

# Run the hoverboard_node container with tmux session
docker run -it --rm \
  --name hoverboard_node_tmux \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/..:/workspace \
  -v /dev:/dev \
  -v ${HOME}/.zshrc:/root/.zshrc \
  -e ROS_DISTRO=humble \
  --privileged \
  ros_humble_zsh tmux 