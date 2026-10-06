#!/bin/bash

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Execute the passed command or start zsh
exec "$@" 