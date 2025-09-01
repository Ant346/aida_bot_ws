#!/bin/bash

# Source ROS2
if [[ -f /opt/ros/$ROS_DISTRO/setup.bash ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Auto-build and source workspace if it doesn't exist
if [[ ! -f /workspace/install/setup.bash ]]; then
    echo "🔨 Building hoverboard_node package..."
    cd /workspace
    colcon build --packages-select hoverboard_node
    echo "✅ Package built successfully!"
fi

# Source the workspace
if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
    echo "🎯 Workspace sourced and ready!"
fi

exec "$@" 