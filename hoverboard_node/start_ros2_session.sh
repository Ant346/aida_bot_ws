#!/bin/bash

# ROS2 Hoverboard Development Session
# This script starts tmux with a pre-configured layout for ROS2 development

SESSION_NAME="hoverboard_dev"

# Check if session already exists
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "Session $SESSION_NAME already exists. Attaching..."
    tmux attach-session -t $SESSION_NAME
    exit 0
fi

# Create new session
echo "Creating new ROS2 development session: $SESSION_NAME"
tmux new-session -d -s $SESSION_NAME -n "main"

# Source ROS2 in the main window
tmux send-keys -t $SESSION_NAME:main "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:main "echo 'ROS2 Humble sourced successfully'" C-m

# Create window for building
tmux new-window -t $SESSION_NAME -n "build"
tmux send-keys -t $SESSION_NAME:build "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:build "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:build "echo 'Ready to build: colcon build --packages-select hoverboard_node'" C-m

# Create window for hoverboard driver
tmux new-window -t $SESSION_NAME -n "driver"
tmux send-keys -t $SESSION_NAME:driver "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:driver "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:driver "echo 'Ready to run: ros2 launch hoverboard_node hoverboard_driver.launch.py'" C-m

# Create window for manual control
tmux new-window -t $SESSION_NAME -n "manual"
tmux send-keys -t $SESSION_NAME:manual "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:manual "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:manual "echo 'Ready to run: ros2 run hoverboard_node manual_wheel_control'" C-m

# Create window for testing
tmux new-window -t $SESSION_NAME -n "test"
tmux send-keys -t $SESSION_NAME:test "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:test "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:test "echo 'Ready to run: ros2 run hoverboard_node test_rs485'" C-m

# Create window for monitoring
tmux new-window -t $SESSION_NAME -n "monitor"
tmux send-keys -t $SESSION_NAME:monitor "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:monitor "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:monitor "echo 'Ready to monitor: ros2 topic list, ros2 topic echo, etc.'" C-m

# Split the monitor window for better monitoring
tmux split-window -h -t $SESSION_NAME:monitor
tmux send-keys -t $SESSION_NAME:monitor.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:monitor.1 "cd /workspace" C-m
tmux send-keys -t $SESSION_NAME:monitor.1 "echo 'Right pane ready for additional monitoring'" C-m

# Go back to main window
tmux select-window -t $SESSION_NAME:main

# Attach to session
echo "Attaching to session $SESSION_NAME..."
echo ""
echo "Available windows:"
echo "  main    - Main development window"
echo "  build   - Building and compilation"
echo "  driver  - Hoverboard driver node"
echo "  manual  - Manual wheel control"
echo "  test    - RS485 testing"
echo "  monitor - ROS2 topic monitoring"
echo ""
echo "Tmux shortcuts:"
echo "  Ctrl+a c     - Create new window"
echo "  Ctrl+a n     - Next window"
echo "  Ctrl+a p     - Previous window"
echo "  Ctrl+a |     - Split pane horizontally"
echo "  Ctrl+a -     - Split pane vertically"
echo "  Alt+Arrow    - Switch between panes"
echo "  Ctrl+a d     - Detach from session"
echo ""

tmux attach-session -t $SESSION_NAME 