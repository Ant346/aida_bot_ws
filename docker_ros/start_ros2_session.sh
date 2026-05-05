#!/bin/bash
SESSION_NAME="aida_ros_dev"
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    exec tmux attach-session -t "$SESSION_NAME"
fi
tmux new-session -s "$SESSION_NAME" -n shell "bash"
