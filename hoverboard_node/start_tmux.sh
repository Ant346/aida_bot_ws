#!/bin/bash

# Start tmux session inside the running hoverboard_node container
docker exec -it aida_bot_ws-hoverboard_node-1 start_ros2_session.sh 