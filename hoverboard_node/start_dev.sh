#!/bin/bash

# Comprehensive development workflow for hoverboard_node
# This script sets up the complete development environment

echo "🚀 Starting hoverboard_node development environment..."

# Set environment variables for Docker Compose Watch
export COMPOSE_DOCKER_CLI_BUILD=1
export DOCKER_BUILDKIT=1

# Start the container with watch mode
echo "📡 Starting container with file watching..."
docker compose watch hoverboard_node &

# Wait a moment for container to start
sleep 3

# Check if container is running
if docker ps | grep -q "aida_bot_ws-hoverboard_node"; then
    echo "✅ Container is running!"
    
    # Build the package
    echo "🔨 Building package..."
    docker exec -it aida_bot_ws-hoverboard_node-1 bash -c "
        cd /workspace && 
        colcon build --packages-select hoverboard_node &&
        source install/setup.bash &&
        echo '✅ Package built successfully!'
    "
    
    echo ""
    echo "🎯 Development environment ready!"
    echo ""
    echo "📋 Available commands:"
    echo "  ./hoverboard_node/rebuild.sh                    - Rebuild package"
    echo "  docker exec -it aida_bot_ws-hoverboard_node-1 /bin/zsh  - Access container"
    echo "  docker compose watch --stop                     - Stop watching"
    echo ""
    echo "🧪 Test commands:"
    echo "  ros2 launch hoverboard_node hoverboard_driver.launch.py  - Run driver"
    echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'  - Send velocity"
    echo "  ros2 topic echo /hoverboard_odom                - Monitor odometry"
    echo ""
    echo "🔄 File changes will be automatically synced!"
    
else
    echo "❌ Container failed to start"
    exit 1
fi 