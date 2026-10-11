#!/bin/bash

# Simple script to get into the Docker container
echo "🐳 Getting into Docker container..."

# Check if container is running
if docker ps | grep -q "ds4driver"; then
    echo "✅ Container is running, executing bash..."
    docker exec -it $(docker ps -q --filter "name=ds4driver") /bin/bash -c "
        cd /workspace
        echo '🚀 Sourcing ROS2...'
        source /opt/ros/humble/setup.bash
        
        echo '🔧 Setting up workspace structure...'
        # Create src directory if it doesn't exist
        if [[ ! -d 'src' ]]; then
            mkdir -p src
            echo '📁 Created src directory'
        fi
        
        # Create symbolic links for DS4 driver packages
        if [[ -d 'ds4_driver_submodule/ds4_driver_msgs' ]]; then
            ln -sf ../ds4_driver_submodule/ds4_driver_msgs src/ds4_driver_msgs
            echo '✅ Linked ds4_driver_msgs'
        fi
        
        if [[ -d 'ds4_driver_submodule/ds4_driver' ]]; then
            ln -sf ../ds4_driver_submodule/ds4_driver src/ds4_driver
            echo '✅ Linked ds4_driver'
        fi
        
        echo '🔨 Building workspace...'
        colcon build --symlink-install
        
        echo '🎯 Sourcing workspace...'
        if [ -f install/setup.bash ]; then
            source install/setup.bash
        fi
        
        echo '🚀 Ready! Starting zsh...'
        exec /bin/zsh
    "
else
    echo "❌ Container not running. Starting it first..."
    docker compose up -d ds4driver
    sleep 3
    echo "✅ Container started, executing bash..."
    docker exec -it $(docker ps -q --filter "name=ds4driver") /bin/bash -c "
        cd /workspace
        echo '🚀 Sourcing ROS2...'
        source /opt/ros/humble/setup.bash
        
        echo '🔧 Setting up workspace structure...'
        # Create src directory if it doesn't exist
        if [[ ! -d 'src' ]]; then
            mkdir -p src
            echo '📁 Created src directory'
        fi
        
        # Create symbolic links for DS4 driver packages
        if [[ -d 'ds4_driver_submodule/ds4_driver_msgs' ]]; then
            ln -sf ../ds4_driver_submodule/ds4_driver_msgs src/ds4_driver_msgs
            echo '✅ Linked ds4_driver_msgs'
        fi
        
        if [[ -d 'ds4_driver_submodule/ds4_driver' ]]; then
            ln -sf ../ds4_driver_submodule/ds4_driver src/ds4_driver
            echo '✅ Linked ds4_driver'
        fi
        
        echo '🔨 Building workspace...'
        colcon build --symlink-install
        
        echo '🎯 Sourcing workspace...'
        if [ -f install/setup.bash ]; then
            source install/setup.bash
        fi
        
        echo '🚀 Ready! Starting zsh...'
        exec /bin/zsh
    "
fi 