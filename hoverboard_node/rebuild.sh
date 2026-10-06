#!/bin/bash

# Quick rebuild script for hoverboard_node package
# This rebuilds the package inside the running container

echo "🔨 Rebuilding hoverboard_node package..."

# Execute rebuild command in the container
docker exec -it aida_bot_ws-hoverboard_node-1 bash -c "
cd /workspace && 
colcon build --packages-select hoverboard_node &&
source install/setup.bash &&
echo '✅ Package rebuilt successfully!'
"

echo ""
echo "🎯 Package is ready to use!"
echo "Run: ros2 launch hoverboard_node hoverboard_driver.launch.py" 