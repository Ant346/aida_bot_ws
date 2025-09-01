#!/bin/bash

# Source ROS2
if [[ -f /opt/ros/$ROS_DISTRO/setup.bash ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "🚀 ROS2 $ROS_DISTRO sourced successfully!"
fi

# Function to build workspace
build_workspace() {
    echo "🔨 Building workspace..."
    cd /workspace
    
    # Check if we need to install dependencies
    if [[ -f "src/package.xml" ]]; then
        echo "📦 Installing package dependencies..."
        rosdep install --from-paths src --ignore-src -r -y || echo "⚠️  Some dependencies may not be installed"
    fi
    
    # Build the workspace
    echo "🏗️  Running colcon build..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug || {
        echo "❌ Build failed! Trying to build individual packages..."
        # Try building packages one by one
        for pkg in src/*/; do
            if [[ -f "$pkg/package.xml" ]]; then
                echo "🔨 Building package: $(basename $pkg)"
                colcon build --packages-select $(basename $pkg) --symlink-install || echo "⚠️  Failed to build $(basename $pkg)"
            fi
        done
    }
    
    echo "✅ Build completed!"
}

# Auto-build and source workspace if it doesn't exist
if [[ ! -f /workspace/install/setup.bash ]]; then
    echo "🏗️  Workspace not found. Building from scratch..."
    build_workspace
else
    echo "✅ Workspace already exists!"
fi

# Source the workspace
if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
    echo "🎯 Workspace sourced and ready!"
    
    # Show available packages
    echo "📦 Available packages:"
    ls -la src/ 2>/dev/null || echo "No packages found in src/"
    
    # Show ROS2 topics if available
    echo "📡 ROS2 environment ready!"
else
    echo "⚠️  Workspace not properly built!"
fi

# Set up development environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

echo "🔧 Development environment configured:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "   Workspace: /workspace"

# Execute the command or start zsh
if [[ $# -eq 0 ]]; then
    echo "🐚 Starting zsh shell..."
    exec /bin/zsh
else
    exec "$@"
fi 