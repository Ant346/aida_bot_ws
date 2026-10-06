#!/bin/bash

# Development script for hoverboard_node with Docker Compose Watch
# This enables automatic file synchronization and rebuilding

echo "🚀 Starting hoverboard_node development environment with Watch..."

# Set environment variable for Docker Compose Watch
export COMPOSE_DOCKER_CLI_BUILD=1
export DOCKER_BUILDKIT=1

# Start the container with watch mode
echo "📡 Starting container with file watching..."
docker compose watch hoverboard_node

echo ""
echo "✅ Development environment started!"
echo ""
echo "📋 Available commands:"
echo "  docker compose watch hoverboard_node  - Start with file watching"
echo "  docker compose watch --stop           - Stop watching"
echo "  docker exec -it aida_bot_ws-hoverboard_node-1 /bin/zsh  - Access container"
echo "  ./hoverboard_node/rebuild.sh          - Rebuild package"
echo ""
echo "🔄 File changes will be automatically synced!" 