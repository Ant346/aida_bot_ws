#!/bin/bash

set -e

# Build the hoverboard_node Docker image
docker build -t ros_humble_zsh .

echo "Docker image 'ros_humble_zsh' built successfully!"
echo "To run the container:"
echo "  ./run.sh"
