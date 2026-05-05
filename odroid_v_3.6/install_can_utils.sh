#!/bin/bash

# Script to install can-utils package

echo "=== Installing can-utils ==="

# Update package list
sudo apt-get update

# Install can-utils
sudo apt-get install -y can-utils

echo ""
echo "=== Installation Complete ==="
echo "Now you can use:"
echo "  candump can0  # Monitor CAN messages"
echo "  candump can1  # Monitor CAN messages"
echo "  cansend can0 123#DEADBEEF  # Send CAN message"

