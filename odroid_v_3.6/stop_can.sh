#!/bin/bash

# Script to stop CAN interfaces

set -e

echo "=== Stopping CAN Interfaces ==="

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Bring down CAN interfaces
for can_if in can0 can1; do
    if ip link show "$can_if" &>/dev/null; then
        echo "Bringing down $can_if..."
        ip link set "$can_if" down 2>/dev/null || true
    fi
done

# Kill slcand processes
echo "Stopping slcand processes..."
pkill -f "slcand" 2>/dev/null || true
sleep 0.5

echo "=== CAN Interfaces Stopped ==="

