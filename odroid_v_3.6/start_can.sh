#!/bin/bash

# Script to set up CAN interfaces for canable2 adapters
# This script configures can_front and can_rear interfaces

# Don't use set -e, we want to continue even if one interface fails

# Configuration
CAN_BITRATE=${CAN_BITRATE:-250000}  # Default 250kbps (matches main.py), can be overridden
FRONT_DEVICE="/dev/can_front"
REAR_DEVICE="/dev/can_rear"

echo "=== CAN Interface Setup ==="
echo "Bitrate: $CAN_BITRATE bps"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: Please run as root (use sudo)"
    exit 1
fi

# Check if devices exist and resolve symlinks to real devices
echo "Checking devices..."

# Function to resolve device path
resolve_device() {
    local dev=$1
    if [ ! -e "$dev" ]; then
        echo ""
        return 1
    fi
    # Try multiple methods to resolve symlink
    local resolved=$(readlink "$dev" 2>/dev/null)
    if [ -n "$resolved" ]; then
        # If it's an absolute path, use it; otherwise make it absolute
        if [[ "$resolved" == /* ]]; then
            echo "$resolved"
        else
            echo "/dev/$resolved"
        fi
    else
        # Not a symlink, return as is
        echo "$dev"
    fi
}

FRONT_REAL_DEVICE=$(resolve_device "$FRONT_DEVICE")
if [ -z "$FRONT_REAL_DEVICE" ] || [ ! -e "$FRONT_REAL_DEVICE" ]; then
    echo "WARNING: $FRONT_DEVICE not found or cannot be resolved"
    FRONT_REAL_DEVICE=""
else
    echo "✓ Found $FRONT_DEVICE -> $FRONT_REAL_DEVICE"
fi

REAR_REAL_DEVICE=$(resolve_device "$REAR_DEVICE")
if [ -z "$REAR_REAL_DEVICE" ] || [ ! -e "$REAR_REAL_DEVICE" ]; then
    echo "WARNING: $REAR_DEVICE not found or cannot be resolved"
    REAR_REAL_DEVICE=""
else
    echo "✓ Found $REAR_DEVICE -> $REAR_REAL_DEVICE"
fi
echo ""

# Load CAN kernel modules if not already loaded
echo "Loading CAN kernel modules..."
modprobe can 2>/dev/null || true
modprobe can-raw 2>/dev/null || true
modprobe slcan 2>/dev/null || true

# Function to setup CAN interface
setup_can() {
    local device=$1
    local can_name=$2
    
    # Device should already be resolved, but double-check
    local real_device="$device"
    if [ -L "$device" ]; then
        # If it's still a symlink, try to resolve it
        local resolved=$(readlink "$device" 2>/dev/null)
        if [ -n "$resolved" ]; then
            if [[ "$resolved" == /* ]]; then
                real_device="$resolved"
            else
                real_device="/dev/$resolved"
            fi
        fi
    fi
    
    if [ ! -e "$real_device" ]; then
        echo "Warning: Device $device (resolved to $real_device) not found. Make sure udev rules are set up."
        echo "Run: ./setup_rules.sh"
        return 1
    fi
    
    echo "Setting up $can_name on $real_device..."
    
    # Check if device is already in use
    if lsof "$real_device" 2>/dev/null | grep -q .; then
        echo "  WARNING: Device $real_device is in use by another process"
        lsof "$real_device" 2>/dev/null | head -3
        echo "  Attempting to stop existing processes..."
    fi
    
    # Stop any existing slcan instance for this CAN interface
    pkill -f "slcand.*$can_name" 2>/dev/null || true
    # Also stop by device path
    pkill -f "slcand.*$real_device" 2>/dev/null || true
    # Bring down interface if it exists
    ip link set "$can_name" down 2>/dev/null || true
    sleep 1
    
    # Attach serial device to CAN interface
    # -o: open command, -s8: 8 data bits, -S: baudrate
    # Use real device path, not symlink
    echo "  Running: slcand -o -s8 -S 115200 $real_device $can_name"
    if ! slcand -o -s8 -S 115200 "$real_device" "$can_name"; then
        echo "  ERROR: Failed to attach $real_device to $can_name"
        echo "  Check if device exists and is not in use"
        echo "  Try: ls -la $real_device"
        return 1
    fi
    
    sleep 1  # Give slcand time to initialize
    
    # Bring CAN interface up
    echo "  Running: ip link set $can_name up type can bitrate $CAN_BITRATE"
    if ! ip link set "$can_name" up type can bitrate "$CAN_BITRATE"; then
        echo "  ERROR: Failed to bring $can_name up"
        echo "  Check if interface was created by slcand"
        return 1
    fi
    
    echo "✓ $can_name is up and running"
}

# Setup front CAN interface (use real device if available)
if [ -n "$FRONT_REAL_DEVICE" ] && [ -e "$FRONT_REAL_DEVICE" ]; then
    if setup_can "$FRONT_REAL_DEVICE" "can0"; then
        echo "Front CAN (can0) configured successfully"
    else
        echo "Failed to configure front CAN"
    fi
else
    echo "Skipping front CAN - device not found"
fi

# Setup rear CAN interface (use real device if available)
if [ -n "$REAR_REAL_DEVICE" ] && [ -e "$REAR_REAL_DEVICE" ]; then
    if setup_can "$REAR_REAL_DEVICE" "can1"; then
        echo "Rear CAN (can1) configured successfully"
    else
        echo "Failed to configure rear CAN"
    fi
else
    echo "Skipping rear CAN - device not found"
fi

# Display CAN interface status
echo ""
echo "=== CAN Interface Status ==="
ip link show | grep -E "^[0-9]+: can" || echo "No CAN interfaces found"

echo ""
echo "=== Testing CAN interfaces ==="
if ip link show can0 &>/dev/null; then
    echo "can0: $(ip -s link show can0 | grep -E 'RX|TX' | head -2)"
fi
if ip link show can1 &>/dev/null; then
    echo "can1: $(ip -s link show can1 | grep -E 'RX|TX' | head -2)"
fi

echo ""
echo "=== Setup Complete ==="
echo "To test CAN communication, use:"
echo "  candump can0  # Monitor can0"
echo "  candump can1  # Monitor can1"
echo "  cansend can0 123#DEADBEEF  # Send test message"

