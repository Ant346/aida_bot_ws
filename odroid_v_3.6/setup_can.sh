#!/bin/bash

# Script to monitor and maintain can_rear and can_front interfaces
# Automatically restarts slcand if interface is lost

CAN_BITRATE=${CAN_BITRATE:-250000}  # Default 250kbps, can be overridden
SLCAN_BAUDRATE=${SLCAN_BAUDRATE:-115200}  # Serial baudrate for slcand

# Configuration for both interfaces
declare -A CAN_CONFIG
CAN_CONFIG[can_rear]="/dev/can_rear:206932AA5052"
CAN_CONFIG[can_front]="/dev/can_front:208C33725931"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: Please run as root (use sudo)"
    exit 1
fi

# Load CAN kernel modules if not already loaded
modprobe can 2>/dev/null || true
modprobe can-raw 2>/dev/null || true
modprobe slcan 2>/dev/null || true

# Function to resolve device path
resolve_device() {
    local dev=$1
    if [ ! -e "$dev" ]; then
        echo ""
        return 1
    fi
    local resolved=$(readlink "$dev" 2>/dev/null)
    if [ -n "$resolved" ]; then
        if [[ "$resolved" == /* ]]; then
            echo "$resolved"
        else
            echo "/dev/$resolved"
        fi
    else
        echo "$dev"
    fi
}

# Function to find device by serial number
find_device_by_serial() {
    local target_serial=$1
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        local serial=$(udevadm info -q property -n "$dev" 2>/dev/null | grep "ID_SERIAL_SHORT" | cut -d= -f2)
        if [ "$serial" = "$target_serial" ]; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

# Track last device state to avoid spam
declare -A LAST_DEVICE_STATE
declare -A MISSING_COUNT

# Function to setup and monitor a single CAN interface
monitor_can_interface() {
    local iface=$1
    local dev_symlink=$(echo "${CAN_CONFIG[$iface]}" | cut -d: -f1)
    local serial=$(echo "${CAN_CONFIG[$iface]}" | cut -d: -f2)
    
    # Resolve device path - try symlink first, then find by serial
    local real_device=$(resolve_device "$dev_symlink")
    if [ -z "$real_device" ] || [ ! -e "$real_device" ]; then
        real_device=$(find_device_by_serial "$serial")
        if [ -z "$real_device" ] || [ ! -e "$real_device" ]; then
            # Only log if state changed or every 6th check (1 minute)
            local count=${MISSING_COUNT[$iface]:-0}
            ((count++))
            MISSING_COUNT[$iface]=$count
            
            if [ "$count" -eq 1 ] || [ $((count % 6)) -eq 0 ]; then
                echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Device not found (serial: $serial) [attempt $count]"
            fi
            LAST_DEVICE_STATE[$iface]="missing"
            return 1
        fi
    fi
    
    # Reset missing count if device found
    if [ "${LAST_DEVICE_STATE[$iface]}" = "missing" ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Device found: $real_device"
    fi
    MISSING_COUNT[$iface]=0
    LAST_DEVICE_STATE[$iface]="found"
    
    # Check if interface exists
    if ip link show $iface &>/dev/null; then
        # Check if interface is UP
        local state=$(ip link show $iface 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}')
        if [ "$state" = "UP" ]; then
            # Only log state change, not every check
            if [ "${LAST_DEVICE_STATE[$iface]}" != "up" ]; then
                echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Interface is UP"
                LAST_DEVICE_STATE[$iface]="up"
            fi
            return 0
        else
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Interface is DOWN, bringing UP"
            ip link set $iface type can bitrate $CAN_BITRATE 2>/dev/null || true
            ip link set up $iface 2>/dev/null || true
            LAST_DEVICE_STATE[$iface]="down"
            return 0
        fi
    else
        # Only log if interface was previously up
        if [ "${LAST_DEVICE_STATE[$iface]}" = "up" ] || [ -z "${LAST_DEVICE_STATE[$iface]}" ]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Interface not found, restarting slcand"
        fi
        LAST_DEVICE_STATE[$iface]="not_found"
        
        # Re-resolve device in case it changed
        local current_device=$(resolve_device "$dev_symlink")
        if [ -z "$current_device" ] || [ ! -e "$current_device" ]; then
            current_device=$(find_device_by_serial "$serial")
        fi
        
        if [ -z "$current_device" ] || [ ! -e "$current_device" ]; then
            # Don't spam errors - device will be checked again in next cycle
            return 1
        fi
        
        # Stop any existing slcand processes for this device
        pkill -f "slcand.*$current_device" 2>/dev/null || true
        pkill -f "slcand.*$iface" 2>/dev/null || true
        sleep 1
        
        # Start slcand with proper parameters (matching start_can.sh)
        # -o: open command, -s8: 8 data bits, -S: baudrate
        echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Starting slcand -o -s8 -S $SLCAN_BAUDRATE $current_device $iface"
        if ! slcand -o -s8 -S $SLCAN_BAUDRATE "$current_device" "$iface" 2>/dev/null; then
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] ERROR: Failed to start slcand"
            return 1
        fi
        
        # Wait for interface to appear
        local timeout=15
        while ! ip link show $iface &>/dev/null && [ $timeout -gt 0 ]; do
            sleep 1
            ((timeout--))
        done

        # If interface appeared, configure it
        if ip link show $iface &>/dev/null; then
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Interface created, configuring..."
            ip link set $iface type can bitrate $CAN_BITRATE 2>/dev/null || true
            ip link set up $iface 2>/dev/null || true
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] Interface is UP"
            LAST_DEVICE_STATE[$iface]="up"
            return 0
        else
            echo "$(date '+%Y-%m-%d %H:%M:%S'): [$iface] ERROR: Interface not found after slcand restart (timeout)"
            LAST_DEVICE_STATE[$iface]="timeout"
            return 1
        fi
    fi
}

# Initialize devices on startup
echo "=== CAN Interface Monitor (can_rear + can_front) ==="
echo "CAN Bitrate: $CAN_BITRATE bps"
echo "Serial Baudrate: $SLCAN_BAUDRATE bps"
echo ""

# Resolve and display device information
for iface in "${!CAN_CONFIG[@]}"; do
    dev_symlink=$(echo "${CAN_CONFIG[$iface]}" | cut -d: -f1)
    serial=$(echo "${CAN_CONFIG[$iface]}" | cut -d: -f2)
    
    real_device=$(resolve_device "$dev_symlink")
    if [ -z "$real_device" ] || [ ! -e "$real_device" ]; then
        real_device=$(find_device_by_serial "$serial")
    fi
    
    if [ -n "$real_device" ] && [ -e "$real_device" ]; then
        echo "✓ $iface: $real_device (serial: $serial)"
    else
        echo "✗ $iface: Device not found (serial: $serial)"
    fi
done

echo ""
echo "Starting monitoring loop..."
echo ""

# Main monitoring loop
while true; do
    for iface in "${!CAN_CONFIG[@]}"; do
        monitor_can_interface "$iface"
    done
    sleep 10
done
