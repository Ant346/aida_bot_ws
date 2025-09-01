# Hoverboard Node

A ROS2 package for controlling hoverboard motors with omni-directional capabilities.

## Features

- **Hoverboard Driver**: Main driver for controlling hoverboard motors via serial communication
- **Manual Wheel Control**: Direct control of individual wheels for testing and debugging
- **RS485 Testing**: Test utility for RS485 communication with hoverboard controllers
- **Docker Support**: Complete Docker container with TTY support for development

## Nodes

### hoverboard_driver
Main driver node that:
- Subscribes to `cmd_vel` topic for velocity commands
- Publishes odometry data to `hoverboard_odom` topic
- Provides service for individual wheel testing
- Supports omni-directional movement

### manual_wheel_control
Direct wheel control node that:
- Subscribes to `wheel_speeds` topic (Int32MultiArray)
- Sends direct speed commands to hoverboard controllers
- Useful for testing and debugging

### test_rs485
RS485 communication test utility that:
- Tests serial communication with hoverboard controllers
- Monitors feedback data
- Provides detailed logging of communication

## Parameters

### hoverboard_driver
- `front_serial`: Front hoverboard serial ID (default: '0001')
- `baudrate`: Serial baudrate (default: 115200)
- `timeout`: Serial timeout (default: 0.05)
- `wheel_radius`: Wheel radius in meters (default: 0.095)
- `wheel_base`: Wheel base in meters (default: 0.635)
- `wheel_track`: Wheel track in meters (default: 0.72)
- `max_speed`: Maximum speed (default: 50)
- `test_speed`: Test speed for individual wheel testing (default: 1)

### manual_wheel_control
- `front_port`: Front hoverboard port (default: '/dev/ttyUSB0')
- `rear_port`: Rear hoverboard port (default: '/dev/ttyUSB1')
- `baudrate`: Serial baudrate (default: 115200)
- `timeout`: Serial timeout (default: 0.1)

## Topics

### Subscribed
- `cmd_vel` (geometry_msgs/Twist): Velocity commands
- `wheel_speeds` (std_msgs/Int32MultiArray): Direct wheel speed commands

### Published
- `hoverboard_odom` (geometry_msgs/Twist): Odometry data

### Services
- `hoverboard_omni_driver/test_wheel` (std_srvs/Empty): Test individual wheels

## Docker Usage

### Build the container
```bash
cd hoverboard_node
./build.sh
```

### Run with Docker Compose
```bash
cd aida_bot_ws
docker-compose up hoverboard_node
```

### Run standalone
```bash
cd hoverboard_node
./run.sh
```

### Interactive shell
```bash
docker run -it --rm \
  --name hoverboard_node \
  --network host \
  -v $(pwd)/..:/workspace \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=${DISPLAY} \
  -e ROS_DISTRO=humble \
  --privileged \
  ros_humble_zsh
```

## Building and Running

### Build the package
```bash
cd aida_bot_ws
colcon build --packages-select hoverboard_node
source install/setup.bash
```

### Launch the driver
```bash
ros2 launch hoverboard_node hoverboard_driver.launch.py
```

### Run manual wheel control
```bash
ros2 run hoverboard_node manual_wheel_control
```

### Run RS485 test
```bash
ros2 run hoverboard_node test_rs485
```

## Hardware Requirements

- Hoverboard motors with serial controllers
- USB-to-serial adapters (CH341 or similar)
- Proper udev rules for device access

## udev Rules

Make sure to install the udev rules for proper device access:
```bash
sudo cp hoverboard_driver/99-hoverboard.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## License

This package is licensed under the Apache License 2.0. 