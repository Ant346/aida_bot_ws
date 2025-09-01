# AIDA Bot Workspace

A ROS2 workspace for controlling a hoverboard-based robot using a DualShock 4 (DS4) controller. This project provides Docker containers for easy development and deployment.

## 🚀 Quick Start

### Prerequisites

- Docker and Docker Compose
- Linux system (tested on Ubuntu 22.04)
- DualShock 4 controller (for manual control)
- Hoverboard hardware (optional - simulation mode available)

### 1. Clone and Setup

```bash
git clone <repository-url>
cd aida_bot_ws
```

### 2. Build and Run

```bash
# Build all containers
docker compose build

# Start all services
docker compose up -d

# View logs
docker compose logs -f
```

## 📦 Project Structure

```
aida_bot_ws/
├── compose.yaml              # Docker Compose configuration
├── ds4_driver/              # DS4 controller driver
│   ├── docker/              # DS4 driver Docker files
│   └── ds4_driver/          # DS4 driver ROS2 package
├── hoverboard_node/         # Hoverboard control package
│   ├── dockerfile           # Hoverboard container
│   ├── entrypoint.bash      # Container startup script
│   ├── hoverboard_driver/   # Hoverboard driver code
│   └── hoverboard_node/     # ROS2 package structure
└── README.md               # This file
```

## 🎮 Services

### DS4 Driver (`ds4driver`)

The DS4 controller driver that publishes:
- `/status` - Raw controller data
- `/cmd_vel` - Velocity commands (Twist messages)
- `/imu` - IMU data from controller
- `/battery` - Battery status

**Features:**
- Automatic device detection
- Configurable button mappings
- Deadzone support
- IMU data processing

### Hoverboard Node (`hoverboard_node`)

The hoverboard control system that:
- Subscribes to `/cmd_vel` for velocity commands
- Controls hoverboard motors via serial communication
- Publishes odometry data
- Supports simulation mode for testing without hardware

**Features:**
- Omni-directional control
- Serial communication with hoverboard hardware
- Simulation mode for development
- Automatic workspace building
- Interactive zsh shell

## 🛠️ Development

### Accessing Containers

```bash
# Access hoverboard node container
docker exec -it aida_bot_ws-hoverboard_node-1 /bin/zsh

# Access DS4 driver container
docker exec -it aida_bot_ws-ds4driver-1 /bin/zsh
```

### Building the Workspace

The hoverboard container automatically builds the workspace on first startup. To rebuild:

```bash
# From inside the hoverboard container
cd /workspace
colcon build --packages-select hoverboard_node
source install/setup.zsh
```

### Running Individual Components

```bash
# Start hoverboard driver
ros2 launch hoverboard_node hoverboard_driver.launch.py

# Start DS4 twist node (already running via compose)
ros2 launch ds4_driver ds4_twist.launch.xml

# Test cmd_vel publishing
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once
```

## 🎯 Usage

### 1. Basic Control

1. **Start the system:**
   ```bash
   docker compose up -d
   ```

2. **Connect DS4 controller** (via USB or Bluetooth)

3. **Access hoverboard container:**
   ```bash
   docker exec -it aida_bot_ws-hoverboard_node-1 /bin/zsh
   ```

4. **Launch hoverboard driver:**
   ```bash
   ros2 launch hoverboard_node hoverboard_driver.launch.py
   ```

5. **Control with DS4 controller** - The robot will respond to controller input

### 2. Simulation Mode

For testing without hardware, enable simulation mode:

```bash
# Edit the launch file to set simulation_mode: True
# Or modify the parameter in hoverboard_driver.launch.py
```

### 3. Manual Testing

Test individual components:

```bash
# Check topics
ros2 topic list
ros2 topic info /cmd_vel

# Monitor cmd_vel messages
ros2 topic echo /cmd_vel

# Test hoverboard communication
ros2 service call /hoverboard_omni_driver/test_wheel std_srvs/srv/Empty
```

## 🔧 Configuration

### DS4 Controller Mapping

The DS4 controller mapping is configured in `ds4_driver/config/twist_6dof.yaml`:

- **Right Stick Y-axis:** Forward/Backward movement (linear.x)
- **Left Stick X-axis:** Left/Right movement (linear.y)
- **L1/R1 Buttons:** Yaw rotation (angular.z) - rotate left/right
- **No vertical movement** - configured for 4-wheel omni ground robot

### Hoverboard Parameters

Key parameters in `hoverboard_driver.launch.py`:

- `wheel_radius`: 0.095 meters
- `wheel_base`: 0.635 meters  
- `wheel_track`: 0.72 meters
- `max_speed`: 50 (units)
- `simulation_mode`: False/True

### Serial Communication

The hoverboard driver communicates via:
- **Device:** CH341 USB-to-UART bridge
- **Baudrate:** 115200
- **Protocol:** Custom binary protocol

## 🐛 Troubleshooting

### Common Issues

1. **Controller not detected:**
   ```bash
   # Check device permissions
   ls -la /dev/input/js*
   sudo chmod 666 /dev/input/js0
   ```

2. **Serial port not found:**
   ```bash
   # Check USB devices
   lsusb | grep CH341
   ls -la /dev/serial/by-id/
   ```

3. **Permission denied:**
   ```bash
   # Add user to input group
   sudo usermod -a -G input $USER
   # Reboot or log out/in
   ```

4. **Container networking issues:**
   ```bash
   # Check container connectivity
   docker compose ps
   docker compose logs ds4driver
   docker compose logs hoverboard_node
   ```

### Debug Commands

```bash
# Check ROS2 topics
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel

# Check node status
ros2 node list
ros2 node info /hoverboard_driver

# Check parameters
ros2 param list
ros2 param get /hoverboard_driver simulation_mode

# Monitor system resources
htop
docker stats
```

## 📋 Hardware Requirements

### Minimum Setup
- Linux system with Docker
- DualShock 4 controller
- USB connection for controller

### Full Setup
- Hoverboard hardware
- CH341 USB-to-UART bridge
- Motor drivers and power supply
- Physical robot chassis

## 🔄 Development Workflow

1. **Make code changes** in the mounted volumes
2. **Rebuild workspace** (automatic in hoverboard container)
3. **Test changes** using ROS2 tools
4. **Iterate** and refine

### Useful Development Commands

```bash
# Quick rebuild
cd /workspace && colcon build --packages-select hoverboard_node

# Source workspace
source install/setup.zsh

# Run tests
colcon test --packages-select hoverboard_node

# Check code style
flake8 hoverboard_driver/
```

## 📝 License

[Add your license information here]

## 🤝 Contributing

[Add contribution guidelines here]

## 📞 Support

[Add support contact information here] # aida_bot_ws
