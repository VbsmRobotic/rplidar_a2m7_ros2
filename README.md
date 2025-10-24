# RPLIDAR A2M7 ROS2 Driver

A complete ROS2 driver package for the RPLIDAR A2M7 laser scanner with real hardware integration.

## Features

- **Real RPLIDAR SDK Integration**: Direct communication with A2M7 hardware
- **High Performance**: 15,900 points per scan at 2Hz frequency
- **TF Support**: Automatic transform publishing for RViz2 compatibility
- **Optimized QoS**: Reliable message delivery without queue overflow
- **XML Launch Files**: Clean launch configuration
- **YAML Parameters**: All settings configurable via YAML files

## Hardware Specifications

| Specification | Value |
|---------------|-------|
| **Model** | RPLIDAR A2M7 |
| **Baud Rate** | 256000 |
| **Max Range** | 16.0 meters |
| **Point Count** | 15,900 points/scan |
| **Scan Mode** | Sensitivity |
| **USB Interface** | CP2102 USB-to-serial |
| **Firmware** | 1.27 |
| **Hardware Rev** | 6 |

## Installation

```bash
# Clone the repository
git clone https://github.com/VbsmRobotic/rplidar_a2m7_ros2.git
cd rplidar_a2m7_ros2

# Build the package
colcon build
source install/setup.bash
```

## Usage

### Launch the RPLIDAR A2M7 Node

```bash
# Source your workspace
source install/setup.bash

# Launch the RPLIDAR A2M7 node
ros2 launch rplidar_a2m7 rplidar_a2m7.launch.xml
```

### Launch with RViz2 Visualization

```bash
# Launch RPLIDAR A2M7
ros2 launch rplidar_a2m7 rplidar_a2m7.launch.xml

# In another terminal, launch RViz2
rviz2 -d install/rplidar_a2m7/share/rplidar_a2m7/rviz/rplidar_a2m7.rviz
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D laser scan data |
| `/rplidar_a2m7/status` | `std_msgs/msg/String` | Node status information |

## Frame ID

The default frame ID for published `LaserScan` messages is `laser`.

## Configuration

All parameters are configured in `config/rplidar_a2m7_params.yaml`:

- **Serial Port**: `/dev/ttyUSB1` (auto-detected)
- **Baud Rate**: 256000 (A2M7 specific)
- **Scan Frequency**: 2.0 Hz (optimized for RViz2)
- **Max Distance**: 16.0 meters
- **Point Number**: 15900

## Requirements

- ROS2 Humble/Iron
- RPLIDAR A2M7 hardware
- CP2102 USB-to-serial driver

## Author

**VbsmRobotic**  
GitHub: https://github.com/VbsmRobotic/rplidar_a2m7_ros2