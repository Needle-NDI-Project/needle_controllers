# Keyboard Teleop Needle Controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [Hardware Requirements](#hardware-requirements)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Configuration](#configuration)
7. [Usage](#usage)
8. [Implementation Details](#implementation-details)
9. [Troubleshooting](#troubleshooting)

## Overview

The Keyboard Teleop Needle Controller enables manual control of robotic needles using standard keyboard input. It translates keyboard commands into velocity controls through ROS2's `geometry_msgs/Twist` messages, providing an accessible and intuitive interface for needle manipulation.

### Features

- Direct velocity control interface through keyboard input
- Compatible with standard ROS2 keyboard teleoperation nodes
- Real-time response
- Proper ROS2 lifecycle management
- Thread-safe implementation

## Package Structure

```plaintext
keyboard_teleop_needle_controller/
├── CMakeLists.txt
├── keyboard_teleop_plugin.xml
├── include/
│   └── keyboard_teleop_needle_controller/
│       ├── keyboard_teleop_controller.hpp
│       └── visibility_control.h
├── package.xml
└── src/
    └── keyboard_teleop_controller.cpp
```

## Hardware Requirements

- Standard keyboard input device
- ROS2-compatible computer running Linux
- Access to the needle system hardware interface

## Dependencies

```xml
<depend>control_msgs</depend>
<depend>controller_interface</depend>
<depend>eigen3_cmake_module</depend>
<depend>geometry_msgs</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
```

## Installation

### Controller Installation

```bash
cd ~/needle_ws
colcon build --packages-select keyboard_teleop_needle_controller
source install/setup.bash
```

### Keyboard Teleoperation Node

Install the standard ROS2 keyboard teleoperation package:

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

## Configuration

### Controller Parameters

```yaml
keyboard_teleop_needle_controller:
  ros__parameters:
    robot_description: ""
    robot_base_link: "base_link"
    end_effector_link: "needle_tip"
    interface_name: "velocity"
    joints: ["joint1", "joint2", "joint3"]
```

### Default Key Mappings

Using the standard `teleop_twist_keyboard` node:

- 'i': Move forward
- ',' or 'k': Move backward
- 'j': Move left
- 'l': Move right
- 'u': Move up
- 'o': Move down
- 'q': Quit

## Usage

### Launch Configuration

```xml
<controller name="keyboard_teleop_needle_controller"
            type="needle_controllers/KeyboardTeleopController">
  <param name="robot_description">robot_needle.urdf</param>
  <param name="robot_base_link">base_link</param>
  <param name="end_effector_link">needle_tip</param>
  <param name="interface_name">velocity</param>
  <param name="joints">['joint1','joint2','joint3']</param>
</controller>
```

### Runtime Control

```bash
# Start the keyboard teleoperation node
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/keyboard_teleop_needle_controller/cmd_vel

# Load and activate the controller
ros2 control load_controller keyboard_teleop_needle_controller
ros2 control switch_controllers --start keyboard_teleop_needle_controller
```

### Safety Features

- Automatic velocity scaling
- Emergency stop on controller deactivation
- Bounds checking on inputs
- NaN value filtering

## Implementation Details

### Class Structure

```cpp
class KeyboardTeleopController : public controller_interface::ControllerInterface
{
protected:
  // Subscriber for keyboard-generated Twist messages
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tgt_velocity_sub_;

  // State variables
  std::vector<std::string> joint_names_;
  Eigen::Vector3d simulated_joint_cmd_;

  // Interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      joint_cmd_vel_handles_;
};
```

### Key Methods

#### Velocity Callback

```cpp
void targetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Map keyboard commands to joint velocities
  simulated_joint_cmd_[0] = msg->linear.x;  // Forward/backward
  simulated_joint_cmd_[1] = msg->linear.y;  // Left/right
  simulated_joint_cmd_[2] = msg->linear.z;  // Up/down
}
```

### Real-time Considerations

- Non-blocking operations in callback
- Pre-allocated command vectors
- Lock-free command updates
- Minimal memory allocation during runtime

## Troubleshooting

### Common Issues

1. **No Movement**

   ```plaintext
   Problem: Keyboard input not affecting needle movement
   Solution:
   - Check controller activation status
   - Verify topic connections
   - Confirm keyboard focus in terminal
   ```

2. **Incorrect Movement**

   ```plaintext
   Problem: Needle moves in unexpected directions
   Solution:
   - Verify coordinate frame settings
   - Check axis mappings in configuration
   - Confirm joint order matches hardware
   ```

3. **Latency Issues**

   ```plaintext
   Problem: Delayed response to keyboard input
   Solution:
   - Check system load
   - Verify update rate settings
   - Monitor network performance
   ```

### Debugging Tools

```bash
# Monitor keyboard commands
ros2 topic echo /keyboard_teleop_needle_controller/cmd_vel

# Check controller status
ros2 control list_controllers

# View hardware interfaces
ros2 control list_hardware_interfaces

# Enable debug logging
ros2 run keyboard_teleop_needle_controller keyboard_controller \
    --ros-args --log-level debug
```

### Testing Tools

```bash
# Test keyboard input
ros2 topic pub /keyboard_teleop_needle_controller/cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.1, y: 0.0, z: 0.0}"
```
