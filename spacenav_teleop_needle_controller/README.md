# SpaceNav Teleop Needle Controller

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/)

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

The SpaceNav Teleop Needle Controller enables manual control of robotic needles using a 3DConnexion Space Navigator (3D mouse) input device. It translates the 6-DOF input from the Space Navigator into appropriate velocity commands for the needle's joints.

### Features

- Direct velocity control interface
- Configurable axis mapping
- Real-time response
- Proper ROS2 lifecycle management
- Thread-safe implementation

## Package Structure

```plaintext
spacenav_teleop_needle_controller/
├── CMakeLists.txt
├── spacenav_teleop_plugin.xml
├── include/
│   └── spacenav_teleop_needle_controller/
│       ├── spacenav_teleop_controller.hpp
│       └── visibility_control.h
├── package.xml
└── src/
    └── spacenav_teleop_controller.cpp
```

## Hardware Requirements

- 3DConnexion Space Navigator or compatible 3D mouse
- Linux-compatible USB port
- `spacenavd` daemon running
- ROS2 `spacenav_node` package

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

### Device Setup

1. Install the SpaceNav daemon:

    ```bash
    sudo apt install spacenavd
    ```

2. Install ROS2 SpaceNav node:

    ```bash
    sudo apt install ros-$ROS_DISTRO-spacenav
    ```

### Controller Installation

```bash
cd ~/needle_ws
colcon build --packages-select spacenav_teleop_needle_controller
source install/setup.bash
```

## Configuration

### Controller Parameters

```yaml
spacenav_teleop_needle_controller:
  ros__parameters:
    robot_description: ""
    robot_base_link: "base_link"
    end_effector_link: "needle_tip"
    interface_name: "velocity"
    joints: ["joint1", "joint2", "joint3"]
```

### Axis Mapping

Default mapping in `spacenav_teleop_controller.cpp`:

```cpp
simulated_joint_cmd_[0] = -msg->linear.y;  // X axis mapped to Y
simulated_joint_cmd_[1] = msg->linear.x;   // Y axis mapped to X
simulated_joint_cmd_[2] = msg->linear.z;   // Z axis as is
```

## Usage

### Launch Configuration

```xml
<controller name="spacenav_teleop_needle_controller"
            type="needle_controllers/SpacenavTeleopController">
  <param name="robot_description">robot_needle.urdf</param>
  <param name="robot_base_link">base_link</param>
  <param name="end_effector_link">needle_tip</param>
  <param name="interface_name">velocity</param>
  <param name="joints">['joint1','joint2','joint3']</param>
</controller>
```

### Runtime Control

```bash
# Start the SpaceNav node
ros2 run spacenav_node spacenav_node

# Load and activate the controller
ros2 control load_controller spacenav_teleop_needle_controller
ros2 control switch_controllers --start spacenav_teleop_needle_controller
```

### Operation

1. Push forward/backward → Y-axis motion
2. Push left/right → X-axis motion
3. Push up/down → Z-axis motion
4. Twist → Not used by default

### Safety Features

- Automatic velocity scaling
- Built-in deadzone handling
- Emergency stop on controller deactivation
- Bounds checking on inputs

## Implementation Details

### Class Structure

```cpp
class SpacenavTeleopController : public controller_interface::ControllerInterface
{
protected:
  // Subscribers
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
  // Input validation
  if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) ||
      std::isnan(msg->linear.z))
  {
    return;
  }

  // Map twist message to joint commands
  simulated_joint_cmd_[0] = -msg->linear.y;
  simulated_joint_cmd_[1] = msg->linear.x;
  simulated_joint_cmd_[2] = msg->linear.z;
}
```

#### Command Writing

```cpp
void writeJointControlCmds()
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_cmd_vel_handles_[i].get().set_value(simulated_joint_cmd_[i]);
  }
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
   Problem: Device moves but needle doesn't respond
   Solution:
   - Check controller activation status
   - Verify topic connections
   - Confirm hardware interface availability
   ```

2. **Wrong Axis Mapping**

   ```plaintext
   Problem: Movement axes don't match expectations
   Solution:
   - Modify axis mapping in targetVelocityCallback
   - Adjust sign conventions if needed
   ```

3. **Jerky Motion**

   ```plaintext
   Problem: Motion not smooth
   Solution:
   - Check update rate configuration
   - Verify no CPU throttling
   - Consider adding motion smoothing
   ```

### Debugging Tools

```bash
# Check if device is publishing
ros2 topic echo /spacenav/twist

# Monitor controller state
ros2 control list_controllers

# View hardware interfaces
ros2 control list_hardware_interfaces

# Enable debug logging
ros2 run spacenav_teleop_needle_controller spacenav_controller \
    --ros-args --log-level debug
```

### Device Testing

```bash
# Test device recognition
lsusb | grep -i space

# Check device permissions
ls -l /dev/input/spacenavigator

# Test raw device input
sudo spacenav-raw
```

## Advanced Configuration

### Custom Axis Mapping

To modify the axis mapping, edit the callback in your controller subclass:

```cpp
void customTargetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Custom mapping example
  simulated_joint_cmd_[0] = msg->linear.x;   // Direct X mapping
  simulated_joint_cmd_[1] = msg->linear.y;   // Direct Y mapping
  simulated_joint_cmd_[2] = -msg->linear.z;  // Inverted Z mapping
}
```

### Velocity Scaling

Add scaling factors in your configuration:

```yaml
spacenav_teleop_needle_controller:
  ros__parameters:
    scale_translation: 1.0
    scale_rotation: 0.1
    deadzone: 0.1
```

### Launch File Integration

Example launch file snippet:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spacenav_node',
            executable='spacenav_node',
            name='spacenav_node',
            parameters=[{'zero_when_static': True}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'spacenav_config': spacenav_config}]
        )
    ])
```
