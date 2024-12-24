# Broyden Needle Controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [Theory of Operation](#theory-of-operation)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Configuration](#configuration)
7. [Usage](#usage)
8. [API Reference](#api-reference)
9. [Troubleshooting](#troubleshooting)

## Overview

The Broyden Needle Controller implements a velocity-based controller using Broyden's method for numerical Jacobian estimation. This controller is designed for autonomous needle steering applications where the relationship between joint velocities and tip motion needs to be continuously estimated and updated.

## Package Structure

```plaintext
broyden_needle_controller/
├── CMakeLists.txt
├── broyden_plugin.xml
├── include/
│   └── broyden_needle_controller/
│       ├── broyden_controller.hpp
│       └── visibility_control.h
├── package.xml
└── src/
    └── broyden_controller.cpp
```

## Theory of Operation

### Broyden's Method

The controller uses Broyden's method to update an estimate of the Jacobian matrix relating joint velocities to tip motion:

```math
J_{k+1} = J_k + \frac{(y_k - J_k \cdot x_k)}{x_k^T x_k} x_k^T
```

where:

- `J_k` is the current Jacobian estimate
- `x_k` is the change in joint position
- `y_k` is the observed change in tip position

### Control Flow

1. Receive desired tip position (`cmd_tip`)
2. Receive measured tip position (`msr_tip`)
3. Calculate position error
4. Update Jacobian estimate
5. Compute velocity commands
6. Apply commands to joints

## Dependencies

```xml
<depend>control_msgs</depend>
<depend>controller_interface</depend>
<depend>eigen3_cmake_module</depend>
<depend>geometry_msgs</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
```

## Installation

This package is typically built as part of the needle_controllers workspace:

```bash
cd ~/needle_ws
colcon build --packages-select broyden_needle_controller
source install/setup.bash
```

## Configuration

### Controller Parameters

```yaml
broyden_needle_controller:
  ros__parameters:
    robot_description: ""
    robot_base_link: "base_link"
    end_effector_link: "needle_tip"
    interface_name: "velocity"
    joints: ["joint1", "joint2", "joint3"]
```

### Topics

- **Input**:
  - `~/cmd_tip` (geometry_msgs/PointStamped)
  - `~/msr_tip` (geometry_msgs/PointStamped)

- **Output**:
  - Joint velocity commands via ROS2 Control interfaces

## Usage

### Launch Configuration

```xml
<controller name="broyden_needle_controller" type="needle_controllers/BroydenController">
  <param name="robot_description">robot_needle.urdf</param>
  <param name="robot_base_link">base_link</param>
  <param name="end_effector_link">needle_tip</param>
  <param name="interface_name">velocity</param>
  <param name="joints">['joint1','joint2','joint3']</param>
</controller>
```

### Runtime Control

```bash
# Load the controller
ros2 control load_controller broyden_needle_controller

# Activate the controller
ros2 control switch_controllers --start broyden_needle_controller

# Send a target point
ros2 topic pub /broyden_needle_controller/cmd_tip geometry_msgs/msg/PointStamped \
  "header: {frame_id: 'base_link'}
   point: {x: 0.1, y: 0.0, z: 0.0}"
```

## API Reference

### Class Overview

```cpp
class BroydenController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(...) override;
  controller_interface::CallbackReturn on_activate(...) override;
  controller_interface::CallbackReturn on_deactivate(...) override;
  controller_interface::return_type update(...) override;
};
```

### Key Methods

#### broydenUpdate

```cpp
void broydenUpdate(const Eigen::Vector3d &x_j, const Eigen::Vector3d &y_j)
```

Updates the Jacobian estimate based on observed motion.

#### writeJointControlCmds

```cpp
void writeJointControlCmds()
```

Applies computed velocity commands to the hardware interfaces.

## Troubleshooting

### Common Issues

1. **Jacobian Singularity**

   ```plaintext
   Problem: "Jacobian condition number too high"
   Solution: Check for near-singular configurations or adjust damping
   ```

2. **Missing Points**

   ```plaintext
   Problem: "Target or measured point not received"
   Solution: Verify topic publications and transformations
   ```

3. **Interface Errors**

   ```plaintext
   Problem: "Expected N interfaces, got M"
   Solution: Check controller configuration matches hardware
   ```

### Debugging

```bash
# Monitor controller state
ros2 control list_controllers

# Check interface availability
ros2 control list_hardware_interfaces

# Enable debug logging
ros2 run broyden_needle_controller broyden_controller --ros-args --log-level debug
```
