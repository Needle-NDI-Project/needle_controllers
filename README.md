# Needle Controllers

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Controllers](#controllers)
6. [Usage](#usage)
7. [Technical Details](#technical-details)

## Overview

The **Needle Controllers** repository provides ROS2 controllers for robotic needle manipulation systems. It includes controllers for both autonomous and teleoperated needle control, implementing advanced algorithms for precise needle steering and position control.

### Features

- Broyden-based velocity controller for autonomous needle steering
- Space Navigator teleoperation interface for manual control
- Keyboard teleoperation interface for accessible control
- Real-time performance with ROS2 Control framework integration
- Support for position and velocity command interfaces
- Thread-safe implementations with proper lifecycle management

## Repository Structure

```plaintext
needle_controllers/
├── broyden_needle_controller/          # Broyden-based autonomous controller
├── spacenav_teleop_needle_controller/  # Space Navigator teleoperation controller
├── keyboard_teleop_needle_controller/  # Keyboard teleoperation controller
└── README.md                          # This file
```

## Prerequisites

### Required Software

- Ubuntu 22.04 or later
- ROS2 Humble or later
- C++17 compatible compiler
- CMake 3.8+

### Dependencies

```xml
<depend>controller_interface</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>geometry_msgs</depend>
<depend>eigen3_cmake_module</depend>
<depend>Eigen3</depend>
```

## Installation

1. Create a new workspace:

    ```bash
    mkdir -p needle_ws/src
    cd needle_ws/src
    ```

2. Clone the repository:

    ```bash
    git clone https://github.com/your-org/needle_controllers.git
    ```

3. Install dependencies:

    ```bash
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the workspace:

    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

5. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Controllers

### Broyden Needle Controller

A velocity-based controller implementing Broyden's method for numerical Jacobian estimation:

- Subscribes to desired and measured tip positions
- Updates internal Jacobian estimate
- Generates smooth velocity commands

See [broyden_needle_controller/README.md](broyden_needle_controller/README.md) for details.

### SpaceNav Teleop Controller

A teleoperation controller for manual needle guidance using a 3D Space Navigator:

- Maps 3D mouse inputs to needle velocities
- Direct velocity control interface
- Configurable axis mapping

See [spacenav_teleop_needle_controller/README.md](spacenav_teleop_needle_controller/README.md) for details.

### Keyboard Teleop Controller

A teleoperation controller for accessible needle control using standard keyboard input:

- Maps keyboard commands to needle velocities
- Compatible with standard ROS2 keyboard teleoperation nodes
- Intuitive directional control

See [keyboard_teleop_needle_controller/README.md](keyboard_teleop_needle_controller/README.md) for details.

## Usage

### Loading Controllers

```bash
# Load Broyden controller
ros2 control load_controller broyden_needle_controller --type needle_controllers/BroydenController

# Load SpaceNav controller
ros2 control load_controller spacenav_teleop_needle_controller --type needle_controllers/SpacenavTeleopController

# Load Keyboard controller
ros2 control load_controller keyboard_teleop_needle_controller --type needle_controllers/KeyboardTeleopController
```

### Activating Controllers

```bash
# Activate Broyden controller
ros2 control switch_controllers --start broyden_needle_controller

# Activate SpaceNav controller
ros2 control switch_controllers --start spacenav_teleop_needle_controller

# Activate Keyboard controller
ros2 control switch_controllers --start keyboard_teleop_needle_controller
```

### Configuration

Example controller configurations in YAML:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

broyden_needle_controller:
  ros__parameters:
    joints: ['joint1', 'joint2', 'joint3']
    interface_name: velocity
```

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

keyboard_teleop_needle_controller:
  ros__parameters:
    joints: ['joint1', 'joint2', 'joint3']
    interface_name: velocity
    robot_description: "robot_description"
    robot_base_link: "base_link"
    end_effector_link: "needle_tip"
```

## Technical Details

### Performance Considerations

- Update rates: Configurable up to 1kHz
- Thread-safe implementations
- Real-time compatible
- Lock-free data structures where possible

### Safety Features

- Velocity limits enforcement
- Workspace boundary checking
- Emergency stop handling
- Proper cleanup on deactivation
- Input validation and filtering
