# Table of Contents

- [Table of Contents](#table-of-contents)
- [Robile Safety Behaviour Tree](#robile-safety-behaviour-tree)
  - [Features](#features)
  - [Safety Behaviors](#safety-behaviors)
    - [🚨 Collision Avoidance](#-collision-avoidance)
    - [🔋 Battery Charging Mode](#-battery-charging-mode)
    - [🟢 Normal Operation](#-normal-operation)
  - [Node Structure](#node-structure)
  - [Behavior Tree Structure](#behavior-tree-structure)
  - [Safety Behaviors](#safety-behaviors-1)
- [📋 Prerequisites](#-prerequisites)
  - [Required ROS2 packages:](#required-ros2-packages)
  - [Install py\_trees and related packages](#install-py_trees-and-related-packages)
  - [Verify installation](#verify-installation)
- [Installation](#installation)
  - [Build the package](#build-the-package)
  - [Start the Robile Simulation](#start-the-robile-simulation)
  - [Run the safety node](#run-the-safety-node)
  - [In another terminal, monitor the state](#in-another-terminal-monitor-the-state)
- [🔍 Monitoring](#-monitoring)
  - [Node Status](#node-status)
  - [Monitor commands](#monitor-commands)
  - [Monitor inputs](#monitor-inputs)
  - [Check frequencies](#check-frequencies)
  - [Output Color Coding](#output-color-coding)
  - [Understanding the Output](#understanding-the-output)
- [Configuration Guide](#configuration-guide)
  - [Adjustable Parameters](#adjustable-parameters)
    - [Quick Reference Table](#quick-reference-table)
- [🧪 Testing](#-testing)
  - [Test Structure](#test-structure)
  - [Running Tests](#running-tests)
    - [Using colcon (recommended for ROS2)](#using-colcon-recommended-for-ros2)
    - [Using pytest directly](#using-pytest-directly)
    - [Run specific test file](#run-specific-test-file)
    - [Run with coverage](#run-with-coverage)
  - [Test Suites](#test-suites)
    - [Behaviour Tests](#behaviour-tests)
    - [Structure Tests](#structure-tests)
  - [Test Coverage Details](#test-coverage-details)
    - [BatteryStatus2bb Tests](#batterystatus2bb-tests)
    - [LaserScan2bb Tests](#laserscan2bb-tests)
    - [Rotate Tests](#rotate-tests)
    - [StopMotion Tests](#stopmotion-tests)
    - [StateMonitor Tests](#statemonitor-tests)
    - [Tree Structure Tests](#tree-structure-tests)
- [🔄 Continuous Integration](#-continuous-integration)
  - [CI Workflow](#ci-workflow)
  - [Triggering CI](#triggering-ci)
    - [Automatic Triggers](#automatic-triggers)
    - [Manual Trigger](#manual-trigger)
  - [CI Pipeline Steps](#ci-pipeline-steps)
  - [CI Environment](#ci-environment)
  - [CI Configuration File](#ci-configuration-file)
  - [Viewing CI Results](#viewing-ci-results)
  - [Adding CI Badge to Repository](#adding-ci-badge-to-repository)
- [Robile Safety State Machine (SMACH)](#robile-safety-state-machine-smach)
  - [Overview](#overview)
  - [State Machine Architecture](#state-machine-architecture)
    - [States](#states)
    - [State Transitions](#state-transitions)
  - [Prerequisites](#prerequisites)
  - [Quick Start Guide](#quick-start-guide)
    - [1. Start the Robile Simulation](#1-start-the-robile-simulation)
    - [2. Run the Safety State Machine](#2-run-the-safety-state-machine)
    - [3. Expected Behavior](#3-expected-behavior)
    - [4. Triggering State Transitions](#4-triggering-state-transitions)
      - [Collision Detection](#collision-detection)
      - [Low Battery](#low-battery)
      - [Emergency Shutdown](#emergency-shutdown)
  - [Visualizing the State Machine](#visualizing-the-state-machine)
    - [Using SMACH Viewer](#using-smach-viewer)
  - [Development](#development)
    - [Adding New States](#adding-new-states)
  - [References](#references)





# Robile Safety Behaviour Tree

A ROS2 node implementing safety functionalities for the Robile robot using a behaviour tree architecture. The node monitors battery levels and laser scan data to ensure safe operation, triggering appropriate responses when safety thresholds are breached.

## Features

- **Battery Monitoring**: Continuously monitors battery voltage and triggers rotation behavior when levels drop below threshold
- **Collision Avoidance**: Uses laser scan data to detect obstacles and performs emergency stops
- **Priority-based Safety**: Implements a priority system where collision avoidance takes precedence over low battery responses
- **Real-time State Monitoring**: Provides clear visual feedback of the robot's current state and state transitions

## Safety Behaviors

### 🚨 Collision Avoidance
- **Trigger**: Obstacle detected within 1.0 meter range
- **Action**: Immediate emergency stop of all robot motion
- **Priority**: Highest priority - overrides all other behaviors

### 🔋 Battery Charging Mode  
- **Trigger**: Battery level drops below 30%
- **Action**: Rotates robot in place at 1.0 rad/s angular velocity
- **Priority**: Medium priority - overridden by collision detection

### 🟢 Normal Operation
- **Trigger**: No safety concerns detected
- **Action**: Robot remains idle, ready for normal operation
- **Priority**: Default state

## Node Structure

- **Node Name**: `safety_bt`
- **Package**: `robile_safety`
- **Input Topics**:
  - `/battery_voltage` (std_msgs/Float32) - Battery voltage monitoring
  - `/scan` (sensor_msgs/LaserScan) - Laser scan data for obstacle detection
- **Output Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for robot control

## Behavior Tree Structure
```
root (Parallel)
├── Topics2BB (Sequence)
│ ├── BatteryStatus2bb → Writes: battery, battery_low_warning
│ └── LaserScan2bb → Writes: laser_scan, collision_warning, min_distance
└── Priorities (Selector)
├── CollisionPriority (Sequence)
│ ├── CollisionDetected? (Condition)
│ └── StopMotion (Emergency Stop)
├── BatteryPriority (Sequence)
│ ├── BatteryLow? (Condition)
│ └── Rotate (Rotation Action)
└── Idle (Running - Default)
```

## Safety Behaviors

| State                 | Trigger         | Action             | Priority |
| --------------------- | --------------- | ------------------ | -------- |
| 🚨 COLLISION_AVOIDANCE | Obstacle < 1.0m | Emergency stop     | Highest  |
| 🔋 BATTERY_CHARGING    | Battery < 30%   | Rotate @ 1.0 rad/s | Medium   |
| 🟢 NORMAL_OPERATION    | No issues       | Idle               | Default  |

# 📋 Prerequisites

- ROS2 Humble (or compatible distribution)
- Python 3.8+
## Required ROS2 packages:
  ```bash
  sudo apt install ros-${ROS_DISTRO}-py-trees-ros \
                   ros-${ROS_DISTRO}-py-trees \
                   ros-${ROS_DISTRO}-geometry-msgs \
                   ros-${ROS_DISTRO}-std-msgs \
                   ros-${ROS_DISTRO}-sensor-msgs
  ```
## Install py_trees and related packages
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-py-trees-ros ros-${ROS_DISTRO}-py-trees
```

## Verify installation
```bash
python3 -c "import py_trees, py_trees_ros; print('Dependencies OK')"
```

# Installation

Clone or place the `robile_safety` file in your ROS2 workspace src
```bash
git clone https://github.com/SuhailRafi/robile_safety.git
```

## Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select robile_safety
source install/setup.bash
```
## Start the Robile Simulation

Follow your existing guide to launch the Robile robot in Gazebo simulation.

## Run the safety node

```bash
ros2 run robile_safety safety_bt
```

## In another terminal, monitor the state
```bash
ros2 topic echo /battery_voltage
ros2 topic echo /scan

# # Normal battery
ros2 topic pub /battery_voltage std_msgs/Float32 "data: 45.0" -r 1

# Low battery (triggers rotation)
ros2 topic pub /battery_voltage std_msgs/Float32 "data: 25.0" -r 1
```
# 🔍 Monitoring
## Node Status
```bash
ros2 node list
ros2 node info /safety_bt
```
## Monitor commands
```bash
ros2 topic echo /cmd_vel
```
## Monitor inputs
```bash
ros2 topic echo /battery_voltage
ros2 topic echo /scan
```
## Check frequencies
```bash
ros2 topic hz /battery_voltage
ros2 topic hz /scan
```

## Output Color Coding

| Color    | State               | Meaning                     |
| -------- | ------------------- | --------------------------- |
| 🟢 Green  | NORMAL_OPERATION    | All systems normal          |
| 🟡 Yellow | BATTERY_CHARGING    | Low battery response active |
| 🔴 Red    | COLLISION_AVOIDANCE | Emergency stop active       |
| ⚫ Gray   | UNKNOWN             | Initial/unknown state       |

## Understanding the Output

- **Tick Counter**: Increments every 100ms (configurable)
- **Time**: Seconds since node startup
- **Battery Status**: 🟢 = normal, 🟡 = low, 🔴 = critical
- **Obstacle Status**: 🟢 = safe, 🟡 = warning, 🔴 = danger
- **State Transitions**: Highlighted with border for visibility

The output is designed to be easily readable in real-time while providing comprehensive debugging information when needed.

# Configuration Guide

## Adjustable Parameters

The Robile Safety Behaviour Tree can be customized through various parameters to match your specific hardware and safety requirements.

### Quick Reference Table

| Parameter         | Class              | Default Value | Description                      | Safe Range      |
| ----------------- | ------------------ | ------------- | -------------------------------- | --------------- |
| Battery Threshold | `BatteryStatus2bb` | 30.0%         | Low battery warning level        | 10.0% - 50.0%   |
| Safe Distance     | `LaserScan2bb`     | 1.00m         | Minimum obstacle distance        | 0.5m - 5.0m     |
| Rotation Speed    | `Rotate`           | 1.0 rad/s     | Angular velocity for low battery | 0.1 - 2.0 rad/s |
| Tick Frequency    | `main`             | 100ms         | Behavior tree update period      | 50ms - 500ms    |

# 🧪 Testing

The package includes a comprehensive test suite to validate all behaviour tree components and ensure safety-critical functionality works correctly.

## Test Structure

```
test/
├── test_behaviours/          # Unit tests for individual behaviours
│   ├── __init__.py
│   ├── test_battery_status.py    # BatteryStatus2bb behaviour tests
│   ├── test_laser_scan.py        # LaserScan2bb behaviour tests
│   ├── test_rotate.py            # Rotate behaviour tests
│   ├── test_state_monitor.py     # StateMonitor tests
│   └── test_stop_motion.py       # StopMotion behaviour tests
└── test_structure/           # Integration tests for tree structure
    ├── __init__.py
    └── test_tree_structure.py    # Behavior tree structure validation
```

## Running Tests

### Using colcon (recommended for ROS2)
```bash
cd ~/ros2_ws
colcon build --packages-select robile_safety
colcon test --packages-select robile_safety --event-handlers console_direct+
colcon test-result --verbose
```

### Using pytest directly
```bash
cd ~/ros2_ws/src/robile_safety
pytest test/ -v
```

### Run specific test file
```bash
pytest test/test_behaviours/test_battery_status.py -v
```

### Run with coverage
```bash
pytest test/ -v --cov=robile_safety --cov-report=html
```

## Test Suites

### Behaviour Tests

| Test File | Behaviour | Tests Covered |
|-----------|-----------|---------------|
| `test_battery_status.py` | `BatteryStatus2bb` | Initialization, threshold logic, blackboard persistence, warning states |
| `test_laser_scan.py` | `LaserScan2bb` | Scan processing, collision detection, NaN/infinity handling, edge cases |
| `test_rotate.py` | `Rotate` | Setup, update cycle, velocity publishing, termination |
| `test_stop_motion.py` | `StopMotion` | Emergency stop, zero velocity publishing, topic configuration |
| `test_state_monitor.py` | `StateMonitor` | State determination, transitions, history tracking, color coding |

### Structure Tests

| Test File | Tests Covered |
|-----------|---------------|
| `test_tree_structure.py` | Tree creation, root structure, Topics2BB branch, Priorities branch, collision/battery priority ordering, visualization |

## Test Coverage Details

### BatteryStatus2bb Tests
- ✅ Initialization with default and custom thresholds
- ✅ Topic name configuration
- ✅ Battery warning logic with parametrized levels (0%, 25%, 29.9%, 30%, 30.1%, 50%, 100%)
- ✅ Parent update status handling (SUCCESS, RUNNING, FAILURE)
- ✅ Blackboard value persistence

### LaserScan2bb Tests
- ✅ Initialization and safe range configuration
- ✅ Collision detection with various distance arrays
- ✅ Handling of `inf`, `NaN`, and negative values
- ✅ Empty scan handling
- ✅ Minimum distance calculation

### Rotate Tests
- ✅ Behaviour initialization with angular velocity
- ✅ Publisher setup with ROS2 node
- ✅ Twist message publishing during update
- ✅ Zero velocity on termination
- ✅ Error handling for missing node/publisher

### StopMotion Tests
- ✅ Initialization with default and custom topics
- ✅ Publisher setup
- ✅ All velocity components set to zero
- ✅ Proper Twist message generation
- ✅ Graceful termination

### StateMonitor Tests
- ✅ State determination (NORMAL, COLLISION, BATTERY, UNKNOWN)
- ✅ Priority handling (collision > battery)
- ✅ State change detection and history
- ✅ Color code generation for terminal output
- ✅ Exception handling for missing blackboard values

### Tree Structure Tests
- ✅ Root node creation as Parallel composite
- ✅ Topics2BB branch with BatteryStatus2bb and LaserScan2bb
- ✅ Priorities branch as Selector with correct ordering
- ✅ CollisionPriority sequence with condition and StopMotion
- ✅ BatteryPriority sequence with condition and Rotate
- ✅ Idle behaviour as default fallback
- ✅ Tree visualization generation

# 🔄 Continuous Integration

This package uses GitHub Actions for automated testing on every push and pull request.

## CI Workflow

The CI pipeline is defined in `.github/workflows/ros2_ci.yml` and runs on:
- **Push** to `main` branch
- **Pull requests** targeting `main` branch
- **Manual trigger** via workflow dispatch

## Triggering CI

### Automatic Triggers
- Push commits to `main` branch
- Open/update pull requests to `main`

### Manual Trigger
1. Go to the repository's **Actions** tab on GitHub
2. Select **ROS2 CI** workflow
3. Click **Run workflow**

## CI Pipeline Steps

| Step | Description |
|------|-------------|
| **Checkout** | Clone the repository using `actions/checkout@v4` |
| **Install Tools** | Install `rosdep` and `colcon` build tools |
| **Install Dependencies** | Run `rosdep install` to install ROS2 package dependencies |
| **Build** | Build the package with `colcon build --packages-select robile_safety` |
| **Test** | Run tests with `colcon test` and output results |

## CI Environment

- **Runner**: Ubuntu 22.04
- **Container**: `ros:humble-ros-base` Docker image
- **ROS Distribution**: Humble Hawksbill

## CI Configuration File

The workflow is defined in `.github/workflows/ros2_ci.yml`. Key configuration:

- **Container**: Uses official `ros:humble-ros-base` Docker image for consistent builds
- **Dependencies**: Managed via `rosdep` for automatic resolution
- **Build System**: Uses `colcon` for ROS2 package building and testing

## Viewing CI Results

1. Navigate to the **Actions** tab in the GitHub repository
2. Select a workflow run to view detailed logs
3. Check the **Run tests with colcon** step for test output
4. View `colcon test-result --verbose` output for detailed test results

## Adding CI Badge to Repository

Add this badge to your README to show CI status:
```markdown
![ROS2 CI](https://github.com/SuhailRafi/robile_safety/actions/workflows/ros2_ci.yml/badge.svg)
```



# Robile Safety State Machine (SMACH)

A ROS2 SMACH-based implementation for managing safety functionalities of the Robile robot, including battery monitoring, collision detection, automatic charging, and manual control intervention.

## Overview

This state machine provides autonomous safety management for the Robile robot with the following capabilities:

- **Battery Monitoring**: Continuously monitors battery voltage and initiates charging when below threshold
- **Collision Detection**: Detects obstacles using laser scan data and stops robot motion
- **Automatic Charging**: Rotates the robot base to simulate charging behavior
- **Manual Control**: Allows operator intervention after collision detection
- **Emergency Shutdown**: Service-based shutdown with restart capability

## State Machine Architecture

### States

1. **MONITOR_BATTERY_AND_COLLISION**: Main monitoring state that checks battery level and obstacle proximity
2. **ROTATE_BASE**: Initiates rotation for charging behavior
3. **CHARGING**: Monitors battery level while rotating until fully charged
4. **MANUAL_CONTROL**: Stops robot and waits for manual clearance after collision
5. **SHUTDOWN**: Handles emergency shutdown and waits for restart signal

### State Transitions

```
MONITOR_BATTERY_AND_COLLISION
  ├─> ROTATE_BASE (battery low)
  │     └─> CHARGING
  │           └─> MONITOR_BATTERY_AND_COLLISION (battery charged)
  ├─> MANUAL_CONTROL (collision detected)
  │     └─> MONITOR_BATTERY_AND_COLLISION (clearance received)
  └─> SHUTDOWN (shutdown requested)
        └─> MONITOR_BATTERY_AND_COLLISION (restart requested)
```

## Prerequisites

- ROS2 Humble
- SMACH for ROS2
- Robile simulation running in Gazebo

## Quick Start Guide

### 1. Start the Robile Simulation

Follow your existing guide to launch the Robile robot in Gazebo simulation.

### 2. Run the Safety State Machine

In a new terminal, navigate to the workspace and run:

```bash
ros2 run robile_safety safety_ssm
```

### 3. Expected Behavior

Upon startup, you should see:

```
[INFO] [safety_state_machine]: Starting Safety State Machine
[INFO] [safety_state_machine]: MonitorBatteryAndCollision state initialized
[WARN] [safety_state_machine]: Executing MonitorBatteryAndCollision state
[INFO] [safety_state_machine]: Battery level: 100.0%, Min distance: inf m
```

The state machine will:
- Monitor battery voltage from `/battery_voltage` topic
- Monitor laser scan data from `/scan` topic
- Automatically transition between states based on conditions

### 4. Triggering State Transitions

#### Collision Detection
When an obstacle is detected within 1m, the robot will:
1. Stop immediately
2. Transition to MANUAL_CONTROL state
3. Wait for manual clearance

To resume operation after collision:
```bash
ros2 topic pub /manual_clearance std_msgs/msg/Bool "data: true" --once
```

#### Low Battery
When battery drops below 30%, the robot will:
1. Transition to ROTATE_BASE state
2. Start rotating (simulating charging)
3. Monitor battery until it exceeds threshold
4. Stop rotation and resume monitoring

#### Emergency Shutdown
To trigger emergency shutdown:
```bash
ros2 service call /request_shutdown std_srvs/srv/Trigger
```

To restart after shutdown:
```bash
ros2 service call /request_restart std_srvs/srv/Trigger
```

## Visualizing the State Machine

### Using SMACH Viewer

Since the SMACH Viewer was not fully ported to Humble, running it is complicated to describe in this README.md. The authors of this package had managed to run it, and there is a video recording of the SMACH Viewer in action available to the evaluator.

## Development

### Adding New States
1. Inherit from `BaseState` class
2. Define outcomes in `__init__`
3. Implement `execute(self, userdata)` method
4. Add state to state machine in `main()`

## References

- [SMACH ROS Wiki](https://wiki.ros.org/smach)
- [SMACH Tutorials](https://wiki.ros.org/smach/Tutorials)
- ROS2 Humble Documentation
