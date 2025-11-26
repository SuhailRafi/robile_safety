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

## Prerequisites

- ROS2 Humble (or compatible distribution)
- `py_trees` and `py_trees_ros` packages
- `geometry_msgs`, `std_msgs`, `sensor_msgs` ROS2 message packages

## Installation

1. Clone or place the `safety_bt.py` file in your ROS2 workspace
2. Ensure it's part of the `robile_safety` package
3. Build your workspace:
   ```bash
   colcon build --packages-select robile_safety

# Build the package
cd ~/ros2_ws
colcon build --packages-select robile_safety
source install/setup.bash

# Run the safety node
ros2 run robile_safety safety_bt

# Run the launch file intead
ros2 launch robile_gazebo gazebo_4_wheel.launch.py