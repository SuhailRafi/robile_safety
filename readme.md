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
  ```
# Build the package
  ```bash
cd ~/ros2_ws
colcon build --packages-select robile_safety
source install/setup.bash
  ```

# Run the safety node
  ```bash
ros2 run robile_safety safety_bt
  ```

# Run the launch file instead
   ```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
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
