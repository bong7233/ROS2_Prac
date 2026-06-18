# FAE Field Guide

Korean version: [07_fae_field_guide.md](../07_fae_field_guide.md)

This guide explains how to use the project as an FAE-oriented demo and troubleshooting portfolio.

## 1. What This Demonstrates

The project demonstrates:

- ROS 2 graph inspection
- topic/service/interface understanding
- safety gate behavior
- diagnostics-based fault isolation
- rosbag-oriented reproduction workflow
- Python field tooling
- C++ runtime discipline

## 2. Normal Motion Demo

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.20}}"
```

Check:

```bash
ros2 topic echo /safety_state
ros2 topic hz /odom
ros2 run amr_tools health_report --duration 3.0
```

Expected:

- `/cmd_vel` passes through the safety monitor.
- `/cmd_vel_safe` is published.
- `/wheel_command` is generated.
- `/motor_state` changes.
- `/odom` updates.

## 3. Estop Scenario

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 topic echo /safety_state
ros2 topic echo /robot_state
```

Expected:

- `command_allowed=false`
- `estop_active=true`
- robot mode becomes `ESTOP`
- safe command is zero

Recover:

```bash
ros2 run amr_tools fault_scenario estop-off
ros2 run amr_tools fault_scenario recover
```

## 4. Battery Critical Scenario

```bash
ros2 run amr_tools fault_scenario battery-critical
ros2 topic echo /battery_state
ros2 topic echo /robot_state
```

Expected:

- battery percentage becomes critical
- system manager enters fault state
- safety monitor blocks motion

Recover:

```bash
ros2 run amr_tools fault_scenario battery-normal
ros2 run amr_tools fault_scenario recover
```

## 5. Motor Fault Scenario

```bash
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 topic echo /motor_state
ros2 topic echo /diagnostics
```

Expected:

- motor fault becomes active
- motor enable becomes false
- safety monitor blocks motion
- diagnostics reports an ERROR

Recover:

```bash
ros2 run amr_tools fault_scenario motor-clear
ros2 run amr_tools fault_scenario recover
```

## 6. Fault Isolation Table

| Symptom | Check First |
| --- | --- |
| Robot does not move | `/safety_state` |
| `/cmd_vel` exists but `/cmd_vel_safe` is zero | safety gate reason |
| `/cmd_vel_safe` exists but motor speed is zero | `/motor_state`, `/wheel_command` |
| Odometry is missing | `/motor_state`, `/odom` |
| UI state is stale | `health_report` |
| Fault reset fails | `/robot_state`, `/safety_state` |

## 7. rosbag Recording

```bash
ros2 bag record \
  /cmd_vel /cmd_vel_safe /wheel_command /motor_state /odom /tf \
  /battery_state /io_state /safety_state /robot_state /diagnostics
```

Use rosbag to reproduce field issues on a development PC.

