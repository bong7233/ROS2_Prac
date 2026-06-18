# AMR System Architecture

Korean version: [02_amr_system_architecture.md](../02_amr_system_architecture.md)

This document describes the intended ROS 2 architecture for a Linux PC based AGV/AMR controller.

## 1. Core Principle

The system is split by responsibility:

- UI is not a hardware driver.
- Hardware drivers do not own robot mode policy.
- Safety gates all motion commands.
- The base controller publishes odometry and TF.
- System manager aggregates mode and faults.
- Python tools support field diagnosis and fault reproduction.

## 2. Runtime Layers

```text
Operator and FAE Tools
  Qt UI later, ros2 CLI, health_report, fault_scenario, rosbag2

Application Layer
  system_manager, safety_monitor

Control Layer
  diff_drive_base_controller

Driver Layer
  battery, IO, motor, lidar later

Hardware Layer
  BMS, IO board, motor drive, laser scanner, estop circuit
```

## 3. Implemented Packages

| Package | Responsibility |
| --- | --- |
| `amr_interfaces` | Custom ROS messages and services |
| `amr_bringup` | Launch and parameter files |
| `amr_battery_driver` | Mock BMS driver and battery fault injection |
| `amr_io_driver` | Mock IO board and input/output services |
| `amr_motor_driver` | Mock motor drive and motor fault injection |
| `amr_safety_monitor` | Motion command gate |
| `amr_base_controller` | Differential-drive kinematics, odometry, TF |
| `amr_system_manager` | Robot mode and fault summary |
| `amr_tools` | Python FAE tools |

## 4. Command Flow

```text
/cmd_vel
  -> safety_monitor
  -> /cmd_vel_safe
  -> diff_drive_base_controller
  -> /wheel_command
  -> mock_motor_driver
  -> /motor_state
  -> diff_drive_base_controller
  -> /odom and /tf
```

## 5. Safety Model

The software safety monitor blocks motion when:

- estop input is active
- protective stop is active
- battery is critical
- motor fault is active
- state communication is stale
- command timeout is active
- motor drive is disabled

In a real robot, the physical estop must still cut motor enable through hardware. ROS 2 safety is a command gate and visibility layer, not the only safety mechanism.

## 6. Diagnostics Strategy

All runtime C++ nodes publish diagnostics. This supports field debugging:

- Which device is unhealthy?
- Is the command stale?
- Is the motor enabled?
- Is a mock fault active?
- Is the robot in FAULT or ESTOP mode?

Python tools summarize those signals for fast field checks.

