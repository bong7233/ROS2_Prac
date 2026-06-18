# ROS 2 AMR Learning Guide

Korean version: [01_ros2_amr_learning_guide.md](../01_ros2_amr_learning_guide.md)

This guide explains how to learn ROS 2 through an AGV/AMR control-system portfolio. The goal is not to run a few tutorial nodes, but to understand how a real robot PC can manage device drivers, safety, control, diagnostics, and operator tooling.

## 1. Mindset Shift

Legacy AGV software often puts UI, serial communication, TCP/IP communication, CAN commands, state machines, logging, and manual control into one large application. That style can work early, but it becomes hard to test, debug, extend, and integrate with ROS tools.

ROS 2 encourages a different structure:

- Device communication belongs in driver nodes.
- Control and safety decisions belong in controller/safety nodes.
- The UI observes state and sends operator commands.
- Nodes communicate through topics, services, actions, parameters, and launch files.
- Mock drivers should use the same interfaces as future real drivers.

## 2. ROS 2 Concepts Used Here

| Concept | Meaning | AMR Example |
| --- | --- | --- |
| Node | One executable responsibility | `mock_battery_driver`, `safety_monitor` |
| Topic | Continuous data stream | `/battery_state`, `/odom` |
| Service | Short request/response | `/set_io`, `/reset_fault` |
| Action | Long-running task with feedback | Nav2 goal later |
| Parameter | Runtime configuration | wheel radius, timeout, IP address |
| Launch | Start multiple nodes together | `mock_robot.launch.py` |
| TF | Coordinate transforms | `odom -> base_link` |
| QoS | Communication policy | sensor vs command behavior |
| Diagnostics | Health reporting | `/diagnostics` |

## 3. Topic, Service, Action Selection

| Need | Use |
| --- | --- |
| Sensor or state stream | Topic |
| Velocity command | Topic |
| IO output command | Service |
| Fault reset | Service |
| Navigation goal | Action |
| Device port, IP, threshold | Parameter |

## 4. Recommended Learning Order

1. Install Ubuntu 24.04 and ROS 2 Jazzy.
2. Learn `ros2 run`, `ros2 launch`, `ros2 topic`, `ros2 service`, and `ros2 param`.
3. Build with `colcon build --symlink-install`.
4. Read `amr_interfaces` first.
5. Run the mock robot launch.
6. Inspect `/battery_state`, `/io_state`, `/motor_state`, `/safety_state`, `/robot_state`, and `/odom`.
7. Run `amr_tools health_report`.
8. Run `amr_tools fault_scenario` and watch safety behavior.
9. Record and replay rosbag data.
10. Add Qt, Gazebo, ros2_control, and Nav2 later.

## 5. Why This Project Starts Before Nav2

Nav2 is valuable, but it needs a reliable robot base first:

- `/cmd_vel` input path
- command timeout
- safety gate
- `/odom`
- `odom -> base_link` TF
- `base_link -> laser` TF later
- diagnostics
- robot state and fault handling

This repository builds that foundation before adding autonomous navigation.

## 6. Python and C++ Roles

C++ is used for runtime nodes that should be deterministic and close to robot behavior:

- drivers
- safety gate
- base controller
- odometry
- system state

Python is used where it is strongest:

- launch files
- health reports
- fault scenario scripts
- rosbag/log analysis
- integration tests
- field automation

This split is especially useful for an FAE portfolio.

