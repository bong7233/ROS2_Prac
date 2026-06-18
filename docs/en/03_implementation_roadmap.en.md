# Implementation Roadmap

Korean version: [03_implementation_roadmap.md](../03_implementation_roadmap.md)

This roadmap shows how the project can grow from a mock AMR stack into a stronger FAE portfolio.

## M0. Documentation Baseline

Completed:

- README
- ROS 2 AMR learning guide
- architecture document
- implementation roadmap
- reference links
- code walkthrough
- build/development guide
- FAE field guide

## M1. Mock Runtime Stack

Completed:

- custom interfaces
- mock battery driver
- mock IO driver
- mock motor driver
- safety monitor
- differential-drive base controller
- system manager
- mock robot launch
- Python health report
- Python fault scenario CLI

## M2. Operator UI

Next recommended step:

- Qt 6 operator UI
- ROS 2 executor thread
- battery/IO/motor/robot state display
- manual jog
- fault reset
- fault scenario buttons for demo mode

## M3. Better Field Tooling

Add:

- Markdown/JSON incident report export
- rosbag auto-record wrapper
- diagnostics timeline parser
- CAN dump parser
- Modbus register viewer

## M4. Robot Description and RViz

Add:

- URDF/xacro
- `robot_state_publisher`
- wheel and laser frames
- RViz config
- TF validation commands

## M5. Simulation

Add:

- Gazebo Harmonic world
- simulated differential-drive robot
- `ros2_control`
- simulated scan data

## M6. Navigation Readiness

Before Nav2:

- stable `/odom`
- `odom -> base_link`
- `base_link -> laser`
- robot footprint
- scan topic
- command timeout
- safety monitor

## M7. Real Hardware Adapters

Add one real adapter at a time:

- serial BMS adapter
- TCP/IP IO adapter
- SocketCAN motor adapter
- vendor LiDAR launch wrapper

Keep ROS interfaces stable while replacing the transport/protocol layer.

