# ROS2_Prac

[![CI](https://github.com/bong7233/ROS2_Prac/actions/workflows/ci.yml/badge.svg)](https://github.com/bong7233/ROS2_Prac/actions/workflows/ci.yml)

Korean documentation: [README.md](README.md)

ROS2_Prac is a ROS 2 portfolio project that redesigns a Linux PC based AGV/AMR control program using modern ROS 2 architecture.

The project starts from a realistic industrial robot layout: a robot PC, serial BMS, TCP/IP IO board, CAN motor drive, and TCP/IP laser scanner. The first implementation does not jump directly into SLAM, Nav2, or OpenCV. Instead, it builds the AMR software foundation that those higher-level systems need: device driver nodes, safety command gating, odometry, diagnostics, launch/config management, and field-support tooling.

## Technology Policy

| Layer | Choice | Reason |
| --- | --- | --- |
| OS | Ubuntu 24.04 LTS | Stable target for ROS 2 Jazzy and common industrial PC deployments. |
| ROS 2 | Jazzy Jalisco LTS | Mature LTS baseline with support until May 2029. |
| Runtime language | C++17 | Appropriate for drivers, control loops, safety gate, odometry, and Qt integration. |
| Tooling language | Python | Best fit for launch, diagnostics, rosbag analysis, test automation, and FAE field scripts. |
| Build | colcon + ament | Standard ROS 2 workspace workflow. |
| Operator UI | Qt 6 | Suitable for Linux operator panels and maintainable CMake integration. |

## Current Implementation

The repository currently contains a runnable mock AMR stack.

GitHub Actions CI runs static checks and a ROS 2 Jazzy `colcon build/test` on `ubuntu-24.04`.

| Package | Language | Purpose |
| --- | --- | --- |
| `amr_interfaces` | ROS IDL | Project-specific messages and services |
| `amr_battery_driver` | C++ | Mock BMS, `BatteryState`, diagnostics, battery fault injection |
| `amr_io_driver` | C++ | Mock IO board, IO state, output service, input fault injection |
| `amr_motor_driver` | C++ | Mock motor drive, wheel feedback, motor fault injection |
| `amr_safety_monitor` | C++ | `/cmd_vel` safety gate and `/safety_state` |
| `amr_base_controller` | C++ | Differential drive wheel command, odometry, TF |
| `amr_system_manager` | C++ | Robot mode and fault aggregation |
| `amr_bringup` | Python/YAML | Launch and parameter files |
| `amr_description` | SDF/URDF/RViz | AMR model for Gazebo and RViz |
| `amr_sim` | Gazebo/YAML/Python launch | Gazebo Harmonic world and ROS-Gazebo bridge |
| `amr_operator_ui` | C++/Qt 6 | Clickable operator console, live robot telemetry, manual jog, mode/fault services |
| `amr_tools` | Python | FAE health report and fault scenario CLI |
| `amr_vision` | Python/OpenCV | ArUco docking-marker detection, mock camera, `/docking_state` |
| `amr_docking` | Python | Alignment/approach controller from `/docking_state`, emits `/cmd_vel` |
| `amr_lidar_driver` | Python | Mock 2D LiDAR simulator, `/scan` (Nav2 input prep) |
| `amr_navigation` | Python | Odom-based waypoint following (pure pursuit), emits `/cmd_vel` |

## Quick Start

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch amr_bringup mock_robot.launch.py
```

Run the Qt operator console in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amr_operator_ui operator_ui.launch.py
```

Click the robot in the workspace view to open the right-side operator panel.

Send a manual command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.30}}"
```

Inspect the robot:

```bash
ros2 topic echo /robot_state
ros2 topic echo /safety_state
ros2 topic hz /odom
ros2 run amr_tools health_report --duration 3.0
```

Run FAE scenarios:

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 run amr_tools fault_scenario battery-critical
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 run amr_tools fault_scenario recover
```

Run the vision docking pipeline (ArUco marker detection):

```bash
ros2 launch amr_vision docking_vision.launch.py
ros2 topic echo /docking_state
ros2 run rqt_image_view rqt_image_view /docking_debug_image
```

Run the fully closed-loop mock docking (no Gazebo): the controller drives
`/cmd_vel` -> safety -> base -> `/odom`; the mock camera re-renders the marker
from the new robot pose, so the loop closes and the robot does
`ALIGN -> APPROACH -> DOCKED`. The mock LiDAR provides `/scan` for the
controller's obstacle stop.

```bash
ros2 launch amr_docking dock_closed_loop.launch.py
```

Publish a mock laser scan on its own (Nav2 input prep without Gazebo):

```bash
ros2 launch amr_lidar_driver mock_lidar.launch.py
ros2 topic echo /scan --once
```

Headless bringup with the core Nav2 inputs (TF, odom, scan), no Gazebo:

```bash
ros2 launch amr_bringup display.launch.py            # add rviz:=true for RViz
ros2 run tf2_tools view_frames                        # odom -> base_link -> lidar_link/camera_link
```

`display.launch.py` adds the sensor static transforms (`base_link -> lidar_link`,
`base_link -> camera_link`) and the mock LiDAR to `mock_robot.launch.py`. The base
controller already publishes `odom -> base_link`, completing the `odom -> base_link
-> laser` chain Nav2 expects.

Waypoint following (odom-based autonomy without Nav2); the robot drives a square:

```bash
ros2 launch amr_navigation waypoint_demo.launch.py        # loop:=true to repeat
ros2 topic echo /cmd_vel
```

The command flows through the safety monitor, and `/odom` feedback closes the
loop. Start/stop with `/enable_navigation` (`std_srvs/SetBool`).

Run Gazebo simulation:

```bash
ros2 launch amr_sim gazebo_amr.launch.py
```

Gazebo is not a project web page. It is a desktop 3D simulator GUI. The launch file opens a Gazebo window with the warehouse world and AMR model, and also opens RViz by default.

## Documentation

Korean:

- [ROS 2 AMR learning guide](docs/01_ros2_amr_learning_guide.md)
- [AMR system architecture](docs/02_amr_system_architecture.md)
- [Implementation roadmap](docs/03_implementation_roadmap.md)
- [Reference links](docs/04_reference_links.md)
- [Code walkthrough](docs/05_code_walkthrough.md)
- [Build and development guide](docs/06_build_and_development_guide.md)
- [FAE field guide](docs/07_fae_field_guide.md)
- [Gazebo simulation guide](docs/08_gazebo_simulation_guide.md)
- [Qt operator UI guide](docs/09_qt_operator_ui_guide.md)
- [Vision docking guide](docs/10_vision_docking_guide.md)
- [Dev environment setup (Windows -> Ubuntu/ROS 2)](docs/11_dev_environment_setup.md)

English:

- [ROS 2 AMR learning guide](docs/en/01_ros2_amr_learning_guide.en.md)
- [AMR system architecture](docs/en/02_amr_system_architecture.en.md)
- [Implementation roadmap](docs/en/03_implementation_roadmap.en.md)
- [Reference links](docs/en/04_reference_links.en.md)
- [Code walkthrough](docs/en/05_code_walkthrough.en.md)
- [Build and development guide](docs/en/06_build_and_development_guide.en.md)
- [FAE field guide](docs/en/07_fae_field_guide.en.md)
- [Gazebo simulation guide](docs/en/08_gazebo_simulation_guide.en.md)
- [Qt operator UI guide](docs/en/09_qt_operator_ui_guide.en.md)
- [Vision docking guide](docs/en/10_vision_docking_guide.en.md)
