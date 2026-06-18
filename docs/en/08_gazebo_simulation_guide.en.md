# Gazebo Simulation Guide

Korean version: [08_gazebo_simulation_guide.md](../08_gazebo_simulation_guide.md)

This guide explains how to visualize and simulate ROS2_Prac in Gazebo Harmonic.

## 1. Where Do You See Gazebo?

Gazebo is not a project web page. It is a desktop 3D simulator GUI.

```bash
ros2 launch amr_sim gazebo_amr.launch.py
```

This launch normally opens two windows:

| Window | Purpose |
| --- | --- |
| Gazebo | 3D physics simulation with the warehouse world, AMR model, obstacles, and LiDAR rays |
| RViz | ROS 2 visualization for `/odom`, `/tf`, `/scan`, and robot model |

On Windows with WSL2, the GUI is displayed through WSLg. If no GUI appears, use an Ubuntu desktop PC or configure WSLg/X server support.

## 2. Install Packages

Ubuntu 24.04 + ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

The recommended pairing for this project is ROS 2 Jazzy with Gazebo Harmonic.

## 3. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. Run

Gazebo + ROS stack + RViz:

```bash
ros2 launch amr_sim gazebo_amr.launch.py
```

Gazebo without RViz:

```bash
ros2 launch amr_sim gazebo_amr.launch.py rviz:=false
```

## 5. Move the Robot

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.35}}"
```

Data flow:

```text
/cmd_vel
  -> safety_monitor
  -> /cmd_vel_safe
  -> ros_gz_bridge
  -> /model/amr_demo_robot/cmd_vel in Gazebo
  -> Gazebo DiffDrive system
  -> /model/amr_demo_robot/odometry
  -> ros_gz_bridge
  -> /odom
```

Only commands that pass the ROS 2 safety gate are sent into Gazebo.

## 6. Inspect

```bash
ros2 topic list
ros2 topic echo /safety_state
ros2 topic echo /robot_state
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run amr_tools health_report --duration 3.0
```

Gazebo internal topics:

```bash
gz topic -l
gz topic -e -t /model/amr_demo_robot/odometry
```

## 7. FAE Fault Scenarios in Gazebo

Estop:

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 topic echo /safety_state
```

Recover:

```bash
ros2 run amr_tools fault_scenario estop-off
ros2 run amr_tools fault_scenario recover
```

Battery critical:

```bash
ros2 run amr_tools fault_scenario battery-critical
ros2 topic echo /robot_state
```

Motor fault:

```bash
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 topic echo /diagnostics
```

## 8. Package Roles

| Package | Role |
| --- | --- |
| `amr_description` | Gazebo SDF model, URDF/xacro, RViz config |
| `amr_sim` | Gazebo world, bridge config, simulation launch |
| `amr_safety_monitor` | Safety gate from `/cmd_vel` to `/cmd_vel_safe` |
| `ros_gz_bridge` | Connect ROS topics and Gazebo topics |
| Gazebo DiffDrive | Physics-based differential drive and odometry |

## 9. Mock Launch vs Gazebo Launch

| Launch | Odometry source | Purpose |
| --- | --- | --- |
| `amr_bringup mock_robot.launch.py` | C++ `diff_drive_base_controller` | Fast ROS 2 logic testing |
| `amr_sim gazebo_amr.launch.py` | Gazebo physics and DiffDrive | 3D simulation and sensor visualization |

The Gazebo launch does not run `amr_base_controller`, because Gazebo publishes `/odom`.

