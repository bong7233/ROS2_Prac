# Build and Development Guide

Korean version: [06_build_and_development_guide.md](../06_build_and_development_guide.md)

This guide explains how to edit, build, and run ROS2_Prac on Linux and Windows.

## 1. Recommended Environments

| Environment | Recommendation |
| --- | --- |
| Ubuntu 24.04 PC | Best option for ROS 2 Jazzy development |
| Windows 11 + WSL2 Ubuntu 24.04 | Recommended for Windows laptops |
| Native Windows ROS 2 | Useful for limited experiments, less ideal for robot PC work |
| Docker | Useful for CI and experiments, needs extra setup for GUI, USB, CAN, and networking |

## 2. Ubuntu Setup

Install ROS 2 Jazzy using the official Ubuntu guide:

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [colcon tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

Install common tools:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  ros-jazzy-desktop \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

## 3. Clone, Build, Run

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/bong7233/ROS2_Prac.git
cd ROS2_Prac

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup mock_robot.launch.py
```

Send a command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.30}}"
```

Inspect:

```bash
ros2 run amr_tools health_report --duration 3.0
ros2 topic echo /safety_state
ros2 topic hz /odom
```

## 4. Build One Package

```bash
colcon build --symlink-install --packages-select amr_safety_monitor
source install/setup.bash
```

After changing custom interfaces, rebuild the full workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Windows Development

The recommended Windows workflow is WSL2 with Ubuntu 24.04.

```powershell
wsl --install -d Ubuntu-24.04
```

Then install ROS 2 Jazzy inside WSL and build the project there.

Recommended:

```bash
cd ~
mkdir -p ros2_ws
cd ros2_ws
git clone https://github.com/bong7233/ROS2_Prac.git
```

Avoid building under `/mnt/c/...` because colcon builds can be slower and symlink behavior can be less convenient.

Use VS Code with:

- WSL extension
- C/C++
- CMake Tools
- Python
- optional ROS extension

Native Windows ROS 2 can be used for experiments, but this project targets a Linux robot PC, so WSL2 or Ubuntu is preferred.

## 6. Useful Commands

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 interface show amr_interfaces/msg/SafetyState
ros2 interface show amr_interfaces/srv/InjectMotorFault
ros2 param list /safety_monitor
ros2 run tf2_ros tf2_echo odom base_link
```
