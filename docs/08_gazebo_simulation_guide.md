# Gazebo 시뮬레이션 가이드

English version: [Gazebo Simulation Guide](en/08_gazebo_simulation_guide.en.md)

이 문서는 ROS2_Prac 프로젝트를 Gazebo Harmonic에서 시각화하고 시뮬레이션하는 방법을 설명합니다.

## 1. Gazebo는 어디서 보는가?

Gazebo는 특정 웹페이지가 아닙니다. Ubuntu 데스크톱에서 실행되는 3D 시뮬레이터 GUI 애플리케이션입니다.

```bash
ros2 launch amr_sim gazebo_amr.launch.py
```

위 명령을 실행하면 보통 두 개의 창이 열립니다.

| 창 | 역할 |
| --- | --- |
| Gazebo | 3D 물리 시뮬레이션 창. 창고 world, 로봇, 장애물, LiDAR ray를 볼 수 있습니다. |
| RViz | ROS 2 시각화 창. `/odom`, `/tf`, `/scan`, robot model을 ROS 관점에서 볼 수 있습니다. |

WSL2에서 실행한다면 Windows 11의 WSLg가 Gazebo GUI를 표시합니다. GUI가 열리지 않으면 Ubuntu 데스크톱 PC 또는 WSLg/X server 설정이 필요합니다.

## 2. 설치 패키지

Ubuntu 24.04 + ROS 2 Jazzy 기준:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

Gazebo와 ROS 조합은 공식적으로 `ROS 2 Jazzy + Gazebo Harmonic`이 권장 조합입니다.

## 3. 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. 실행

Gazebo + ROS stack + RViz:

```bash
ros2 launch amr_sim gazebo_amr.launch.py
```

RViz 없이 Gazebo만 보고 싶을 때:

```bash
ros2 launch amr_sim gazebo_amr.launch.py rviz:=false
```

Gazebo가 열리면 상단의 play 버튼이 눌려 있는지 확인합니다. 이 프로젝트 launch는 `gz sim -r` 옵션을 사용하므로 기본적으로 실행 상태로 시작합니다.

## 5. 로봇 움직이기

다른 터미널:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.35}}"
```

흐름:

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

즉, ROS 2 safety gate를 통과한 명령만 Gazebo 로봇으로 들어갑니다.

## 6. 확인 명령

ROS topic:

```bash
ros2 topic list
ros2 topic echo /safety_state
ros2 topic echo /robot_state
ros2 topic hz /odom
ros2 topic hz /scan
```

TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Python health report:

```bash
ros2 run amr_tools health_report --duration 3.0
```

Gazebo 내부 topic:

```bash
gz topic -l
gz topic -e -t /model/amr_demo_robot/odometry
```

## 7. FAE 장애 시나리오를 Gazebo에서 보기

비상정지:

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 topic echo /safety_state
```

예상:

- Gazebo 로봇이 멈춥니다.
- `/cmd_vel_safe`가 zero command가 됩니다.
- `/robot_state`가 `ESTOP`이 됩니다.

복구:

```bash
ros2 run amr_tools fault_scenario estop-off
ros2 run amr_tools fault_scenario recover
```

배터리 critical:

```bash
ros2 run amr_tools fault_scenario battery-critical
ros2 topic echo /robot_state
```

모터 fault:

```bash
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 topic echo /diagnostics
```

## 8. 현재 Gazebo 연동 구조

| Package | 역할 |
| --- | --- |
| `amr_description` | Gazebo SDF 모델, RViz용 URDF/xacro, RViz config |
| `amr_sim` | Gazebo world, bridge config, simulation launch |
| `amr_safety_monitor` | `/cmd_vel`을 검증하고 `/cmd_vel_safe` publish |
| `ros_gz_bridge` | ROS topic과 Gazebo topic 연결 |
| Gazebo DiffDrive | 물리 기반 differential drive 움직임과 odometry publish |

## 9. Mock launch와 Gazebo launch 차이

| Launch | Odometry source | Purpose |
| --- | --- | --- |
| `amr_bringup mock_robot.launch.py` | C++ `diff_drive_base_controller` | ROS 2 로직만 빠르게 학습/검증 |
| `amr_sim gazebo_amr.launch.py` | Gazebo physics + DiffDrive | 3D 물리 시뮬레이션과 센서 시각화 |

Gazebo launch에서는 `amr_base_controller`를 실행하지 않습니다. Gazebo가 `/odom`을 만들기 때문입니다. 대신 `safety_monitor`는 그대로 사용해서 실제 AMR 구조처럼 안전 게이트를 유지합니다.

## 10. 다음 확장

이제 추가하면 좋은 기능:

- Gazebo joint state bridge
- 더 현실적인 caster/wheel friction
- warehouse map 저장
- Nav2 bringup
- SLAM Toolbox
- simulated docking station
- Qt UI에서 Gazebo 시뮬레이션 상태 표시

