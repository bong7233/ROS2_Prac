# 빌드와 개발 가이드

이 문서는 ROS2_Prac 프로젝트를 Linux와 Windows PC에서 수정, 빌드, 실행하는 방법을 설명합니다. 실제 현업 ROS 2 개발은 Linux가 기본이고, Windows PC는 WSL2 또는 원격 Linux 개발 환경과 함께 쓰는 구성이 가장 안정적입니다.

English version: [Build and Development Guide](en/06_build_and_development_guide.en.md)

## 1. 권장 개발 환경

| 환경 | 권장도 | 설명 |
| --- | --- | --- |
| Ubuntu 24.04 LTS PC | 가장 권장 | ROS 2 Jazzy 개발, 빌드, 실행을 가장 단순하게 할 수 있습니다. |
| Windows 11 + WSL2 Ubuntu 24.04 | 권장 | Windows 노트북에서 VS Code로 개발하면서 Linux ROS 2 환경을 사용할 수 있습니다. |
| Windows native ROS 2 | 보조 | 가능하지만 패키지, 드라이버, Qt, CAN, 현장 네트워크까지 고려하면 WSL2/Linux가 더 낫습니다. |
| Docker | 보조 | CI나 실험에는 좋지만 USB, CAN, GUI, DDS 네트워크 설정은 추가 작업이 필요합니다. |

이 프로젝트의 기준은 `Ubuntu 24.04 + ROS 2 Jazzy + colcon + ament_cmake/ament_python`입니다.

## 2. Linux PC에서 준비

### 2.1 ROS 2 Jazzy 설치

공식 설치 문서를 기준으로 ROS 2 Jazzy를 설치합니다.

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [colcon tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

기본 개발 패키지:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  qt6-base-dev qt6-tools-dev qt6-tools-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  ros-jazzy-desktop \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros
```

처음 한 번만 rosdep을 초기화합니다.

```bash
sudo rosdep init
rosdep update
```

이미 초기화되어 있으면 `sudo rosdep init`은 실패할 수 있습니다. 그 경우 `rosdep update`만 실행하면 됩니다.

### 2.2 저장소 클론

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/bong7233/ROS2_Prac.git
cd ROS2_Prac
```

### 2.3 의존성 설치

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 2.4 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

패키지 하나만 빌드할 때:

```bash
colcon build --symlink-install --packages-select amr_safety_monitor
source install/setup.bash
```

깨끗하게 다시 빌드할 때:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. 실행 방법

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash
ros2 launch amr_bringup mock_robot.launch.py
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.30}}"
```

터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash

ros2 run amr_tools health_report --duration 3.0
ros2 topic echo /robot_state
ros2 topic echo /safety_state
ros2 topic hz /odom
```

터미널 4에서 Qt 운영 UI를 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ROS2_Prac
source install/setup.bash

ros2 launch amr_operator_ui operator_ui.launch.py
```

## 4. FAE 시나리오 실행

비상정지 입력:

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 topic echo /safety_state
ros2 run amr_tools fault_scenario estop-off
```

배터리 critical:

```bash
ros2 run amr_tools fault_scenario battery-critical
ros2 topic echo /robot_state
ros2 run amr_tools fault_scenario battery-normal
```

모터 fault:

```bash
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 topic echo /motor_state
ros2 run amr_tools fault_scenario recover
```

rosbag 기록:

```bash
ros2 bag record \
  /cmd_vel /cmd_vel_safe /wheel_command /motor_state /odom \
  /battery_state /io_state /safety_state /robot_state /diagnostics
```

## 5. 코드 수정 방법

추천 수정 순서:

1. interface 변경이 필요하면 `amr_interfaces`를 먼저 수정합니다.
2. driver node의 topic/service 계약을 바꾸지 않고 내부 구현을 수정합니다.
3. parameter는 `src/amr_bringup/config/mock_robot.yaml`에 추가합니다.
4. launch 변경은 `src/amr_bringup/launch/mock_robot.launch.py`에서 합니다.
5. Python 도구는 `src/amr_tools/amr_tools/` 아래에 추가합니다.
6. README와 관련 docs를 함께 갱신합니다.

C++ 노드 수정 후:

```bash
colcon build --symlink-install --packages-select amr_motor_driver
source install/setup.bash
```

interface 수정 후에는 의존 패키지까지 다시 빌드하는 것이 안전합니다.

```bash
colcon build --symlink-install
source install/setup.bash
```

## 6. Windows PC에서 개발하는 방법

### 6.1 권장: Windows 11 + WSL2

Windows PC에서는 WSL2에 Ubuntu 24.04를 설치하고, 그 안에 ROS 2 Jazzy를 설치하는 방법을 권장합니다.

1. Windows Terminal 또는 PowerShell에서 WSL2 설치

```powershell
wsl --install -d Ubuntu-24.04
```

2. WSL Ubuntu 실행 후 ROS 2 Jazzy 설치

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions
```

3. VS Code 사용 시 권장 확장

- Remote Development
- WSL
- C/C++
- CMake Tools
- Python
- ROS, 선택 사항

4. WSL 안의 Linux 파일시스템에서 작업

권장:

```bash
cd ~
mkdir -p ros2_ws
cd ros2_ws
git clone https://github.com/bong7233/ROS2_Prac.git
```

비권장:

```bash
cd /mnt/c/Users/<name>/...
```

`/mnt/c` 아래에서 colcon 빌드를 하면 파일 I/O가 느리고 권한/심볼릭 링크 문제가 생길 수 있습니다.

### 6.2 Windows native ROS 2

Windows native에서도 ROS 2를 설치할 수 있지만, 이 프로젝트는 Linux robot PC를 목표로 합니다. CAN, Serial, TCP/IP 장치, Qt 배포, DDS 네트워크, 현장 로깅까지 생각하면 WSL2 또는 실제 Ubuntu PC가 더 현실적입니다.

Windows native는 다음 용도로만 권장합니다.

- 문서 수정
- 코드 리뷰
- 간단한 Python 도구 개발
- GitHub 작업

실제 빌드와 실행은 WSL2 Ubuntu 또는 Ubuntu PC에서 하는 것이 좋습니다.

## 7. 자주 쓰는 명령

패키지 목록:

```bash
ros2 pkg list | grep amr
```

노드 목록:

```bash
ros2 node list
```

토픽 목록:

```bash
ros2 topic list
```

서비스 목록:

```bash
ros2 service list
```

인터페이스 확인:

```bash
ros2 interface show amr_interfaces/msg/SafetyState
ros2 interface show amr_interfaces/srv/InjectMotorFault
```

파라미터 확인:

```bash
ros2 param list /safety_monitor
ros2 param get /safety_monitor command_timeout_ms
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Qt 운영 UI:

```bash
ros2 launch amr_operator_ui operator_ui.launch.py
```

## 8. 빌드 문제 해결

`package not found`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

custom interface import 실패:

```bash
colcon build --symlink-install
source install/setup.bash
```

rosdep 실패:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Python entry point가 안 보일 때:

```bash
colcon build --symlink-install --packages-select amr_tools
source install/setup.bash
ros2 run amr_tools health_report --duration 1.0
```

## 9. 현업식 개발 루틴

1. 작은 단위로 수정합니다.
2. package 단위로 빠르게 빌드합니다.
3. launch로 통합 실행합니다.
4. `health_report`로 상태를 봅니다.
5. fault scenario를 하나 실행합니다.
6. rosbag으로 기록합니다.
7. README/docs를 같이 갱신합니다.
8. 커밋 메시지는 기능 단위로 작성합니다.

## 10. GitHub Actions CI

이 저장소는 `.github/workflows/ci.yml`을 사용합니다.

CI에서 확인하는 것:

- Python launch/tool 문법 검사
- XML/SDF/URDF/xacro 파싱 검사
- Ubuntu 24.04 runner에서 ROS 2 Jazzy apt repository 설정
- `rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy`
- `colcon build --symlink-install`
- `colcon test`

CI 결과는 GitHub 저장소의 `Actions` 탭에서 볼 수 있습니다.
