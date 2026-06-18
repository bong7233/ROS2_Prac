# ROS 2 AMR Learning Guide

English version: [ROS 2 AMR Learning Guide](en/01_ros2_amr_learning_guide.en.md)

이 문서는 ROS 2를 처음 사용하는 사람이 AGV/AMR 제어 프로그램을 구현할 수 있도록 학습 순서와 설계 기준을 정리한 가이드입니다. 목표는 단순히 예제 노드를 실행하는 것이 아니라, 실제 로봇 PC에서 돌아갈 수 있는 구조를 이해하고 포트폴리오 코드로 연결하는 것입니다.

## 1. Mindset Shift

레거시 AGV 제어 프로그램은 흔히 하나의 큰 프로그램 안에 UI, Serial 통신, TCP/IP 통신, CAN 통신, 상태 머신, 로그, 수동 조작, 자동 운전 로직이 섞여 있습니다. 이런 구조는 처음 만들 때는 빠르지만 시간이 지나면 다음 문제가 생깁니다.

- 장치 하나가 죽어도 전체 프로그램 영향 범위를 알기 어렵습니다.
- UI 코드와 제어 코드가 얽혀 테스트하기 어렵습니다.
- 통신 재연결, fault reset, 로그 재현이 일관되지 않습니다.
- 나중에 SLAM/Nav2 같은 ROS 생태계 기능을 붙이기 어렵습니다.

ROS 2에서는 한 프로그램을 크게 만드는 대신, 책임이 작은 노드들이 표준 인터페이스로 연결된 graph를 만듭니다. 배터리 노드는 배터리만 읽고, IO 노드는 IO만 관리하고, 모터 노드는 드라이브만 담당하고, UI는 이 노드들의 상태를 구독하거나 service/action을 호출합니다.

핵심은 "프로그램 하나를 잘 짠다"가 아니라 "로봇 시스템을 노드들의 계약으로 설계한다"입니다.

## 2. What ROS 2 Is

ROS 2는 로봇 애플리케이션을 만들기 위한 middleware, build system, 메시지 정의, CLI 도구, visualization, logging 생태계입니다. 특정 SLAM 알고리즘 하나가 아니라, 로봇 프로그램들이 서로 통신하고 관리되는 표준 방식에 가깝습니다.

ROS 2에서 가장 먼저 익혀야 할 단위는 아래와 같습니다.

| Concept | Meaning | AMR Example |
| --- | --- | --- |
| Node | 하나의 실행 책임 단위 | `amr_battery_driver`, `amr_io_driver` |
| Topic | 지속적으로 흐르는 데이터 | `/battery_state`, `/odom`, `/scan` |
| Service | 짧은 요청/응답 | `/reset_fault`, `/set_io` |
| Action | 오래 걸리고 중간 feedback이 필요한 작업 | Nav2의 `NavigateToPose`, 미래의 docking |
| Parameter | 실행 중 읽는 설정값 | serial port, CAN interface, wheel radius |
| Launch | 여러 노드와 설정을 함께 시작 | `mock_robot.launch.py` |
| Package | 빌드/배포 단위 | `amr_interfaces`, `amr_bringup` |
| Workspace | 여러 package를 모아 빌드하는 공간 | 이 저장소 전체 |
| TF | 로봇 좌표계 변환 | `map`, `odom`, `base_link`, `laser` |
| QoS | 통신 품질 정책 | 센서 데이터는 유실 허용, 상태는 reliable |
| Lifecycle | 노드 상태 전이 모델 | configure, activate, deactivate, cleanup |

## 3. Topic, Service, Action 선택법

처음 ROS 2를 배울 때 가장 많이 헷갈리는 부분입니다.

| Need | Use | Reason |
| --- | --- | --- |
| 계속 변하는 센서값 | Topic | 주기적으로 publish하고 필요한 노드가 subscribe합니다. |
| 로봇 속도 명령 | Topic | 최신 명령이 중요하고 계속 갱신됩니다. |
| IO 출력 1회 변경 | Service | "출력 3번 ON 해줘"처럼 성공/실패 응답이 필요합니다. |
| Fault reset | Service | 즉시 처리하고 결과를 돌려주면 됩니다. |
| 목적지 이동 | Action | 오래 걸리고 진행률, 취소, 결과가 필요합니다. |
| 배터리 전압 임계값 | Parameter | 실행 환경마다 바뀌는 설정값입니다. |

이 프로젝트의 초기 단계에서는 topic과 service만으로 충분합니다. Nav2를 붙이는 단계에서 action을 본격적으로 사용합니다.

## 4. Recommended Learning Order

### Phase 0: Linux and ROS 2 Environment

먼저 Ubuntu 24.04 LTS에서 ROS 2 Jazzy를 설치하고 CLI 흐름에 익숙해집니다.

학습 목표:

- `source /opt/ros/jazzy/setup.bash`의 의미 이해
- `ros2 run`, `ros2 launch`, `ros2 node list`, `ros2 topic list` 사용
- `colcon build`와 workspace overlay 이해
- package와 executable의 차이 이해

연습 명령:

```bash
source /opt/ros/jazzy/setup.bash
ros2 doctor
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

### Phase 1: Node, Topic, Service

처음 작성할 코드는 AMR과 직접 관련된 mock node가 좋습니다. 튜토리얼의 `talker/listener`보다 포트폴리오 목적이 선명합니다.

추천 첫 노드:

- `mock_battery_driver`: 24.0V에서 서서히 전압이 내려가는 `BatteryState` publish
- `mock_io_driver`: digital input/output 상태 publish, `/set_io` service 제공
- `mock_motor_driver`: wheel velocity command를 받아 wheel state publish

학습 목표:

- publisher/subscriber 생성
- timer callback
- service server/client
- parameter 선언과 읽기
- `rclcpp` logging

### Phase 2: Interfaces

표준 메시지를 우선 사용하고, 표준 메시지로 부족할 때만 custom interface를 만듭니다.

표준 메시지 우선순위:

- Battery: `sensor_msgs/msg/BatteryState`
- Velocity command: `geometry_msgs/msg/Twist`
- Odometry: `nav_msgs/msg/Odometry`
- Laser: `sensor_msgs/msg/LaserScan`
- Diagnostics: `diagnostic_msgs/msg/DiagnosticArray`
- Joint state: `sensor_msgs/msg/JointState`

Custom interface 후보:

- `amr_interfaces/msg/RobotState`
- `amr_interfaces/msg/IoState`
- `amr_interfaces/msg/MotorState`
- `amr_interfaces/srv/SetDigitalOutput`
- `amr_interfaces/srv/ResetFault`

원칙은 "외부 ROS 생태계가 이미 아는 데이터는 표준 메시지를 쓰고, 우리 로봇 도메인에만 있는 상태는 custom으로 만든다"입니다.

### Phase 3: Launch and Parameters

현장 로봇은 포트, IP, CAN interface, wheel radius, gear ratio, battery threshold가 모두 설정값입니다. 코드에 하드코딩하면 포트폴리오 점수가 깎입니다.

좋은 구조:

```text
amr_bringup/
  launch/
    mock_robot.launch.py
    real_robot.launch.py
  config/
    robot.yaml
    devices.mock.yaml
    devices.real.yaml
    diagnostics.yaml
```

예상 parameter:

```yaml
amr_battery_driver:
  ros__parameters:
    port: "/dev/ttyUSB_BMS"
    baudrate: 115200
    publish_rate_hz: 2.0
    low_voltage: 22.0

amr_motor_driver:
  ros__parameters:
    can_interface: "can0"
    wheel_radius_m: 0.1
    wheel_separation_m: 0.55
    command_timeout_ms: 200
```

### Phase 4: Lifecycle Nodes

AMR에서는 "프로세스가 켜졌다"와 "로봇이 운전 가능하다"가 다릅니다. 배터리 연결, IO 통신, 모터 enable, fault clear 같은 순서가 필요합니다.

Lifecycle 상태를 AMR bringup에 적용하면 다음처럼 생각할 수 있습니다.

| Lifecycle State | AMR Meaning |
| --- | --- |
| Unconfigured | 노드는 생성됐지만 장치 연결 전 |
| Inactive | 장치 연결/설정 완료, 아직 제어 출력 안 함 |
| Active | 주기 처리, publish, command 처리 가능 |
| Finalized | 종료 또는 복구 불가 |

초기에는 모든 노드를 lifecycle로 만들 필요는 없습니다. 포트폴리오에서 가장 설득력 있는 적용 대상은 아래입니다.

- `amr_system_manager`
- `amr_safety_monitor`
- `amr_motor_driver`
- `amr_io_driver`

### Phase 5: Diagnostics and Field Debugging

FAE 포지션에서 특히 중요한 것은 "문제가 났을 때 어떤 데이터를 보고 판단할 수 있는가"입니다.

각 driver node는 최소한 다음 상태를 diagnostics로 내보내야 합니다.

| Node | Diagnostic Keys |
| --- | --- |
| Battery | connection, voltage, current, soc, temperature, last_rx_age |
| IO | connection, input_count, output_count, estop_input, last_rx_age |
| Motor | can_state, drive_enable, fault_code, wheel_velocity, command_age |
| LiDAR | connection, scan_rate, last_scan_age |
| Safety | estop, command_timeout, battery_fault, motor_fault, current_mode |

ROS 2에서 현장 재현을 위해 꼭 사용할 도구:

```bash
ros2 topic hz /odom
ros2 topic echo /diagnostics
ros2 topic info /scan --verbose
ros2 service call /reset_fault std_srvs/srv/Trigger
ros2 bag record /cmd_vel /odom /battery_state /diagnostics
ros2 bag play <bag_folder>
```

### Phase 6: Qt UI Integration

Qt UI는 ROS graph의 일부가 될 수 있지만, UI가 직접 Serial/TCP/CAN을 잡으면 안 됩니다. UI는 operator interface이고, 장치 통신은 driver node의 책임이어야 합니다.

권장 구조:

- Qt main thread는 화면만 담당합니다.
- ROS 2 executor는 별도 thread에서 spin합니다.
- ROS callback에서 Qt widget을 직접 만지지 않고 signal/slot으로 전달합니다.
- UI command는 service client 또는 topic publisher로 보냅니다.
- 긴 작업은 action client로 보냅니다. Nav2 목적지 이동이 대표 예입니다.

UI에서 처음 구현할 기능:

- robot mode 표시
- battery voltage/SOC 표시
- IO input/output table
- motor fault 표시
- manual jog buttons
- enable/disable, fault reset button
- diagnostics summary
- topic alive indicator

## 5. AMR에서 자주 쓰는 ROS 2 데이터 흐름

### Manual Jog

```text
Qt UI button
  -> /cmd_vel
  -> amr_safety_monitor
  -> /cmd_vel_safe
  -> amr_base_controller
  -> amr_motor_driver
  -> CAN motor drive
```

### Battery Monitoring

```text
BMS serial frame
  -> amr_battery_driver
  -> /battery_state
  -> /diagnostics
  -> Qt UI and rosbag2
```

### IO Control

```text
Qt UI
  -> /set_io service
  -> amr_io_driver
  -> TCP/IP IO board
  -> /io_state
  -> Qt UI and safety monitor
```

### Odometry

```text
Wheel encoder
  -> motor driver wheel state
  -> base controller
  -> /odom
  -> odom to base_link TF
  -> RViz, robot_localization, Nav2 later
```

## 6. Why Not Start With Nav2

Nav2는 AMR 포트폴리오에 매우 강력한 주제지만, 처음부터 Nav2를 붙이면 문제가 생겼을 때 원인을 분리하기 어렵습니다. Nav2가 잘 동작하려면 이미 다음이 준비돼 있어야 합니다.

- 안정적인 `/cmd_vel` 처리
- 정확한 `/odom`
- `odom -> base_link` TF
- `base_link -> laser` TF
- URDF/footprint
- `/scan` sensor data
- command timeout과 safety stop
- lifecycle bringup 이해

따라서 이 프로젝트는 먼저 "Nav2를 받을 수 있는 AMR base"를 만들고, 그 다음 Nav2를 붙입니다. 이 순서가 실무적으로도 더 건강합니다.

## 7. Recommended Development Environment

초기 개발 환경:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  ros-jazzy-desktop
```

Qt UI 개발:

```bash
sudo apt install -y \
  qt6-base-dev qt6-tools-dev qt6-tools-dev-tools \
  libqt6svg6-dev
```

나중에 필요한 ROS 패키지:

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox
```

CAN 개발 시 Linux 쪽 준비:

```bash
sudo apt install -y can-utils
sudo ip link set can0 up type can bitrate 500000
candump can0
```

실제 장비가 없을 때도 `vcan0`로 개발할 수 있습니다.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump vcan0
```

## 8. First Coding Target

코드를 시작할 때는 아래 순서를 권장합니다.

1. `amr_interfaces` 생성
2. `amr_battery_driver` mock node 생성
3. `amr_io_driver` mock node 생성
4. `amr_bringup` launch/config 생성
5. `ros2 topic echo`, `ros2 service call`로 검증
6. `amr_system_manager`가 mock nodes 상태를 모니터링
7. `amr_operator_ui`가 topic/service를 통해 상태 표시와 명령 수행

가장 첫 번째 pull request 또는 첫 번째 커밋은 "배터리 mock driver + README 실행법" 정도가 좋습니다. 작지만 ROS 2의 기본 단위가 모두 들어갑니다.

## 9. Study Checklist

아래 질문에 스스로 답할 수 있으면 다음 구현 단계로 넘어가도 됩니다.

- ROS 2 node와 package의 차이를 설명할 수 있는가?
- topic/service/action 중 무엇을 언제 쓰는지 설명할 수 있는가?
- `colcon build` 후 `source install/setup.bash`가 왜 필요한지 설명할 수 있는가?
- parameter YAML이 launch에서 어떻게 node로 전달되는지 이해했는가?
- `cmd_vel`과 `odom`의 역할 차이를 설명할 수 있는가?
- `map`, `odom`, `base_link`, `laser` frame의 의미를 설명할 수 있는가?
- QoS mismatch가 왜 topic 통신 실패처럼 보일 수 있는지 알고 있는가?
- lifecycle node가 일반 node와 어떻게 다른지 알고 있는가?
- UI가 장치 포트를 직접 열지 않아야 하는 이유를 설명할 수 있는가?
- rosbag2로 현장 문제를 재현하는 흐름을 설명할 수 있는가?

## 10. Questions To Answer Before Real Hardware

실제 장비 연동으로 가기 전에 아래 정보가 있으면 구조를 더 정확히 잡을 수 있습니다.

- 구동 방식: differential drive, mecanum, steering, fork AGV, ackermann 중 무엇인가?
- 모터 드라이브 프로토콜: raw CAN frame인가, CANopen/CiA402인가?
- encoder 값은 모터 드라이브에서 읽는가, 별도 encoder board에서 읽는가?
- BMS serial frame protocol 문서가 있는가?
- IO board TCP/IP protocol이 Modbus TCP인지, vendor custom인지?
- LiDAR 모델과 ROS 2 vendor driver 유무는 무엇인가?
- 실제 로봇 없이 개발할 기간이 긴가, 바로 장비를 붙일 수 있는가?
- UI는 현장 operator용인지, 개발/debug용인지, 둘 다인지?
