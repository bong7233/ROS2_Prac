# AMR System Architecture

English version: [AMR System Architecture](en/02_amr_system_architecture.en.md)

이 문서는 AGV/AMR 제어 프로그램을 ROS 2 기반으로 설계할 때의 권장 구조를 설명합니다. 기준은 "실제 로봇 PC에서 운용 가능하고, 포트폴리오로 설명 가능하며, 이후 Nav2와 실제 하드웨어로 확장 가능한 구조"입니다.

## 1. Architectural Principle

ROS 2 AMR 제어 프로그램의 기본 원칙은 다음입니다.

- 장치 통신은 device driver node가 담당합니다.
- 제어 판단은 controller/safety/system node가 담당합니다.
- UI는 상태 표시와 operator command만 담당합니다.
- 모든 노드는 topic/service/action/parameter 계약으로 연결합니다.
- 실제 장비가 없어도 mock driver로 같은 interface를 검증합니다.
- 하드웨어 safety는 ROS 2보다 우선합니다. ROS 2 safety monitor는 2차 보호와 상태 진단입니다.

레거시 구조에서 흔한 "UI 버튼 callback 안에서 TCP 패킷 보내고, Serial 읽고, CAN 출력까지 하는" 방식을 피하는 것이 핵심입니다.

## 2. Layered Architecture

```text
Operator / Field Layer
  Qt UI, RViz, ros2 CLI, rosbag2, diagnostics tools

Application Layer
  system manager, safety monitor, manual control, future mission manager

Navigation Layer
  Nav2, SLAM Toolbox, robot_localization
  이 프로젝트에서는 후순위

Robot Control Layer
  base controller, ros2_control controllers, odometry publisher

Device Driver Layer
  battery serial, IO TCP/IP, motor CAN, lidar vendor driver

Hardware Layer
  BMS, IO board, motor drive, laser scanner, estop, relays
```

이 레이어 구조를 지키면 각 문제의 위치를 빠르게 좁힐 수 있습니다.

예를 들어 `/battery_state`가 안 나오면 UI 문제가 아니라 battery driver 또는 serial 문제입니다. `/cmd_vel`은 나오는데 로봇이 안 움직이면 safety gate, base controller, motor driver, CAN, drive enable 순서로 확인하면 됩니다.

## 3. Package Responsibilities

| Package | Responsibility | Notes |
| --- | --- | --- |
| `amr_interfaces` | custom msg/srv/action | 표준 메시지로 부족한 상태만 정의 |
| `amr_bringup` | launch, YAML config | mock/real bringup 분리 |
| `amr_description` | URDF, xacro, mesh, TF config | RViz/Gazebo/Nav2의 기반 |
| `amr_system_manager` | lifecycle orchestration, mode transition | INIT, MANUAL, AUTO, FAULT 관리 |
| `amr_safety_monitor` | command gate, estop/fault aggregation | `/cmd_vel`을 검증 후 `/cmd_vel_safe`로 전달 |
| `amr_base_controller` | kinematics, odometry, wheel command | 추후 `ros2_control`로 일부 대체 가능 |
| `amr_battery_driver` | BMS serial driver | `BatteryState`, diagnostics |
| `amr_io_driver` | IO board TCP/IP driver | `IoState`, set output service |
| `amr_motor_driver` | CAN motor drive driver | wheel command/state, fault |
| `amr_lidar_driver` | LiDAR driver wrapper | vendor driver 설정/launch 포함 |
| `amr_operator_ui` | Qt operator UI | ROS interface client 역할 |
| `amr_diagnostics` | diagnostics helper/aggregation | field debug view |
| `amr_sim` | mock hardware, Gazebo assets | real hardware 없이 검증 |

## 4. Node Design

### amr_system_manager

역할:

- 전체 robot mode 관리
- lifecycle node configure/activate/deactivate 순서 제어
- fault reset 정책 제어
- UI에 robot state 제공
- bringup 실패 시 어느 장치가 실패했는지 표시

주요 상태:

```text
BOOT -> INIT -> MANUAL -> AUTO_READY -> AUTO_RUNNING
              -> FAULT
              -> ESTOP
```

권장 interface:

| Name | Type | Direction |
| --- | --- | --- |
| `/robot_state` | `amr_interfaces/msg/RobotState` | publish |
| `/set_mode` | `amr_interfaces/srv/SetMode` | service server |
| `/reset_fault` | `std_srvs/srv/Trigger` | service server |
| diagnostics | `diagnostic_msgs/msg/DiagnosticArray` | publish |

### amr_safety_monitor

역할:

- `/cmd_vel` command timeout 검사
- estop, bumper, safety input 감시
- battery low/fault, motor fault 반영
- manual/auto mode에 따라 command 허용 여부 결정
- 안전하지 않으면 zero velocity publish

중요 원칙:

- 물리적 estop은 반드시 하드웨어 회로로 모터 enable을 끊어야 합니다.
- ROS 2 safety monitor는 상태 표시와 command gate입니다.
- network delay나 process crash에도 로봇이 멈추도록 motor driver 쪽 command timeout이 따로 있어야 합니다.

권장 interface:

| Name | Type | Direction |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | subscribe |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | publish |
| `/io_state` | `amr_interfaces/msg/IoState` | subscribe |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | subscribe |
| `/motor_state` | `amr_interfaces/msg/MotorState` | subscribe |
| `/safety_state` | custom msg | publish |

### amr_base_controller

역할:

- `/cmd_vel_safe`를 wheel velocity command로 변환
- wheel encoder state로 odometry 계산
- `/odom` publish
- `odom -> base_link` TF publish

differential drive 기준 기본 계산:

```text
v_left  = linear_x - angular_z * wheel_separation / 2
v_right = linear_x + angular_z * wheel_separation / 2

wheel_angular_left  = v_left / wheel_radius
wheel_angular_right = v_right / wheel_radius
```

실제 구현에서는 gear ratio, encoder resolution, drive unit, acceleration limit, command timeout을 모두 parameter로 둡니다.

### amr_motor_driver

역할:

- SocketCAN 또는 vendor library로 CAN frame 송수신
- motor enable/disable
- wheel velocity command 전송
- encoder, current, temperature, fault code 수신
- drive fault diagnostics publish

CANopen/CiA402 드라이브라면 추후 `ros2_canopen` 또는 ros2_control hardware interface 적용을 검토할 수 있습니다. vendor custom raw CAN이면 먼저 SocketCAN 기반으로 protocol adapter를 명확히 분리합니다.

권장 내부 분리:

```text
MotorDriverNode
  - ROS interface
  - Command timeout
  - Diagnostics

MotorProtocol
  - encode command frame
  - decode status frame
  - fault code mapping

CanTransport
  - SocketCAN open/read/write
  - reconnect
```

### amr_battery_driver

역할:

- Serial port open/reconnect
- BMS frame parser
- checksum validation
- `sensor_msgs/msg/BatteryState` publish
- low voltage, communication timeout diagnostics

BatteryState에 넣을 후보:

- `voltage`
- `current`
- `percentage`
- `temperature`
- `power_supply_status`
- `power_supply_health`

### amr_io_driver

역할:

- TCP/IP IO board 연결
- digital input state publish
- output set service 제공
- reconnect/backoff
- estop, bumper, charging contact 같은 safety input 표시

Modbus TCP 장비라면 protocol adapter를 Modbus client로 구현하고, vendor custom이면 frame encoder/decoder를 분리합니다.

### amr_operator_ui

역할:

- operator가 로봇 상태를 이해하도록 표시
- manual jog command publish
- output command service call
- fault reset service call
- diagnostics summary 표시
- 미래에는 Nav2 action client로 goal 전송

UI가 하지 말아야 할 일:

- Serial/TCP/CAN port 직접 open
- safety 판단 직접 수행
- motor command를 driver에 우회 전송
- ROS callback thread에서 widget 직접 수정

## 5. Topic Map

| Topic | Message | Rate | QoS | Notes |
| --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | event/10-50 Hz | reliable, depth 1 | UI or Nav2 command |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | event/10-50 Hz | reliable, depth 1 | safety-filtered |
| `/odom` | `nav_msgs/msg/Odometry` | 30-100 Hz | reliable, depth 10 | local odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | 30-100 Hz | default TF QoS | dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | latched | transient local | static transforms |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1-5 Hz | reliable, depth 10 | BMS telemetry |
| `/io_state` | custom | 5-20 Hz | reliable, depth 10 | digital IO |
| `/motor_state` | custom | 10-100 Hz | reliable, depth 10 | drive state |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10-40 Hz | sensor data QoS | LiDAR |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | reliable, depth 10 | health |
| `/robot_state` | custom | 1-10 Hz | reliable, depth 10 | mode/fault |

## 6. Service Map

| Service | Type | Server | Purpose |
| --- | --- | --- | --- |
| `/set_mode` | custom | system manager | mode transition request |
| `/reset_fault` | `std_srvs/srv/Trigger` | system manager | fault reset request |
| `/set_io` | custom | IO driver | digital output command |
| `/motor_enable` | `std_srvs/srv/SetBool` | motor driver | drive enable/disable |
| `/clear_motor_fault` | `std_srvs/srv/Trigger` | motor driver | drive fault clear |
| `/reload_config` | `std_srvs/srv/Trigger` | selected nodes | optional development helper |

## 7. Parameters

모든 장치 설정은 YAML parameter로 관리합니다.

```yaml
amr_base_controller:
  ros__parameters:
    wheel_radius_m: 0.1
    wheel_separation_m: 0.55
    max_linear_velocity_mps: 1.0
    max_angular_velocity_radps: 1.5
    command_timeout_ms: 200
    publish_tf: true

amr_safety_monitor:
  ros__parameters:
    command_timeout_ms: 300
    low_battery_percentage: 0.2
    critical_battery_percentage: 0.1
    require_motor_enabled: true
```

parameter 설계 원칙:

- 현장에서 바뀌는 값은 parameter입니다.
- protocol frame layout처럼 코드 의미에 가까운 값은 코드/문서에 둡니다.
- 안전 임계값은 parameter로 두되 default를 보수적으로 둡니다.
- parameter 이름은 단위까지 포함합니다. 예: `_m`, `_ms`, `_hz`.

## 8. Lifecycle Bringup Sequence

권장 bringup 순서:

1. `amr_system_manager` 시작
2. driver nodes 생성
3. battery/io/motor/lidar configure
4. 통신 연결 확인
5. safety monitor configure
6. base controller configure
7. diagnostics active
8. motor driver active, but motor disabled
9. UI active
10. operator enable 또는 mode change 후 command 허용

shutdown 순서:

1. safety monitor가 zero command publish
2. base controller command 중지
3. motor driver disable
4. device drivers deactivate
5. logs flush
6. process shutdown

## 9. QoS Policy

QoS는 ROS 2에서 "연결됐는데 데이터가 안 보이는" 상황의 주요 원인입니다.

권장 기본값:

| Data | Reliability | Durability | Depth | Reason |
| --- | --- | --- | --- | --- |
| Command | reliable | volatile | 1 | 최신 명령만 중요 |
| State | reliable | volatile | 10 | UI/진단에서 놓치면 안 됨 |
| High-rate sensor | best effort or sensor data profile | volatile | small | scan/image는 일부 유실 허용 |
| Map/static config | reliable | transient local | 1 | 늦게 붙은 노드도 마지막 값 필요 |
| Diagnostics | reliable | volatile | 10 | 낮은 주기의 상태 정보 |

처음 구현에서는 default QoS를 쓰되, LiDAR와 Nav2를 붙일 때 sensor data QoS를 명시적으로 이해해야 합니다.

## 10. TF and Frame Tree

AMR의 최소 frame tree:

```text
map
  -> odom
    -> base_link
      -> base_footprint
      -> laser
      -> imu_link
```

초기 단계에서는 `map -> odom`은 비워두거나 identity로 두지 않습니다. Nav2/SLAM 단계에서 localization이 담당합니다.

초기 구현 책임:

- `robot_state_publisher`: URDF 기반 static/dynamic robot link TF
- `amr_base_controller`: `odom -> base_link`
- LiDAR mount: `base_link -> laser` static transform

Nav2가 요구하는 핵심은 `odom -> base_link`와 `base_link -> laser`가 안정적으로 존재하는 것입니다.

## 11. Safety Model

중요한 safety 원칙:

- ROS 2 process가 죽어도 로봇은 멈춰야 합니다.
- motor drive command timeout은 반드시 driver 또는 drive 자체에 있어야 합니다.
- estop은 하드웨어 회로로 motor enable을 차단해야 합니다.
- software safety는 command gate, diagnostics, operator visibility 역할입니다.
- UI button은 safety를 우회할 수 없어야 합니다.

소프트웨어 safety monitor가 zero command를 내는 조건:

- estop input active
- bumper or safety scanner protective stop
- motor fault active
- battery critical
- command timeout
- invalid robot mode
- localization/nav fault, later
- manual deadman released, if configured

## 12. Testing Strategy

### Unit Test

- protocol parser checksum
- wheel kinematics
- state transition validation
- parameter validation
- fault code mapping

### Node Test

- mock serial/TCP/CAN input
- topic publish rate
- service response
- command timeout behavior
- lifecycle transition result

### Integration Test

- `mock_robot.launch.py` 실행
- `/cmd_vel` publish 후 `/odom` 변화 확인
- BMS timeout 발생 시 diagnostics fault 확인
- estop input mock 시 `/cmd_vel_safe` zero 확인

### Field Test Later

- CAN bus disconnect
- IO board power cycle
- BMS checksum error
- motor fault clear sequence
- low battery threshold
- rosbag replay

## 13. Deployment Notes

실제 로봇 PC에 올릴 때 고려할 내용:

- `/dev/ttyUSB*` 대신 udev rule로 `/dev/ttyBMS` 같은 stable name 사용
- `can0` bitrate systemd/networkd 설정
- IO board, LiDAR static IP 관리
- ROS_DOMAIN_ID 설정
- DDS discovery가 현장 네트워크에 미치는 영향 확인
- systemd service로 bringup 자동 시작
- 로그 저장 위치와 용량 제한
- crash dump 또는 core dump 정책
- NTP/chrony 시간 동기화

## 14. Portfolio Review Points

이 아키텍처를 코드로 구현하면 다음 포인트를 강조할 수 있습니다.

- ROS 2 표준 interface를 우선 사용했다.
- 실제 장비 없이도 mock driver로 integration test가 가능하다.
- UI와 driver를 분리해서 유지보수성을 높였다.
- lifecycle로 bringup/shutdown/fault reset 순서를 설계했다.
- diagnostics와 rosbag2로 현장 디버깅 관점을 반영했다.
- Nav2를 무작정 붙이지 않고, Nav2가 요구하는 base interface를 먼저 만들었다.
