# FAE 현장 지원 가이드

이 문서는 ROS2_Prac 프로젝트를 FAE 포트폴리오로 설명할 때 도움이 되는 현장 점검, 장애 재현, 복구 시나리오를 정리합니다.

English version: [FAE Field Guide](en/07_fae_field_guide.en.md)

## 1. FAE 관점에서 중요한 능력

FAE 포지션에서는 코드를 작성하는 능력뿐 아니라, 현장에서 문제가 생겼을 때 원인을 빠르게 좁히는 능력이 중요합니다.

이 프로젝트에서 보여줄 수 있는 역량:

- ROS 2 graph 상태 확인
- topic/service/interface 계약 이해
- 장치 통신 장애와 software fault 구분
- safety gate 동작 검증
- diagnostics 기반 fault isolation
- rosbag2로 문제 재현
- Python으로 현장 점검 자동화
- C++ runtime node와 Python field tool 역할 분리

## 2. 현장 점검 순서

로봇이 움직이지 않는다고 가정하면 아래 순서로 봅니다.

1. ROS graph가 살아 있는가?

```bash
ros2 node list
ros2 topic list
ros2 service list
```

2. 주요 상태 topic이 살아 있는가?

```bash
ros2 run amr_tools health_report --duration 3.0
```

3. safety가 command를 막고 있는가?

```bash
ros2 topic echo /safety_state
```

4. robot mode가 fault인가?

```bash
ros2 topic echo /robot_state
```

5. 모터가 명령을 받고 있는가?

```bash
ros2 topic echo /wheel_command
ros2 topic echo /motor_state
```

6. odometry가 나오는가?

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 3. FAE 데모 시나리오

### 3.1 정상 주행

목표:

- `/cmd_vel`이 `/cmd_vel_safe`로 통과한다.
- `/wheel_command`가 생성된다.
- `/motor_state`가 바뀐다.
- `/odom`이 증가한다.

명령:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.20}}"
```

확인:

```bash
ros2 topic echo /safety_state
ros2 topic hz /odom
ros2 run amr_tools health_report --duration 3.0
```

설명 포인트:

- UI나 Nav2가 낸 명령은 직접 모터로 가지 않는다.
- safety monitor가 먼저 검증한 뒤 `/cmd_vel_safe`를 publish한다.
- base controller는 motor feedback으로 odom을 계산한다.

### 3.2 비상정지 입력

명령:

```bash
ros2 run amr_tools fault_scenario estop-on
ros2 topic echo /safety_state
ros2 topic echo /robot_state
```

예상 결과:

- `command_allowed=false`
- `estop_active=true`
- robot mode가 `ESTOP`으로 바뀐다.
- `/cmd_vel_safe`는 zero command가 된다.

복구:

```bash
ros2 run amr_tools fault_scenario estop-off
ros2 run amr_tools fault_scenario recover
```

설명 포인트:

- 실제 로봇에서는 estop이 하드웨어로 motor enable을 차단해야 한다.
- ROS 2 safety monitor는 command gate와 상태 표시 역할이다.

### 3.3 배터리 critical

명령:

```bash
ros2 run amr_tools fault_scenario battery-critical
ros2 topic echo /battery_state
ros2 topic echo /robot_state
```

예상 결과:

- battery percentage가 critical 영역으로 떨어진다.
- system manager가 fault 상태로 전환한다.
- safety monitor가 command를 차단한다.

복구:

```bash
ros2 run amr_tools fault_scenario battery-normal
ros2 run amr_tools fault_scenario recover
```

설명 포인트:

- 배터리 상태는 표준 `sensor_msgs/msg/BatteryState`로 publish한다.
- AMR 운영 상태는 custom `RobotState`로 요약한다.

### 3.4 모터 fault

명령:

```bash
ros2 run amr_tools fault_scenario motor-fault --fault-code 2310
ros2 topic echo /motor_state
ros2 topic echo /diagnostics
```

예상 결과:

- motor fault가 active 된다.
- motor enable이 false가 된다.
- safety monitor가 command를 차단한다.
- diagnostics에 ERROR가 표시된다.

복구:

```bash
ros2 run amr_tools fault_scenario motor-clear
ros2 run amr_tools fault_scenario recover
```

설명 포인트:

- 모터 fault는 driver node에서 감지하고 publish한다.
- system manager는 여러 장치 상태를 모아 robot fault로 승격한다.

## 4. 장애 격리 표

| 증상 | 먼저 볼 topic/service | 가능 원인 |
| --- | --- | --- |
| 로봇이 움직이지 않음 | `/safety_state` | estop, battery critical, motor fault, command timeout |
| `/cmd_vel`은 있는데 `/cmd_vel_safe`가 0 | `/safety_state` | safety gate 차단 |
| `/cmd_vel_safe`는 있는데 모터 속도 0 | `/motor_state`, `/wheel_command` | motor disabled, fault, command timeout |
| odom이 안 나옴 | `/motor_state`, `/odom` | motor feedback 없음, base controller 문제 |
| UI 상태가 오래됨 | `health_report` | topic stale, node down |
| fault reset 실패 | `/robot_state`, `/safety_state` | estop 또는 critical condition이 아직 남아 있음 |

## 5. rosbag 기록 전략

문제가 재현되면 아래 topic을 기록합니다.

```bash
ros2 bag record \
  /cmd_vel /cmd_vel_safe /wheel_command /motor_state /odom /tf \
  /battery_state /io_state /safety_state /robot_state /diagnostics
```

기록 후 확인:

```bash
ros2 bag info <bag_folder>
ros2 bag play <bag_folder>
```

FAE 설명 포인트:

- rosbag은 현장 이슈를 개발 PC에서 재현하게 해준다.
- safety_state와 diagnostics를 함께 기록해야 원인 분석이 쉽다.
- high-rate sensor data는 용량이 커질 수 있으므로 필요한 topic만 고른다.

## 6. Python 도구의 포트폴리오 가치

이 프로젝트의 Python 도구:

- `health_report`: graph 상태와 diagnostics 요약
- `fault_scenario`: 장애 주입과 복구 시나리오 실행

Python이 중요한 이유:

- 현장 자동화 스크립트를 빠르게 만들 수 있다.
- 반복 점검을 표준화할 수 있다.
- rosbag, CSV, JSON, 로그 분석에 강하다.
- 고객사/현장별 점검 절차를 도구화하기 좋다.

면접 설명 예시:

> Runtime node는 C++로 구현하고, FAE 현장 점검과 장애 재현은 Python CLI로 만들었습니다. 이렇게 분리하면 제어 루프 안정성과 현장 대응 속도를 모두 가져갈 수 있습니다.

## 7. 다음에 추가하면 좋은 FAE 기능

- health report를 JSON/Markdown 파일로 저장
- rosbag 자동 기록 시작/종료 도구
- 장치별 protocol simulator
- CAN frame dump parser
- Modbus TCP register map viewer
- diagnostics timeline viewer
- Qt UI의 field support tab

