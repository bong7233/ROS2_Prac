# Implementation Roadmap

English version: [Implementation Roadmap](en/03_implementation_roadmap.en.md)

이 문서는 ROS2_Prac 프로젝트를 어떤 순서로 구현하면 학습과 포트폴리오 완성도가 함께 올라가는지 정리한 로드맵입니다. 각 단계는 "무엇을 배웠는지", "무엇을 보여줄 수 있는지", "다음 단계의 기반이 되는지"를 기준으로 나눴습니다.

## M0. Documentation Baseline

목표:

- 프로젝트 의도와 기술 스택 확정
- ROS 2 AMR 구조를 글과 다이어그램으로 설명
- 구현 전 설계 기준 마련

산출물:

- `README.md`
- `docs/01_ros2_amr_learning_guide.md`
- `docs/02_amr_system_architecture.md`
- `docs/03_implementation_roadmap.md`
- `docs/04_reference_links.md`

완료 기준:

- README만 봐도 프로젝트 목표가 이해된다.
- 왜 Jazzy LTS를 선택했는지 설명되어 있다.
- 어떤 package를 만들지 이름과 책임이 정리되어 있다.
- SLAM/Nav2를 왜 뒤로 미뤘는지 설명되어 있다.

## M1. Workspace and First ROS 2 Node

목표:

- ROS 2 C++ workspace 구조에 익숙해지기
- `colcon`, `ament_cmake`, `package.xml`, `CMakeLists.txt` 이해
- 첫 node를 package로 만들고 launch로 실행

구현:

- `src/amr_bringup`
- `src/amr_battery_driver`
- `mock_battery_driver_node`
- `mock_battery.launch.py`

완료 기준:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup mock_battery.launch.py
ros2 topic echo /battery_state
```

포트폴리오 포인트:

- ROS 2 workspace 생성부터 package 실행까지 직접 구성했다.
- 표준 `BatteryState` 메시지를 사용했다.
- parameter로 publish rate와 초기 전압을 바꿀 수 있다.

## M2. Custom Interfaces and Mock IO

목표:

- custom msg/srv 작성
- service server/client 이해
- IO board를 실제 장비 없이 mock으로 표현

구현:

- `src/amr_interfaces`
- `IoState.msg`
- `SetDigitalOutput.srv`
- `FaultState.msg`, 필요 시
- `src/amr_io_driver`
- `mock_io_driver_node`

완료 기준:

```bash
ros2 interface show amr_interfaces/msg/IoState
ros2 topic echo /io_state
ros2 service call /set_io amr_interfaces/srv/SetDigitalOutput "{channel: 1, value: true}"
```

포트폴리오 포인트:

- 표준 메시지와 custom 메시지의 경계를 이해했다.
- TCP/IP IO board를 붙이기 전 service 계약을 먼저 만들었다.

## M3. Manual Drive and Odometry Mock

목표:

- `/cmd_vel` 기반 AMR manual control 구조 구현
- differential drive kinematics 이해
- `/odom`과 TF publish 구조 이해

구현:

- `src/amr_base_controller`
- `src/amr_motor_driver`
- mock wheel state
- `/cmd_vel` subscribe
- `/odom` publish
- `odom -> base_link` TF publish

완료 기준:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

포트폴리오 포인트:

- Nav2 전 단계에서 가장 중요한 base interface를 직접 구현했다.
- wheel radius, wheel separation을 parameter로 분리했다.
- RViz에서 robot movement를 확인할 수 있다.

## M4. Safety Monitor

목표:

- `/cmd_vel`을 직접 motor로 보내지 않고 safety gate를 통과시키기
- estop, fault, command timeout 반영
- safety-critical 사고방식 표현

구현:

- `src/amr_safety_monitor`
- `/cmd_vel` subscribe
- `/cmd_vel_safe` publish
- `/safety_state` publish
- IO/Battery/Motor state subscribe
- command timeout

완료 기준:

- 정상 상태에서 `/cmd_vel`이 `/cmd_vel_safe`로 전달된다.
- estop input mock이 active면 `/cmd_vel_safe`가 zero가 된다.
- command가 일정 시간 없으면 zero command가 유지된다.
- battery critical이면 manual jog가 차단된다.

포트폴리오 포인트:

- UI 또는 Nav2 명령이 safety를 우회하지 못하는 구조를 만들었다.
- 하드웨어 estop과 software safety의 차이를 문서화했다.

## M5. Diagnostics and Lifecycle

목표:

- 현장 디버깅 가능한 상태 출력 만들기
- managed lifecycle로 bringup/shutdown 순서 관리
- fault reset 흐름 구현

구현:

- `diagnostic_updater` 적용
- `amr_system_manager`
- lifecycle node 또는 lifecycle-aware manager
- `/reset_fault`
- `/robot_state`

완료 기준:

```bash
ros2 topic echo /diagnostics
ros2 lifecycle nodes
ros2 service call /reset_fault std_srvs/srv/Trigger
```

테스트 시나리오:

- mock battery timeout 발생
- diagnostics level WARN/ERROR 전환
- fault reset 후 정상 복귀
- lifecycle deactivate 시 command 중단

포트폴리오 포인트:

- FAE 관점의 field troubleshooting 구조를 갖췄다.
- 로그와 diagnostics로 장애 원인을 좁힐 수 있다.

## M6. Qt Operator UI

목표:

- ROS 2 graph와 Qt UI를 안전하게 연결
- operator 관점의 상태 표시와 manual command 제공

구현:

- `src/amr_operator_ui`
- Qt 6 Widgets 또는 QML 중 하나 선택
- ROS executor background thread
- battery/io/motor/robot state 표시
- manual jog
- IO output command
- fault reset

완료 기준:

- UI가 `/battery_state`, `/io_state`, `/robot_state`, `/diagnostics`를 표시한다.
- UI 버튼으로 `/cmd_vel` 또는 manual command를 보낸다.
- UI에서 `/set_io`, `/reset_fault` service를 호출한다.
- ROS callback에서 UI widget 직접 접근 없이 signal/slot으로 업데이트한다.

포트폴리오 포인트:

- Linux PC 기반 operator UI 경험을 ROS 2 방식으로 보여준다.
- UI와 hardware driver가 분리된 구조를 설명할 수 있다.

## M7. Robot Description and RViz

목표:

- URDF/xacro와 TF tree 이해
- RViz에서 AMR 상태 시각화
- Nav2/Gazebo 준비 기반 만들기

구현:

- `src/amr_description`
- base footprint, wheel, laser link
- `robot_state_publisher`
- RViz config
- launch integration

완료 기준:

```bash
ros2 launch amr_bringup display.launch.py
rviz2
```

확인:

- `base_link`, `laser`, wheel links 표시
- `/odom` 이동에 따라 base가 움직임
- `/scan` mock 또는 vendor scan 표시

포트폴리오 포인트:

- 로봇 좌표계와 센서 mounting을 이해했다.
- Nav2 입력 조건 중 TF/URDF 기반을 준비했다.

## M8. Gazebo and ros2_control Preparation

목표:

- 실제 장비 없이 주행 검증 가능하게 만들기
- ros2_control의 hardware interface 개념 이해
- mock driver에서 simulation으로 확장

구현:

- `src/amr_sim`
- Gazebo Harmonic world
- differential drive simulation
- ros2_control config
- controller manager launch

완료 기준:

- Gazebo에서 AMR model spawn
- `/cmd_vel_safe`로 시뮬레이션 로봇 움직임
- `/odom`, `/tf`, `/scan` 확인

포트폴리오 포인트:

- 실제 하드웨어 전에도 제어/상태/UI를 통합 검증할 수 있다.
- real hardware와 simulation interface를 맞추는 방향을 잡았다.

## M9. Nav2 Readiness

목표:

- Nav2를 붙이기 전에 base가 조건을 만족하는지 검증
- SLAM/Nav2 도입 전 문제 범위 최소화

필수 조건:

- `/cmd_vel` input path 준비
- `/odom` 안정 출력
- `odom -> base_link` TF
- `base_link -> laser` TF
- `/scan`
- robot footprint
- command timeout
- lifecycle bringup 이해

완료 기준:

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_tools view_frames
ros2 launch nav2_bringup navigation_launch.py
```

포트폴리오 포인트:

- Nav2를 "그냥 실행"한 것이 아니라, 필요한 robot interface를 직접 준비했다.

## M10. Real Hardware Candidate

목표:

- 실제 장치 또는 장치 simulator와 통신
- 프로토콜 parser/transport 분리
- field failure scenario 검증

구현 후보:

- BMS serial real driver
- IO board TCP real driver
- SocketCAN motor transport
- vendor LiDAR driver launch wrapper

완료 기준:

- 장치 연결 끊김/복구 확인
- checksum/frame error diagnostics
- command timeout 안전 정지
- rosbag2로 문제 상황 기록

포트폴리오 포인트:

- 실제 AMR 장치 통신 경험을 ROS 2 구조로 보여준다.
- 현장 트러블슈팅 사고방식을 코드로 표현한다.

## Beyond the Baseline (Implemented)

baseline mock 스택 이후 추가된 인지/행동/센서 단계다. 핵심 수학은 ROS 의존성 없는 순수 Python으로 분리해 단위 테스트했고, ROS 노드는 그 위의 얇은 래퍼다.

- `amr_vision` - OpenCV ArUco 도킹 마커 인식. mock 카메라로 하드웨어 없이 검출 파이프라인 동작. `/docking_state` 발행. (`docs/10`)
- `amr_docking` - `/docking_state` 기반 정렬·접근 컨트롤러(`ALIGN/APPROACH/DOCKED`), `/scan` 장애물 정지(`BLOCKED`), odom 기반 완전 폐루프 데모. `/cmd_vel`은 기존 safety monitor를 그대로 거친다.
- `amr_lidar_driver` - mock 2D LiDAR 시뮬레이터. Gazebo 없이 `/scan` 제공(Nav2 입력 준비).
- `amr_tools` - health report에 docking/scan 추가, 요약 로직 순수 함수화 + 테스트.

다음 후보: system_manager `CHARGING` 모드 연동, `Dock.action` 액션 서버 승격, Gazebo 카메라 센서 통합, 순수 mock 스택의 TF(robot_state_publisher) 정리로 Nav2 readiness 완성.

## Suggested GitHub Issues

초기 issue로 나누기 좋은 작업:

- Create ROS 2 Jazzy workspace skeleton
- Add `amr_interfaces` package
- Implement mock battery driver
- Implement mock IO driver with `SetDigitalOutput`
- Add mock bringup launch
- Implement safety monitor command timeout
- Add differential drive odometry publisher
- Add diagnostics to battery/io/motor nodes
- Add lifecycle-aware system manager
- Add Qt operator UI skeleton
- Add URDF/xacro robot description
- Add RViz config
- Add Gazebo simulation package
- Add Nav2 readiness checklist launch

## Interview Talking Points

면접에서 이 프로젝트를 설명할 때 사용할 수 있는 문장:

- "단일 제어 프로그램이 아니라 ROS 2 node graph로 장치 통신, safety, UI, 제어를 분리했습니다."
- "처음부터 Nav2를 붙이지 않고, Nav2가 요구하는 `/cmd_vel`, `/odom`, TF, sensor interface를 먼저 만들었습니다."
- "배터리, IO, 모터, LiDAR를 mock driver로 먼저 구현해서 실제 장비 없이도 통합 테스트할 수 있게 했습니다."
- "FAE 업무에서 중요한 diagnostics, rosbag2, command timeout, reconnect, fault reset 시나리오를 설계에 포함했습니다."
- "Qt UI는 장치 포트를 직접 열지 않고 ROS 2 topic/service/action client로만 시스템과 통신하게 했습니다."

## Definition of Done for Portfolio

이 프로젝트가 포트폴리오로 충분히 설득력 있으려면 최소한 아래가 동작해야 합니다.

- `colcon build` 성공
- mock robot launch 한 번으로 주요 노드 실행
- Qt UI에서 battery/io/motor/robot state 표시
- UI manual jog가 `/cmd_vel`을 내고 `/odom`이 변함
- estop mock 입력 시 주행 명령이 차단됨
- `/diagnostics`에서 장치 상태 확인 가능
- rosbag2로 주요 topic 기록 가능
- README에 실행 방법과 아키텍처 설명이 있음
- docs에 왜 이런 구조인지 설명되어 있음
