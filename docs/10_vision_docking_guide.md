# Vision Docking Guide (amr_vision)

English version: [Vision Docking Guide](en/10_vision_docking_guide.en.md)

이 문서는 `amr_vision` 패키지가 제공하는 OpenCV 기반 ArUco 도킹 마커 인식 기능을 설명합니다. 실제 AMR이 충전 스테이션에 정밀 정렬할 때 흔히 쓰는 fiducial marker 도킹을, 이 프로젝트의 mock 우선 철학에 맞춰 하드웨어 없이 검증 가능하게 구현했습니다.

## Goal

- 카메라 영상에서 도킹용 ArUco 마커를 검출한다.
- 마커까지의 거리/횡방향 오차/방위각을 로봇 base frame 기준으로 계산한다.
- 다른 노드(향후 도킹 컨트롤러, system manager)가 사용할 수 있도록 `/docking_state`로 발행하고, 기존 스택과 동일하게 `/diagnostics`로 상태를 보고한다.
- 핵심 기하 계산은 ROS 의존성이 없는 순수 Python으로 분리해서, ROS 없이도 단위 테스트할 수 있게 한다.

## Node Graph

```text
mock_dock_camera ──/image──────────▶ aruco_docking ──/docking_state─────▶ (docking controller, UI)
                └─/camera_info──────▶              ├─/diagnostics───────▶ (health_report, UI)
                                                   └─/docking_debug_image▶ (rqt_image_view, RViz)
```

- `mock_dock_camera_node`: 마커를 가상 장면에 렌더링해서 `sensor_msgs/Image`와 `sensor_msgs/CameraInfo`를 발행한다. `approach_speed_mps`를 주면 마커가 서서히 가까워져 로봇이 도킹에 접근하는 상황을 흉내 낸다.
- `aruco_docking_node`: 영상을 받아 마커를 검출하고 `amr_interfaces/DockingState`를 발행한다. 실제 카메라 드라이버(`usb_cam`, `v4l2_camera` 등)로 `mock_dock_camera`만 교체하면 동일한 검출 노드를 그대로 쓸 수 있다.

## Frames and Conventions

- 카메라 광학 좌표계(REP-103): `x` 오른쪽, `y` 아래, `z` 전방.
- 로봇 base 좌표계(REP-103): `x` 전방, `y` 좌측, `z` 위.

`solvePnP`는 마커 자세를 카메라 광학 좌표계로 돌려준다. `amr_vision`은 이를 base 좌표계로 변환해서 보고하므로, 도킹 컨트롤러는 "전방 거리", "좌측 오차", "좌향 방위각"처럼 로봇 관점으로 바로 해석할 수 있다.

| 필드 | 의미 |
| --- | --- |
| `detected` | 대상 마커 검출 여부 |
| `marker_id` | 검출된 마커 ID |
| `range_m` | 마커까지 전방 거리 |
| `lateral_offset_m` | 횡방향 오차, 좌측이 양수 |
| `bearing_rad` | 마커 중심 방위각, 좌측이 양수 |
| `aligned` | 거리/횡오차/방위각이 모두 허용 범위면 true |

## Why translation only (no yaw yet)

단일 평면 마커의 면 방향(dock-face yaw)은 잘 알려진 *planar pose ambiguity* 때문에 신뢰도가 낮고, 특정 코너 배치에서는 solver가 비정상 값을 내기도 한다. 그래서 이번 단계에서는 translation 기반 신호(거리/횡오차/방위각)만 보고한다. 이 세 값만으로도 횡정렬 + 직진 접근 도킹 컨트롤러를 만들 수 있다. 면 방향까지 안정적으로 복원하려면 마커 보드(여러 마커) 또는 접근 중 모션 기반 disambiguation이 필요하며, 이는 다음 증분 작업으로 남겨 두었다.

비정상 자세(NaN/Inf)는 검출 단계에서 걸러내어 `detected=false`로 처리한다.

## Parameters

`config/docking.yaml` 참고.

`mock_dock_camera`:

| Parameter | Default | 설명 |
| --- | --- | --- |
| `image_width` / `image_height` | 640 / 480 | 영상 크기 |
| `horizontal_fov_deg` | 70.0 | 수평 화각(내부 K 계산용) |
| `marker_id` | 3 | 렌더링할 마커 ID |
| `marker_length_m` | 0.20 | 마커 한 변 길이 |
| `marker_forward_m` / `marker_left_m` / `marker_up_m` | 2.0 / 0.1 / 0.0 | base frame 기준 마커 위치 |
| `approach_speed_mps` | 0.15 | 0보다 크면 마커가 `min_forward_m`까지 접근 |

`aruco_docking`:

| Parameter | Default | 설명 |
| --- | --- | --- |
| `marker_id` | 3 | 추적 대상 마커 ID |
| `marker_length_m` | 0.20 | 마커 한 변 길이(거리 추정 스케일) |
| `publish_debug_image` | true | 주석 영상 발행 여부 |
| `aligned_max_range_m` | 0.6 | 도킹 완료 거리 상한 |
| `aligned_min_range_m` | 0.15 | 도킹 완료 거리 하한 |
| `aligned_max_lateral_m` | 0.03 | 횡오차 허용치 |
| `aligned_max_bearing_rad` | 0.05 | 방위각 허용치 |

## Quick Start

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch amr_vision docking_vision.launch.py
```

다른 터미널에서 도킹 상태를 확인한다.

```bash
ros2 topic echo /docking_state
ros2 topic hz /image
ros2 run rqt_image_view rqt_image_view /docking_debug_image
```

마커가 접근하면서 `range_m`이 줄어들고, 충분히 가까워지고 정렬되면 `aligned`가 true로 바뀐다.

실제 카메라로 바꾸려면 `mock_dock_camera` 대신 카메라 드라이버를 실행하고 토픽만 맞춰 주면 된다.

```bash
ros2 run amr_vision aruco_docking_node --ros-args \
  -r image:=/camera/image_raw -r camera_info:=/camera/camera_info \
  -p marker_id:=3 -p marker_length_m:=0.20
```

## Testing without ROS

도킹 기하 로직(`amr_vision/marker_docking.py`, `amr_vision/aruco_compat.py`)은 ROS를 import하지 않으므로 OpenCV와 NumPy만 있으면 단위 테스트가 가능하다.

```bash
cd src/amr_vision
python3 -m pytest test -q
```

테스트는 알려진 자세에 마커를 합성 렌더링한 뒤 `검출 → solvePnP → 도킹 오차` 전체 경로를 통과시켜, 거리/횡오차/방위각이 정답과 일치하는지 검증한다. 렌더링 경로와 검출 경로가 같은 코드를 공유하므로 회귀를 함께 잡아낸다.

`aruco_compat`는 OpenCV 4.6(Ubuntu 24.04 / ROS 2 Jazzy의 `python3-opencv`)과 4.7+(pip wheel)의 ArUco API 차이를 feature detection으로 흡수하므로, CI와 개발 PC에서 동일한 코드가 동작한다.

## Docking Controller (amr_docking)

`amr_docking` 패키지는 `/docking_state`를 받아 마커에 정렬하고 접근하는 `/cmd_vel`을 생성한다. 제어 법칙(`amr_docking/docking_controller.py`)은 perception과 마찬가지로 ROS 의존성 없이 분리되어 단위 테스트된다.

동작 단계:

- `DOCKED`: 검출기가 `aligned`를 보고하면 정지·유지.
- `SEARCH`: 마커 미검출이면 정지(또는 `search_yaw_rate_radps`>0이면 천천히 회전).
- `ALIGN`: 마커는 보이나 방위각이 크면 제자리 회전으로 먼저 정렬.
- `APPROACH`: 마커가 정면이면 전진(거리에 비례해 감속)하며 방위각 보정.
- `BLOCKED`: `/scan` 접근 코리도어에 장애물이 있으면 정지. 도킹 구조물 자체는 `dock_margin_m`로 무시해 도크를 장애물로 오인하지 않는다(`enable_obstacle_stop`으로 끌 수 있음).

생성된 `/cmd_vel`은 manual jog/Nav2와 동일하게 safety monitor를 거치므로, estop/battery/timeout 게이트가 그대로 적용된다. 기본값은 `auto_start: false`이며 `/enable_docking`(`std_srvs/SetBool`)로 시작·정지한다. `dock_demo.launch.py`는 perception과 컨트롤러를 함께 띄우고 컨트롤러를 자동 시작한다.

```bash
ros2 launch amr_docking dock_demo.launch.py
ros2 topic echo /cmd_vel
```

제어 법칙 테스트:

```bash
cd src/amr_docking
python3 -m pytest test -q
```

## Closed-Loop Demo (no Gazebo)

`mock_dock_camera`는 `use_odom: true`일 때 마커를 world(odom) 좌표에 고정해 두고, `/odom`으로 들어오는 로봇 실제 위치 기준으로 다시 렌더링한다. 따라서 컨트롤러가 `/cmd_vel`을 내면 `safety → base → /odom → 카메라 → 검출 → /docking_state → 컨트롤러`로 루프가 실제로 닫힌다. Gazebo 없이 mock 스택만으로 동작한다.

```bash
ros2 launch amr_docking dock_closed_loop.launch.py
ros2 topic echo /docking_state
ros2 topic echo /cmd_vel
```

로봇은 원점에서 출발하고 마커는 앞쪽 약간 옆(`marker_world_x`, `marker_world_y`)에 있어 `ALIGN → APPROACH → DOCKED` 진행을 볼 수 있다. world→카메라 변환 기하(`world_marker_to_camera_center`)도 ROS 없이 단위 테스트된다.

## Next Steps

- `amr_description`에 `camera_link`/`camera_optical_frame`을 추가해 두었다. 다음은 Gazebo 카메라 센서를 붙여 sim에서 실제 영상으로 검출까지 돌리는 것.
- `system_manager`가 `/docking_state`를 구독해 `/robot_state.docked`로 도킹 여부를 보고한다(상태 메시지에 `(docked)` 표시). 다음은 도킹 완료 시 `CHARGING` 모드 자동 전환.
- 도킹을 `Dock.action`(피드백 phase/range, 결과) 액션 서버로 승격.
- 마커 보드(여러 마커)로 dock-face yaw까지 안정적으로 복원.
- `amr_description`에 카메라 링크/광학 프레임 추가, Gazebo 카메라 센서로 sim 통합.
- 실제 카메라 드라이버(`v4l2_camera`/`usb_cam`) 연동.
