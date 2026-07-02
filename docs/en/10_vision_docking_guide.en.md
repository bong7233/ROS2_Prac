# Vision Docking Guide (amr_vision / amr_docking)

This guide covers the OpenCV ArUco docking-marker perception (`amr_vision`) and
the docking behaviour controller (`amr_docking`). Together they implement the
fiducial-marker docking that real AMRs use to align with a charging station,
built the project's mock-first way so it runs with no hardware.

## Goal

- Detect a docking ArUco marker in a camera image.
- Express the docking error (range / lateral offset / bearing) in the robot base
  frame so a controller can act on it.
- Drive `/cmd_vel` to align with and approach the dock, stopping for obstacles.
- Keep the geometry and control math ROS-free so they are unit testable.

## Node Graph

```text
mock_dock_camera ─/image,/camera_info─▶ aruco_docking ─/docking_state─▶ docking_controller ─/cmd_vel─▶ safety ─▶ base
        ▲                                                                      ▲
        └────────────────── /odom (closed loop) ───────────────────┬──────────┘
                                                       mock_lidar ─/scan (obstacle stop)
```

- `mock_dock_camera_node` renders a marker and publishes `Image` + `CameraInfo`.
  With `use_odom` it places the marker at a fixed world pose and re-renders it
  from the live robot pose, closing the loop without Gazebo.
- `aruco_docking_node` detects the marker and publishes `amr_interfaces/DockingState`
  plus `/diagnostics` and an annotated debug image.
- `docking_controller_node` turns the docking error into `/cmd_vel` through the
  same safety-gated path as manual jog and Nav2.

## Frames

- Camera optical frame (REP-103): x right, y down, z forward.
- Robot base frame (REP-103): x forward, y left, z up.

`solvePnP` returns the marker pose in the optical frame; everything is remapped
to the base frame so the controller reasons in robot terms.

## Why translation only (no yaw)

A single planar marker's face yaw suffers from the planar pose ambiguity and can
yield non-finite poses for some corner layouts, so only translation-based signals
(range, lateral, bearing) are reported; pose estimation uses `SOLVEPNP_ITERATIVE`
and non-finite results are filtered. Reliable dock-face yaw needs a multi-marker
board and is left for a later increment.

## Controller phases

- `DOCKED` - aligned: stop and hold.
- `BLOCKED` - obstacle in the `/scan` corridor (the dock itself is ignored via
  `dock_margin_m`): stop.
- `SEARCH` - no marker: stop (or rotate if `search_yaw_rate_radps` > 0).
- `ALIGN` - bearing too large: rotate in place first.
- `APPROACH` - marker ahead: drive forward, slowing with range.

The controller starts disabled; toggle it with `/enable_docking`
(`std_srvs/SetBool`). `dock_demo` and `dock_closed_loop` auto-start it.

## Quick Start

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Perception only
ros2 launch amr_vision docking_vision.launch.py
ros2 topic echo /docking_state

# Fully closed-loop mock docking (no Gazebo)
ros2 launch amr_docking dock_closed_loop.launch.py
```

Swap `mock_dock_camera` for a real driver (`v4l2_camera` / `usb_cam`) and remap
`image` / `camera_info` to reuse the same detector.

## Testing without ROS

The geometry (`amr_vision/marker_docking.py`, `amr_vision/aruco_compat.py`), the
control law (`amr_docking/docking_controller.py`), and the scan model
(`amr_lidar_driver/scan_model.py`) import no ROS, so they run under plain pytest
with OpenCV/NumPy:

```bash
cd src/amr_vision && python3 -m pytest test -q
cd src/amr_docking && python3 -m pytest test -q
cd src/amr_lidar_driver && python3 -m pytest test -q
```

`aruco_compat` hides the OpenCV 4.6 (Jazzy) vs 4.7+ ArUco API differences so the
same code runs in CI and on a developer machine.

## Next Steps

- Mock camera reacting to `/cmd_vel_safe` as well as `/odom` for a tighter loop.
- Integrate with the `system_manager` `CHARGING` mode and docking sequence.
- Promote docking to a `Dock.action` server (feedback: phase/range; result).
- Add a camera link/optical frame to `amr_description` and a Gazebo camera sensor.
