# Reference Links and Version Rationale

English version: [Reference Links and Version Rationale](en/04_reference_links.en.md)

이 문서는 ROS2_Prac의 기술 선택 근거와 참고할 공식 문서를 정리합니다. 기준일은 2026-06-19입니다.

## 1. ROS 2 Distribution Choice

### Recommended for this project: ROS 2 Jazzy Jalisco

이 프로젝트는 ROS 2 Jazzy Jalisco를 기본 배포판으로 사용합니다.

선택 이유:

- Jazzy는 LTS 배포판이며 EOL이 2029-05입니다.
- Ubuntu 24.04 LTS와 잘 맞습니다.
- 2026-06 기준으로 최신 LTS인 Lyrical Luth도 존재하지만, 출시된 지 약 한 달밖에 지나지 않았으므로 포트폴리오와 현업 친화성 측면에서는 Jazzy가 더 보수적입니다.
- Humble Hawksbill도 여전히 지원되지만 2027-05 EOL이라 새 프로젝트에는 남은 지원 기간이 짧습니다.

참고:

- [ROS 2 distributions list](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst)
- [ROS 2 Jazzy Jalisco Released - Open Robotics](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released)
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 2. ROS 2 Core Concepts

학습과 구현에서 계속 참고할 문서:

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Creating a ROS 2 package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Using colcon to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS 2 Jazzy Windows binary installation](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)
- [ROS 2 launch tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 QoS settings](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 managed node lifecycle design](https://design.ros2.org/articles/node_lifecycle.html)
- [tf2 concept](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html)

## 3. AMR Control and Hardware Abstraction

- [ros2_control documentation](https://control.ros.org/jazzy/)
- [Writing a ros2_control hardware component](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [ros2_control hardware interface types](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- [ros2_control demos](https://github.com/ros-controls/ros2_control_demos)

이 프로젝트에서는 처음부터 ros2_control을 강제하지 않습니다. 먼저 AMR의 기본 topic/service와 mock driver를 구현한 뒤, motor/base 부분을 ros2_control로 옮기는 순서를 권장합니다.

## 4. Robot Description and TF

- [robot_state_publisher package](https://docs.ros.org/en/jazzy/p/robot_state_publisher/)
- [Using URDF with robot_state_publisher](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-cpp.html)
- [Nav2 setup guide: URDF and robot state publisher](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html)

AMR에서 TF는 선택 기능이 아니라 기본 인프라입니다. 최소한 `odom -> base_link`, `base_link -> laser` 관계를 안정적으로 제공해야 합니다.

## 5. Diagnostics and Field Debugging

- [diagnostic_updater package](https://docs.ros.org/en/jazzy/p/diagnostic_updater/)
- [diagnostic_msgs package](https://docs.ros.org/en/jazzy/p/diagnostic_msgs/)
- [rosbag2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)

FAE 포트폴리오에서는 diagnostics가 매우 중요합니다. "장치가 연결되어 있나?", "마지막 수신이 몇 ms 전인가?", "fault code가 무엇인가?"를 UI와 로그에서 설명할 수 있어야 합니다.

## 6. Simulation and Navigation Later

- [Gazebo with ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Simulation-Gazebo.html)
- [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/)
- [Use ROS 2 to interact with Gazebo](https://gazebosim.org/docs/latest/ros2_integration/)
- [Gazebo DiffDrive system API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Nav2 documentation](https://docs.nav2.org/)
- [Nav2 first-time robot setup guide](https://docs.nav2.org/setup_guides/index.html)
- [Nav2 odometry setup](https://docs.nav2.org/setup_guides/odom/setup_odom.html)
- [Nav2 robot_localization guide](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

Nav2는 이 프로젝트의 후반 목표입니다. 먼저 robot base가 Nav2 입력 조건을 만족해야 합니다.

## 7. Qt UI

- [Qt 6 supported platforms](https://doc.qt.io/qt-6/supported-platforms.html)
- [Qt CMake manual](https://doc.qt.io/qt-6/cmake-manual.html)
- [Qt Widgets overview](https://doc.qt.io/qt-6/qtwidgets-index.html)
- [Qt QML overview](https://doc.qt.io/qt-6/qtqml-index.html)

Ubuntu 24.04의 distro package와 Qt Online Installer 사이에는 버전 차이가 있을 수 있습니다. 포트폴리오 초기에는 빌드 재현성을 위해 apt 기반 Qt6를 우선하고, UI 요구가 커질 때 Qt Online Installer 또는 특정 Qt LTS를 검토하는 편이 안전합니다.

## 8. Hardware Communication References

실제 구현 시 참고할 Linux/ROS 주변 도구:

- [SocketCAN kernel documentation](https://docs.kernel.org/networking/can.html)
- [can-utils](https://github.com/linux-can/can-utils)
- [Boost.Asio](https://www.boost.org/doc/libs/release/doc/html/boost_asio.html)
- [Modbus protocol overview](https://modbus.org/specs.php)

모터 드라이브가 CANopen/CiA402라면 protocol을 직접 전부 구현하기 전에 ROS 2 CANopen 생태계를 검토합니다. vendor custom CAN이면 frame encoder/decoder와 SocketCAN transport를 분리해 테스트 가능하게 만듭니다.

## 9. Version Policy Summary

| Area | Policy |
| --- | --- |
| ROS 2 distro | Jazzy LTS first |
| OS | Ubuntu 24.04 LTS |
| Language | C++17 first, Python helper only |
| UI | Qt 6 |
| Build | colcon + ament_cmake |
| Simulation | Gazebo Harmonic when simulation starts |
| Navigation | Nav2 after base interface is stable |
| Control | Start simple, migrate motor/base to ros2_control when ready |
| Hardware | Mock first, real transport later |
