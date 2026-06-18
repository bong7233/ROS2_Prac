# Reference Links and Version Rationale

Korean version: [04_reference_links.md](../04_reference_links.md)

The project uses conservative, well-supported technology choices for a ROS 2 portfolio.

## Version Policy

| Area | Choice |
| --- | --- |
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco LTS |
| Runtime language | C++17 |
| Tooling language | Python |
| UI later | Qt 6 |
| Build | colcon + ament |
| Navigation later | Nav2 |
| Simulation later | Gazebo Harmonic |

## Official References

- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/index.html)
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Windows installation](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)
- [colcon tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS 2 QoS concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 lifecycle design](https://design.ros2.org/articles/node_lifecycle.html)
- [tf2 concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html)
- [diagnostic_updater](https://docs.ros.org/en/jazzy/p/diagnostic_updater/)
- [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/)
- [Use ROS 2 to interact with Gazebo](https://gazebosim.org/docs/latest/ros2_integration/)
- [Gazebo DiffDrive system API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [ros2_control documentation](https://control.ros.org/jazzy/)
- [Nav2 documentation](https://docs.nav2.org/)
- [Qt 6 supported platforms](https://doc.qt.io/qt-6/supported-platforms.html)

## Why Jazzy

Jazzy is a strong baseline because it is an LTS release, targets Ubuntu 24.04, and has a mature ROS ecosystem. Newer distributions may exist, but a portfolio intended for industrial and FAE roles benefits from stable documentation and package availability.
