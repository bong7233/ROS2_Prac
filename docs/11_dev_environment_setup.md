# Dev Environment Setup (Windows 11 -> Ubuntu/ROS 2)

이 문서는 Windows 11 PC만 있는 상황에서 이 프로젝트(ROS 2 Jazzy, OpenCV, 향후 Gazebo/Isaac Sim)를 개발·빌드·디버깅하기 위한 환경 선택과 구체적 설치 절차를 정리한다.

## 핵심 전제: GPU가 선택을 좌우한다

- OpenCV: 어떤 환경이든 가볍게 동작한다.
- Gazebo Harmonic: OpenGL/Vulkan 가속 필요. 내장그래픽으로 가벼운 월드는 가능하나 무거우면 느리다.
- Isaac Sim: **NVIDIA RTX GPU(최소 RTX 2070급, 권장 3070/4070+), VRAM 8GB+, RAM 32GB+** 가 사실상 필수다. 저가 별도 머신으로는 불가능하다. 따라서 "지금 PC에 쓸만한 RTX GPU가 있는가"가 가장 중요한 변수다.

## 옵션 비교

| 방식 | 장점 | 단점 | 비용 |
| --- | --- | --- | --- |
| WSL2 | 설치 즉시·무료, 리부팅 없이 윈도우와 병행, VS Code 연동, CPU 빌드 거의 네이티브, CUDA 패스스루 | Gazebo는 WSLg로 동작하나 GPU 센서 렌더링 불완전 가능, Isaac Sim 비권장, 실장비 USB 연동 번거로움 | 0원 |
| 외장 SSD 우분투 | 본체 GPU 그대로 사용(RTX면 Isaac Sim 네이티브 가능), 네이티브 성능, 내장 디스크 무수정·되돌리기 쉬움 | 부팅 전환 필요, USB가 내장 NVMe보다 약간 느림, NVIDIA 드라이버 설치 필요 | 8~15만원 |
| 듀얼부팅 | 네이티브 최고 성능, 본체 GPU 100%, 무료 | 파티션/부트로더 리스크, 윈도우 업데이트가 GRUB 덮기도 함, 공간 필요 | 0원 |
| FreeDOS 노트북 | 전용 리눅스 머신, 완전 분리 | 50만원대 사양 약함, Gazebo 버벅, Isaac Sim 불가 | 40~50만원 |

## 추천 순위와 전략

1. **WSL2 (지금 바로, 무료)** - 현재 작업의 대부분(ROS 2 빌드/디버깅, OpenCV, RViz, Qt, 가벼운 Gazebo).
2. **외장 SSD 우분투 (~10만원)** - 무거운 Gazebo·네이티브 성능·실장비 연동이 필요할 때. WSL2와 병행이 가성비 최적.
3. **듀얼부팅 (무료)** - 최고 성능을 원하고 파티션 작업 리스크를 감수할 수 있을 때.
4. **FreeDOS 노트북 (비추천)** - 본체보다 약한 머신을 사는 셈.

50만원을 노트북에 쓰지 말고, WSL2(0원) + 필요 시 외장 SSD(~10만원)로 끝낸 뒤 남는 예산은 향후 GPU 업그레이드나 Isaac Sim 클라우드(NVIDIA Omniverse Cloud, GPU 인스턴스 대여)에 쓰는 편이 효율적이다. 본체에 RTX가 있으면 Isaac Sim은 윈도우 네이티브로 돌리고 ROS 2를 WSL2/네트워크로 브리지하는 방법도 있다.

## WSL2에 ROS 2 Jazzy + 이 레포 설치

PowerShell(관리자)에서:

```powershell
wsl --install -d Ubuntu-24.04
```

재부팅 후 Ubuntu 24.04 셸에서:

```bash
# 1) 로케일
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2) ROS 2 apt 저장소
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}')
. /etc/os-release
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 3) ROS 2 Jazzy + 빌드 도구
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

레포 클론 후 워크스페이스 빌드:

```bash
git clone https://github.com/bong7233/ROS2_Prac.git
cd ROS2_Prac
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

실행 예시:

```bash
ros2 launch amr_bringup mock_robot.launch.py            # mock 로봇 스택
ros2 launch amr_docking dock_closed_loop.launch.py      # 폐루프 도킹 데모
ros2 launch amr_lidar_driver mock_lidar.launch.py       # mock LiDAR /scan
```

GUI(RViz/rqt/Qt UI)는 Windows 11 + WSL2의 WSLg로 별도 X 서버 없이 바로 창이 뜬다. Gazebo도 WSLg로 실행되지만 3D가 무거우면 외장 SSD/네이티브가 낫다.

## ROS 없이 가능한 작업 (개발 PC/WSL 어디서나)

이 프로젝트는 비전 기하, 도킹 제어 법칙, 스캔 모델을 ROS 의존성 없는 순수 Python으로 분리해 두어, ROS 설치 전에도 핵심 로직을 테스트할 수 있다.

```bash
pip install opencv-contrib-python-headless numpy pytest
cd src/amr_vision        && python3 -m pytest test -q
cd ../amr_docking        && python3 -m pytest test -q
cd ../amr_lidar_driver   && python3 -m pytest test -q
cd ../amr_tools          && python3 -m pytest test/test_report_model.py -q
```

## Isaac Sim 주의

Isaac Sim은 RTX GPU가 없으면 로컬에서 돌릴 수 없다. 현 PC에 RTX가 없다면 (1) 향후 GPU 업그레이드, (2) 클라우드 GPU 사용을 고려한다. ROS 2 학습·이 프로젝트의 모든 단계는 Isaac Sim 없이도 진행 가능하므로, Isaac Sim은 가장 마지막 단계로 미루는 것이 합리적이다.
