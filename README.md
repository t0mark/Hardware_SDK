# rby1

RB-Y1 humanoid robot ROS2 package (description, URDF, launch, hardware nodes).

## 환경

- **U-PC**: Custom Jetson (Custom JetPack) — 일부 제한 사항 있음
  - Web Browser 미지원 (Firefox, Chromium)
  - Joystick 미지원 (8Bitdo 연결 불가 확인)

## 의존성 설치

### RB-Y1 SDK

```bash
sudo apt install python3-pip
pip install conan

git clone --recurse-submodules git@github.com:RainbowRobotics/rby1-sdk.git
cd rby1-sdk

conan install . -s build_type=Release -b missing -of build
cd build
cmake .. -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=./conan_toolchain.cmake \
  -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
  -DCMAKE_BUILD_TYPE=Release
cmake --build .
make install
```

### ROS2 Humble 패키지

```bash
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-rclcpp-action \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-tf2-ros \
  ros-humble-control-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-kdl-kinematics-plugin \
  libcurl4-openssl-dev
```

## 빌드

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

## 사용법

```bash
# U-PC (RB-Y1 온보드) — LiDAR + 이더넷 직결
ros2 launch rby1 bringup.launch.py use_lidar:=true robot_ip:=192.168.30.1

# Laptop — AIR_ValidationLab3 네트워크
ros2 launch rby1 bringup.launch.py robot_ip:=192.168.3.25

# 종료
ros2 launch rby1 finish.launch.py robot_ip:=192.168.3.25
```

### 런치 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_ip` | `192.168.30.1:50051` | 로봇 gRPC 주소 (host:port) |
| `model` | `a` | 로봇 모델 (`a` / `m` / `ub`) |
| `end_effector` | `original` | 엔드이펙터 타입 (`original` / `inspire`) |
| `rate` | `50.0` | 관절 상태 퍼블리시 주기 [Hz] |
| `rviz` | `true` | RViz 실행 여부 |
| `use_lidar` | `false` | LiDAR 노드 실행 여부 |
