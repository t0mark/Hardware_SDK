# 1. 공식 ROS 2 Humble 데스크탑 풀 이미지 사용
FROM osrf/ros:humble-desktop-full

# 기본 셸 설정
SHELL ["/bin/bash", "-c"]

# 2. 시스템 업데이트 및 필수 패키지 설치
# 사용자가 요청한 ROS 패키지 및 개발 도구 통합
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-dev-tools \
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
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    libcurl4-openssl-dev \
    python3-pip \
    curl \
    wget \
    gedit \
    nano \
    sudo \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 3. Conan 및 SDK 빌드 환경 구축
RUN pip install conan

# SDK 소스 가져오기
WORKDIR /
RUN git clone https://github.com/RainbowRobotics/rby1-sdk --recurse-submodules

# SDK 빌드 시 충돌 방지를 위한 기존 Protobuf 제거
RUN apt-get remove -y libprotobuf-dev protobuf-compiler || true

# Conan 설정 및 의존성 설치
RUN conan profile detect
WORKDIR /rby1-sdk
RUN conan install . -s build_type=Release -b missing -of build

# 4. SDK 컴파일 및 시스템 설치
WORKDIR /rby1-sdk/build
RUN cmake .. -G "Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=./conan_toolchain.cmake \
    -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=ON

RUN cmake --build . && make install

# 5. ROS SDK 구축
RUN mkdir -p /workspace/ros_ws/src && \
    git clone -b plat/custom-rby1 https://github.com/t0mark/hardware-sdk /workspace/ros_ws/src/rby1

# colcon build 수행
WORKDIR /workspace/ros_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 기본 작업 디렉토리 설정
WORKDIR /workspace

# 5. 환경 변수 및 ROS 소스 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/ros_ws/install/setup.bash" >> ~/.bashrc