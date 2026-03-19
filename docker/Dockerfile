FROM osrf/ros:humble-desktop-full

# Update & upgrade
RUN apt update && apt upgrade -y

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    wget \
    gedit \
    nano \  
    sudo \
    git \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 humble setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install conan
RUN pip install conan

# Clone rby1-sdk
WORKDIR /
RUN git clone https://github.com/RainbowRobotics/rby1-sdk --recurse-submodules

# Remove system protobuf to avoid version conflict with conan's protobuf
RUN apt-get remove -y libprotobuf-dev protobuf-compiler || true

# Conan install
RUN conan profile detect
WORKDIR /rby1-sdk
RUN conan install . -s build_type=Release -b missing -of build

# Build & install (CMake < 3.23, Option 2)
WORKDIR /rby1-sdk/build
RUN cmake .. -G "Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=./conan_toolchain.cmake \
    -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=ON
RUN cmake --build .
RUN make install

RUN apt install -y \
    ros-humble-rqt*

# MoveIt2 + 관련 패키지 (kdl_kinematics_plugin은 moveit에 포함)
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-control-msgs \
    && rm -rf /var/lib/apt/lists/*