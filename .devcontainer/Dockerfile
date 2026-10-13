FROM nvidia/cuda:13.2.1-devel-ubuntu24.04

ENV ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    TZ=America/Chicago \
    ROS_LOG_DIR="/ws/log/runtime" \
    PATH="/devtools:${PATH}" \
    CC=clang \
    CXX=clang++ \
    CMAKE_GENERATOR=Ninja \
    CMAKE_EXPORT_COMPILE_COMMANDS=ON \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache \
    CCACHE_DIR=/ws/.ccache \
    LDFLAGS='-fuse-ld=mold'

# ROS 2 + Isaac ROS + NVIDIA VPI apt sources.
# This image builds on BOTH arches:
#   * arm64  -> Jetson / JetPack 7.2 (the robot).  Isaac ROS suite "noble-jetpack",
#              VPI + cuda-toolkit-13-2 from repo.download.nvidia.com/jetson/common.
#   * amd64  -> desktop dev container.  Isaac ROS suite "noble", VPI etc. from
#              repo.download.nvidia.com/jetson/x86_64/noble.
# "noble-jetpack" has no amd64 packages and plain "noble" has no arm64 Isaac ROS
# packages, hence the split. Suite r39.2 == Jetson Linux 39.2.x (JetPack 7.2.x);
# bump it when you move JetPack minor. Both NVIDIA repos are signed on r39.2.
RUN apt-get update && apt-get install -y software-properties-common curl gnupg && \
    apt-add-repository universe && \
    ARCH="$(dpkg --print-architecture)" && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /tmp/ros2-apt-source.deb && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o /usr/share/keyrings/nvidia-l4t.gpg && \
    if [ "$ARCH" = "arm64" ]; then \
        ISAAC_SUITE="noble-jetpack"; L4T_PATH="jetson/common"; \
    else \
        ISAAC_SUITE="noble"; L4T_PATH="jetson/x86_64/noble"; \
    fi && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.6 ${ISAAC_SUITE} main" > /etc/apt/sources.list.d/nvidia-isaac-ros.list && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-l4t.gpg] https://repo.download.nvidia.com/${L4T_PATH} r39.2 main" > /etc/apt/sources.list.d/nvidia-l4t.list && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    { apt-mark unhold libcublas-13-2 libcublas-dev-13-2 || true; } && \
    apt-get upgrade -y && apt-get install -y \
        ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-librealsense2 \
        ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-ament-clang-format \
        ros-${ROS_DISTRO}-apriltag-msgs \
        ros-${ROS_DISTRO}-isaac-ros-apriltag \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-rclc \
        ros-${ROS_DISTRO}-gtsam \
        ros-${ROS_DISTRO}-apriltag-detector \
        libboost-all-dev \
        libasio-dev \
        libfastcdr-dev \
        clang \
        libomp-dev \
        mold \
        ccache \
        clangd \
        clang-tidy \
        ninja-build \
        gdb \
        bash-completion \
        sudo \
        python3-argcomplete && \
    if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
        apt-get purge -y cuda-compat-13-2 cuda-compat-orin-13-2 && \
        rm -rf /usr/local/cuda-*/compat /usr/local/cuda-*/compat_orin; \
    fi && \
    rm -rf /var/lib/apt/lists/*

RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    register-python-argcomplete ros2 > /etc/bash_completion.d/ros2 && \
    register-python-argcomplete colcon > /etc/bash_completion.d/colcon

WORKDIR /ws
