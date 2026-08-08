FROM nvidia/cuda:12.9.0-devel-ubuntu22.04

ENV ROS_DISTRO=humble \
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

# See https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html 
# ROS2 + Isaac Ros setup, the base image does not have ROS2
RUN apt-get update && apt-get install -y software-properties-common curl && \
    apt-add-repository universe && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') &&\
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" &&\
    dpkg -i /tmp/ros2-apt-source.deb &&\
    rm -rf /tmp/ros2-apt-source.deb &&\
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" > /etc/apt/sources.list.d/nvidia-isaac-ros.list &&\
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        ros-humble-ros-base \
        ros-dev-tools \
        ros-${ROS_DISTRO}-rviz2  \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-librealsense2 \
        ros-${ROS_DISTRO}-ament-cmake-clang-format  \
        ros-${ROS_DISTRO}-gtsam  \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-ament-clang-format  \
        ros-${ROS_DISTRO}-apriltag-detector  \
        ros-${ROS_DISTRO}-isaac-ros-apriltag \
        ros-${ROS_DISTRO}-apriltag-msgs \
        ros-${ROS_DISTRO}-v4l2-camera \
        libboost-all-dev  \
        libasio-dev \
        libfastcdr-dev \
        libopencv-dev \
        clang \
        mold \
        ccache \
        clangd \
        ninja-build \
        gdb \
        bash-completion \
        python3-argcomplete \ 
        python3-pip && \
    pip3 install --upgrade cmake && \
    pip3 install numpy==1.26.4 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y sudo && \
    rm -rf /var/lib/apt/lists/* && \
    useradd -m -s /bin/bash -u 1000 -U ubuntu && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ${HOME}/.bashrc && \
    register-python-argcomplete3 ros2 > /etc/bash_completion.d/ros2 && \
    register-python-argcomplete3 colcon > /etc/bash_completion.d/colcon

WORKDIR /ws
