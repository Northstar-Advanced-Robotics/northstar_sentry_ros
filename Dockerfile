# JetPack 7.2.1 ships CUDA 13.2.1 on Ubuntu 24.04 (noble). This tag is
# multi-arch (linux/arm64 + linux/amd64), so the same Dockerfile builds on the
# Jetson (arm64) and on a desktop dev machine (amd64); the arch-specific apt
# sources below are selected at build time with `dpkg --print-architecture`.
FROM nvidia/cuda:13.2.1-devel-ubuntu24.04

# CHANGED: jazzy (Isaac ROS 4.x is Jazzy-only; Humble is not supported)
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
#
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

# CHANGED: libopencv-dev REMOVED from this list.
# Isaac ROS 4.6 is built against OpenCV 4.6.0 and pulls it in itself. Ubuntu's
# libopencv-dev is 4.6.x on noble, but installing it explicitly can win the
# dependency resolution and shadow NVIDIA's build. Let Isaac ROS supply it.
# (On a JetPack 7.2 *host*, NVIDIA tells you to `apt remove libopencv* opencv*`
# for the same reason.)
# CHANGED: the nvidia/cuda base image holds libcublas-13-2 / libcublas-dev-13-2
# at 13.4.0.1-1 (see `apt-mark showhold`). The NVIDIA repo's cuda-libraries-13-2
# -- pulled in transitively by ros-jazzy-isaac-ros-* via cuda-toolkit-13-2 --
# requires libcublas-13-2 >= 13.4.1.3, so the hold has to be released first or
# apt reports "held broken packages". `|| true` in case a future base image
# stops holding them.
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
        # CHANGED: re-added. A Jazzy build of both now exists in the ROS repo
        # (gtsam 4.2.0, apriltag-detector 3.1.0); tagslam/package.xml depends on
        # `gtsam` and `apriltag_detector`.
        ros-${ROS_DISTRO}-gtsam \
        ros-${ROS_DISTRO}-apriltag-detector \
        libboost-all-dev \
        libasio-dev \
        libfastcdr-dev \
        clang \
        # CHANGED: added. CUDA 13.2 bundles Thrust 3.2, whose scan/copy_if
        # dispatch headers `#include <omp.h>` unconditionally. GCC ships omp.h
        # but the CUDA host compiler here is clang, which needs its own copy;
        # libomp-dev drops it in clang's resource dir. (Not needed on JetPack 6
        # / CUDA 12.6 -- older Thrust only pulled omp.h when the OMP backend was
        # actually selected.)
        libomp-dev \
        mold \
        ccache \
        clangd \
        clang-tidy \
        ninja-build \
        gdb \
        bash-completion \
        python3-argcomplete \
        python3-pip && \
    # CHANGED: noble enforces PEP 668, so system pip installs need this flag.
    # CHANGED: numpy pin dropped -- 1.26.4 was a Humble-era constraint; Jazzy
    # expects numpy 2.x. Re-pin only if something concrete breaks.
    pip3 install --break-system-packages --upgrade cmake && \
    # CHANGED: on arm64/Jetson, drop the CUDA forward-compat packages. The CUDA
    # driver comes from the L4T host (mounted in), so cuda-compat-13-2 /
    # cuda-compat-orin-13-2 are dead weight -- and nvidia-container-toolkit 1.19.x
    # panics in its `cudacompat` hook parsing their libcuda.so headers, so the
    # container won't start under `--runtime=nvidia`. On amd64/desktop the
    # forward-compat libs are useful (newer container CUDA on an older host
    # driver), and cuda-compat-orin doesn't exist -- so leave them.
    if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
        apt-get purge -y cuda-compat-13-2 cuda-compat-orin-13-2 && \
        rm -rf /usr/local/cuda-*/compat /usr/local/cuda-*/compat_orin; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# CHANGED: Ubuntu 24.04 base images ALREADY ship a `ubuntu` user at UID 1000,
# so the old `useradd -u 1000 ubuntu` fails the build. Reuse the existing user.
RUN apt-get update && apt-get install -y sudo && \
    rm -rf /var/lib/apt/lists/* && \
    if ! id -u ubuntu >/dev/null 2>&1; then \
        useradd -m -s /bin/bash -u 1000 -U ubuntu; \
    else \
        usermod -s /bin/bash ubuntu && mkdir -p /home/ubuntu && chown ubuntu:ubuntu /home/ubuntu; \
    fi && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

# CHANGED: noble ships `register-python-argcomplete`, not the `3`-suffixed name.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    register-python-argcomplete ros2 > /etc/bash_completion.d/ros2 && \
    register-python-argcomplete colcon > /etc/bash_completion.d/colcon

WORKDIR /ws