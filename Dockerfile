# CHANGED: JetPack 7.2.1 ships CUDA 13.2.1 on Ubuntu 24.04 (noble).
# VERIFY: confirm this exact tag exists for linux/arm64 before committing:
#   docker manifest inspect nvidia/cuda:13.2.1-devel-ubuntu24.04 | grep arm64
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

# ROS 2 + Isaac ROS apt sources.
# The ros-apt-source .deb auto-detects the codename, so it picks up noble here.
RUN apt-get update && apt-get install -y software-properties-common curl gnupg && \
    apt-add-repository universe && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -rf /tmp/ros2-apt-source.deb && \
    # CHANGED: release-4.6 + "noble-jetpack" suite (was release-3 / jammy).
    # noble-jetpack is the Jetson JetPack 7.2 channel; plain "noble" is the x86 channel.
    # Swap release-4.6 -> release-4 if you want to float on the latest 4.x minor.
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.6 noble-jetpack main" > /etc/apt/sources.list.d/nvidia-isaac-ros.list && \
    # CHANGED: added the Jetson/L4T repo. The nvidia/cuda base only carries the
    # generic sbsa CUDA repo -- it has no VPI (libnvvpi4 / vpi4-dev) and not the
    # cuda-toolkit-13-2 metapackage that Isaac ROS nitros/common pin. Those live
    # in repo.download.nvidia.com/jetson/common. Suite r39.2 == JetPack 7.2.x
    # (Jetson Linux 39.2.x); bump this when you move JetPack minor again.
    # This repo IS signed on r39.2 (InRelease + Release.gpg present), so the
    # jetson-ota-public key + signed-by is enough -- no [trusted=yes] needed.
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o /usr/share/keyrings/nvidia-l4t.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-l4t.gpg] https://repo.download.nvidia.com/jetson/common r39.2 main" > /etc/apt/sources.list.d/nvidia-l4t.list && \
    rm -rf /var/lib/apt/lists/*

# CHANGED: libopencv-dev REMOVED from this list.
# Isaac ROS 4.6 is built against OpenCV 4.6.0 and pulls it in itself. Ubuntu's
# libopencv-dev is 4.6.x on noble, but installing it explicitly can win the
# dependency resolution and shadow NVIDIA's build. Let Isaac ROS supply it.
# (On a JetPack 7.2 *host*, NVIDIA tells you to `apt remove libopencv* opencv*`
# for the same reason.)
# CHANGED: the nvidia/cuda base image holds libcublas-13-2 / libcublas-dev-13-2
# at 13.4.0.1-1 (see `apt-mark showhold`). The Jetson repo's cuda-libraries-13-2
# -- pulled in transitively by ros-jazzy-isaac-ros-* via cuda-toolkit-13-2 --
# requires libcublas-13-2 >= 13.4.1.3, so the hold has to be released first or
# apt reports "held broken packages".
RUN apt-get update && \
    apt-mark unhold libcublas-13-2 libcublas-dev-13-2 && \
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
    # CHANGED: drop the CUDA forward-compat packages. The nvidia/cuda base ships
    # cuda-compat-13-2 and the Jetson repo adds cuda-compat-orin-13-2; on a
    # Jetson the CUDA driver is provided by the L4T host and mounted in, so these
    # are dead weight. Worse, nvidia-container-toolkit 1.19.x panics in its
    # `cudacompat` hook while parsing their libcuda.so headers, which makes the
    # container fail to start under `--runtime=nvidia` / `--gpus all`.
    # NVIDIA's own l4t base images do not carry these.
    apt-get purge -y cuda-compat-13-2 cuda-compat-orin-13-2 && \
    rm -rf /usr/local/cuda-*/compat /usr/local/cuda-*/compat_orin && \
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