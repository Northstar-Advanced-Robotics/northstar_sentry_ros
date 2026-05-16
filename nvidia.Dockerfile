ARG ROS_DISTRO=jazzy
FROM nvidia/cuda:12.6.3-devel-ubuntu24.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y software-properties-common && \
  apt-add-repository universe && \
  apt-get update && apt-get install curl -y &&\
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') &&\
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" &&\
  dpkg -i /tmp/ros2-apt-source.deb 

ENV ROS_DISTRO=jazzy

# heavy stuff here
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-jazzy-ros-base \
    ros-dev-tools \
    ros-${ROS_DISTRO}-rviz2  \
    ros-${ROS_DISTRO}-librealsense2  \
    ros-${ROS_DISTRO}-realsense2-*  \
    ros-${ROS_DISTRO}-ament-cmake-clang-format  \
    ros-${ROS_DISTRO}-gtsam  \
    ros-${ROS_DISTRO}-apriltag-detector  \
    ros-${ROS_DISTRO}-apriltag-detector-umich  \
    ros-${ROS_DISTRO}-ament-clang-format  \
    ros-${ROS_DISTRO}-robot-localization \
    libboost-all-dev  

RUN apt-get update -y && \
  apt-get install -y libasio-dev \
  clangd \
  ros-${ROS_DISTRO}-librealsense2 \
  ros-${ROS_DISTRO}-apriltag-ros \
  ros-${ROS_DISTRO}-v4l2-camera \
  libfastcdr-dev \
  libopencv-dev \
  bash-completion \
  python3-argcomplete \
  ninja-build \
  gdb 

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ${HOME}/.bashrc && \
  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/ubuntu/.bashrc && \
  register-python-argcomplete ros2 > /etc/bash_completion.d/ros2 && \
  register-python-argcomplete colcon > /etc/bash_completion.d/colcon 

ENV PATH="/devtools:${PATH}"
ENV ROS_LOG_DIR="/home/ubuntu/realsense_ws/logs"
