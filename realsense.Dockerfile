ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base
ENV ROS_DISTRO=${ROS_DISTRO}

RUN ["/bin/bash", "-c", "\
  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ${HOME}/.bashrc && \
  echo 'eval \"$(register-python-argcomplete ros2)\"' >> ${HOME}/.bashrc && \
  echo 'eval \"$(register-python-argcomplete colcon)\"' >> ${HOME}/.bashrc && \
  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/ubuntu/.bashrc && \
  echo 'eval \"$(register-python-argcomplete ros2)\"' >> /home/ubuntu/.bashrc && \
  echo 'eval \"$(register-python-argcomplete colcon)\"' >> /home/ubuntu/.bashrc \
"]

RUN ["/bin/bash", "-c", "\
  apt-get update -y && \
  apt-get install ros-${ROS_DISTRO}-rviz2 -y && \
  apt-get install ros-${ROS_DISTRO}-librealsense2 -y && \
  apt-get install ros-${ROS_DISTRO}-realsense2-* -y && \
  apt-get install ros-${ROS_DISTRO}-ament-cmake-clang-format -y && \
  apt-get install libboost-all-dev -y && \
  apt-get install ros-${ROS_DISTRO}-gtsam -y && \
  apt-get install ros-${ROS_DISTRO}-apriltag-detector -y && \
  apt-get install ros-${ROS_DISTRO}-apriltag-detector-umich -y && \
  apt-get install ros-${ROS_DISTRO}-ament-clang-format -y && \
  apt-get install ros-${ROS_DISTRO}-robot-localization -y \
"]

RUN ["/bin/bash", "-c", "\
  apt-get update -y && \
  apt-get install libasio-dev -y && \
  apt-get install clangd -y && \
  apt-get install ros-${ROS_DISTRO}-apriltag-ros -y && \
  apt-get install ninja-build -y \
"]

ENV PATH="/devtools:${PATH}"
