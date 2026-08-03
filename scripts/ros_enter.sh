#!/bin/bash

if [ -n "$(docker ps -f "name=^/ros-container$" -q )" ]; then
  docker exec -it ros-container bash -i
else
  docker run -it --rm --net=host --ipc=host --pid=host --privileged \
    -e="DISPLAY" -e="TERM" -e="QT_X11_NO_MITSHM=1" -e="COLORTERM" \
    --runtime=nvidia \
    --group-add video \
    --group-add dialout \
    -v="/run/udev:/run/udev:ro" \
    -v="/dev:/dev" \
    -v="/opt/nvidia/vpi3:/opt/nvidia/vpi3:ro" \
    -e="NVIDIA_DRIVER_CAPABILITIES"=all \
    -e="FASTRTPS_DEFAULT_PROFILES_FILE"=/opt/ros/fastdds.xml \
    -v="$HOME/Docker/realsense_ws/fastdds.xml:/opt/ros/fastdds.xml" \
    -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v="${HOME}/.Xauthority:/home/ubuntu/.Xauthority" \
    -v="${HOME}/Docker/realsense_ws:/home/ubuntu/realsense_ws" \
    -w="/home/ubuntu/realsense_ws" \
    --user=$(id -u):$(id -g) \
    --name="ros-container" \
    -v="${HOME}/.tar-installs:/devtools" \
    -v="${HOME}/.config/helix:/home/ubuntu/.config/helix" \
    sentry:latest bash -i
fi
