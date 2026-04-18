#!/bin/bash

if [ -n "$(docker ps -f "name=^/ros-container$" -q )" ]; then
  docker exec -it ros-container bash
else
  docker run -it --rm --net=host --ipc=host --pid=host --privileged \
    -e="DISPLAY" -e="TERM" -e="QT_X11_NO_MITSHM=1" -e="COLORTERM" \
    --gpus all \
    -e="NVIDIA_DRIVER_CAPABILITIES"=all \
    -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v="${HOME}/.Xauthority:/home/ubuntu/.Xauthority" \
    -v="/dev:/dev" \
    -v="/run/udev:/run/udev:ro" \
    -v="${HOME}/Docker/realsense_ws:/home/ubuntu/realsense_ws" \
    -v="${HOME}/.tar-installs:/devtools" \
    -v="${HOME}/.config/helix:/home/ubuntu/.config/helix" \
    -v="${HOME}/.vscode-server:/home/ubuntu/.vscode-server" \
    -v="${HOME}/.vscode-server:/root/.vscode-server" \
    -w="/home/ubuntu/realsense_ws" \
    --device=/dev/ttyTHS1 \
    --group-add dialout \
    --shm-size=4g \
    --userns=host \
    --user=$(id -u):$(id -g) \
    --name="ros-container" \
    ros:nvidia bash -i
fi
