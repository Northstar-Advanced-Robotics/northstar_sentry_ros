#!/bin/bash

export DISPLAY=:1
export XAUTHORITY=/home/northstar_agx/.Xauthority

# WAIT for the physical X11 display socket to exist before continuing
while [ ! -e /tmp/.X11-unix/X1 ]; do
  echo "Waiting for Graphical Desktop to load..."
  sleep 2
done

arducam="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0"
cam1="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_825513025416-video-index0"
cam2="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_825513025424-video-index0"
cam3="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_415_Intel_R__RealSense_TM__Depth_Camera_415_844513021088-video-index0"

if [ -e "$arducam" ] && [ -e "$cam1" ] && [ -e "$cam2" ] && [ -e "$cam3" ]; then
  docker run --rm --net=host --ipc=host --pid=host --privileged \
    -e="DISPLAY" -e="TERM" -e="QT_X11_NO_MITSHM=1" -e="COLORTERM" \
    --runtime=nvidia \
    --shm-size=8g \
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
    sentry:latest bash -c "source /opt/ros/humble/setup.bash && ./scripts/run.sh localization"
else
  echo "A camera is not plugged in"
fi

