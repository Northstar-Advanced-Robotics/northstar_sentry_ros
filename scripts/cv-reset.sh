#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

while true;
do
  ros2 run detector detector
  echo Restarting CV
done
