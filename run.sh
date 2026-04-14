#!/bin/bash

if (("$#" != 1)); then 
  echo "You need to provide a package name"  
  exit 1
fi

colcon build --symlink-install --packages-up-to "$1" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja 
source install/setup.bash
ros2 launch "$1" launch.py
