#!/bin/bash

HERE=$(dirname ${BASH_SOURCE[0]})

if [[ "$USER" == 'northstar_agx' ]]; then
  echo "run this inside the container"
  exit 1
fi

if (("$#" != 1)); then 
  echo "You need to provide a package name"  
  exit 1
fi

colcon build --symlink-install --event-handlers console_direct+ --packages-up-to "$1" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_COLOR_DIAGNOSTICS=ON -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ${HERE}/../install/setup.bash
ros2 launch "$1" launch.py
