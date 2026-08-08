#!/bin/bash
HERE=$(dirname ${BASH_SOURCE[0]})

if [[ "$USER" == 'northstar_agx' ]]; then
  echo "run this inside the container"
  exit 1
fi

colcon build --symlink-install --event-handlers console_direct+ --packages-up-to localization --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ${HERE}/../install/setup.bash
ros2 launch localization headless-launch.py
