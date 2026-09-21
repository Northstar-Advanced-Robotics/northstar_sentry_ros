#!/bin/bash
# Build the full localization stack then launch it headless (no rviz) -
# run INSIDE the container.
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_sentry_env.sh"
if ! sentry_in_container; then
  echo "Run this inside the container (./scripts/ros_enter.sh)." >&2
  exit 1
fi

cd "$SENTRY_REPO"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

colcon build --symlink-install --event-handlers console_direct+ \
  --packages-up-to localization \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
exec ros2 launch localization headless-launch.py
