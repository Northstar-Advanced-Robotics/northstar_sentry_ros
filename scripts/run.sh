#!/bin/bash
# Build a package (and its deps) then launch it - run INSIDE the container.
#   ./scripts/run.sh <package_name>        # runs <package>/launch/launch.py
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_sentry_env.sh"
if ! sentry_in_container; then
  echo "Run this inside the container (./scripts/ros_enter.sh)." >&2
  exit 1
fi

if (($# != 1)); then
  echo "Usage: $0 <package_name>" >&2
  exit 1
fi

cd "$SENTRY_REPO"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

colcon build --symlink-install --event-handlers console_direct+ \
  --packages-up-to "$1" \
  --cmake-args -G Ninja -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
exec ros2 launch "$1" launch.py
