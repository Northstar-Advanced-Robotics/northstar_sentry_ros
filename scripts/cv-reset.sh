#!/bin/bash
# Keep the detector (CV) node alive - restart it whenever it exits.
# Run INSIDE the container; the cv-watchdog.service invokes this via docker exec.
set -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_sentry_env.sh"
cd "$SENTRY_REPO"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source install/setup.bash

while true; do
  ros2 run detector detector
  echo "Restarting CV..."
  sleep 1
done
