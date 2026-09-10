#!/bin/bash
# Boot entry point for the localization stack WITHOUT rviz
# (localization-headless.service). Runs on the HOST.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_sentry_env.sh"

# Headless, but give X up to 60s in case a driver still wants a display.
sentry_wait_for_x 60 || echo "No X server after 60s - continuing headless."

arducam="/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0"
cam1="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_825513025416-video-index0"
cam2="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_825513025424-video-index0"
cam3="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_415_Intel_R__RealSense_TM__Depth_Camera_415_844513021088-video-index0"

if [ ! -e "$arducam" ] || [ ! -e "$cam1" ] || [ ! -e "$cam2" ] || [ ! -e "$cam3" ]; then
  echo "A camera is not plugged in - aborting." >&2
  exit 1
fi

docker rm -f "$SENTRY_CONTAINER" 2>/dev/null || true
sentry_docker_args
exec docker run --rm "${SENTRY_DOCKER_ARGS[@]}" "$SENTRY_IMAGE" \
  bash -c './scripts/headless.sh'
