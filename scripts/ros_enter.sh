#!/bin/bash
# Enter the Northstar Sentry ROS 2 container (Jazzy / JetPack 7).
#
#   * repo is bind-mounted at /ws  (== Dockerfile WORKDIR; CCACHE_DIR=/ws/.ccache,
#     ROS_LOG_DIR=/ws/log/runtime, all .gitignore'd)
#   * runs as the host uid:gid; HOME -> /ws/.container_home (see _sentry_env.sh)
#   * ROS 2 Jazzy is auto-sourced by /etc/bash.bashrc, so `bash -i` is enough
#
# Override the image tag with:  SENTRY_IMAGE=my:tag ./scripts/ros_enter.sh
#
# X11: if GUI apps (rviz2) can't open the display, run once on the host:
#   xhost +local:
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_sentry_env.sh"

# Can we even reach the docker daemon?
if ! docker version >/dev/null 2>&1; then
  echo "Cannot talk to the Docker daemon:" >&2
  docker version 2>&1 | sed 's/^/  /' >&2
  echo "If this is a permission error, add yourself to the docker group:" >&2
  echo "  sudo usermod -aG docker \$USER && newgrp docker" >&2
  exit 1
fi

# Reuse a running container if one is already up.
if [ -n "$(docker ps -f "name=^/${SENTRY_CONTAINER}$" -q)" ]; then
  exec docker exec -it "$SENTRY_CONTAINER" bash -i
fi

if ! docker image inspect "$SENTRY_IMAGE" >/dev/null 2>&1; then
  echo "Image '$SENTRY_IMAGE' not found. Build it first:" >&2
  echo "  docker build -t $SENTRY_IMAGE \"$SENTRY_REPO\"" >&2
  exit 1
fi

sentry_docker_args
exec docker run -it --rm "${SENTRY_DOCKER_ARGS[@]}" "$SENTRY_IMAGE" bash -i
