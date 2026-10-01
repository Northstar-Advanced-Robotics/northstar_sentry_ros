#!/bin/bash
# Shared launch config for the Northstar Sentry container (Jazzy / JetPack 7).
# Source this - do not execute it:  source "$(dirname ...)/scripts/_sentry_env.sh"
#
# After sourcing, call `sentry_docker_args` to populate the SENTRY_DOCKER_ARGS
# array with every `docker run` flag common to ros_enter.sh / startup*.sh, then:
#   docker run ... "${SENTRY_DOCKER_ARGS[@]}" "$SENTRY_IMAGE" <cmd>

SENTRY_IMAGE="${SENTRY_IMAGE:-sentry:latest}"
SENTRY_CONTAINER="${SENTRY_CONTAINER:-ros-container}"
SENTRY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SENTRY_CONTAINER_HOME="$SENTRY_REPO/.container_home"

sentry_docker_args() {
  mkdir -p "$SENTRY_CONTAINER_HOME"

  SENTRY_DOCKER_ARGS=(
    --name "$SENTRY_CONTAINER"
    --net=host --ipc=host --pid=host --privileged
    --runtime=nvidia
    --shm-size=8g
    -e NVIDIA_DRIVER_CAPABILITIES=all
    -e DISPLAY -e TERM -e COLORTERM -e QT_X11_NO_MITSHM=1
    # Run as the host user so files written to /ws stay yours. The host has no
    # matching passwd entry, so mount it in and redirect HOME to a gitignored
    # dir inside the repo (dotfiles persist between runs, repo root stays clean).
    --user "$(id -u):$(id -g)"
    -e HOME=/ws/.container_home
    -v /etc/passwd:/etc/passwd:ro
    -v /etc/group:/etc/group:ro
    --group-add video --group-add dialout
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v /run/udev:/run/udev:ro
    -v /dev:/dev
    -v "$SENTRY_REPO:/ws"
    -w /ws
  )

  # FastDDS profile (repo copy).
  if [ -e "$SENTRY_REPO/fastdds.xml" ]; then
    SENTRY_DOCKER_ARGS+=(
      -v "$SENTRY_REPO/fastdds.xml:/opt/ros/fastdds.xml:ro"
      -e "FASTRTPS_DEFAULT_PROFILES_FILE=/opt/ros/fastdds.xml"
    )
  fi

  # X11 cookie (optional - `xhost +local:` also works).
  if [ -e "$HOME/.Xauthority" ]; then
    SENTRY_DOCKER_ARGS+=(
      -v "$HOME/.Xauthority:/tmp/.host.Xauthority:ro"
      -e "XAUTHORITY=/tmp/.host.Xauthority"
    )
  fi

  [ -d "$HOME/.tar-installs" ]  && SENTRY_DOCKER_ARGS+=( -v "$HOME/.tar-installs:/devtools" )
  [ -d "$HOME/.config/helix" ] && SENTRY_DOCKER_ARGS+=(
    -v "$HOME/.config/helix:/ws/.container_home/.config/helix" )

  # VPI 4 + PVA backend from the JetPack 7 host (JP6 used /opt/nvidia/vpi3).
  local p
  for p in /opt/nvidia/vpi4 /opt/nvidia/pva-sdk-2.9 /opt/nvidia/pva-allow-2; do
    [ -d "$p" ] && SENTRY_DOCKER_ARGS+=( -v "$p:$p:ro" )
  done
}

# True when running inside the sentry container (used by the build/run scripts
# to refuse to run on the host - the old `$USER == northstar_agx` check broke
# once the container started running as the host user too).
sentry_in_container() { [ -f /.dockerenv ]; }

# Wait for an X server and export a working DISPLAY + XAUTHORITY.
#   sentry_wait_for_x [max_seconds]   (0/omitted = wait forever)
# The display number is not consistent across setups (:0 on some, :1 on others),
# so detect it from the socket rather than hard-coding.
sentry_wait_for_x() {
  local max="${1:-0}" waited=0
  while ! ls /tmp/.X11-unix/X* >/dev/null 2>&1; do
    if [ "$max" -gt 0 ] && [ "$waited" -ge "$max" ]; then return 1; fi
    echo "Waiting for graphical desktop..."
    sleep 2; waited=$((waited + 2))
  done
  if [ -z "${DISPLAY:-}" ] || [ ! -e "/tmp/.X11-unix/X${DISPLAY##*:}" ]; then
    local sock; sock="$(ls /tmp/.X11-unix/X* 2>/dev/null | head -1)"
    export DISPLAY=":${sock##*/X}"
  fi
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
  echo "Using DISPLAY=$DISPLAY"
}
