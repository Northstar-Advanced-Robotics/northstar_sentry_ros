#!/usr/bin/env bash
#
# CLI entry point for the dev container. Builds the image if needed, then drops
# you into a shell inside it. Run from anywhere in the repo:
#
#     ./scripts/dev.sh            # interactive shell
#     ./scripts/dev.sh cmake -B build   # run one command and exit
#
# NOTE: container flags live in two places and must be kept in sync:
#   - RUN_FLAGS below                      -> CLI workflow
#   - "runArgs" in .devcontainer/devcontainer.json -> VS Code workflow
# If you add a device or network flag to one, add it to the other.

set -euo pipefail

IMAGE="northstar/sentry:dev"
DOCKERFILE=".devcontainer/Dockerfile"

RUN_FLAGS=(
  # "--gpus",
	# "all"
)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# Prefer docker, fall back to podman.
ENGINE="docker"
command -v docker >/dev/null 2>&1 || ENGINE="podman"

# SELinux (Fedora/RHEL) and podman need the :Z relabel on bind mounts.
MOUNT_OPT=""
if [ "$ENGINE" = "podman" ] || [ -d /sys/fs/selinux ]; then
  MOUNT_OPT=":Z"
fi

"$ENGINE" build -t "$IMAGE" -f "$DOCKERFILE" .

exec "$ENGINE" run -it --rm \
  -v "${REPO_ROOT}:/ws${MOUNT_OPT}" \
  -w /ws \
  ${RUN_FLAGS[@]+"${RUN_FLAGS[@]}"} \
  "$IMAGE" "${@:-bash}"
