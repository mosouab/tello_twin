#!/usr/bin/env bash
# Run a shell command inside the `jazzy` distrobox with ROS 2 Jazzy, the venv and the workspace sourced.
# Usage (from anywhere on the host): scripts/in_jazzy.sh "<command>"
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec distrobox enter jazzy -- bash -c '
  source /opt/ros/jazzy/setup.bash
  source "$HOME/venvs/tello/bin/activate"
  cd "$0"
  # Limit colcon to src/ (see colcon_defaults.yaml) unless the caller chose another defaults file.
  export COLCON_DEFAULTS_FILE="${COLCON_DEFAULTS_FILE:-$PWD/colcon_defaults.yaml}"
  if [ -f install/setup.bash ]; then source install/setup.bash; fi
  set -e
  eval "$1"' "$ROOT" "$1"
