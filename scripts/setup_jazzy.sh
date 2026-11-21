#!/usr/bin/env bash
# Run on the HOST (CachyOS). Creates the `jazzy` distrobox (Ubuntu 24.04) and installs everything inside it.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! distrobox list | grep -qw jazzy; then
  distrobox create --yes --name jazzy --image docker.io/library/ubuntu:24.04
fi
distrobox enter jazzy -- bash "$HERE/container_install.sh"
