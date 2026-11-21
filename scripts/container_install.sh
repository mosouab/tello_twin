#!/usr/bin/env bash
# Runs INSIDE the jazzy distrobox: ROS 2 Jazzy, Gazebo Harmonic, Python tools and the venv.
set -euo pipefail
APT="sudo env DEBIAN_FRONTEND=noninteractive apt-get"
$APT update
$APT install -y software-properties-common curl
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  V=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
$APT update
$APT install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap ros-dev-tools \
  python3-venv python3-pytest python3-numpy python3-yaml python3-scipy xterm
$APT install -y ros-jazzy-vrpn-mocap \
  || echo "WARNING: ros-jazzy-vrpn-mocap not available from apt; Plan 4 builds it from source."
if [ ! -d "$HOME/venvs/tello" ]; then
  python3 -m venv --system-site-packages "$HOME/venvs/tello"
fi
echo "container ready"
