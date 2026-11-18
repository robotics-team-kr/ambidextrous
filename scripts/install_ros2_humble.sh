#!/usr/bin/env bash
# ROS 2 Humble + MoveIt 2 + Gazebo Fortress 설치 (Ubuntu 22.04 "jammy" 전용).
#
#   sudo bash scripts/install_ros2_humble.sh
#
# 하는 일:
#   1. ROS 2 apt 저장소(jammy)가 없으면 추가합니다. 이미 있으면 건드리지 않습니다.
#   2. 이 워크스페이스가 쓰는 ROS 패키지를 apt 로 설치합니다.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "sudo 로 실행하세요:  sudo bash $0" >&2
  exit 1
fi

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
  echo "이 스크립트는 Ubuntu 22.04(jammy) 전용입니다. 현재: ${VERSION_CODENAME:-unknown}" >&2
  exit 1
fi

KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ros2.list

if [ ! -s "$KEYRING" ]; then
  apt-get update
  apt-get install -y curl ca-certificates
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$KEYRING"
fi

if ! grep -rqs "packages.ros.org/ros2/ubuntu.* jammy " /etc/apt/sources.list /etc/apt/sources.list.d/; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu jammy main" > "$LIST"
fi

apt-get update
apt-get install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-description \
  python3-colcon-common-extensions

echo
echo "설치 완료. 새 터미널에서:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd $(dirname "$(dirname "$(readlink -f "$0")")") && colcon build --symlink-install"
