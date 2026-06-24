#!/usr/bin/env bash
set -Eeuo pipefail

packages=(
  ros-jazzy-fastcdr
  ros-jazzy-fastrtps
  ros-jazzy-fastrtps-cmake-module
  ros-jazzy-rmw-dds-common
  ros-jazzy-rmw-fastrtps-cpp
  ros-jazzy-rmw-fastrtps-shared-cpp
  ros-jazzy-rosidl-dynamic-typesupport-fastrtps
  ros-jazzy-rosidl-typesupport-fastrtps-c
  ros-jazzy-rosidl-typesupport-fastrtps-cpp
)

printf 'Installing ROS Jazzy Fast-DDS ABI alignment packages:\n'
printf '  %s\n' "${packages[@]}"
sudo apt-get install -y "${packages[@]}"
