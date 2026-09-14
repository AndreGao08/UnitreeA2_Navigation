#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-pcl-ros \
  ros-humble-sensor-msgs-py \
  python3-pytest \
  libeigen3-dev \
  libpcl-dev
