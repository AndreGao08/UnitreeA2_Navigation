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
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-pcl-ros \
  ros-humble-sensor-msgs-py \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  python3-pytest \
  python3-numpy \
  libboost-all-dev \
  libeigen3-dev \
  libpcl-dev \
  libgtest-dev \
  libnanoflann-dev \
  libssl-dev \
  libyaml-cpp-dev
