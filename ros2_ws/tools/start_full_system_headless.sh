#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

exec ros2 launch zhirong_bringup system.launch.py \
  gui:=false \
  navigation_rviz:=false
