#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

params_file="$(ros2 pkg prefix zhirong_ppo)/share/zhirong_ppo/config/ppo_collision_monitor.yaml"

exec ros2 launch zhirong_bringup simulation.launch.py \
  gui:=false \
  rviz:=false \
  slam:=false \
  safety_monitor:=true \
  collision_monitor_params:="${params_file}"
