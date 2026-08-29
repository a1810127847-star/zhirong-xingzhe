#!/usr/bin/env bash
set -e

export DISPLAY="${DISPLAY:-:10}"
export XAUTHORITY="${XAUTHORITY:-/home/lddcyfy/.Xauthority}"
export QT_X11_NO_MITSHM=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=/home/lddcyfy/zhirong_xingzhe_ws/install/zhirong_gazebo/share/zhirong_gazebo/models

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

exec xterm \
  -T "Zhirong Complete System" \
  -hold \
  -e bash --noprofile --norc -c '
    . /opt/ros/humble/setup.bash
    . /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
    export GAZEBO_MODEL_DATABASE_URI=
    export GAZEBO_MODEL_PATH=/home/lddcyfy/zhirong_xingzhe_ws/install/zhirong_gazebo/share/zhirong_gazebo/models
    exec ros2 launch zhirong_bringup system.launch.py
  '
