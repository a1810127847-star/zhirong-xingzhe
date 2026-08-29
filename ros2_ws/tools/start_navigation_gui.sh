#!/usr/bin/env bash
set -eo pipefail

export DISPLAY=:10
export XAUTHORITY=/home/lddcyfy/.Xauthority
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=/home/lddcyfy/zhirong_xingzhe_ws/install/zhirong_gazebo/share/zhirong_gazebo/models

exec xterm \
  -T "Zhirong Nav2 Navigation" \
  -hold \
  -e bash --noprofile --norc -c \
  ". /opt/ros/humble/setup.bash; \
   . /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash; \
   export GAZEBO_MODEL_DATABASE_URI=; \
   export GAZEBO_MODEL_PATH=/home/lddcyfy/zhirong_xingzhe_ws/install/zhirong_gazebo/share/zhirong_gazebo/models; \
   exec ros2 launch zhirong_bringup navigation.launch.py"
