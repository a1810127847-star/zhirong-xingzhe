#!/usr/bin/env bash
set -eo pipefail

export DISPLAY=:10
export XAUTHORITY=/home/lddcyfy/.Xauthority

exec xterm \
  -T "Zhirong SLAM Mapping" \
  -hold \
  -e bash --noprofile --norc -c \
  ". /opt/ros/humble/setup.bash; \
   . /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash; \
   exec ros2 launch zhirong_bringup mapping.launch.py"
