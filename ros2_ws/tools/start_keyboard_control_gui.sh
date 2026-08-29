#!/usr/bin/env bash
set -eo pipefail

export DISPLAY=:10
export XAUTHORITY=/home/lddcyfy/.Xauthority

. /opt/ros/humble/setup.bash
. /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

exec ros2 run zhirong_bringup keyboard_hold_teleop.py
