#!/usr/bin/env bash
set -eo pipefail

. /opt/ros/humble/setup.bash
. /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

exec ros2 run zhirong_bringup gamepad_udp_teleop.py
